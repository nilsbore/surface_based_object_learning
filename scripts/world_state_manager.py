#!/usr/bin/env python

# general stuff
import roslib
import rospy
import sys
import argparse
import os
from random import randint
import cv2

# view store STUFF
from mongodb_store.message_store import MessageStoreProxy
from soma_io.mongo import MongoDocument, MongoTransformable, MongoConnection
from soma_llsd_msgs.msg import Segment,Observation,Scene

# ROS stuff
from sensor_msgs.msg import PointCloud2, PointField
from segment_processing import SegmentProcessor
from recognition_manager import ObjectRecognitionManager
from view_registration import ViewAlignmentManager
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, JointState
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import Pose,Point,Quaternion
import tf

# WS stuff
from soma_io.observation import *
from soma_io.geometry import *
from soma_io.state import World, Object
from world_modeling.srv import *

# SOMA2 stuff
from soma2_msgs.msg import SOMA2Object
from soma_manager.srv import *

# recog stuff
from recognition_srv_definitions.srv import *

# people tracker stuff #
from bayes_people_tracker.msg import PeopleTracker
from vision_people_logging.msg import LoggingUBD
from human_trajectory.msg import Trajectories

import uuid
import python_pcd




class WorldStateManager:

    def __init__(self,db_hostname,db_port):
        rospy.init_node('world_state_modeling', anonymous = False)
        self.setup_clean = False
        rospy.loginfo("Manager Online")
        # make a segment tracker
        rospy.loginfo("looking for camera info topic")

        self.world_model = World(server_host=db_hostname,server_port=int(db_port))
        rospy.loginfo("world model done")

        self.segment_tracker = SegmentProcessor()
        self.pending_obs = []
        self.cur_sequence_obj_ids = []
        self.cur_view_soma_ids = []
        self.cur_observation_data = None

        rospy.loginfo("setting up services")
        update_world_state = rospy.Service('update_world_model',WorldUpdate,self.object_segment_callback)
        rospy.loginfo("world update service running")
        update_person_state = rospy.Service('update_person_model',PersonUpdate,self.person_segment_callback)
        rospy.loginfo("person update service running")

        begin_observations = rospy.Service('/begin_observations',Trigger,self.begin_obs)
        end_observations = rospy.Service('/end_observations',Trigger,self.end_obs)

        rospy.loginfo("setting up SOMA services")
        rospy.loginfo("getting SOMA insert service")
        rospy.wait_for_service('soma2/insert_objects')
        rospy.loginfo("done")
        self.soma_insert = rospy.ServiceProxy('soma2/insert_objects',SOMA2InsertObjs)

        rospy.loginfo("getting SOMA query service")
        rospy.wait_for_service('soma2/query_db')
        rospy.loginfo("done")
        self.soma_get = rospy.ServiceProxy('soma2/query_db',SOMA2QueryObjs)

        rospy.loginfo("getting SOMA update service")
        rospy.wait_for_service('/soma2/update_object')
        rospy.loginfo("done")
        self.soma_update = rospy.ServiceProxy('soma2/update_object',SOMA2UpdateObject)
        self.recog_manager = None
        rospy.loginfo("setting up view alignment manager")
        self.view_alignment_manager = ViewAlignmentManager()

        rospy.loginfo("getting LLSD services")
        rospy.wait_for_service('/soma_llsd/insert_scene')
        self.view_store_insert = rospy.ServiceProxy('/soma_llsd/insert_scene',InsertScene)

        rospy.wait_for_service('/soma_llsd/get_segment')
        self.get_segment = rospy.ServiceProxy('/soma_llsd/get_segment',GetSegment)

        rospy.wait_for_service('/soma_llsd/insert_segment')
        self.insert_segment = rospy.ServiceProxy('/soma_llsd/insert_segment',InsertSegment)

        rospy.wait_for_service('/soma_llsd/add_observations_to_segment')
        self.append_obs_to_segment = rospy.ServiceProxy('/soma_llsd/add_observations_to_segment',AddObservationsToSegment)

        self.clean_up_obs()

        rospy.loginfo("-- node setup completed --")
        self.setup_clean = True

        #rospy.spin()

    def person_segment_callback(self,req):
        if(self.setup_clean is False):
            rospy.loginfo("-- world_modeling node is missing one or more key services, cannot act --")
        else:
            pid = req.id
            self.cur_waypoint = req.waypoint
            self.assign_people(pid)
            return PersonUpdateResponse(True)

    def clean_up_obs(self):
        rospy.loginfo("running cleanup")
        self.pending_obs = []
        self.cur_sequence_obj_ids = []
        self.cur_view_soma_ids = []
        self.segment_tracker.reset()
        self.cur_episode_id = str(uuid.uuid4())


    def begin_obs(self,req):
        rospy.loginfo("-- received signal to begin sequence of observations --")
        if(self.setup_clean):
            rospy.loginfo("ready to go")
        else:
            rospy.loginfo("ERROR: node setup not completed yet, wait a sec and try again")
            return
        self.clean_up_obs()
        return TriggerResponse(True,"Observations Beginning: Assuming all subsequent observations are from the same sequence.")

    def end_obs(self,req):
        rospy.loginfo("-- received signal to terminate sequence of observations --")
        rospy.loginfo("")
        self.do_postprocessing()
        return TriggerResponse(True,"Observations Ending: Assuming all previous observations were from the same sequence.")

    def get_camera_info_topic(self):
        camera_msg = None
        try:
            camera_msg = rospy.wait_for_message("/head_xtion/depth_registered/sw_registered/camera_info",  CameraInfo, timeout=2)
            return "/head_xtion/depth_registered/sw_registered/camera_info"
        except Exception,e:
            rospy.loginfo("couldn't find /head_xtion/depth_registered/sw_registered/camera_info")

        if(not camera_msg):
            try:
                camera_msg = rospy.wait_for_message("/head_xtion/depth_registered/camera_info",  CameraInfo, timeout=2)
                return "/head_xtion/depth_registered/camera_info"
            except Exception,e:
                rospy.loginfo("couldn't find /head_xtion/depth_registered/camera_info")

        return None

    def assign_people(self,pid):
        rospy.loginfo("assigning")
        # do we have a low-level object with this key?
            # if so get it out
            # if not, create it
        exists = self.world_model.does_object_exist(pid)
        cur_person = None

        if(exists):
            cur_person = self.world_model.get_object(pid)

        if(not cur_person):
            rospy.loginfo("creating person entry")
            cur_person = self.world_model.create_object(pid)
            cur_person._parent = self.cur_waypoint
        else:
            rospy.loginfo("got this person already")


        # record this observation
        DEFAULT_TOPICS = [("/vision_logging_service/log", LoggingUBD),
                          ("/people_trajectory/trajectories/batch", Trajectories),
                          ("/robot_pose", geometry_msgs.msg.Pose),
                          ("/people_tracker/positions", PeopleTracker)]



        person_observation = Observation.make_observation(DEFAULT_TOPICS)
        cur_person.add_observation(person_observation)


        people_tracker_output = person_observation.get_message("/people_tracker/positions")
        # get the idx of the thing we want
        person_idx = ''
        try:
            person_idx = people_tracker_output.uuids.index(pid)
        except:
            rospy.logwarn(
                "Can not capture %s information, the person has gone" % pid
            )

        soma_objs=self.get_soma_objects_with_id(cur_person.key)
        cur_soma_person = None

        if(soma_objs.objects):
            rospy.loginfo("soma has this person")
            # we have a soma object with this id
            # retrieve it
            cur_soma_person = soma_objs.objects[0] # will only ever return 1 anyway, as keys are unique
        else:
            rospy.loginfo("soma doesn't have this person")
            # if this object is unknown, lets register a new unknown object in SOMA2
            # we do not have a soma object with this id
            # create it
            cur_soma_person = SOMA2Object()
            cur_soma_person.id = cur_person.key
            cur_soma_person.type = "person"
            cur_soma_person.waypoint = self.cur_waypoint

            # either way we want to record this, so just do it here?
            #cur_soma_person.cloud = cur_scene_segment_instance.segmented_pc_mapframe

            if person_idx != '':
                cur_soma_person.pose = people_tracker_output.poses[person_idx]

            msg = rospy.wait_for_message("/robot_pose",  geometry_msgs.msg.Pose, timeout=3.0)
            cur_soma_person.sweepCenter = msg

            rospy.loginfo("inserting person detection into SOMA")
            res = self.soma_insert([cur_soma_person])

        # update this object in some way
        # TODO: HOW?


    def flush_observation(self,data):
        print("-- flushing observation from dataset through system --")
        self.process_cloud(data['cloud'],data['data'][3],data)

    def register_with_view_store(self,cloud,extra_data=None):
        try:
            if(self.cur_observation_data is None):
                self.populate_observation_data(cloud,extra_data)


            # INSERT INTO THE VIEW STORE
            scene = self.view_store_insert(self.cur_episode_id,
            self.cur_observation_data['waypoint'],
            "{}", # meta_data
            self.cur_observation_data['timestamp'],
            self.cur_observation_data['tf'],
            self.cur_observation_data['scene_cloud'],
            self.cur_observation_data['rgb_image'],
            self.cur_observation_data['depth_image'],
            self.cur_observation_data['camera_info'],
            self.cur_observation_data['robot_pose'])

            if(scene.result is True):
                rospy.loginfo("successfully added scene to view store")
                self.cur_scene_id = scene.response.id
            else:
                rospy.logerr("couldn't add scene to view store, this is catastrophic")

        except Exception,e:
            rospy.loginfo("failed to add view to view store")


    def process_cloud(self,cloud,waypoint,extra_data=None):
        try:
            rospy.loginfo("---- Storing view in View Store ----")
            self.cur_waypoint = waypoint
            self.populate_observation_data(cloud,extra_data)
            self.register_with_view_store(cloud)


            rospy.loginfo("---- Segmenting Scene ----")
            scene = self.segment_tracker.add_unsegmented_scene(self.cur_observation_data,extra_data)
            if(scene.clean_setup is True):

                scene.waypoint = waypoint

                if(self.recog_manager):
                    rospy.loginfo("---- Running Object Recognition ----")
                    recognition = self.recog_manager.recognise_scene(cloud)
                    if(recognition is True):
                        self.recog_manager.assign_labels(scene)
                else:
                    rospy.logwarn("Object recognition service not found, try restarting is the node running?")

                self.assign_segments(scene,self.segment_tracker.prev_scene,extra_data)
                self.pending_obs.append(scene)

                rospy.loginfo("have: " + str(len(self.pending_obs)) + " clouds waiting to be processed")

                return WorldUpdateResponse(True,self.cur_view_soma_ids)
            else:
                rospy.loginfo("Error in processing scene")

        except Exception,e:
            rospy.logerr("Unable to segment and process this scene")
            rospy.logerr(e)

    def object_segment_callback(self, req):
        result = WorldUpdateResponse(False,self.cur_view_soma_ids)
        if(self.setup_clean is False):
            rospy.logerr("-- world_modeling node is missing one or more key services, cannot act --")
            rospy.logerr("-- run services and then re-start me --")
            return result
        else:

            if(req.input is None):
                rospy.logwarn("-- This point cloud looks empty, is the service being called correctly? ---")
                rospy.logwarn("-- Stopping Processing ---")
                return result

            result = self.process_cloud(req.input,req.waypoint)

            return result

    def do_postprocessing(self):
        rospy.loginfo("-- beginning post-processing, attempting view alignment and object label updates -- ")

        if(len(self.cur_sequence_obj_ids) == 0):
            rospy.loginfo("-- no segments found in this scene, or if they were they were filtered out by the SOMA region or height filters ")

        for object_id in self.cur_sequence_obj_ids:
            soma_object = self.get_soma_objects_with_id(object_id)
            segment = self.get_segment(object_id)

            observations = segment.observations
            rospy.loginfo("observations for " + str(object_id) + " = " + str(len(observations)))
            if(len(observations) >= 2):
                rospy.loginfo("processing...")
                # update world model
                try:
                    rospy.loginfo("updating world model")
                    merged_cloud = self.view_alignment_manager.register_views(segment)
                    rospy.loginfo("updating SOMA obj")
                    soma_objects.objects[0].cloud = merged_cloud

                    self.soma_update(object=soma_objects.objects[0],db_id=str(object_id))
                except Exception,e:
                    rospy.logerr("problem updating object models in world/SOMA db. Unable to register merged clouds")
                    rospy.logerr(e)
                    continue
            else:
                rospy.loginfo("not running view alignment, only one view")

            #rospy.loginfo("attempting to update object's recognition label")
            #try:
            #    soma_objects.objects[0].type = str(world_object.label)
            #    self.soma_update(object=soma_objects.objects[0],db_id=str(object_id))
            #    rospy.loginfo("done! this object recognised as a " + str(world_object.label) + " with confidence: " + str(world_object.label_confidence))
            #except Exception,e:
            #    rospy.logerr("Problem updating SOMA object label.")
            #    rospy.logerr(e)

        rospy.loginfo("post-processing complete")


    def add_soma_object(self,obj):
        rospy.loginfo("getting service")
        rospy.wait_for_service('soma2/insert_objects')
        rospy.loginfo("done")
        soma_insert = rospy.ServiceProxy('soma2/insert_objects',SOMA2InsertObjs)
        soma_insert(obj)

    def get_soma_objects_with_id(self,id):
        rospy.loginfo("looking for SOMA objects with id: " + str(id))
        query = SOMA2QueryObjsRequest()

        query.query_type = 0
        query.usetimestep = False
        query.uselowertime =  False
        query.useuppertime =  False
        query.usedates =  False
        query.useweekday =  False
        query.useroi =  False

        query.objectids = ([id])
        query.objecttypes=['']

        response = self.soma_get(query)


        return response

    def populate_observation_data(self,scene,extra_data=None):
        self.cur_observation_data = {}
        if(extra_data is None):
            rospy.loginfo("*** Making observation using live robot data")
            try:
                self.camera_info_topic = self.get_camera_info_topic()
                self.cur_observation_data['rgb_image'] = rospy.wait_for_message("/head_xtion/rgb/image_rect_color", Image, timeout=10.0)
                self.cur_observation_data['depth_image'] = rospy.wait_for_message("/head_xtion/depth/image_rect", Image, timeout=10.0)
                self.cur_observation_data['camera_info'] = rospy.wait_for_message(self.camera_info_topic, CameraInfo, timeout=10.0)
                self.cur_observation_data['scene_cloud'] = scene.unfiltered_cloud
                self.cur_observation_data['waypoint'] = self.cur_waypoint
                self.cur_observation_data['timestamp'] = int(rospy.Time.now().to_sec())
                self.cur_observation_data['robot_pose'] = rospy.wait_for_message("/robot_pose", geometry_msgs.msg.Pose, timeout=10.0)
                self.cur_observation_data['tf'] = rospy.wait_for_message("/tf", TFMessage, timeout=10.0)
            except rospy.ROSException, e:
                rospy.logwarn("Failed to get some observation data")
                rospy.logwarn(e)
        else:
                rospy.loginfo("*** Making observation using historic robot data")
                self.cur_observation_data['rgb_image'] = extra_data['rgb_image']
                self.cur_observation_data['camera_info'] = extra_data['camera_info']
                self.cur_observation_data['scene_cloud'] = extra_data['cloud']
                self.cur_observation_data['robot_pose'] = extra_data['robot_pose']
                self.cur_observation_data['metadata'] = extra_data['data']
                self.cur_observation_data['tf'] = extra_data['tf']
                self.cur_observation_data['depth_image'] = extra_data['depth_image']

        return self.cur_observation_data

    def assign_segments(self,scene,prev_scene,extra_data=None):
        rospy.loginfo("assigning")
        cur_scene = scene
        self.cur_view_soma_ids = []

        # if this is not scene 0, ie. we have a previous scene to compare to
        if not prev_scene:
            rospy.loginfo("don't have previous scene to compare to, assuming first run")

        if not cur_scene:
            rospy.loginfo("don't have anything in the current scene...")
            rospy.loginfo("did segmentation fail?")
            return

        # we iterate over the INSTANCES OF segmentS visible in the current scene
        # and see if we can link them to previously seen segments in prior views
        for cur_scene_segment_instance in cur_scene.segment_list:

            # this object is a Segment in the database
            # whereas cur_scene_segment_instance is an instance of a segment in memory in the current view
            target_db_segment = None

            # if there are previous views to look at it
            if(prev_scene):
                rospy.loginfo("seeing if prev scene contains: " + str(cur_scene_segment_instance.segment_id))
                for pc in prev_scene.segment_list:
                    rospy.loginfo(pc.segment_id)

                if(prev_scene.contains_segment_id(cur_scene_segment_instance.segment_id)):
                    rospy.loginfo("getting EXISTING segment")
                    target_db_segment = self.get_segment(cur_scene_segment_instance.id)

            # if this the first view, or a new segment
            if not target_db_segment:
                rospy.loginfo("creating NEW segment")

                # inserting a blank segment linked to the current scene we're observing
                request = self.insert_segment("{}",self.cur_scene_id,[])
                if(request.result is False):
                    rospy.logerr("Unable to insert segment, this is catastrophic")
                    return

                cur_scene_segment_instance.segment_id = request.response.id
                target_db_segment = request.response
                self.cur_sequence_obj_ids.append(target_db_segment.id)

            # from here we've either added this as a new object to the scene
            # or retreived the data for it in a previous scene
            if(target_db_segment):
                # so first add a new observation to it, in all cases
                rospy.loginfo("making observation")
                # add an observation for the object

                new_segment_observation = Observation()
                #new_segment_observation.id =  I'm ignoring this because if it's left blank, the service generates one for you
                new_segment_observation.timestamp = self.cur_observation_data['timestamp'],
                new_segment_observation.meta_data = "{}"

                new_segment_observation.pose =  cur_scene_segment_instance.map_centroid # centroid in map co-ordinates
                new_segment_observation.map_cloud =  cur_scene_segment_instance.segmented_pc_mapframe #segmented cloud in map co-ordinates
                new_segment_observation.camera_cloud = cur_scene_segment_instance.segmented_pc_camframe # segmented  cloud in camera co-ordinates
                new_segment_observation.room_cloud =   None # segmented cloud aligned to meta-room

                new_segment_observation.rgb_cropped = cur_scene_segment_instance.cropped_image
                new_segment_observation.depth_cropped = None
                new_segment_observation.rgb_masked = cur_scene_segment_instance.rgb_mask
                new_segment_observation.depth_masked = None

                self.append_obs_to_segment(target_db_segment.id,[new_segment_observation])

                cur_soma_obj = None
                soma_objs = self.get_soma_objects_with_id(target_db_segment.id)

                if(soma_objs.objects):
                    rospy.loginfo("soma has this object")
                    cur_soma_obj = soma_objs.objects[0]
                    # nothing to do in this case?

                else:
                    rospy.loginfo("soma doesn't have this object")
                    # if this object is unknown, lets register a new unknown object in SOMA2
                    # we do not have a soma object with this id
                    # create it
                    try:
                        cur_soma_obj = SOMA2Object()
                        cur_soma_obj.id = target_db_segment.segment_id
                        cur_soma_obj.type = "unknown"
                        cur_soma_obj.waypoint = self.cur_observation_data['waypoint']

                        # either way we want to record this, so just do it here?
                        cur_soma_obj.cloud = new_segment_observation.map_cloud
                        cur_soma_obj.pose = new_segment_observation.pose

                        if(extra_data is not None):
                            cur_soma_obj.logtimestamp = self.cur_observation_data['timestamp'] # jesus christ

                        cur_soma_obj.sweepCenter = self.cur_observation_data['robot_pose']

                        rospy.loginfo("inserting into SOMA")
                        res = self.soma_insert([cur_soma_obj])
                    except Exception, e:
                        rospy.logerr("unable to insert into SOMA. Is the database server running?")
                        rospy.logerr(e)

                # record all SOMA objects seen in this view
                self.cur_view_soma_ids.append(target_db_segment.id)

                rospy.loginfo("done")

        rospy.loginfo("DB Update Complete")
        rospy.loginfo("")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(prog='world_state_manager.py')
    parser.add_argument("db_hostname", nargs=1, help='DB Hostname')
    parser.add_argument('db_port', nargs=1, help="DB Port")

    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    if(len(sys.argv) < 2):
        rospy.loginfo("not enough args, need db hostname and port")
    else:
        hostname = str(vars(args)['db_hostname'][0])
        port = str(vars(args)['db_port'][0])

        rospy.loginfo("got db_hostname as: " + hostname + " got db_port as: " + port)
        world_state_manager = WorldStateManager(hostname,port)
