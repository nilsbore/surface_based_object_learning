#!/usr/bin/env python

# general stuff
import roslib
import rospy
import sys
import argparse
import os
from random import randint
import cv2
import json
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
import tf2_ros
import tf, tf2_msgs.msg

# WS stuff
from surface_based_object_learning.srv import *
from util import TransformationStore

# soma stuff
from soma_msgs.msg import SOMAObject
from soma_manager.srv import *
from soma_llsd.srv import *

# recog stuff
from recognition_srv_definitions.srv import *

import uuid
#import python_pcd

class LearningCore:

    def __init__(self,db_hostname,db_port):
        rospy.init_node('surface_based_object_learning', anonymous = False)
        self.setup_clean = False
        rospy.loginfo("LEARNING CORE: Manager Online")
        # make a segment tracker
        rospy.loginfo("LEARNING CORE: looking for camera info topic")

        self.segment_processor = SegmentProcessor()
        self.pending_obs = []
        self.cur_sequence_obj_ids = []
        self.cur_view_soma_ids = []
        self.cur_observation_data = None
        self.queued_soma_objs = []

        rospy.loginfo("LEARNING CORE: setting up services")
        process = rospy.Service('/surface_based_object_learning/process_scene',ProcessScene,self.process_scene_callback)
        rospy.loginfo("LEARNING CORE: scene processing service running")

        begin_observations = rospy.Service('/surface_based_object_learning/begin_observation_sequence',Trigger,self.begin_obs)
        end_observations = rospy.Service('/surface_based_object_learning/end_observation_sequence',Trigger,self.end_obs)

        rospy.loginfo("LEARNING CORE: setting up SOMA services")
        rospy.loginfo("LEARNING CORE: getting SOMA insert service")
        rospy.wait_for_service('soma/insert_objects')
        rospy.loginfo("LEARNING CORE: done")
        self.soma_insert = rospy.ServiceProxy('soma/insert_objects',SOMAInsertObjs)

        rospy.loginfo("LEARNING CORE: getting SOMA query service")
        rospy.wait_for_service('soma/query_objects')
        rospy.loginfo("LEARNING CORE: done")
        self.soma_get = rospy.ServiceProxy('soma/query_objects',SOMAQueryObjs)

        rospy.loginfo("LEARNING CORE: getting SOMA update service")
        rospy.wait_for_service('/soma/update_object')
        rospy.loginfo("LEARNING CORE: done")
        self.soma_update = rospy.ServiceProxy('soma/update_object',SOMAUpdateObject)
        self.recog_manager = None
        rospy.loginfo("LEARNING CORE: setting up view alignment manager")
        self.view_alignment_manager = ViewAlignmentManager()

        rospy.loginfo("LEARNING CORE: getting LLSD services")
        rospy.wait_for_service('/soma_llsd/insert_scene')
        self.view_store_insert = rospy.ServiceProxy('/soma_llsd/insert_scene',InsertScene)

        rospy.wait_for_service('/soma_llsd/get_segment')
        self.get_segment = rospy.ServiceProxy('/soma_llsd/get_segment',GetSegment)

        rospy.wait_for_service('/soma_llsd/insert_segment')
        self.insert_segment = rospy.ServiceProxy('/soma_llsd/insert_segment',InsertSegment)

        rospy.wait_for_service('/soma_llsd/add_observations_to_segment')
        self.append_obs_to_segment = rospy.ServiceProxy('/soma_llsd/add_observations_to_segment',AddObservationsToSegment)

        self.clean_up_obs()

        rospy.loginfo("LEARNING CORE: -- node setup completed --")
        self.setup_clean = True

        rospy.spin()

    def clean_up_obs(self):
        rospy.loginfo("LEARNING CORE: running cleanup")
        self.pending_obs = []
        self.cur_sequence_obj_ids = []
        self.cur_view_soma_ids = []
        self.cur_observation_data = None
        self.segment_processor.reset()
        self.cur_episode_id = str(uuid.uuid4())
        self.queued_soma_objs = []
        rospy.loginfo("LEARNING CORE: -- new episode id: " + self.cur_episode_id)


    def begin_obs(self,req):
        rospy.loginfo("LEARNING CORE: -- received signal to begin sequence of observations --")
        if(self.setup_clean):
            rospy.loginfo("LEARNING CORE: ready to go")
        else:
            rospy.loginfo("LEARNING CORE: ERROR: node setup not completed yet, wait a sec and try again")
            return
        self.clean_up_obs()
        return TriggerResponse(True,"Observations Beginning: Assuming all subsequent observations are from the same sequence.")

    def end_obs(self,req):
        rospy.loginfo("LEARNING CORE: -- received signal to terminate sequence of observations --")
        rospy.loginfo("LEARNING CORE: ")
        self.do_postprocessing()
        return TriggerResponse(True,"Observations Ending: Assuming all previous observations were from the same sequence.")

    def get_camera_info_topic(self):
        camera_msg = None
        try:
            camera_msg = rospy.wait_for_message("/head_xtion/depth_registered/sw_registered/camera_info",  CameraInfo, timeout=2)
            return "/head_xtion/depth_registered/sw_registered/camera_info"
        except Exception,e:
            rospy.loginfo("LEARNING CORE: couldn't find /head_xtion/depth_registered/sw_registered/camera_info")

        if(not camera_msg):
            try:
                camera_msg = rospy.wait_for_message("/head_xtion/depth_registered/camera_info",  CameraInfo, timeout=2)
                rospy.loginfo("LEARNING CORE: found /head_xtion/depth_registered/camera_info")
                return "/head_xtion/depth_registered/camera_info"
            except Exception,e:
                rospy.loginfo("LEARNING CORE: couldn't find /head_xtion/depth_registered/camera_info")

        return None


    def flush_observation(self,data):
        print("-- flushing observation from dataset through system --")
        self.process_scene(data['cloud'],data['data'][3],data)

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
                rospy.loginfo("LEARNING CORE: successfully added scene to view store")
                self.cur_scene_id = scene.response.id
            else:
                rospy.logerr("couldn't add scene to view store, this is catastrophic")

        except Exception,e:
            rospy.loginfo("LEARNING CORE: failed to add view to view store")


    def process_scene(self,cloud,waypoint,extra_data=None):
        #try:
        rospy.loginfo("LEARNING CORE: ---- Storing view in View Store ----")
        self.cur_waypoint = waypoint
        self.populate_observation_data(cloud,extra_data)
        self.register_with_view_store(cloud)


        rospy.loginfo("LEARNING CORE: ---- Segmenting Scene ----")
        scene = self.segment_processor.add_unsegmented_scene(self.cur_observation_data,extra_data)
        if(scene.clean_setup is True):

            scene.waypoint = waypoint

            if(self.recog_manager):
                rospy.loginfo("LEARNING CORE: ---- Running Object Recognition ----")
                recognition = self.recog_manager.recognise_scene(cloud)
                if(recognition is True):
                    self.recog_manager.assign_labels(scene)
            else:
                rospy.logwarn("Object recognition service not found, try restarting is the node running?")

            self.assign_segments(scene,self.segment_processor.prev_scene,extra_data)
            self.pending_obs.append(scene)

            rospy.loginfo("LEARNING CORE: have: " + str(len(self.pending_obs)) + " view(s) waiting to be processed")

            return ProcessSceneResponse(True,self.cur_view_soma_ids)
        else:
            rospy.loginfo("LEARNING CORE: Error in processing scene")

        #except Exception,e:
        #    rospy.logerr("Unable to segment and process this scene")
        #    rospy.logerr(e)

    def process_scene_callback(self, req):
        result = ProcessSceneResponse(False,self.cur_view_soma_ids)
        if(self.setup_clean is False):
            rospy.logerr("-- surface_based_object_learning node is missing one or more key services, cannot act --")
            rospy.logerr("-- run services and then re-start me --")
            return result
        else:

            if(req.input is None):
                rospy.logwarn("-- This point cloud looks empty, is the service being called correctly? ---")
                rospy.logwarn("-- Stopping Processing ---")
                return result

            result = self.process_scene(req.input,req.waypoint)

            return result

    def do_postprocessing(self):
        rospy.loginfo("LEARNING CORE: -- beginning post-processing, attempting view alignment and object label updates -- ")

        if(self.queued_soma_objs):
            rospy.loginfo("-- inserting " + str(len(self.queued_soma_objs)) + " objects into soma")
            for k in self.queued_soma_objs:
                print(k.id)
            res = self.soma_insert(self.queued_soma_objs)
            print(res)


        if(len(self.cur_sequence_obj_ids) == 0):
            rospy.loginfo("LEARNING CORE: -- no segments found in this scene, or if they were they were filtered out by the SOMA region or height filters ")

        for object_id in self.cur_sequence_obj_ids:
            rospy.loginfo("trying to get soma id: " + object_id)
            soma_object = self.get_soma_objects_with_id(object_id)
            rospy.loginfo("trying to get segment:-" + object_id+"-")
            segment = self.get_segment(object_id)
            if(segment.result is False):
                rospy.logerr("Could not access low-level data for object")
                rospy.logerr("This is catastrophic")
                continue
            rospy.loginfo("done")

            observations = segment.response.observations
            rospy.loginfo("LEARNING CORE: observations for " + str(object_id) + " = " + str(len(observations)))
            if(len(observations) >= 2):
                rospy.loginfo("LEARNING CORE: processing...")
                # update world model
                try:
                    rospy.loginfo("LEARNING CORE: updating world model")
                    merged_cloud = self.view_alignment_manager.register_views(segment.response.observations)
                    rospy.loginfo("LEARNING CORE: updating SOMA obj")
                    soma_object.objects[0].cloud = merged_cloud

                    self.soma_update(object=soma_object.objects[0],db_id=str(object_id))
                except Exception,e:
                    rospy.logerr("problem updating object models in world/SOMA db. Unable to register merged clouds")
                    rospy.logerr(e)
                    continue
            else:
                rospy.loginfo("LEARNING CORE: not running view alignment, only one view")

            #rospy.loginfo("LEARNING CORE: attempting to update object's recognition label")
            #try:
            #    soma_objects.objects[0].type = str(world_object.label)
            #    self.soma_update(object=soma_objects.objects[0],db_id=str(object_id))
            #    rospy.loginfo("LEARNING CORE: done! this object recognised as a " + str(world_object.label) + " with confidence: " + str(world_object.label_confidence))
            #except Exception,e:
            #    rospy.logerr("Problem updating SOMA object label.")
            #    rospy.logerr(e)

        rospy.loginfo("LEARNING CORE: post-processing complete")


    def add_soma_object(self,obj):
        rospy.loginfo("LEARNING CORE: getting service")
        rospy.wait_for_service('soma/insert_objects')
        rospy.loginfo("LEARNING CORE: done")
        soma_insert = rospy.ServiceProxy('soma/insert_objects',SOMAInsertObjs)
        soma_insert(obj)

    def get_soma_objects_with_id(self,id):
        rospy.loginfo("LEARNING CORE: looking for SOMA objects with id: " + str(id))
        query = SOMAQueryObjsRequest()
        query.query_type = 0
        query.objectids = ([id])
        #query.objecttypes=['']

        response = self.soma_get(query)


        return response

    def populate_observation_data(self,scene,extra_data=None):
        self.cur_observation_data = {}
        if(extra_data is None):
            rospy.loginfo("LEARNING CORE: *** Making observation using live robot data")
            try:
                self.camera_info_topic = self.get_camera_info_topic()
                self.cur_observation_data['rgb_image'] = rospy.wait_for_message("/head_xtion/rgb/image_rect_color", Image, timeout=10.0)
                self.cur_observation_data['depth_image'] = rospy.wait_for_message("/head_xtion/depth/image_rect", Image, timeout=10.0)
                self.cur_observation_data['camera_info'] = rospy.wait_for_message(self.camera_info_topic, CameraInfo, timeout=10.0)
                self.cur_observation_data['scene_cloud'] = scene
                self.cur_observation_data['waypoint'] = self.cur_waypoint
                self.cur_observation_data['timestamp'] = int(rospy.Time.now().to_sec())
                self.cur_observation_data['robot_pose'] = rospy.wait_for_message("/robot_pose", geometry_msgs.msg.Pose, timeout=10.0)

                # populates the tf entry with a few seconds worth of tf data
                listener = TransformationStore()
                listener.create_live()
                print("waiting for listener")
                rospy.sleep(2)
                listener.kill()
                self.cur_observation_data['tf'] = listener.get_as_msg()

            except rospy.ROSException, e:
                rospy.logwarn("Failed to get some observation data")
                rospy.logwarn(e)
        else:
                rospy.loginfo("LEARNING CORE: *** Making observation using historic robot data")
                self.cur_observation_data['rgb_image'] = extra_data['rgb_image']
                self.cur_observation_data['camera_info'] = extra_data['camera_info']
                self.cur_observation_data['scene_cloud'] = extra_data['cloud']
                self.cur_observation_data['robot_pose'] = extra_data['robot_pose']
                self.cur_observation_data['metadata'] = extra_data['data']
                self.cur_observation_data['tf'] = extra_data['tf']
                self.cur_observation_data['depth_image'] = extra_data['depth_image']

        return self.cur_observation_data

    def assign_segments(self,scene,prev_scene,extra_data=None):
        rospy.loginfo("LEARNING CORE: Assigning segments")
        cur_scene = scene
        self.cur_view_soma_ids = []
        have_previous_scene = False
        data_dict = {}
        data_dict["source"] = "surface_based_object_learning"
        data_dict["waypoint"] = self.cur_observation_data['waypoint']

        meta_data = json.dumps(data_dict)

        # if this is not scene 0, ie. we have a previous scene to compare to
        if(prev_scene is not None):
            if(prev_scene != scene):
                have_previous_scene = True

        if(have_previous_scene):
            rospy.loginfo("LEARNING CORE: We have a previous scene")
            rospy.loginfo("LEARNING CORE: Current scene ID: " + scene.scene_id)
            rospy.loginfo("LEARNING CORE: Previous scene ID: " + prev_scene.scene_id)

        else:
            rospy.loginfo("LEARNING CORE: we do not have a previous scene")


        if not cur_scene:
            rospy.loginfo("LEARNING CORE: don't have anything in the current scene...")
            rospy.loginfo("LEARNING CORE: did segmentation fail?")
            return

        # we iterate over the INSTANCES OF segmentS visible in the current scene
        # and see if we can link them to previously seen segments in prior views
        for cur_scene_segment_instance in cur_scene.segment_list:

            # this object is a Segment in the database
            # whereas cur_scene_segment_instance is an instance of a segment in memory in the current view
            target_db_segment = None

            # if there are previous views to look at it
            if(have_previous_scene):
                rospy.loginfo("LEARNING CORE: seeing if prev scene contains: " + str(cur_scene_segment_instance.segment_id))
                for pc in prev_scene.segment_list:
                    rospy.loginfo(pc.segment_id)

                if(prev_scene.contains_segment_id(cur_scene_segment_instance.segment_id)):
                    rospy.loginfo("LEARNING CORE: getting EXISTING segment")
                    get_segment_req = self.get_segment(cur_scene_segment_instance.segment_id)
                    if(get_segment_req.result is False):
                        rospy.logerr("Failed to retreive segment, this is catastrophic")
                        return
                    else:
                        target_db_segment = get_segment_req.response

            # if this the first view, or a new segment
            if not target_db_segment:
                rospy.loginfo("LEARNING CORE: creating NEW segment")
                request = self.insert_segment(meta_data,self.cur_scene_id,[])
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
                rospy.loginfo("LEARNING CORE: making observation")
                # add an observation for the object

                new_segment_observation = Observation()
                #new_segment_observation.id =  I'm ignoring this because if it's left blank, the service generates one for you
                new_segment_observation.timestamp = self.cur_observation_data['timestamp']
                new_segment_observation.meta_data = meta_data

                new_segment_observation.pose =  cur_scene_segment_instance.map_centroid # centroid in map co-ordinates
                new_segment_observation.map_cloud =  cur_scene_segment_instance.segmented_pc_mapframe #segmented cloud in map co-ordinates
                new_segment_observation.camera_cloud = cur_scene_segment_instance.segmented_pc_camframe # segmented  cloud in camera co-ordinates
                #new_segment_observation.room_cloud = None # segmented cloud aligned to meta-room

                new_segment_observation.rgb_cropped = cur_scene_segment_instance.cropped_rgb_image
                new_segment_observation.depth_cropped =  cur_scene_segment_instance.cropped_depth_image
                new_segment_observation.image_mask = cur_scene_segment_instance.image_mask

                self.append_obs_to_segment(target_db_segment.id,[new_segment_observation],self.cur_scene_id)

                # do some sanity checking
                get_segment_req = self.get_segment(cur_scene_segment_instance.segment_id)
                target_db_segment = get_segment_req.response
                rospy.loginfo("LEARNING CORE: segment: " + target_db_segment.id + " now has " + str(len(target_db_segment.observations)) + " observations")

                cur_soma_obj = None
                soma_objs = self.get_soma_objects_with_id(target_db_segment.id)

                if(soma_objs.objects):
                    cur_soma_obj = soma_objs.objects[0]
                    # nothing to do in this case?

                for k in self.queued_soma_objs:
                    if(k.id is target_db_segment.id):
                        cur_soma_obj = k
                        break

                if(cur_soma_obj):
                    rospy.loginfo("LEARNING CORE: soma has this object")

                else:
                    rospy.loginfo("LEARNING CORE: soma doesn't have this object")
                    # if this object is unknown, lets register a new unknown object in soma
                    #  have a soma object with this id
                    # create it
                    try:
                        cur_soma_obj = SOMAObject()
                        cur_soma_obj.id = target_db_segment.id
                        rospy.loginfo("LEARNING CORE: segment: " + target_db_segment.id + " will have SOMA id " + target_db_segment.id)

                        cur_soma_obj.type = "unknown"
                        #cur_soma_obj.waypoint = self.cur_observation_data['waypoint']



                        cur_soma_obj.metadata = meta_data

                        # either way we want to record this, so just do it here?
                        cur_soma_obj.cloud = new_segment_observation.map_cloud
                        cur_soma_obj.pose = new_segment_observation.pose

                        if(extra_data is not None):
                            cur_soma_obj.logtimestamp = self.cur_observation_data['timestamp'] # jesus christ

                        #cur_soma_obj.sweepCenter = self.cur_observation_data['robot_pose']

                        rospy.loginfo("LEARNING CORE: inserting into SOMA once views are done")

                        self.queued_soma_objs.append(cur_soma_obj)
                        #res = self.soma_insert([cur_soma_obj])


                    except Exception, e:
                        rospy.logerr("unable to insert into SOMA. Is the database server running?")
                        rospy.logerr(e)

                # record all SOMA objects seen in this view
                self.cur_view_soma_ids.append(target_db_segment.id)

                rospy.loginfo("LEARNING CORE: done")

        rospy.loginfo("LEARNING CORE: DB Update Complete")
        rospy.loginfo("LEARNING CORE: ")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(prog='world_state_manager.py')
    parser.add_argument("db_hostname", nargs=1, help='DB Hostname')
    parser.add_argument('db_port', nargs=1, help="DB Port")

    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    if(len(sys.argv) < 2):
        rospy.loginfo("LEARNING CORE: not enough args, need db hostname and port")
    else:
        hostname = str(vars(args)['db_hostname'][0])
        port = str(vars(args)['db_port'][0])

        rospy.loginfo("LEARNING CORE: got db_hostname as: " + hostname + " got db_port as: " + port)
        world_state_manager = LearningCore(hostname,port)
