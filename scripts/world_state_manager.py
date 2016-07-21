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


# ROS stuff
from sensor_msgs.msg import PointCloud2, PointField
from cluster_tracker import SOMAClusterTracker
from recognition_manager import ObjectRecognitionManager
from view_registration import ViewAlignmentManager
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, JointState
from std_srvs.srv import Trigger, TriggerResponse

# WS stuff
from soma_io.observation import *
from soma_io.geometry import *
from soma_io.state import World, Object
import soma_io.geometry as ws_geom
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

talk = True

class StoredView(MongoDocument):
    def __init__(self, mongo=None):
        super(StoredView, self).__init__()
        if mongo is not None:
            # this will create the document live..
            self._connect(mongo)
        self.id = None
        self.point_cloud = None
        self.rgb_image = None
        self.robot_pose = None
        self.camera_info = None
        self.tf = None
        self.view_episode_id = None

    @classmethod
    def _mongo_encode(cls, class_object):
        doc = {}
        doc.update(class_object.__dict__)
        try:
            doc.pop("_MongoDocument__mongo")
            doc.pop("_MongoDocument__connected")
        except KeyError:
            print "Warning: no no no"
        doc["__pyobject_class_type"] = class_object.get_pyoboject_class_string()
        doc = copy.deepcopy(doc)
        return doc

class WorldStateManager:

    def __init__(self,db_hostname,db_port):
        rospy.init_node('world_state_modeling', anonymous = False)
        self.setup_clean = False
        if(talk): rospy.loginfo("Manager Online")
        # make a cluster tracker
        rospy.loginfo("looking for camera info topic")

        self.world_model = World(server_host=db_hostname,server_port=int(db_port))
        if(talk): rospy.loginfo("world model done")

        self.cluster_tracker = SOMAClusterTracker()
        self.pending_obs = []
        self.cur_sequence_obj_ids = []
        self.cur_view_soma_ids = []
        self.cur_observation_data = None

        ## set up things for the view store ##
        self.view_store = MongoConnection("surface_view_store", db_hostname, int(db_port))

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
        self.cluster_tracker.reset()
        self.view_episode_id = str(uuid.uuid4())

        try:
            # TODO: have to hack this due to issue with world_model code I'd rather not touch for now
            query = {'_life_end': None}
            db = self.world_model._mongo.database.Objects.find({"__pyobject_class_type": Object.get_pyoboject_class_string(),'_life_end': None,})
            if(db):
                for d in db:
                    # uh well, this is weird? can't acces the object directly from this query
                    # but can if I request it through the model
                    # TODO: fix this insanity
                    wo = self.world_model.get_object(d.key)
                    rospy.loginfo("cut object: " + d.key)
                    wo.cut()
        except Exception,e:
            rospy.logerr("Failed to clean up obs due to DB error")
            rospy.logerr(e)

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

    def get_camera_info_topic_as_string(self):
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
            #cur_soma_person.cloud = cur_scene_cluster.segmented_pc_mapframe

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

    def create_view_store_msg(self,data):
        message_proxy = MessageStoreProxy(collection="surface_view_store")
        msg_id = message_proxy.insert(data)
        pc_msg = MessageStoreObject(
            database=message_proxy.database,
            collection=message_proxy.collection,
            obj_id=msg_id,
            typ=data._type)
        return pc_msg

    def place_in_view_store(self,cloud,extra_data=None):
        try:
            if(self.cur_observation_data is None):
                self.get_observation_data(cloud,extra_data)

            new_view = StoredView()
            new_view.point_cloud = self.create_view_store_msg(cloud)
            new_view.id = str(uuid.uuid4())
            new_view.rgb_image = self.create_view_store_msg(self.cur_observation_data['rgb_image'])
            new_view.robot_pose = self.create_view_store_msg(self.cur_observation_data['robot_pose'])
            new_view.camera_info = self.create_view_store_msg(self.cur_observation_data['camera_info'])
            new_view.tf = self.create_view_store_msg(self.cur_observation_data['tf'])
            new_view.view_episode_id = self.view_episode_id;
            new_id = self.view_store.database.views.insert(StoredView._mongo_encode(new_view))
            new_view.id = new_id

        #    new_view._connect(self.view_store)
            rospy.loginfo("successfully added view to view store")
        except Exception,e:
            rospy.loginfo("failed to add view to view store")


    def process_cloud(self,cloud,waypoint,extra_data=None):
        try:
            rospy.loginfo("---- Storing view in View Store ----")
            self.get_observation_data(cloud,extra_data)
            self.place_in_view_store(cloud)

            rospy.loginfo("---- Segmenting Scene ----")
            scene = self.cluster_tracker.add_unsegmented_scene(cloud,extra_data)
            if(scene.clean_setup is True):

                scene.waypoint = waypoint

                if(self.recog_manager):
                    rospy.loginfo("---- Running Object Recognition ----")
                    recognition = self.recog_manager.recognise_scene(cloud)
                    if(recognition is True):
                        self.recog_manager.assign_labels(scene)
                else:
                    rospy.logwarn("Object recognition service not found, try restarting is the node running?")

                self.assign_clusters(scene,self.cluster_tracker.prev_scene,extra_data)
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
            soma_objects = self.get_soma_objects_with_id(object_id)
            rospy.loginfo("attempting to process object: " + str(object_id))

            # should be impossible, but just in case

            if(not soma_objects.objects[0]):
                rospy.loginfo("SOMA object doesn't exist")
                continue
            else:
                rospy.loginfo("got this SOMA object")

            if(not self.world_model.does_object_exist(object_id)):
                rospy.logerr("WORLD object doesn't exist")
                continue

            try:
                world_object = self.world_model.get_object(object_id)
            except rospy.ServiceException, e:
                rospy.logerr("DB ERROR")
                continue

            observations = world_object._observations
            rospy.loginfo("observations for " + str(object_id) + " = " + str(len(observations)))
            if(len(observations) >= 2):
                rospy.loginfo("processing...")
                # update world model
                try:
                    rospy.loginfo("updating world model")
                    merged_cloud = self.view_alignment_manager.register_views(observations)
                    rospy.loginfo("got merged cloud, putting it into the message store")
                    message_proxy = MessageStoreProxy(collection="ws_merged_aligned_clouds")
                    msg_id = message_proxy.insert(merged_cloud)
                    mso = MessageStoreObject(
                        database=message_proxy.database,
                        collection=message_proxy.collection,
                        obj_id=msg_id,
                        typ=merged_cloud._type)
                    world_object._point_cloud = mso

                    # update SOMA
                    rospy.loginfo("updating SOMA obj itself")
                    soma_objects.objects[0].cloud = merged_cloud

                    self.soma_update(object=soma_objects.objects[0],db_id=str(object_id))
                except Exception,e:
                    rospy.logerr("problem updating object models in world/SOMA db. Unable to register merged clouds")
                    rospy.logerr(e)
                    continue
            else:
                rospy.loginfo("not running view alignment, only one view")

            rospy.loginfo("attempting to update object's recognition label")

            try:
                soma_objects.objects[0].type = str(world_object.label)
                self.soma_update(object=soma_objects.objects[0],db_id=str(object_id))
                rospy.loginfo("done! this object recognised as a " + str(world_object.label) + " with confidence: " + str(world_object.label_confidence))
            except Exception,e:
                rospy.logerr("Problem updating SOMA object label.")
                rospy.logerr(e)

        rospy.loginfo("post-processing complete")


    def cluster_is_live(self,cluster_id,waypoint):
        if(talk): rospy.loginfo("seeing if object exists:" + str(cluster_id) +" in: " + waypoint)
        exists = self.world_model.does_object_exist(cluster_id)
        db = self.world_model._mongo.database.Objects.find({"__pyobject_class_type": Object.get_pyoboject_class_string(),'key': cluster_id,})
        if(db):
            print("uhhh i got something")
            if(db.count() != 0):
                print("True!")
                return True
            else:
                print("False!")
                return False
        else:
            print("i didn't get anything")


        #if(exists):
        #    live_objects = map(lambda x: x.name, self.world_model.get_children(waypoint, {'_life_end': None,}))
        #    if(talk): rospy.loginfo("live objects:" + str(live_objects))
        #    if(talk): rospy.loginfo("cid: " + str(cluster_id))
        #    if(cluster_id in live_objects):
        #        if(talk): rospy.loginfo("it does!")
        #        return True

        if(talk): rospy.loginfo("nope")
        return False

    def cluster_exists(self,cluster_id):
        return self.world_model.does_object_exist(cluster_id)

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

    def get_observation_data(self,scene,extra_data=None):
        self.cur_observation_data = {}
        if(extra_data is None):
            rospy.loginfo("*** Making observation using live robot data")
            try:
                self.camera_info_topic = self.get_camera_info_topic_as_string()
                self.cur_observation_data['rgb_image'] = rospy.wait_for_message("/head_xtion/rgb/image_rect_color", Image, timeout=10.0)
                self.cur_observation_data['camera_info'] = rospy.wait_for_message(self.camera_info_topic, CameraInfo, timeout=10.0)
                self.cur_observation_data['scene_cloud'] = scene.unfiltered_cloud
                #self.cur_observation_data['ptu_state'] = rospy.wait_for_message("/ptu/state", JointState, timeout=10.0)
                self.cur_observation_data['robot_pose'] = rospy.wait_for_message("/robot_pose", geometry_msgs.msg.Pose, timeout=10.0)

                transforms = TransformationStore.create_live()
                rospy.sleep(1) # drink in some TF
                tf_data =  transforms.pickle_to_msg()
                msg_id = message_proxy.insert(tf_data)
                tf_store  = MessageStoreObject(
                    database=message_proxy.database,
                    collection=message_proxy.collection,
                    obj_id=msg_id,
                    typ=tf_data._type)

                self.cur_observation_data['tf'] = tf_store

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
        return self.cur_observation_data

    def assign_clusters(self,scene,prev_scene,extra_data=None):
        if(talk): rospy.loginfo("assigning")
        cur_scene = scene
        self.cur_view_soma_ids = []




        if(talk): rospy.loginfo("waiting for insert service")

        rospy.wait_for_service('/message_store/insert')

        if(talk): rospy.loginfo("gotcha")

        # if this is not scene 0, ie. we have a previous scene to compare to
        if not prev_scene:
            rospy.loginfo("don't have previous scene to compare to, assuming first run")

        if not cur_scene:
            rospy.loginfo("don't have anything in the current scene...")
            rospy.loginfo("did segmentation fail?")
            return

        for cur_scene_cluster in cur_scene.cluster_list:

            cur_cluster = None

            if(prev_scene):
                rospy.loginfo("seeing if prev scene contains: " + str(cur_scene_cluster.cluster_id))
                for pc in prev_scene.cluster_list:
                    rospy.loginfo(pc.cluster_id)
                if(prev_scene.contains_cluster_id(cur_scene_cluster.cluster_id)):
                    # do we have a living world model for this cluster already?
                    if(self.cluster_is_live(cur_scene_cluster.cluster_id,cur_scene.waypoint)):
                        #   fetch the world_model for the cluster
                        if(talk): rospy.loginfo("got existing object")
                        rospy.loginfo("getting EXISTING cluster with id: " + cur_scene_cluster.cluster_id)
                        cur_cluster = self.world_model.get_object(cur_scene_cluster.cluster_id)

            if not cur_cluster:
                if(talk): rospy.loginfo("creating NEW cluster with id: " + str(cur_scene_cluster.cluster_id))
                cur_cluster = self.world_model.create_object(cur_scene_cluster.cluster_id)
                cur_cluster._parent = cur_scene.waypoint
                cur_cluster.label = "unknown"
                cur_cluster.label_confidence = 0.0
                self.cur_sequence_obj_ids.append(cur_scene_cluster.cluster_id)

            # from here we've either added this as a new object to the scene
            # or retreived the data for it in a previous scene
            if(cur_cluster):
                # so first add a new observation to it, in all cases
                if(talk): rospy.loginfo("making observation")
                # add an observation for the object

                #DEFAULT_TOPICS = [("/head_xtion/rgb/image_rect_color", Image),
                #                  (self.camera_info_topic, CameraInfo),
                #                  ("/head_xtion/depth_registered/points", PointCloud2),
                #                  ("/ptu/state", JointState),
                #                  ("/robot_pose", geometry_msgs.msg.Pose)]
                #cloud_observation = Observation.make_observation(DEFAULT_TOPICS)

                OBSERVATIONS = [("/head_xtion/rgb/image_rect_color",self.cur_observation_data['rgb_image']),
                                ("/robot_pose",self.cur_observation_data['robot_pose']),
                                ("/camera_info",self.cur_observation_data['camera_info']),
                                ("/head_xtion/depth_registered/points",self.cur_observation_data['scene_cloud']),
                                ("/tf",self.cur_observation_data['tf'])]

                cloud_observation = Observation.make_observation_from_messages(OBSERVATIONS)



                # centroid of this object
                ws_pose = ws_geom.Pose()
                ws_pose.position.x = cur_scene_cluster.map_centroid[0]
                ws_pose.position.y = cur_scene_cluster.map_centroid[1]
                ws_pose.position.z = cur_scene_cluster.map_centroid[2]

                rospy.loginfo("observation made")

                #if(talk): rospy.loginfo("POSE")
                #if(talk): rospy.loginfo(pose.position)
                cur_cluster.add_pose(ws_pose)

                # store a bunch of image stuff about the cluster
                cloud_observation.add_message(cur_scene_cluster.segmented_pc_mapframe,"object_cloud_mapframe")
                cloud_observation.add_message(cur_scene_cluster.segmented_pc_camframe,"object_cloud_camframe")

                message_proxy = MessageStoreProxy(collection="ws_observations")
                msg_id = message_proxy.insert(cur_scene_cluster.segmented_pc_mapframe)
                mso = MessageStoreObject(
                    database=message_proxy.database,
                    collection=message_proxy.collection,
                    obj_id=msg_id,
                    typ=cur_scene_cluster.segmented_pc_mapframe._type)

                cur_cluster._point_cloud = mso
                cur_cluster.view_episode_id = self.view_episode_id

                cloud_observation.add_message(cur_scene_cluster.cropped_image,"image_cropped")
                cloud_observation.add_message(cur_scene_cluster.rgb_mask,"rgb_mask")

                if(extra_data is not None):
                    cur_cluster._life_start = extra_data['data'][1]
                    cur_cluster._life_end = extra_data['data'][0]

                cur_cluster.add_observation(cloud_observation)

                cur_soma_obj = None

                soma_objs = self.get_soma_objects_with_id(cur_cluster.key)

                if(soma_objs.objects):
                    rospy.loginfo("soma has this object")
                    # we have a soma object with this id
                    # retrieve it
                    cur_soma_obj = soma_objs.objects[0]
                else:
                    rospy.loginfo("soma doesn't have this object")
                    # if this object is unknown, lets register a new unknown object in SOMA2
                    # we do not have a soma object with this id
                    # create it
                    try:
                        cur_soma_obj = SOMA2Object()
                        cur_soma_obj.id = cur_cluster.key
                        cur_soma_obj.type = "unknown"
                        cur_soma_obj.waypoint = cur_scene.waypoint

                        # either way we want to record this, so just do it here?
                        cur_soma_obj.cloud = cur_scene_cluster.segmented_pc_mapframe

                        soma_pose = geometry_msgs.msg.Pose()
                        soma_pose.position.x = cur_scene_cluster.map_centroid[0]
                        soma_pose.position.y = cur_scene_cluster.map_centroid[1]
                        soma_pose.position.z = cur_scene_cluster.map_centroid[2]

                        cur_soma_obj.pose = soma_pose
                        if(extra_data is not None):
                            cur_soma_obj.logtimestamp = int(float(cur_cluster._life_end)) # jesus christ

                        cur_soma_obj.sweepCenter = self.cur_observation_data['robot_pose']

                        # TODO: everything is unknown for now, but later on we'll change this to a
                        # class or instance distribution
                        rospy.loginfo("inserting into SOMA")
                        res = self.soma_insert([cur_soma_obj])
                        #rospy.loginfo("result: ")
                        #rospy.loginfo(res)
                    except Exception, e:
                        rospy.logerr("unable to insert into SOMA. Is the database server running?")
                        rospy.logerr(e)

                # record all SOMA objects seen in this view
                self.cur_view_soma_ids.append(cur_cluster.key)

                if(talk): rospy.loginfo("done")
        # next we need to clean up the scene, and mark anything no longer observable
        # as not live
        if(prev_scene and cur_scene):
            for prev_scene_cluster in prev_scene.cluster_list:
                # if the cluster observed in the previous scene is not in the current scene
                if not cur_scene.contains_cluster_id(prev_scene_cluster.cluster_id):
                    if(talk): rospy.loginfo("cutting object from previous scene")
                    # set the internal model to not live
                    try:
                        prev_cluster = self.world_model.get_object(prev_scene_cluster.cluster_id)
                        prev_cluster.cut()
                    except Exception, e:
                        rospy.logerr("failed to cut object, is the database server OK?")
                        rospy.logerr("restart of node possibly required")

            else:
                if(talk): rospy.loginfo("object still live, not cutting")

        rospy.loginfo("World Update Complete")
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
