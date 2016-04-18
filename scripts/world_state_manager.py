import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from cluster_tracker import SOMAClusterTracker
from world_modeling.srv import *

# WS stuff
from soma_io.observation import Observation, TransformationStore
from soma_io.geometry import *
from soma_io.state import World, Object
from soma_io.geometry import Pose


talk = False

class WorldStateManager:
    def __init__(self):
        rospy.init_node('world_state_modeling', anonymous = False)

        if(talk): print("Manager Online")

        # make a cluster tracker
        self.world_model = World(server_host="woody",server_port=62345)
        if(talk): print("world model done")

        self.cluster_tracker = SOMAClusterTracker()

        # TODO: THIS NEEDS TO BE *REGISTERED* EVENTUALLY
        self.transform_store = TransformationStore()
        self.transform_store.create_live()

        # get the current point cloud
        if(talk): print("waiting for pc")
        rospy.wait_for_message('/head_xtion/depth_registered/points',PointCloud2)
        if(talk): print("got it")

        # callback chain to deal with storing *objects*
        #self.o_sub = rospy.Subscriber("/head_xtion/depth_registered/points", PointCloud2, self.segment_callback)
        print("setting up service")
        update_world_state = rospy.Service('update_world_model',WorldUpdate,self.segment_callback)
        print("done")

        if(talk): print("done!")
        if(talk): print("loading WS")

        #self.world_model.create_object("base_room")
        rospy.sleep(1.)

        #room = self.world_model.get_object("base_room")
        #if(talk): print(room)

        #if(talk): print("done if(talk): printing room")

        #if(talk): print("getting obj")
        #obj = self.world_model.get_object("test_object")
        #obj._parent = room.name
        #if(talk): print("done")
        #if(talk): print(obj)

        #live_objects = map(lambda x: x.name, self.world_model.get_children(self.get_cur_metaroom(), {'_life_end': None,}))
        #if(talk): print("Node started, setting any live objects to not-live")

        #objs = room.get_children_names()
        #if(talk): print(obj)

        #if(talk): print("doing something esle")
    #   d = self.world_model.get_children(self.room_name, {'_life_end': -1})
        #if(talk): print(d)

        #obj._parent = room.name

        #if(talk): print("done")children(self.room_name, {'_life_end': None,}))

        #if(talk): print(live_objects)

        # send the point cloud to the tracker

        # get back some clusters

        # SCENE CONSISTENCY
        # see if these clusters should be tagged to existing live objects in the world state
            # are there any live objects?
            # yes: see if we can assign current clusters to them
            # no: see if we can create some new live objects from the clusters we have

        # set old objects not in the scene to not live

        # if so, add new observation of existing clusters, keep them live

        # create new object?

        # OBJECT LABELING

        # see if we can run a classifier on any existing clusters

        # add any label to the world object

        rospy.spin()

    def segment_callback(self,data):
        print("got data")
        # handles service calls containing point clouds
        data = data.input

        if(talk): print("got cloud:" + str(data.header.seq))
        try:
            self.cluster_tracker.add_unsegmented_scene(data)
            self.assign_clusters()
            return WorldUpdateResponse(True)
        except rospy.ServiceException, e:
            if(talk): print "service call failed: %s"%e
            return WorldUpdateResponse(False)



    def cluster_is_live(self,cluster_id):
        if(talk): print("seeing if object exists:" + str(cluster_id) +" in: " + self.get_cur_metaroom())
        exists = self.world_model.does_object_exist(cluster_id)

        if(exists):
            live_objects = map(lambda x: x.name, self.world_model.get_children(self.get_cur_metaroom(), {'_life_end': None,}))
            if(talk): print("live objects:" + str(live_objects))
            if(talk): print("cid: " + str(cluster_id))
            if(cluster_id in live_objects):
                if(talk): print("it does!")
                return True

        if(talk): print("nope")
        return False

    def cluster_exists(self,cluster_id):
        return self.world_model.does_object_exist(cluster_id)

    def get_cur_metaroom(self):
        return "base_room"

    def assign_clusters(self):
        if(talk): print("assigning")
        cur_scene = self.cluster_tracker.cur_scene
        prev_scene = self.cluster_tracker.prev_scene

        if(talk): print("waiting for insert service")

        rospy.wait_for_service('/message_store/insert')

        if(talk): print("gotcha")

        # if this is not scene 0, ie. we have a previous scene to compare to
        if(prev_scene and cur_scene):

            # set up tf
            # TODO: THIS NEEDS TO BE *REGISTERED* EVENTUALLY
            #transform_store = TransformationStore()
            #transform_store.create_live()

        # get all the cluster IDs from current scene

        #transform_store = TransformationStore()
        #transform_store.create_live()

        # for each cluster in this scene
            for cur_scene_cluster in cur_scene.cluster_list:
            # is this a new cluster, or was it in the last scene?

                cur_obj = None

                if(prev_scene.contains_cluster_id(cur_scene_cluster.cluster_id)):
                    # do we have a living world model for this cluster already?
                    if(self.cluster_is_live(cur_scene_cluster.cluster_id)):
                        #   fetch the world_model for the cluster
                        if(talk): print("got existing object")
                        cur_obj = self.world_model.get_object(cur_scene_cluster.cluster_id)
                        #   attach observation from current point to world_model
                        #   set to live
                    else:
                        # here we know the object doesn't exist and/or isn't live
                        # BUT we do know that it's in the scene, because it's part of the segmentation output
                        if(talk): print("creating object")
                        cur_obj = self.world_model.create_object(cur_scene_cluster.cluster_id)
                        cur_obj._parent = self.get_cur_metaroom()
                        #cur_room = self.world_model.get_object("base_room")
                        #cur_room.add_child(self.cur_obj)

                    # from here we've either added this as a new object to the scene
                    # or retreived the data for it in a previous scene

                    if(cur_obj):


                        # so first add a new observation to it, in all cases
                        if(talk): print("making observation")
                        # add an observation for the object
                        cloud_observation = Observation.make_observation_from_messages(
                                   [("/tf", self.transform_store.pickle_to_msg()),
                                     ("/head_xtion/depth_registered/points", cur_scene_cluster.cloud)])
                        cur_obj.add_observation(cloud_observation)

                        # centroid of this object, in the head_xtion_rgb_optical_frame

                        pose = Pose()
                        pose.position.x = cur_scene_cluster.local_centroid[0]
                        pose.position.y = cur_scene_cluster.local_centroid[1]
                        pose.position.z = cur_scene_cluster.local_centroid[2]

                        if(talk): print("POSE")
                        if(talk): print(pose.position)
                        cur_obj.add_pose(pose)


                        # raw pointcloud2 message, segmented from the rest of the scene for this object
                        #cloud_observation.add_message(cur_scene_cluster.cloud,"object_cloud")


                        if(talk): print("done")

                        # next step: can we classify this object, OR do we have a classification for it already?
                        # TODO: OBJECT LEARNING STUFF GOES HERE

                        # if the object we received has a classification already

                        # if the object is not classified
                            # try to classify it

                                # it's been classified
                                    # assign classification to object

                                # it hasn't been classified
                                    # what now?


                else:
                    if(talk): print("eh")


            # next we need to clean up the scene, and mark anything no longer observable
            # as not live
            for prev_scene_cluster in prev_scene.cluster_list:
                # if the cluster observed in the previous scene is not in the current scene
                if not cur_scene.contains_cluster_id(prev_scene_cluster.cluster_id):

                    if(talk): print("cutting object")
                    # set the internal model to not live
                    try:
                        prev_obj = self.world_model.get_object(prev_scene_cluster.cluster_id)
                        prev_obj.cut()
                    except Exception, e:
                        # we don't even relaly care about this, if it's not in the db
                        # we're actually good to go
                        print("err")
                        print(e)

                else:
                    if(talk): print("object still live, not cutting")

            # do some cleanup in case of crashes or some other errors
            live_objects = map(lambda x: x.name, self.world_model.get_children(self.get_cur_metaroom(), {'_life_end': None,}))
            for o in live_objects:
                if not cur_scene.contains_cluster_id(o):
                    if(talk): print("killing dangling object")
                    dangling_obj = self.world_model.get_object(o)
                    dangling_obj.cut()

if __name__ == '__main__':
    world_state_manager = WorldStateManager()
