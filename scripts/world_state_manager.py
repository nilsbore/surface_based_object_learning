import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from cluster_tracker import SOMAClusterTracker
from world_modeling.srv import *
from geometry_msgs.msg import Pose

# WS stuff
from soma_io.observation import Observation, TransformationStore
from soma_io.geometry import *
from soma_io.state import World, Object
from soma_io.geometry import Pose

# SOMA2 stuff
from soma2_msgs.msg import SOMA2Object
from soma_manager.srv import *

talk = True

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
        # self.o_sub = rospy.Subscriber("/head_xtion/depth_registered/points", PointCloud2, self.segment_callback)
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

    def add_soma_object(self,obj):
        print("getting service")
        rospy.wait_for_service('soma2/insert_objects')
        print("done")
        soma_insert = rospy.ServiceProxy('soma2/insert_objects',SOMA2InsertObjs)
        soma_insert(obj)


    def get_soma_objects_with_id(self,id):
        print("getting soma service")
        rospy.wait_for_service('soma2/query_db')
        print("done")
        soma_query = rospy.ServiceProxy('soma2/query_db',SOMA2QueryObjs)
        print("done")

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

        response = soma_query(query)

        return response

    def get_image_masks(self):
        msg = rospy.wait_for_message("/robot_pose",  geometry_msgs.msg.Pose, timeout=3.0)

    def assign_clusters(self):
        if(talk): print("assigning")
        cur_scene = self.cluster_tracker.cur_scene
        prev_scene = self.cluster_tracker.prev_scene

        if(talk): print("waiting for insert service")

        rospy.wait_for_service('/message_store/insert')

        if(talk): print("gotcha")


        # if this is not scene 0, ie. we have a previous scene to compare to
        if(prev_scene and cur_scene):
        # get all the cluster IDs from current scene
        # for each cluster in this scene
            for cur_scene_cluster in cur_scene.cluster_list:
            # is this a new cluster, or was it in the last scene?
                cur_cluster = None

                if(prev_scene.contains_cluster_id(cur_scene_cluster.cluster_id)):
                    # do we have a living world model for this cluster already?
                    if(self.cluster_is_live(cur_scene_cluster.cluster_id)):
                        #   fetch the world_model for the cluster
                        if(talk): print("got existing object")
                        cur_cluster = self.world_model.get_object(cur_scene_cluster.cluster_id)
                    else:
                        # here we know the object doesn't exist and/or isn't live
                        # BUT we do know that it's in the scene, because it's part of the segmentation output
                        if(talk): print("creating object")
                        cur_cluster = self.world_model.create_object(cur_scene_cluster.cluster_id)

                        #TODO: do this properly
                        cur_cluster._parent = self.get_cur_metaroom()


                # from here we've either added this as a new object to the scene
                # or retreived the data for it in a previous scene
                if(cur_cluster):
                    # so first add a new observation to it, in all cases
                    if(talk): print("making observation")
                    # add an observation for the object
                    cloud_observation = Observation.make_observation()
                    cur_cluster.add_observation(cloud_observation)

                    # centroid of this object, in the head_xtion_rgb_optical_frame
                    pose = Pose()
                    pose.position.x = cur_scene_cluster.local_centroid[0]
                    pose.position.y = cur_scene_cluster.local_centroid[1]
                    pose.position.z = cur_scene_cluster.local_centroid[2]

                    if(talk): print("POSE")
                    if(talk): print(pose.position)
                    cur_cluster.add_pose(pose)


                    # store the segmented point cloud for this cluster
                    cloud_observation.add_message(cur_scene_cluster.cloud,"object_cloud")
                    # NOTE: Not registered to meta-room yet


                    if(talk): print("done")

                    # next step: can we classify this object, OR do we have a classification for it already?

                    # send this cluster to object recogniser
                    # if we get a distribution back

                    # else
                        #
                        # SOMA INTEGRATION

                    if(False):
                        # see if we have a soma object by this name
                        soma_objs = self.get_soma_objects_with_id(cur_cluster.key)
                        cur_soma_obj = None
                        if(soma_objs.objects):
                            print("soma has this object")
                            # we have a soma object with this id
                            # retrieve it
                            cur_soma_obj = soma_objs.objects[0]
                    else:
                        print("soma doesn't have this object")
                        # if this object is unknown, lets register a new unknown object in SOMA2
                        # we do not have a soma object with this id
                        # create it
                        cur_soma_obj = SOMA2Object()
                        cur_soma_obj.id = cur_cluster.key

                        # TODO: everything is unknown for now, but later on we'll change this to a
                        # class or instance distribution
                        cur_soma_obj.type = "unknown"

                        # either way we want to record this, so just do it here?
                        cur_soma_obj.cloud = cur_scene_cluster.cloud
                        cur_soma_obj.pose = pose
                        msg = rospy.wait_for_message("/robot_pose",  geometry_msgs.msg.Pose, timeout=3.0)
                        new_obj.sweepCenter = msg



                #else:
                #    print("if you're reading this, something went horribly wrong as this should be impossible to reach.")

                #else:
                    #if(talk): print("eh")


            # next we need to clean up the scene, and mark anything no longer observable
            # as not live
            for prev_scene_cluster in prev_scene.cluster_list:
                # if the cluster observed in the previous scene is not in the current scene
                if not cur_scene.contains_cluster_id(prev_scene_cluster.cluster_id):

                    if(talk): print("cutting object")
                    # set the internal model to not live
                    try:
                        prev_cluster = self.world_model.get_object(prev_scene_cluster.cluster_id)
                        prev_cluster.cut()
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
                    #class distribution for this object, retrieve
                        # the SOMA object that matches it
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
                        prev_cluster = self.world_model.get_object(prev_scene_cluster.cluster_id)
                        prev_cluster.cut()
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
