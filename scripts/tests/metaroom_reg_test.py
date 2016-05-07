import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from world_modeling.srv import *
from soma_io.observation import Observation, TransformationStore
from soma_io.geometry import *
from soma_io.state import World, Object
from soma_io.observation import *
# SOMA2 stuff
from soma2_msgs.msg import SOMA2Object
from soma_manager.srv import *
from geometry_msgs.msg import Pose, Transform, Vector3, Quaternion
import sensor_msgs.point_cloud2 as pc2
import python_pcd
import tf
# reg stuff #
from observation_registration_services.srv import *
import PyKDL
import tf2_ros

def transform_to_kdl(t):
     return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                  t.transform.rotation.z, t.transform.rotation.w),
                        PyKDL.Vector(t.transform.translation.x,
                                     t.transform.translation.y,
                                     t.transform.translation.z))

def transform_cloud(cloud,translation,rotation):
    tr = Transform()
    tr.translation = translation
    tr.rotation = rotation

    tr_s = TransformStamped()
    tr_s.header = std_msgs.msg.Header()
    tr_s.header.stamp = rospy.Time.now()
    tr_s.header.frame_id = cloud.header.frame_id
    tr_s.child_frame_id = "head_xtion_rgb_frame"
    tr_s.transform = tr

    t_kdl = transform_to_kdl(tr_s)
    points_out = []
    for p_in in pc2.read_points(cloud):
        p_out = t_kdl * PyKDL.Vector(p_in[0], p_in[1], p_in[2])
        points_out.append([p_out[0],p_out[1],p_out[2],p_in[3]])

    res = pc2.create_cloud(tr_s.header, cloud.fields, points_out)
    return res

def merge_pcs(clouds):
    combined_cloud_points = []
    for c in clouds:
        raw_cloud = pc2.read_points(c)
        int_data = list(raw_cloud)
        for point in int_data:
            x = point[0]
            y = point[1]
            z = point[2]
            colour = point[3]
            combined_cloud_points.append((x,y,z,colour))
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = clouds[0].header.frame_id
    combined_cloud = pc2.create_cloud(header, clouds[0].fields, combined_cloud_points)
    return combined_cloud

if __name__ == '__main__':
    rospy.init_node('test_soma2', anonymous = False)
    print("off we go!")

    world_model = World(server_host='localhost',server_port=62345)

    obj = world_model.get_object("c481c98b-6c79-48c8-94cc-a66673bd2de2")
    print("done")

    observations = obj._observations

    seg_clouds = []
    obs_clouds = []
    trans = []
    time = []
    for o in observations:
        seg_clouds.append(o.get_message('object_cloud_camframe'))
        obs_clouds.append(o.get_message('/head_xtion/depth_registered/points'))
        tf_p = o.get_message('/tf')
        t_st = TransformationStore().msg_to_transformer(tf_p)
        print(t_st)
        trans.append(t_s)
        time.append(o.stamp)

    print("got: " + str(len(seg_clouds)) + " clouds for object")

    print("getting metaroom reg service")

    reg_serv = rospy.ServiceProxy('additional_view_registration_server',AdditionalViewRegistrationService)
    print("got it")


    print("running service call")
#    obs_xml = "/home/jxy/.semanticMap/20160505/patrol_run_1/room_5/room.xml"
#    response = reg_serv(observation_xml=obs_xml,additional_views=input_clouds,additional_views_odometry_transforms=input_transforms)
    response = reg_serv(additional_views=obs_clouds)

    view_trans = response.additional_view_transforms

    cloud_id = 0
    print("-- aligning clouds -- ")
    for transform,obs_cloud,seg_cloud in zip(view_trans,obs_clouds,seg_clouds):
        print("---")
        rot = transform.rotation
        trs = transform.translation
        print("rotation: ")
        print(rot)
        print("translation: ")
        print(trs)
        transformed_obs_cloud = transform_cloud(obs_cloud,trs,rot)
        transformed_seg_cloud = transform_cloud(seg_cloud,trs,rot)
        cloud_id+=1
        print("")
        python_pcd.write_pcd("obs"+str(cloud_id)+".pcd", transformed_obs_cloud)
        python_pcd.write_pcd("seg"+str(cloud_id)+".pcd", transformed_seg_cloud)

    merged_cloud = merge_pcs(seg_clouds)
    python_pcd.write_pcd("merged.pcd", merged_cloud)



    print("done")
    #rospy.spin()
