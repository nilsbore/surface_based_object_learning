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

class ViewAlignmentManager:





    def __init__(self):
        #rospy.init_node('world_modeling_view_alignment', anonymous = True)
        print("---created view alignment manager --")
        print("waiting for view alignment service additional_view_registration_server from strands_3d_mapping")
        self.reg_serv = rospy.ServiceProxy('additional_view_registration_server',AdditionalViewRegistrationService)
        print("got it")


    def transform_to_kdl(self,t):
         return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                      t.transform.rotation.z, t.transform.rotation.w),
                            PyKDL.Vector(t.transform.translation.x,
                                         t.transform.translation.y,
                                         t.transform.translation.z))

    def transform_cloud(self,cloud,translation,rotation):
        tr = Transform()
        tr.translation = translation
        tr.rotation = rotation

        tr_s = TransformStamped()
        tr_s.header = std_msgs.msg.Header()
        tr_s.header.stamp = rospy.Time.now()
        tr_s.header.frame_id = "head_xtion_rgb_frame"
        tr_s.child_frame_id = "head_xtion_rgb_optical_frame"
        tr_s.transform = tr

        t_kdl = self.transform_to_kdl(tr_s)
        points_out = []
        for p_in in pc2.read_points(cloud):
            p_out = t_kdl * PyKDL.Vector(p_in[0], p_in[1], p_in[2])
            points_out.append([p_out[0],p_out[1],p_out[2],p_in[3]])

        res = pc2.create_cloud(tr_s.header, cloud.fields, points_out)
        return res

    def merge_pcs(self,clouds):
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


    def register_views(self,observations,merge_and_write=False):
        seg_clouds = []
        obs_clouds = []
        obs_transforms = []
        time = []
        for o in observations:
            tf_p = o.get_message('/tf')
            t_st = TransformationStore().msg_to_transformer(tf_p)

            cam_cloud = o.get_message('object_cloud_camframe')
            obs_cloud = o.get_message('/head_xtion/depth_registered/points')
            t,r = t_st.lookupTransform("head_xtion_rgb_frame","head_xtion_rgb_optical_frame",rospy.Time(0))
            cam_trans = geometry_msgs.msg.Transform()
            cam_trans.translation.x = t[0]
            cam_trans.translation.y = t[1]
            cam_trans.translation.z = t[2]

            cam_trans.rotation.x = r[0]
            cam_trans.rotation.y = r[1]
            cam_trans.rotation.z = r[2]
            cam_trans.rotation.w = r[3]

            cam_cloud = self.transform_cloud(cam_cloud,cam_trans.translation,cam_trans.rotation)
            obs_cloud = self.transform_cloud(obs_cloud,cam_trans.translation,cam_trans.rotation)

            seg_clouds.append(cam_cloud)
            obs_clouds.append(obs_cloud)


            print("looking for transform")
            trans,rot = t_st.lookupTransform("map","head_xtion_rgb_frame",rospy.Time(0))

            cur_trans = geometry_msgs.msg.Transform()
            cur_trans.translation.x = trans[0]
            cur_trans.translation.y = trans[1]
            cur_trans.translation.z = trans[2]

            cur_trans.rotation.x = rot[0]
            cur_trans.rotation.y = rot[1]
            cur_trans.rotation.z = rot[2]
            cur_trans.rotation.w = rot[3]

            print(cur_trans)
            obs_transforms.append(cur_trans)


        print("got: " + str(len(seg_clouds)) + " clouds for object")

        print("running service call")
        response = self.reg_serv(additional_views=obs_clouds,additional_views_odometry_transforms=obs_transforms)

        view_trans = response.additional_view_transforms
        print(view_trans)

        cloud_id = 0
        transformed_obs_clouds = []
        transformed_seg_clouds = []
        print("-- aligning clouds -- ")
        for transform,obs_cloud,seg_cloud in zip(view_trans,obs_clouds,seg_clouds):
            rot = transform.rotation
            trs = transform.translation

            transformed_obs_cloud = self.transform_cloud(obs_cloud,trs,rot)
            transformed_seg_cloud = self.transform_cloud(seg_cloud,trs,rot)

            transformed_obs_clouds.append(transformed_obs_cloud)
            transformed_seg_clouds.append(transformed_seg_cloud)

            cloud_id+=1
            #python_pcd.write_pcd("obs"+str(cloud_id)+".pcd", transformed_obs_cloud)
            #python_pcd.write_pcd("seg"+str(cloud_id)+".pcd", transformed_seg_cloud)

            merged_cloud = self.merge_pcs(transformed_seg_clouds)

        if(merge_and_write):
            print("-- merging and writing clouds to files --")
            merged_cloud = self.merge_pcs(obs_clouds)
            python_pcd.write_pcd("merged_obs_non_aligned.pcd", merged_cloud)

            merged_cloud = self.merge_pcs(transformed_obs_clouds)
            python_pcd.write_pcd("merged_obs_aligned.pcd", merged_cloud)

            merged_cloud = self.merge_pcs(seg_clouds)
            python_pcd.write_pcd("merged_seg_non_aligned.pcd", merged_cloud)

            python_pcd.write_pcd("merged_seg_aligned.pcd", merged_cloud)

        return merged_cloud

if __name__ == '__main__':
    print("hi")
    rospy.init_node('test2', anonymous = False)
    world_model = World(server_host='localhost',server_port=62345)
    obj = world_model.get_object("c481c98b-6c79-48c8-94cc-a66673bd2de2")
    print("got object")
    print("observations: " + str(len(obj._observations)))
    vr = ViewAlignmentManager()
    merged_cloud = vr.register_views(obj._observations, False)
