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
from view_registration import ViewAlignmentManager
import os
from roi_filter import ROIFilter

if __name__ == '__main__':
    rospy.init_node('test_soma2', anonymous = False)
    print("off we go!")

    base_dir = "/home/jxy/tsc_stuff/pod_test/"
    episode_list = []
    for episode in os.listdir(base_dir):
        # for each episode
        print("EPISODE: " + episode)
        ep = []
        for obj in os.listdir(base_dir+"/"+episode):
            print("\tOBJ: " + obj)
            for observation in os.listdir(base_dir+"/"+episode+"/"+obj):
                #print("\t\tOBSERVATION: " + observation)
                target = base_dir+"/"+episode+"/"+obj+"/"+observation
                if(os.path.isfile(target+"/"+"image.p")):
                    print("\t\tOBSERVATION: " + observation)
                    out = {}
                    out['camera_info'] = pickle.load(open(target+"/"+"camera_info.p",'rb'))
                    out['robot_pose']  = pickle.load(open(target+"/"+"robot_pose.p",'rb'))
                    out['tf']  = pickle.load(open(target+"/"+"tf.p",'rb'))
                    out['cloud']  = pickle.load(open(target+"/"+"cloud.p",'rb'))
                    out['rgb_image']  = pickle.load(open(target+"/"+"image.p",'rb'))
                    out['data'] = pickle.load(open(target+"/"+"data.p",'rb'))
                    #python_pcd.write_pcd("views/"+str(uuid.uuid4())+".pcd",out['cloud'])
                    print(out['data'])
                    ep.append(out)

                else:
                    print("\t\tNO OBS HERE")
        episode_list.append(ep)

    #python_pcd.write_pcd("merged_obs_aligned.pcd", merged_cloud)

    clds = []
    tfs = []
    for k in episode_list:
        for v in k:
            clds.append(v['cloud'])
            tfs.append(v['tf'])

    #merged_cloud = merge_pcs(seg_clouds)
    #python_pcd.write_pcd("merged_seg_non_aligned.pcd", merged_cloud)
    print(str(len(clds)))
    print(str(len(tfs)))
    vam = ViewAlignmentManager()
    wp = "/home/jxy/.semanticMap/20160629/patrol_run_60/room_5/room.xml"
    new_clouds = vam.register_current_view_to_waypoint(clds,wp,tfs)
    rf = ROIFilter()
    for cl in new_clouds:
        idx = new_clouds.index(cl)
        python_pcd.write_pcd(str(idx)+"_aligned_pre_filter"+".pcd",cl,overwrite=True)
        post_mr = rf.filter_full_cloud(cl,None)
        python_pcd.write_pcd(str(idx)+"_aligned_post_filter"+".pcd",post_mr,overwrite=True)


    #for c,t in zip(new_clouds,tfs):
    #    idx = clds.index(c)
        #python_pcd.write_pcd(str(idx)+".pcd",c,overwrite=True)
        #pre_mr = rf.filter_full_cloud(c,t)
        #python_pcd.write_pcd(str(idx)+"_pre_mr"+".pcd", pre_mr,overwrite=True)
        #python_pcd.write_pcd(str(idx)+"_post_mr"+".pcd",c,overwrite=True)





    print("done")
    #rospy.spin()
