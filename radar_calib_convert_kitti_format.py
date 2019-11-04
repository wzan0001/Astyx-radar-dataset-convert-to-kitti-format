#####################################################
##将astyx的calibration 数据转为kitti格式              ##
#####################################################
import json
import math
import os
import numpy as np

def radar_to_camera_convert(T):
    radar_to_camera_rotation=np.linalg.inv(T[0:3,0:3])#rotation matrix
    
    radar_to_camera_translate=T[0:3,3]
    radar_to_camera_translate=np.dot(radar_to_camera_rotation,radar_to_camera_translate.T)
    radar_to_camera_translate=-1*radar_to_camera_translate#translate vector

    radar_to_camera_translate=np.reshape(radar_to_camera_translate,(3,1))
    radar_to_camera=np.hstack((radar_to_camera_rotation,radar_to_camera_translate))
    #print(radar_to_camera)
    return radar_to_camera

def write_to_txt(K,T,save_name):
    K_write=K.reshape((1,12))[0]
    K_write=str(K_write).strip('[').strip(']').replace('\n','')
    K_write=K_write.split()
    K_write=' '.join(K_write)

    T_to_write=T.reshape((1,12))[0]
    T_to_write=str(T_to_write).strip('[').strip(']').replace('\n','')
    T_to_write=T_to_write.split()
    T_to_write=' '.join(T_to_write)
    with open(save_name,'w') as f:
        f.write('P0: 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00'+'\n')
        f.write('P1: 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00'+'\n')
        f.write('P2: '+K_write+'\n')
        f.write('P3: 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00'+'\n')
        f.write('R0_rect: 1.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 1.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 1.000000000000e+00'+'\n')
        f.write('Tr_velo_to_cam: '+T_to_write+'\n')
        f.write('Tr_imu_to_velo: 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00'+'\n')

def calib_convert(save_dir,read_dir):
    name_list=[]
    for file in os.listdir(read_dir):
        name_list.append(file)

    for name in name_list:
        read_name=read_dir+name
        save_name=save_dir+name[0:6]+'.txt'
        print(save_name)

        with open(read_name,mode='r')as read_json_file_name:
            read_object=json.load(read_json_file_name)#dict
            camera2radar=read_object['sensors'][2]#dict
            K=np.array(camera2radar['calib_data']['K'])#camera intrinsics
            K=np.hstack((K,np.array([[0.0],[0.0],[0.0]])))

            camera_to_radar=np.array(camera2radar['calib_data']['T_to_ref_COS'])#camera to radar
            radar_to_camera=radar_to_camera_convert(camera_to_radar)#radar to camera
            write_to_txt(K,radar_to_camera,save_name)
            
