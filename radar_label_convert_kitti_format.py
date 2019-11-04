#####################################################
##将radar 数据转为kitti格式                          ##
#####################################################
import json
import math
import os
import numpy as np
import utils

def rotMat2quatern(R):
        # transform the rotation matrix into quatern
        q = np.zeros(4)
        K = np.zeros([4, 4])
        K[0, 0] = 1 / 3 * (R[0, 0] - R[1, 1] - R[2, 2])
        K[0, 1] = 1 / 3 * (R[1, 0] + R[0, 1])
        K[0, 2] = 1 / 3 * (R[2, 0] + R[0, 2])
        K[0, 3] = 1 / 3 * (R[1, 2] - R[2, 1])
        K[1, 0] = 1 / 3 * (R[1, 0] + R[0, 1])
        K[1, 1] = 1 / 3 * (R[1, 1] - R[0, 0] - R[2, 2])
        K[1, 2] = 1 / 3 * (R[2, 1] + R[1, 2])
        K[1, 3] = 1 / 3 * (R[2, 0] - R[0, 2])
        K[2, 0] = 1 / 3 * (R[2, 0] + R[0, 2])
        K[2, 1] = 1 / 3 * (R[2, 1] + R[1, 2])
        K[2, 2] = 1 / 3 * (R[2, 2] - R[0, 0] - R[1, 1])
        K[2, 3] = 1 / 3 * (R[0, 1] - R[1, 0])
        K[3, 0] = 1 / 3 * (R[1, 2] - R[2, 1])
        K[3, 1] = 1 / 3 * (R[2, 0] - R[0, 2])
        K[3, 2] = 1 / 3 * (R[0, 1] - R[1, 0])
        K[3, 3] = 1 / 3 * (R[0, 0] + R[1, 1] + R[2, 2])
        D, V = np.linalg.eig(K)
        pp = 0
        for i in range(1, 4):
            if(D[i] > D[pp]):
                pp = i
        q = V[:, pp]
        q = np.array([q[3], q[0], q[1], q[2]])
        #print(q)
        return q

def qaut_to_angle(quat):
    x=quat[0]
    y=quat[1]
    z=quat[2]
    w=quat[3]

    rol = math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))#the rol is the yaw angle!
    #pith = math.asin(2*(w*y-z*z))
    #yaw = math.atan2(2*(w*z+x*y),1-2*(z*z+y*y))
    return rol


def quaternionToRotationMatrix(quat):
    q = quat.copy()
    q=np.array(q)
    n = np.dot(q, q)
    if n < np.finfo(q.dtype).eps:
        rot_matrix=np.identity(4)
        return rot_matrix
    q = q * np.sqrt(2.0 / n)
    q = np.outer(q, q)
    rot_matrix = np.array(
        [[1.0 - q[2, 2] - q[3, 3], q[1, 2] + q[3, 0], q[1, 3] - q[2, 0]],
         [q[1, 2] - q[3, 0], 1.0 - q[1, 1] - q[3, 3], q[2, 3] + q[1, 0]],
         [q[1, 3] + q[2, 0], q[2, 3] - q[1, 0], 1.0 - q[1, 1] - q[2, 2]]],
        dtype=q.dtype)
    return rot_matrix


def radarcoordToCameracoordYaw(quat,frame_calib):
    radar_quat_to_mat=quaternionToRotationMatrix(quat)
    radar_to_camera_mat=np.array(frame_calib.tr_velodyne_to_cam)
    radar_to_camera_mat=radar_to_camera_mat[:,0:3]
    rot_mat=np.dot(radar_to_camera_mat,radar_quat_to_mat)
    rot_quat=rotMat2quatern(rot_mat)
    angles=qaut_to_angle(rot_quat)
    return angles


def label_convert(save_dir,read_dir,calib_dir):
    name_list=[]
    for file in os.listdir(read_dir):
        name_list.append(file)

    for name in name_list:
        read_name=read_dir+name
        save_name=save_dir+name[0:6]+'.txt'
        img_idx=int(name[0:6])
        print(save_name)
        frame_calib = utils.read_calibration(calib_dir, img_idx)

        with open(save_name,mode='w')as save_txt_file_name:
            with open(read_name,mode='r')as read_json_file_name:
                read_object=json.load(read_json_file_name)#dict
                objts=read_object['objects']#list

                for oo in objts:

                    obj=oo#dict
                    anotation=[]
                    if obj['classname']=='Other Vehicle':
                        anotation.append('Other_Vehicle')
                    else:
                        anotation.append(obj['classname'])
                    anotation.append('0')#truncated unused
                    anotation.append(str(obj['occlusion']))
                    anotation.append('-10')#alpha unused
                    anotation.append('0')#2d box unuseds
                    anotation.append('0')
                    anotation.append('0')
                    anotation.append('0')
                    dim=obj['dimension3d']
                    anotation.append(str(dim[2]))#h
                    anotation.append(str(dim[1]))#w
                    anotation.append(str(dim[0]))#l

                    centerpoint=np.array(obj['center3d'])
                    centerpoint=np.reshape(centerpoint,(1,3))
                    camera_centerpoint = utils.radar_to_cam_frame(centerpoint, frame_calib)#transform to camera coordinate
                    anotation.append(str(camera_centerpoint[0][0]))
                    anotation.append(str(camera_centerpoint[0][1]+dim[2]*0.5))#top centor point
                    anotation.append(str(camera_centerpoint[0][2]))

                    orientation_quat=obj['orientation_quat']#quaterns
                    yaw_ang=radarcoordToCameracoordYaw(orientation_quat,frame_calib)
                    anotation.append(str(yaw_ang))
                    anotation.append('0')
                    str_anot=' '.join(anotation)
                    #print(str_anot)
                    save_txt_file_name.write(str_anot+'\n')
