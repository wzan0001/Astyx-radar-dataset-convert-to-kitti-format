import os
import csv
import numpy as np


class FrameCalibrationData:
    def __init__(self):
        self.p0 = []
        self.p1 = []
        self.p2 = []
        self.p3 = []
        self.r0_rect = []
        self.tr_velodyne_to_cam = []



def read_calibration(calib_dir, img_idx):
    """Reads in Calibration file from Dataset.
    """
    frame_calibration_info = FrameCalibrationData()

    data_file = open(calib_dir + "/%06d.txt" % img_idx, 'r')
    data_reader = csv.reader(data_file, delimiter=' ')
    data = []

    for row in data_reader:
        data.append(row)

    data_file.close()

    p_all = []

    for i in range(4):
        p = data[i]
        p = p[1:]
        p = [float(p[i]) for i in range(len(p))]
        p = np.reshape(p, (3, 4))
        p_all.append(p)

    frame_calibration_info.p0 = p_all[0]
    frame_calibration_info.p1 = p_all[1]
    frame_calibration_info.p2 = p_all[2]
    frame_calibration_info.p3 = p_all[3]

    # Read in rectification matrix
    tr_rect = data[4]
    tr_rect = tr_rect[1:]
    tr_rect = [float(tr_rect[i]) for i in range(len(tr_rect))]
    frame_calibration_info.r0_rect = np.reshape(tr_rect, (3, 3))

    # Read in velodyne to cam matrix
    tr_v2c = data[5]
    tr_v2c = tr_v2c[1:]
    tr_v2c = [float(tr_v2c[i]) for i in range(len(tr_v2c))]
    frame_calibration_info.tr_velodyne_to_cam = np.reshape(tr_v2c, (3, 4))

    return frame_calibration_info


def radar_to_cam_frame(xyz_radar, frame_calib):
    """Transforms the points to the camera frame.
    """
    r0_rect_mat = frame_calib.r0_rect
    r0_rect_mat = np.pad(r0_rect_mat, ((0, 1), (0, 1)),'constant', constant_values=0)
    r0_rect_mat[3, 3] = 1

    tf_mat = frame_calib.tr_velodyne_to_cam
    tf_mat = np.pad(tf_mat, ((0, 1), (0, 0)),'constant', constant_values=0)
    tf_mat[3, 3] = 1
    one_pad = np.ones(xyz_radar.shape[0]).reshape(-1, 1)
    xyz_radar = np.append(xyz_radar, one_pad, axis=1)
    rectified = np.dot(r0_rect_mat, tf_mat)
    ret_xyz = np.dot(rectified, xyz_radar.T)
    # Change to N x 3 array for consistency.
    return ret_xyz[0:3].T
