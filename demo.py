import radar_calib_convert_kitti_format
import radar_label_convert_kitti_format
import os

if __name__=='__main__':
    ###################################################
    ###                                              ##
    ###       You need input the root_dir!           ##
    ###################################################

    root_dir='~/dataset_astyx_hires2019/dataset_astyx_hires2019/'

    label_read_dir=root_dir+'groundtruth_obj3d/'
    calib_read_dir=root_dir+'calibration/'
    if  os.path.exists(label_read_dir) and os.path.exists(calib_read_dir):
        label_save_dir=root_dir+'kitti_format_label/'
        os.makedirs(label_save_dir)
        calib_save_dir=root_dir+'kitti_format_calib/'
        os.makedirs(calib_save_dir)

        radar_calib_convert_kitti_format.calib_convert(calib_save_dir,calib_read_dir)
        radar_label_convert_kitti_format.label_convert(label_save_dir,label_read_dir,calib_save_dir)
        print('Finish!')
    else:
        print('Path Error!')
