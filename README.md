# Astyx-radar-dataset-convert-to-kitti-format
This repository can transform the astyx HiRes2019 Dataset into Kitti format, This is not an official tool.

Implemented and tested on Ubuntu 16.04 with Python 3.6. Note that! Only transform the groundtruth_obj3d files and calibration files.

## Getting started
Unzip the dataset_astyx_hires2019,the folder should look something like the following:
```bash
dataset_astyx_hires2019
    dataset_astyx_demo
    dataset_astyx_hires2019  #this is the root_dir
        calibration
        camera_front
        groundtruth_obj3d
        lidar_vlp16
        radar_64555
        dataset.json
    licenes.txt
    readme.txt
```
Add the root path of dataset in demo.py：
```bash
root_dir=' your_path /dataset_astyx_hires2019/dataset_astyx_hires2019/'
```
run the demo.py
```bash
python demo.py
```
the result will find in root_dir, 'kitti_format_label' and 'kitti_format_calib'
