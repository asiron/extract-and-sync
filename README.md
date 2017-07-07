# extract_and_sync.py
Copyright (C) 2017 - Maciej Żurad, University of Luxembourg

Tool for extracting images from a ROSBAG and syncing them with a given TF transform

### Usage

If you want to specify both source and targe frame ids, run:

```
./sync_position_with_image.sh \
  -f source-frame-id \
  -t target-frame-id \
  -i ros-image-topic \
  -b /path/to/a/rosbag.bag \
  -o /path/to/output-dir
```
By default, the source frame id is `/map` and the target frame is `DJI/gimbal_base/camera`. If you want to have the gimbal transformation included in your TF tree, check out my [dji_gimbal](https://github.com/snt-robotics/dji_sdk_utils/tree/master/dji_gimbal) repository.

You can also output the transformation's orientation in Euler angles. If you wish to do so, pass in `-e` flag.

The output directory will have the following structure:

```
/path/to/output-dir
|   image_00000.png
|   image_00001.png
|   ...
|   pos_00000.txt
|   pos_00001.txt
|   ...
└───numpy
    |   poses.npy
```

To print help, run:
```
./sync_position_with_image.sh -h
```

### Required Python packages

```
numpy
tqdm
tf2_ros
cv2
tf_conversions
```
