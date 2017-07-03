#!/bin/bash

MAP_FRAME_ID=/map
CAMERA_FRAME_ID=/DJI/gimbal_base/camera
ROSBAG=$1
OUTPUT=$2

ALL_TRANSFORMS_DIR=${OUTPUT}/numpy
mkdir -p ${ALL_TRANSFORMS_DIR}

NUMPY_FILE=${ALL_TRANSFORMS_DIR}/poses.npy

python tf_extractor.py -f {MAP_FRAME_ID} -t ${CAMERA_FRAME_ID} -v -o ${NUMPY_FILE} &
echo 'TF extraction node started!'
rosbag play $ROSBAG
echo 'Finished extracting TF transforms'
kill -INT %1
echo 'Starting image extract and sync'
python extract_and_sync.py -b ${ROSBAG} -a ${NUMPY_FILE} ${OUTPUT}
echo 'Finished!'
