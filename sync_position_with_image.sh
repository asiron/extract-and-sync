#!/bin/bash

MAP_FRAME_ID=map
CAMERA_FRAME_ID=DJI/gimbal_base/camera
IMAGE_TOPIC=/dji_sdk/image_raw

ROSBAG_RATE=2

ROSBAG=$1
OUTPUT=$2

PROGRAMNAME=$0

function usage {
    echo "usage: $PROGRAMNAME <rosbag-file> <output-dir>"
    echo
    echo "   <rosbag-file> Path to a ROSBAG file containing TF and image topics"
    echo "   <output-dir>  Path to an output directory"
    echo
    exit 1
}

if [ "$#" -ne 2 ]; then usage; fi
if ! [ -e "$1" ]; then echo -e "Rosbag file does not exist!\n"; usage; fi

echo "Extracting transform from ${MAP_FRAME_ID} to ${CAMERA_FRAME_ID}"
echo "Syncing the transform with ${IMAGE_TOPIC}"

ALL_TRANSFORMS_DIR=${OUTPUT}/numpy
mkdir -p ${ALL_TRANSFORMS_DIR}

NUMPY_FILE=${ALL_TRANSFORMS_DIR}/poses.npy

python tf_extractor.py -f ${MAP_FRAME_ID} -t ${CAMERA_FRAME_ID} -v -o ${NUMPY_FILE} &
echo 'TF extraction node started!'
rosbag play -d 3 -r ${ROSBAG_RATE} $ROSBAG
echo 'Finished extracting TF transforms'
kill -INT %1
echo 'Starting image extract and sync'
python extract_and_sync.py -b ${ROSBAG} -a ${NUMPY_FILE} -t ${IMAGE_TOPIC} ${OUTPUT}
echo 'Finished!'
