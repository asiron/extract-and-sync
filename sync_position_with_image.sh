#!/bin/bash

PROGRAMNAME=$0

DEFAULT_FROM_FRAME_ID=map
DEFAULT_TO_FRAME_ID=DJI/gimbal_base/camera
DEFAULT_IMAGE_TOPIC=/dji_sdk/image_raw
DEFAULT_ROTATION_CHOICE=quat

ROSBAG_RATE=1

function error_exit
{
  echo "${PROGRAMNAME}: ${1:-"Unknown Error"}" 1>&2
  exit 1
}

function usage {
    echo "usage: ${PROGRAMNAME} [-e] [-f FROM] [-t TO] [-i TOPIC] [-b ROSBAG] [-o OUTPUT]"
    echo 
    echo "Extracts TF transformations FROM a given frame TO a given frame and"
    echo "synchronizes them with images with from a given image TOPIC."
    echo
    echo "   [-f FROM]      FROM frame id"
    echo "   [-t TO]        TO frame id"
    echo "   [-i TOPIC]     Image topic name"
    echo "   [-b ROSBAG]    Path to a ROSBAG file containing TF and image topics"
    echo "   [-o OUTPUT]    Path to an output directory"
    echo "   [-e]           Output orientation as Euler angles"
    echo "   [-h]           Prints this help"
    echo
    exit 1
}

rotation_choice=${DEFAULT_ROTATION_CHOICE}
from_frame_id=${DEFAULT_FROM_FRAME_ID}
to_frame_id=${DEFAULT_TO_FRAME_ID}
image_topic=${DEFAULT_IMAGE_TOPIC}

while getopts 'f:t:b:o:i:eh' flag; do
  case "${flag}" in
    f) from_frame_id="${OPTARG}" ;;
    t) to_frame_id="${OPTARG}" ;;
    i) image_topic="${OPTARG}" ;;
    b) rosbag="${OPTARG}" ;;
    o) output_dir="${OPTARG}" ;;
    e) rotation_choice='euler' ;;
    h) usage ;;
    *) error_exit "Unexpected option ${flag}" ;;
  esac
done

if [[ -z $rosbag ]]; then error_exit "Argument [-b ROSBAG] is mandatory"; fi
if [[ -z $output_dir ]]; then error_exit "Argument [-o OUTPUT] is mandatory"; fi

echo "Extracting transform from ${from_frame_id} to ${to_frame_id}"
echo "Syncing the transform with ${image_topic}"

all_transforms_dir=${output_dir}/numpy
mkdir -p ${all_transforms_dir}

echo "Saving all extracted transformations in ${numpy_file}"
numpy_file=${all_transforms_dir}/poses.npy
python tf_extractor.py -r ${rotation_choice} -f ${from_frame_id} -t ${to_frame_id} -v -o ${numpy_file} &

echo 'TF extraction node started!'
rosbag play -d 3 --clock -r ${ROSBAG_RATE} ${rosbag}
sleep 1

echo 'Finished extracting TF transforms'
kill -INT %1

echo 'Starting image extract and sync'
python extract_and_sync.py -b ${rosbag} -a ${numpy_file} -t ${image_topic} ${output_dir}

echo 'Finished!'
