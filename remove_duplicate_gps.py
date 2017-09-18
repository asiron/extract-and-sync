import rosbag, argparse
from tqdm import tqdm

def make_translation_tuple(tran):
  return (tran.x, tran.y, tran.z)

TF_TOPIC = '/tf'
DEFAULT_FROM_FRAME_ID = '/map'
DEFAULT_TO_FRAME_ID = '/DJI/base_link_no_orientation'

if __name__ == '__main__':
  parser = argparse.ArgumentParser(description="ROS Bag RTK TF transforms duplicate removal tool")
  parser.add_argument('input', type=str, help='Input rosbag file')
  parser.add_argument('output', type=str, help='Output rosbag file')
  parser.add_argument('-f', '--from-frame', type=str, default=DEFAULT_FROM_FRAME_ID, help='Parent frame id to be selected')
  parser.add_argument('-t', '--to-frame', type=str, default=DEFAULT_TO_FRAME_ID, help='Child frame id to be selected')
  args = parser.parse_args() 

  from_f, to_f = args.from_frame, args.to_frame

  with rosbag.Bag(args.input, 'r') as input_bag:
    info = input_bag.get_type_and_topic_info()
    msg_count = sum(t.message_count for t in info.topics.values())

    with rosbag.Bag(args.output, 'w') as output_bag:

      prev_translation = (0.0, 0.0, 0.0)

      generator = tqdm(enumerate(input_bag.read_messages()), total=msg_count)
      for n, (topic, msg, stamp) in generator:

        if topic == TF_TOPIC:
          ts = [t for t in msg.transforms if (t.header.frame_id == from_f) and (t.child_frame_id == to_f)]

          cur_translation = make_translation_tuple(ts[0].transform.translation)
          if cur_translation != prev_translation:
            prev_translation = cur_translation
            output_bag.write(topic, msg, stamp)
        else:
          output_bag.write(topic, msg, stamp)
