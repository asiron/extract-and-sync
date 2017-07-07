import rosbag, argparse, cv2, os
import numpy as np

from tqdm import tqdm
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

DEFAULT_IMAGE_TOPIC = "/dji_sdk/image_raw"

if __name__ == '__main__':
  parser = argparse.ArgumentParser(description="ROS Bag image extraction and sync tool")
  parser.add_argument('output', type=str, help='Output directory for acquired traces')
  parser.add_argument('-r', '--threshold', type=float, default=0.2, help='Drop frame if the best transform delay is larger')
  parser.add_argument('-b', '--rosbag', type=str, required=True, help='Path to rosbag file')
  parser.add_argument('-t', '--topic', type=str, help='Image topic', default=DEFAULT_IMAGE_TOPIC)
  parser.add_argument('-a', '--transform-numpy-array', type=str, required=True, help='Numpy array with transforms')
  args = parser.parse_args() 

  topic = args.topic
  topic = topic if topic[0] == '/' else '/' + topic
  
  bag = rosbag.Bag(args.rosbag)
  info = bag.get_type_and_topic_info()
  img_count = info.topics[topic].message_count

  bridge = CvBridge()
  transforms = np.load(args.transform_numpy_array)
  transforms_index = 0

  output_dir = args.output

  if not os.path.exists(output_dir):
    os.makedirs(output_dir)

  synced_ctr = 0
  
  generator = tqdm(enumerate(bag.read_messages(topics=[args.topic])), total=img_count)
  for n, (_, msg, _) in generator:

    cur_image_stamp = msg.header.stamp.to_sec()

    idx = np.searchsorted(transforms[:,0], cur_image_stamp)

    if idx in [0, transforms.shape[0]]:
      delay = abs(cur_image_stamp - transforms[idx,0])
      if delay > args.threshold:
        tqdm.write('Threshold exceeded at first/last transform' + str(delay))
        continue

    idx += np.argmin(np.abs(transforms[idx:idx+1, 0]))
    best_transform_delay = np.abs(cur_image_stamp - transforms[idx, 0])

    generator.set_description('Current best transform delay: {:.3f}'.format(best_transform_delay))
    generator.refresh()

    if best_transform_delay > args.threshold:
      log = '{0:05d}/{1:05d} Best transform exceeded threshold with {2:.7f}, dropping frame...'
      tqdm.write(log.format(n, img_count-1, best_transform_delay))
      continue

    try:
      cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
      tqdm.write(e)
      continue

    image_filepath = 'image_{0:05d}.png'.format(synced_ctr)
    image_filepath = os.path.join(output_dir, image_filepath)
    cv2.imwrite(image_filepath, cv_image)

    transform_filepath = 'pos_{0:05d}.txt'.format(synced_ctr)
    transform_filepath = os.path.join(output_dir, transform_filepath)
    np.savetxt(transform_filepath, cur_transform.reshape((1,-1)), delimiter=',')
    
    synced_ctr += 1

  bag.close()
