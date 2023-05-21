#!/usr/bin/env python3

# stdlib
import argparse

# 3rd-party
import rosbag
from tqdm import tqdm
from colorama import Fore, Style
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, Transform
from pytictoc import TicToc

# local packages

if __name__ == "__main__":
    # Parse command line arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-bfi", "--bagfile_in", help='Full path to the bagfile', type=str, required=True)
    ap.add_argument("-bfo", "--bagfile_out", help='Full path to the bagfile', type=str, required=True)
    ap.add_argument("-p", "--parent", help='Transform parent', type=str, required=True)
    ap.add_argument("-c", "--child", help='Transform child', type=str, required=True)
    args = vars(ap.parse_args())

    # --------------------------------------------------------------------------
    # Initial setup
    # --------------------------------------------------------------------------
    tictoc = TicToc()
    tictoc.tic()
    bag_out = rosbag.Bag(args['bagfile_out'], 'w')

    # --------------------------------------------------------------------------
    # Read the bag input file
    # --------------------------------------------------------------------------
    bag_file = args['bagfile_in']
    print('Loading bagfile ' + bag_file)
    bag = rosbag.Bag(bag_file) # load the bag file
    bag_info = bag.get_type_and_topic_info()
    bag_types = bag_info[0]
    bag_topics = bag_info[1]

    # Get initial stamp to compute mission time
    for topic, msg, stamp in bag.read_messages():
        initial_stamp = stamp
        break

    # --------------------------------------------------------------------------
    # Writing new bagfile
    # --------------------------------------------------------------------------
    print('Converting bagfile. Please wait.')
    found_one = False
    num_found = 0
    for topic, msg, stamp in tqdm(bag.read_messages(), total=bag.get_message_count(), desc='Processing bag messages'):
        mission_time = (stamp - initial_stamp)

        # if mission_time.to_sec() > 30: # just for testing fast, analyze messages only until 10 secs mission time.
            # break
        
        if not topic == '/tf':
            bag_out.write(topic, msg, stamp)
        else:

            idxs_to_remove = []
            for transform_idx, transform in enumerate(msg.transforms): # iterate all transforms

                if transform.header.frame_id == args['parent'] and transform.child_frame_id == args['child']: 

                    # Set a new transform value for rotation
                    # print('Got transform in msg on topic ' + topic + ' with parent ' + transform.header.frame_id + ' and child ' + transform.child_frame_id  + ' at time ' + str((stamp-initial_stamp).to_sec()))
 
                    idxs_to_remove.append(transform_idx) # should remove this transform from the tf msg
                    num_found += 1

                    # publish static transform if haven't do so before
                    if not found_one:
                        tf_static_msg = TFMessage() # publish this transform on topic /tf_static
                        tf_static_msg.transforms.append(transform)
                        bag_out.write('/tf_static', tf_static_msg, stamp)
                        found_one = True


            idxs_to_remove.reverse()
            for idx_to_remove in idxs_to_remove: # remove transform from tf message
                msg.transforms.pop(idx_to_remove)

            if len(msg.transforms) > 0: # after removing the transformation, there are still some in the message
                bag_out.write('/tf', msg, stamp)


    bag.close() # close the bag file.
    bag_out.close() # close the bag file.

    # Print final report
    print('Finished in ' + str(round(tictoc.tocvalue(),2)) + ' seconds.')
    print('Created a /tf_static of transform ' + Fore.BLUE + args['parent'] + Style.RESET_ALL  + ' to ' + Fore.BLUE + args['child'] + Style.RESET_ALL + '\nRemoved ' + str(num_found) + ' transformations from bag.')