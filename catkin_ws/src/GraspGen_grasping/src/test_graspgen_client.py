#!/usr/bin/env python3
import rospy
import actionlib
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from robokudo_msgs.msg import GenericImgProcAnnotatorAction, GenericImgProcAnnotatorGoal

bridge = CvBridge()

rospy.init_node("test_graspgen_client")
client = actionlib.SimpleActionClient(
    "/pose_estimator/find_grasppose_graspgen",
    GenericImgProcAnnotatorAction
)
client.wait_for_server()

goal = GenericImgProcAnnotatorGoal()

# ---- DEPTH IMAGE ---- #
depth = np.load("/GraspGen2004/code/scenes/scene1/depth.npy").astype(np.float32)
goal.depth = bridge.cv2_to_imgmsg(depth, encoding="32FC1")

# ---- RGB IMAGE ---- #
rgb = np.load("/GraspGen2004/code/scenes/scene1/rgb.npy").astype(np.uint8)
goal.rgb = bridge.cv2_to_imgmsg(rgb, encoding="rgb8")

# ---- MASK IMAGE ---- #
mask = np.load("/GraspGen2004/code/scenes/scene1/label.npy").astype(np.uint16)
goal.mask_detections = [bridge.cv2_to_imgmsg(mask, encoding="16UC1")]

client.send_goal(goal)
rospy.loginfo('Client: Sent Grasp request')
client.wait_for_result()
result = client.get_result()
print(result)
if result.success:
    rospy.loginfo('Client: Received Grasp successfully')
else:
    rospy.loginfo('Client: Received Grasp failed')