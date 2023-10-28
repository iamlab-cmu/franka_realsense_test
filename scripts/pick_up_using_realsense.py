from frankapy import FrankaArm
import numpy as np
import argparse
import cv2
import rospy
from cv_bridge import CvBridge
from autolab_core import RigidTransform, Point

from perception import CameraIntrinsics
from sensor_msgs.msg import Image

REALSENSE_INTRINSICS = "config/realsense.intr"
REALSENSE_EE_TF = "config/realsense_ee.tf"

def get_object_center_point_in_world_realsense(
    object_image_center_x,
    object_image_center_y,
    depth_image,
    intrinsics,
    transform,
    current_pose,
):

    object_center = Point(
        np.array([object_image_center_x, object_image_center_y]),
        "realsense_ee",
    )
    object_depth = depth_image[object_image_center_y, object_image_center_x] * 0.001
    print(
        "x, y, z: ({:.4f}, {:.4f}, {:.4f})".format(
            object_image_center_x, object_image_center_y, object_depth
        )
    )

    object_center_point_in_world = current_pose * transform * intrinsics.deproject_pixel(
        object_depth, object_center
    )
    print(object_center_point_in_world)

    return object_center_point_in_world


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--intrinsics_file_path", type=str, default=REALSENSE_INTRINSICS
    )
    parser.add_argument("--extrinsics_file_path", type=str, default=REALSENSE_EE_TF)
    args = parser.parse_args()

    print("Starting robot")
    fa = FrankaArm()

    print("Opening Grippers")
    # Open Gripper
    fa.open_gripper()

    # Reset Pose
    fa.reset_pose()
    # Reset Joints
    fa.reset_joints()

    # Current Pose
    current_pose = fa.get_pose()

    # Move the robot 5 cm up and 20 cm forward
    desired_pose = current_pose.copy()
    current_pose.translation[0] += 0.2
    current_pose.translation[2] += 0.05
    fa.goto_pose(current_pose)

    cv_bridge = CvBridge()
    realsense_intrinsics = CameraIntrinsics.load(args.intrinsics_file_path)
    realsense_to_ee_transform = RigidTransform.load(args.extrinsics_file_path)

    rgb_image_msg = rospy.wait_for_message('/camera/color/image_raw', Image)
    rgb_cv_image = cv_bridge.imgmsg_to_cv2(rgb_image_msg)
    rgb_image = cv2.cvtColor(rgb_cv_image, cv2.COLOR_RGB2BGR)

    depth_image_msg = rospy.wait_for_message('/camera/aligned_depth_to_color/image_raw', Image)
    depth_image = cv_bridge.imgmsg_to_cv2(depth_image_msg)

    object_image_position = np.array([200, 300])

    def onMouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            print("x = %d, y = %d" % (x, y))
            param[0] = x
            param[1] = y

    cv2.namedWindow("image")
    cv2.imshow("image", rgb_image)
    cv2.setMouseCallback("image", onMouse, object_image_position)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    current_pose = fa.get_pose()
    height_offset = 0.1

    object_center_point_in_world = get_object_center_point_in_world_realsense(
        object_image_position[0],
        object_image_position[1],
        depth_image,
        realsense_intrinsics,
        realsense_to_ee_transform,
        current_pose
    )

    object_center_pose = current_pose.copy()

    object_center_pose.translation = [
        object_center_point_in_world[0],
        object_center_point_in_world[1],
        object_center_point_in_world[2],
    ]

    intermediate_robot_pose = object_center_pose.copy()
    intermediate_robot_pose.translation = [
        object_center_point_in_world[0],
        object_center_point_in_world[1],
        object_center_point_in_world[2] + height_offset,
    ]

    # Move to intermediate robot pose
    fa.goto_pose(intermediate_robot_pose)

    fa.goto_pose(object_center_pose, 5, force_thresholds=[10, 10, 10, 10, 10, 10])

    # Close Gripper
    fa.goto_gripper(0.045, grasp=True, force=10.0)

    # Move to intermediate robot pose
    fa.goto_pose(intermediate_robot_pose)

    fa.goto_pose(object_center_pose, 5, force_thresholds=[10, 10, 20, 10, 10, 10])

    print("Opening Grippers")
    # Open Gripper
    fa.open_gripper()

    fa.goto_pose(intermediate_robot_pose)

    # Reset Pose
    fa.reset_pose()
    # Reset Joints
    fa.reset_joints()

