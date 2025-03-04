import rospy
from geometry_msgs.msg import Pose
from hiwonder_interfaces.srv import SetRobotPose, GetRobotPose, SetJointValue
import jetarm_kinematics.transform as transform

# Initialize the ROS node
rospy.init_node('test_jetarm_driver', anonymous=True)

def test_get_current_pose():
    try:
        # Create a ROS service proxy for the 'get_current_pose' service
        get_pose_proxy = rospy.ServiceProxy('/kinematics/get_current_pose', GetRobotPose, persistent=True)
        res = get_pose_proxy()  # Call the service
        if res.success:
            print(f"Current Pose: {res.pose}")
            return True
        else:
            print("Failed to get current pose.")
            return False
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return False

def test_set_pose_target():
    try:
        # Define the target position (x, y, z) and pitch (in degrees)
        target_position = [0.2, -0.1, 0.15]  # Example target position in meters
        pitch = 45  # Example pitch angle in degrees
        pitch_range = [-180, 180]  # Range of allowed pitch values
        resolution = 1  # Resolution of the pitch search (optional)

        # Create a ROS service proxy for setting the pose target
        set_pose_proxy = rospy.ServiceProxy('/kinematics/set_pose_target', SetRobotPose, persistent=True)
        res = set_pose_proxy(target_position, pitch, pitch_range, resolution)

        if res.success:
            print(f"Successfully set pose target. Pulse: {res.pulse}, Current Pulse: {res.current_pulse}")
            return True
        else:
            print("Failed to set pose target.")
            return False
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return False

def test_set_joint_value_target():
    try:
        # Define the joint values (pulse widths, typically between 0 and 1000)
        joint_values = [500, 500, 500, 500, 500]  # Example joint pulse widths
        
        # Create a ROS service proxy for setting the joint target values
        set_joint_value_proxy = rospy.ServiceProxy('/kinematics/set_joint_value_target', SetJointValue, persistent=True)
        res = set_joint_value_proxy(joint_values)

        print(f"Set joint values result: {res.pose}")
        return True
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return False

def test_kinematics():
    # Forward Kinematics: Given joint angles, get end-effector pose (for testing)
    joint_angles = [30, 60, 90, 120, 150]  # Example joint angles in degrees
    joint_angles_radians = [radians(angle) for angle in joint_angles]
    pulse_values = transform.angle2pulse(joint_angles_radians)
    print(f"Forward Kinematics Pulse: {pulse_values}")

    # Inverse Kinematics: Given position and pitch, find joint angles
    position = [0.2, -0.1, 0.15]  # Example position (x, y, z)
    pitch = 45  # Example pitch angle
    res = test_set_pose_target()  # Using test_set_pose_target to compute inverse kinematics
    if res:
        print(f"Inverse Kinematics Result: {res}")

if __name__ == "__main__":
    if test_get_current_pose():
        print("Test Get Current Pose Passed.")
    if test_set_pose_target():
        print("Test Set Pose Target Passed.")
    if test_set_joint_value_target():
        print("Test Set Joint Value Target Passed.")
    test_kinematics()
    rospy.spin()  # Keep the node running until shutdown
