#! /usr/bin/env python

# import sys
# import rospy
# import moveit_commander
# import geometry_msgs.msg

# moveit_commander.roscpp_initialize(sys.argv)
# rospy.init_node('follow_joints', anonymous=True)
# robot = moveit_commander.RobotCommander()

# # arm_group = moveit_commander.MoveGroupCommander("ur5_arm_group")
# # arm_group.set_named_target("allZeros")
# # plan1 = arm_group.go()

# pose_target = geometry_msgs.msg.Pose()
# pose_target.

# rospy.sleep(5)
# moveit_commander.roscpp_shutdown()


import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math


class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('follow_joints', anonymous=True)

        self._planning_group = "ur5_arm_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()


        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


class Ur5Moveit2:

    # Constructor
    def __init__(self):

        # rospy.init_node('follow_joints2', anonymous=True)

        self._planning_group = "ur5_hand_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()


        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit()
    ur5_2 = Ur5Moveit2()

    straight = [0, -0.5, 0, 0 ,0, 0]


    pre_grip_biscuit1 = [-0.594236047507, -0.745891395877, 1.06834668824, 1.22625815142, 1.50436552291, -1.90903060023]
    pre_grip_biscuit2 = [-0.594085652731, -0.61945185253, 1.06823315957, 1.22634375452, 1.50444709373, -1.90893594952]
    grip_biscuit = [0.20]

    pre_grip_smallest_object1 = [0.240110921295, -0.948216227982, 1.59930656569, 0.821843789592, 1.55759493902, -2.0354579121]
    pre_grip_smallest_object2 = [0.240267092741, -0.847099003708, 1.59918647694, 0.745865154344, 1.5577731668, -2.03546001496]
    grip_smallest_object1 = [0.35]
    grip_smallest_object2 = [0.45]

    pre_grip_middle_object1 = [-0.231141052171, -0.693073623043, 1.22601857956, 1.08072120889, 1.55778288848, -0.991267182435]
    pre_grip_middle_object2 = [-0.23125126038, -0.637813857581, 1.22604897089, 1.08058505903, 1.55784362898, -0.991235210806]
    grip_middle_object1 = [0.2]


    pre_drop_redbox = [1.41723177748, -0.755882625351, 1.13384178385, 0.732255471498, 1.55776113755, -0.991339390529]
    drop_red_box = [0.0]

    pre_drop_blue_box = [-1.73198645651, -0.84709409624, 1.15683231513, 0.74594617151, 1.55761627015, -2.03541839745]
    drop_blue_box = [0.0]

    pre_drop_blue_box_2 = [-1.4645396179, -0.685019689982, 0.838594072515, 1.22634326884, 1.50439211908, -1.90894921109]
    drop_blue_box_2 = [0.0]

    ur5.set_joint_angles(straight)
    rospy.sleep(2)


    ur5.set_joint_angles(pre_grip_biscuit1)
    rospy.sleep(2)
    ur5.set_joint_angles(pre_grip_biscuit2)
    rospy.sleep(2)
    ur5_2.set_joint_angles(grip_biscuit)
    rospy.sleep(2)
    ur5.set_joint_angles(pre_drop_blue_box_2)
    rospy.sleep(2)
    ur5_2.set_joint_angles(drop_blue_box_2)
    rospy.sleep(2)

    ur5.set_joint_angles(pre_grip_smallest_object1)
    rospy.sleep(2)
    ur5.set_joint_angles(pre_grip_smallest_object2)
    rospy.sleep(2)
    ur5_2.set_joint_angles(grip_smallest_object1)
    rospy.sleep(2)
    ur5_2.set_joint_angles(grip_smallest_object2)
    rospy.sleep(2)
    ur5.set_joint_angles(pre_drop_blue_box)
    rospy.sleep(2)
    ur5_2.set_joint_angles(drop_blue_box)
    rospy.sleep(2)

    ur5.set_joint_angles(pre_grip_middle_object1)
    rospy.sleep(2)
    ur5.set_joint_angles(pre_grip_middle_object2)
    rospy.sleep(2)
    ur5_2.set_joint_angles(grip_middle_object1)
    rospy.sleep(2)
    ur5.set_joint_angles(pre_drop_redbox)
    rospy.sleep(2)
    ur5_2.set_joint_angles(drop_red_box)
    rospy.sleep(2)

    rospy.spin()
    moveit_commander.roscpp_shutdown()

    del ur5
    del ur5_2


if __name__ == '__main__':
    main()
