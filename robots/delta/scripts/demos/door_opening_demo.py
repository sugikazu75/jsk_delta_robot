#!/usr/bin/env python

from delta.delta_interface import DeltaInterface
from delta.common_states import *
import math
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from smach import State, StateMachine
import smach_ros
import tf
import time

class Wait(BaseState):
    def __init__(self, robot):
        BaseState.__init__(self, robot, outcomes=['success'])
        time.sleep(1.0)

    def execute(self, userdata):
        time.sleep(2.0)
        return 'success'

class Approach(BaseState):
    def __init__(self, robot):
        BaseState.__init__(self, robot, outcomes=['success'])
        self.approach_cog_pos_offset = rospy.get_param("~approach_cog_pos_offset")
        self.approach_yaw_angle = rospy.get_param("~approach_yaw_angle")
        self.approach_wait_time = rospy.get_param("~approach_wait_time")
        self.door_knob_initial_world_pos = rospy.get_param("~door_knob_initial_world_pos")
        time.sleep(1.0)

    def execute(self, userdata):
        # set target pos
        approach_cog_pos = np.array(self.door_knob_initial_world_pos) + np.array(self.approach_cog_pos_offset)
        self.robot.setTargetPosTraj(approach_cog_pos)
        rospy.logwarn("[Approach] go to {}".format(approach_cog_pos))
        start_time = rospy.get_time()
        while(rospy.get_time() - start_time < self.approach_wait_time):
            rospy.logwarn_throttle(3.0, "[Approach] waiting after pos control command")
            pass

        # check pos control convergence
        converge_flag = False
        while not converge_flag:
            cog_odom = self.robot.getCogOdom()
            if self.robot.posControlConverged(target=approach_cog_pos, thresh=0.1):
                converge_flag = True
            else:
                pos_control_error = self.robot.getPosControlError(approach_cog_pos)
                rospy.logwarn_throttle(3.0, "[Approach] waiting for pos control convergence. error: {} {} {}".format(pos_control_error[0], pos_control_error[1], pos_control_error[2]))

        # set target yaw angle
        self.robot.setTargetYawTraj(self.approach_yaw_angle)
        rospy.logwarn("[Approach] set target yaw angle to {}".format(self.approach_yaw_angle))
        start_time = rospy.get_time()
        while(rospy.get_time() - start_time < self.approach_wait_time):
            pass
        while not self.robot.yawControlConverged(target=self.approach_yaw_angle):
            pass

        return 'success'

class PrepareManip(BaseState):
    def __init__(self, robot):
        BaseState.__init__(self, robot, outcomes=['success'])
        self.prepare_manip_joint_angle = rospy.get_param("~prepare_manip_joint_angle")
        self.prepare_manip_wait_time = rospy.get_param("~prepare_manip_wait_time")
        self.prepare_manip_door_knob_offset = rospy.get_param("~prepare_manip_door_knob_offset")
        self.door_knob_initial_world_pos = rospy.get_param("~door_knob_initial_world_pos")

    def execute(self, userdata):
        # set joint angle
        msg = JointState()
        msg.name = ["joint1", "joint2"]
        msg.position = self.prepare_manip_joint_angle
        self.robot.sendJointState(msg)
        rospy.logwarn("[PrepareManip] set joint angle")
        start_time= rospy.get_time()
        while(rospy.get_time() - start_time < self.prepare_manip_wait_time):
            pass

        # set target pos
        target_ef_pos = np.array(self.door_knob_initial_world_pos) + np.array(self.prepare_manip_door_knob_offset)
        (trans, rot) = self.robot.getTransform("cog", "link1")
        world_R_cog = self.robot.getCogRotationMatrix()
        prepare_manip_cog_pos = target_ef_pos - np.dot(world_R_cog, trans)
        self.robot.setTargetPosTraj(prepare_manip_cog_pos)
        start_time= rospy.get_time()
        while(rospy.get_time() - start_time < self.prepare_manip_wait_time):
            pass
        while not self.robot.posControlConverged(target=prepare_manip_cog_pos):
            rospy.logwarn_throttle(3.0, "[PrepareManip] waiting for pos control converged")
            pass

        return 'success'

class Manip(BaseState):
    def __init__(self, robot):
        BaseState.__init__(self, robot, outcomes=['success', 'recover'])
        self.manip_target_pos = rospy.get_param("~manip_target_pos")
        self.manip_wait_time = rospy.get_param("~manip_wait_time")
        self.door_knob_final_world_pos = rospy.get_param("~door_knob_final_world_pos")

    def execute(self, userdata):
        (trans, rot) = self.robot.getTransform("cog", "link1")
        world_R_cog = self.robot.getCogRotationMatrix()
        self.manip_target_pos = self.door_knob_final_world_pos - np.dot(world_R_cog, trans)
        self.robot.setTargetPosTraj(self.manip_target_pos)
        rospy.logwarn("[Manip] set target pos {}".format(self.manip_target_pos))
        start_time = rospy.get_time()
        while(rospy.get_time() - start_time < self.manip_wait_time):
            pass

        if not self.robot.posControlConverged(target=self.manip_target_pos, thresh=0.1):
            return "recover"

        return 'success'

class FinishTask(BaseState):
    def __init__(self, robot):
        BaseState.__init__(self, robot, outcomes=['success'])
        self.finish_task_target_pos = rospy.get_param('~finish_task_target_pos')
        self.finish_task_joint_angle = rospy.get_param('~finish_task_joint_angle')
        self.finish_task_wait_time = rospy.get_param('~finish_task_wait_time')

    def execute(self, userdata):
        # set joint angle
        msg = JointState()
        msg.name = ["joint1", "joint2"]
        msg.position = self.finish_task_joint_angle
        self.robot.sendJointState(msg)
        time.sleep(self.finish_task_wait_time)

        # set target pos
        self.robot.setTargetPosTraj(self.finish_task_target_pos)
        time.sleep(self.finish_task_wait_time)

        return "success"

if __name__ == "__main__":
    rospy.init_node("door_opening_demo")
    robot = DeltaInterface()

    sm = StateMachine(outcomes=['success'])
    with sm:
        StateMachine.add('WAIT_HOVERING', Wait(robot), transitions={'success':'HOVERING'})

        sm_hover = StateMachine(outcomes=['success', ])
        with sm_hover:
            StateMachine.add('START', Start(robot), transitions={'success':'TAKEOFF'})
            StateMachine.add('TAKEOFF', Takeoff(robot), transitions={'success':'HOVERING_CHECK'})
            StateMachine.add('HOVERING_CHECK', HoveringCheck(robot), transitions={'success':'success'})
        StateMachine.add('HOVERING', sm_hover, transitions={'success':'WAIT_TASK'})

        StateMachine.add('WAIT_TASK', Wait(robot), transitions={'success':'APPROACH'})
        StateMachine.add('APPROACH', Approach(robot), transitions={'success':'PREPAREMANIP'})
        StateMachine.add('PREPAREMANIP', PrepareManip(robot), transitions={'success':'MANIP'})
        StateMachine.add('MANIP', Manip(robot), transitions={'success':'FINISHTASK', 'recover':'PREPAREMANIP'})
        StateMachine.add('FINISHTASK', FinishTask(robot), transitions={'success':'LAND'})
        StateMachine.add('LAND', Land(robot), transitions={'success':'success'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()

    rospy.spin()
    sis.stop()
