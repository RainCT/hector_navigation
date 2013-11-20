#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2013 PAL Robotics SL.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Authors:
#   * Siegfried-A. Gevatter <siegfried.gevatter@pal-robotics.com>

import roslib; roslib.load_manifest('hector_exploration_node')
import rospy

from actionlib import SimpleActionClient, GoalStatus

from move_base_msgs.msg import *
from hector_nav_msgs.srv import GetRobotTrajectory

class ExplorationController:

    def __init__(self):
        self._plan_service = rospy.ServiceProxy('get_exploration_path', GetRobotTrajectory)
        self._move_base = SimpleActionClient('move_base', MoveBaseAction)

    def run(self):
        r = rospy.Rate(1 / 7.0)
        while not rospy.is_shutdown():
            self.run_once()
            r.sleep()

    def run_once(self):
        path = self._plan_service().trajectory
        poses = path.poses
        if not path.poses:
            rospy.loginfo('No frontiers left.')
            return
        rospy.loginfo('Moving to frontier...')
        self.move_to_pose(poses[-1])

    def move_to_pose(self, pose_stamped, timeout=20.0):
        goal = MoveBaseGoal()
        goal.target_pose = pose_stamped
        self._move_base.send_goal(goal)
        self._move_base.wait_for_result(rospy.Duration(timeout))
        return self._move_base.get_state() == GoalStatus.SUCCEEDED

if __name__ == '__main__':
    rospy.init_node('exploration_base_controller')
    controller = ExplorationController()
    controller.run()
