#!/usr/bin/env python2
from __future__ import division

import unittest
import rospy
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, \
                            WaypointList
from mavros_msgs.srv import CommandBool, ParamGet, SetMode, WaypointClear, \
                            WaypointPush
from pymavlink import mavutil
from sensor_msgs.msg import NavSatFix, Imu
from six.moves import xrange


class MavrosTestCommon(unittest.TestCase):
    def __init__(self, *args):
        super(MavrosTestCommon, self).__init__(*args)
    def setUp(self):
        #UAV0
        self.altitude = Altitude()
        self.extended_state = ExtendedState()
        self.global_position = NavSatFix()
        self.imu_data = Imu()
        self.home_position = HomePosition()
        self.local_position = PoseStamped()
        self.mission_wp = WaypointList()
        self.state = State()
        self.mav_type = None
        #UAV1
        self.altitude1 = Altitude()
        self.extended_state1 = ExtendedState()
        self.global_position1 = NavSatFix()
        self.imu_data1 = Imu()
        self.home_position1 = HomePosition()
        self.local_position1 = PoseStamped()
        self.mission_wp1 = WaypointList()
        self.state1 = State()
        self.mav_type1 = None
        #UAV2
        self.altitude2 = Altitude()
        self.extended_state2 = ExtendedState()
        self.global_position2 = NavSatFix()
        self.imu_data2 = Imu()
        self.home_position2 = HomePosition()
        self.local_position2 = PoseStamped()
        self.mission_wp2 = WaypointList()
        self.state2 = State()
        self.mav_type2 = None
        self.sub_topics_ready = {
            key: False
            for key in [
                'alt', 'ext_state', 'global_pos', 'home_pos', 'local_pos',
                'mission_wp', 'state', 'imu'
            ]
        }
        self.sub_topics_ready1 = {
            key: False
            for key in [
                'alt', 'ext_state', 'global_pos', 'home_pos', 'local_pos',
                'mission_wp', 'state', 'imu'
            ]
        }
        self.sub_topics_ready2 = {
            key: False
            for key in [
                'alt', 'ext_state', 'global_pos', 'home_pos', 'local_pos',
                'mission_wp', 'state', 'imu'
            ]
        }

        # ROS services
        service_timeout = 30
        rospy.loginfo("waiting for ROS services")
        try:
            #uav0
            rospy.wait_for_service('uav0/mavros/param/get', service_timeout)
            rospy.wait_for_service('uav0/mavros/cmd/arming', service_timeout)
            rospy.wait_for_service('uav0/mavros/mission/push', service_timeout)
            rospy.wait_for_service('uav0/mavros/mission/clear', service_timeout)
            rospy.wait_for_service('uav0/mavros/set_mode', service_timeout)
            #uav1
            rospy.wait_for_service('uav1/mavros/param/get', service_timeout)
            rospy.wait_for_service('uav1/mavros/cmd/arming', service_timeout)
            rospy.wait_for_service('uav1/mavros/mission/push', service_timeout)
            rospy.wait_for_service('uav1/mavros/mission/clear', service_timeout)
            rospy.wait_for_service('uav1/mavros/set_mode', service_timeout)
            #uav2
            rospy.wait_for_service('uav2/mavros/param/get', service_timeout)
            rospy.wait_for_service('uav2/mavros/cmd/arming', service_timeout)
            rospy.wait_for_service('uav2/mavros/mission/push', service_timeout)
            rospy.wait_for_service('uav2/mavros/mission/clear', service_timeout)
            rospy.wait_for_service('uav2/mavros/set_mode', service_timeout)
            rospy.loginfo("ROS services are up")
        except rospy.ROSException:
            self.fail("failed to connect to services")
        #UAV0
        self.get_param_srv = rospy.ServiceProxy('uav0/mavros/param/get', ParamGet)
        self.set_arming_srv = rospy.ServiceProxy('uav0/mavros/cmd/arming',
                                                 CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('uav0/mavros/set_mode', SetMode)
        self.wp_clear_srv = rospy.ServiceProxy('uav0/mavros/mission/clear',
                                               WaypointClear)
        self.wp_push_srv = rospy.ServiceProxy('uav0/mavros/mission/push',
                                              WaypointPush)
        #UAV1
        self.get_param_srv1 = rospy.ServiceProxy('uav1/mavros/param/get', ParamGet)
        self.set_arming_srv1 = rospy.ServiceProxy('uav1/mavros/cmd/arming',
                                                 CommandBool)
        self.set_mode_srv1 = rospy.ServiceProxy('uav1/mavros/set_mode', SetMode)
        self.wp_clear_srv1 = rospy.ServiceProxy('uav1/mavros/mission/clear',
                                               WaypointClear)
        self.wp_push_srv1 = rospy.ServiceProxy('uav1/mavros/mission/push',
                                              WaypointPush)
        #UAV2
        self.get_param_srv2 = rospy.ServiceProxy('uav2/mavros/param/get', ParamGet)
        self.set_arming_srv2 = rospy.ServiceProxy('uav2/mavros/cmd/arming',
                                                 CommandBool)
        self.set_mode_srv2 = rospy.ServiceProxy('uav2/mavros/set_mode', SetMode)
        self.wp_clear_srv2 = rospy.ServiceProxy('uav2/mavros/mission/clear',
                                               WaypointClear)
        self.wp_push_srv2 = rospy.ServiceProxy('uav2/mavros/mission/push',
                                              WaypointPush)

        # ROS subscribers5
        #UAV0
        self.alt_sub = rospy.Subscriber('uav0/mavros/altitude', Altitude,
                                        self.altitude_callback,(0))
        self.ext_state_sub = rospy.Subscriber('uav0/mavros/extended_state',
                                              ExtendedState,
                                              self.extended_state_callback,(0))
        self.global_pos_sub = rospy.Subscriber('uav0/mavros/global_position/global',
                                               NavSatFix,
                                               self.global_position_callback,(0))
        self.imu_data_sub = rospy.Subscriber('uav0/mavros/imu/data',
                                               Imu,
                                               self.imu_data_callback,(0))
        self.home_pos_sub = rospy.Subscriber('uav0/mavros/home_position/home',
                                             HomePosition,
                                             self.home_position_callback,(0))
        self.local_pos_sub = rospy.Subscriber('uav0/mavros/local_position/pose',
                                              PoseStamped,
                                              self.local_position_callback,(0))
        self.mission_wp_sub = rospy.Subscriber(
            'uav0/mavros/mission/waypoints', WaypointList, self.mission_wp_callback,(0))
        self.state_sub = rospy.Subscriber('uav0/mavros/state', State,
                                          self.state_callback,(0))
        #UAV1
        self.alt_sub1 = rospy.Subscriber('uav1/mavros/altitude', Altitude,
                                        self.altitude_callback,(1))
        self.ext_state_sub1 = rospy.Subscriber('uav1/mavros/extended_state',
                                              ExtendedState,
                                              self.extended_state_callback,(1))
        self.global_pos_sub1 = rospy.Subscriber('uav1/mavros/global_position/global',
                                               NavSatFix,
                                               self.global_position_callback,(1))
        self.imu_data_sub1 = rospy.Subscriber('uav1/mavros/imu/data',
                                               Imu,
                                               self.imu_data_callback,(1))
        self.home_pos_sub1 = rospy.Subscriber('uav1/mavros/home_position/home',
                                             HomePosition,
                                             self.home_position_callback,(1))
        self.local_pos_sub1 = rospy.Subscriber('uav1/mavros/local_position/pose',
                                              PoseStamped,
                                              self.local_position_callback,(1))
        self.mission_wp_sub1 = rospy.Subscriber(
            'uav1/mavros/mission/waypoints', WaypointList, self.mission_wp_callback,(1))
        self.state_sub1 = rospy.Subscriber('uav1/mavros/state', State,
                                          self.state_callback,(1))
        #UAV2
        self.alt_sub2 = rospy.Subscriber('uav2/mavros/altitude', Altitude,
                                        self.altitude_callback,(2))
        self.ext_state_sub2 = rospy.Subscriber('uav2/mavros/extended_state',
                                              ExtendedState,
                                              self.extended_state_callback,(2))
        self.global_pos_sub2 = rospy.Subscriber('uav2/mavros/global_position/global',
                                               NavSatFix,
                                               self.global_position_callback,(2))
        self.imu_data_sub2 = rospy.Subscriber('uav2/mavros/imu/data',
                                               Imu,
                                               self.imu_data_callback,(2))
        self.home_pos_sub2 = rospy.Subscriber('uav2/mavros/home_position/home',
                                             HomePosition,
                                             self.home_position_callback,(2))
        self.local_pos_sub2 = rospy.Subscriber('uav2/mavros/local_position/pose',
                                              PoseStamped,
                                              self.local_position_callback,(2))
        self.mission_wp_sub2 = rospy.Subscriber(
            'uav2/mavros/mission/waypoints', WaypointList, self.mission_wp_callback,(2))
        self.state_sub2 = rospy.Subscriber('uav2/mavros/state', State,
                                          self.state_callback,(2))

    def tearDown(self):
        self.log_topic_vars()

    #
    # Callback functions
    #
    def altitude_callback(self,data,args):
	#rospy.loginfo(args)
        if args == 0:
            self.altitude = data
            # amsl has been observed to be nan while other fields are valid
            if not self.sub_topics_ready['alt'] and not math.isnan(data.amsl):
                self.sub_topics_ready['alt'] = True

        elif args == 1:
            self.altitude1 = data
            # amsl has been observed to be nan while other fields are valid
            if not self.sub_topics_ready1['alt'] and not math.isnan(data.amsl):
                self.sub_topics_ready1['alt'] = True

        else:
            self.altitude2 = data
            # amsl has been observed to be nan while other fields are valid
            if not self.sub_topics_ready2['alt'] and not math.isnan(data.amsl):
                self.sub_topics_ready2['alt'] = True

    def extended_state_callback(self, data,args):
        if self.extended_state.vtol_state != data.vtol_state:
            rospy.loginfo("VTOL state changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_VTOL_STATE']
                [self.extended_state.vtol_state].name, mavutil.mavlink.enums[
                    'MAV_VTOL_STATE'][data.vtol_state].name))

        if self.extended_state.landed_state != data.landed_state:
            rospy.loginfo("landed state changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_LANDED_STATE']
                [self.extended_state.landed_state].name, mavutil.mavlink.enums[
                    'MAV_LANDED_STATE'][data.landed_state].name))

        if args == 0:
            self.extended_state = data
            if not self.sub_topics_ready['ext_state']:
                self.sub_topics_ready['ext_state'] = True

        elif args == 1:
            self.extended_state1 = data
            if not self.sub_topics_ready1['ext_state']:
                self.sub_topics_ready1['ext_state'] = True
        else:
            self.extended_state2 = data
            if not self.sub_topics_ready2['ext_state']:
                self.sub_topics_ready2['ext_state'] = True
            
    def global_position_callback(self, data,args):
        if args == 0:
            self.global_position = data
            if not self.sub_topics_ready['global_pos']:
                self.sub_topics_ready['global_pos'] = True
        elif args == 1:
            self.global_position1 = data
            if not self.sub_topics_ready1['global_pos']:
                self.sub_topics_ready1['global_pos'] = True
        else:
            self.global_position2 = data
            if not self.sub_topics_ready2['global_pos']:
                self.sub_topics_ready2['global_pos'] = True

    def imu_data_callback(self, data,args):
        if args == 0:
            self.imu_data = data
            if not self.sub_topics_ready['imu']:
                self.sub_topics_ready['imu'] = True
        elif args == 1:
            self.imu_data1 = data
            if not self.sub_topics_ready1['imu']:
                self.sub_topics_ready1['imu'] = True
        else:
            self.imu_data2 = data
            if not self.sub_topics_ready2['imu']:
                self.sub_topics_ready2['imu'] = True

    def home_position_callback(self, data,args):
        if args == 0:
            self.home_position = data
            if not self.sub_topics_ready['home_pos']:
                self.sub_topics_ready['home_pos'] = True
        elif args == 1:
            self.home_position1 = data
            if not self.sub_topics_ready1['home_pos']:
                self.sub_topics_ready1['home_pos'] = True
        else:
            self.home_position2 = data
            if not self.sub_topics_ready2['home_pos']:
                self.sub_topics_ready2['home_pos'] = True



    def local_position_callback(self, data,args):
        if args == 0:
            self.local_position = data
            if not self.sub_topics_ready['local_pos']:
                self.sub_topics_ready['local_pos'] = True
        elif args == 1:
            self.local_position1 = data
            if not self.sub_topics_ready1['local_pos']:
                self.sub_topics_ready1['local_pos'] = True
        else:
            self.local_position2 = data
            if not self.sub_topics_ready2['local_pos']:
                self.sub_topics_ready2['local_pos'] = True
            

    def mission_wp_callback(self, data,args):
        if args == 0:
            if self.mission_wp.current_seq != data.current_seq:
                rospy.loginfo("current mission waypoint sequence updated: {0}".
                          format(data.current_seq))
            self.mission_wp = data
            if not self.sub_topics_ready['mission_wp']:
                self.sub_topics_ready['mission_wp'] = True
        elif args == 1:
            if self.mission_wp1.current_seq != data.current_seq:
                rospy.loginfo("current mission waypoint sequence updated: {0}".
                          format(data.current_seq))
            self.mission_wp1 = data
            if not self.sub_topics_ready1['mission_wp']:
                self.sub_topics_ready1['mission_wp'] = True
        else:
            if self.mission_wp2.current_seq != data.current_seq:
                rospy.loginfo("current mission waypoint sequence updated: {0}".
                          format(data.current_seq))
            self.mission_wp2 = data
            if not self.sub_topics_ready2['mission_wp']:
                self.sub_topics_ready2['mission_wp'] = True

    def state_callback(self, data,args):
        if args == 0:
            if self.state.armed != data.armed:
                rospy.loginfo("armed state changed from {0} to {1} vehicle 0".format(
                    self.state.armed, data.armed))

            if self.state.connected != data.connected:
                rospy.loginfo("connected changed from {0} to {1} vehicle 1".format(
                    self.state.connected, data.connected))

            if self.state.mode != data.mode:
                rospy.loginfo("mode changed from {0} to {1} vehicle 2".format(
                    self.state.mode, data.mode))

            if self.state.system_status != data.system_status:
                rospy.loginfo("system_status changed from {0} to {1}".format(
                    mavutil.mavlink.enums['MAV_STATE'][
                        self.state.system_status].name, mavutil.mavlink.enums[
                            'MAV_STATE'][data.system_status].name))

            self.state = data
            # mavros publishes a disconnected state message on init
            if not self.sub_topics_ready['state'] and data.connected:
                self.sub_topics_ready['state'] = True
        elif args == 1:
            if self.state1.armed != data.armed:
                rospy.loginfo("armed state changed from {0} to {1}".format(
                    	self.state1.armed, data.armed))

            if self.state1.connected != data.connected:
                rospy.loginfo("connected changed from {0} to {1}".format(
                    self.state1.connected, data.connected))

            if self.state1.mode != data.mode:
                rospy.loginfo("mode changed from {0} to {1}".format(
                    self.state1.mode, data.mode))

            if self.state1.system_status != data.system_status:
                rospy.loginfo("system_status changed from {0} to {1}".format(
                    mavutil.mavlink.enums['MAV_STATE'][
                        self.state1.system_status].name, mavutil.mavlink.enums[
                            'MAV_STATE'][data.system_status].name))

            self.state1 = data
            # mavros publishes a disconnected state message on init
            if not self.sub_topics_ready1['state'] and data.connected:
                self.sub_topics_ready1['state'] = True
        else:
            if self.state2.armed != data.armed:
                rospy.loginfo("armed state changed from {0} to {1}".format(
                    self.state2.armed, data.armed))

            if self.state2.connected != data.connected:
                rospy.loginfo("connected changed from {0} to {1}".format(
                    self.state2.connected, data.connected))

            if self.state2.mode != data.mode:
                rospy.loginfo("mode changed from {0} to {1}".format(
                    self.state2.mode, data.mode))

            if self.state2.system_status != data.system_status:
                rospy.loginfo("system_status changed from {0} to {1}".format(
                    mavutil.mavlink.enums['MAV_STATE'][
                        self.state2.system_status].name, mavutil.mavlink.enums[
                            'MAV_STATE'][data.system_status].name))

            self.state2 = data
            # mavros publishes a disconnected state message on init
            if not self.sub_topics_ready2['state'] and data.connected:
                self.sub_topics_ready2['state'] = True

    #
    # Helper methods
    #
    def set_arm(self, arm, timeout):
        """arm: True to arm or False to disarm, timeout(int): seconds"""
        rospy.loginfo("setting FCU arm: {0}".format(arm))
        old_arm = self.state.armed
        old_arm1 = self.state1.armed
        old_arm2 = self.state2.armed
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        arm_set = [False]*3
        for i in xrange(timeout * loop_freq):
            if self.state.armed == arm:
                arm_set[0] = True
                rospy.loginfo("set arm success vehiccle 0 | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                #break
            else:
                try:
                    res = self.set_arming_srv(arm)
                    if not res.success:
                        rospy.logerr("failed to send arm command vehicle 0")
                except rospy.ServiceException as e:
                    rospy.logerr(e)
            if self.state1.armed == arm:
                arm_set[1] = True
                rospy.loginfo("set arm success vehicle 1| seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                #break
            else:
                try:
                    res = self.set_arming_srv1(arm)
                    if not res.success:
                        rospy.logerr("failed to send arm command vehicle 1")
                except rospy.ServiceException as e:
                    rospy.logerr(e)
            if self.state2.armed == arm:
                arm_set[2] = True
                rospy.loginfo("set arm success vehicle 2| seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                #break
            else:
                try:
                    res = self.set_arming_srv2(arm)
                    if not res.success:
                        rospy.logerr("failed to send arm command vehicle 2")
                except rospy.ServiceException as e:
                    rospy.logerr(e)
            

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(arm_set[0], (
            "failed to set arm vehicle 0| new arm: {0}, old arm: {1} | timeout(seconds): {2}".
            format(arm, old_arm, timeout)))
        self.assertTrue(arm_set[1], (
            "failed to set arm vehicle 1| new arm: {0}, old arm: {1} | timeout(seconds): {2}".
            format(arm, old_arm1, timeout)))
        self.assertTrue(arm_set[2], (
            "failed to set arm vehicle 2| new arm: {0}, old arm: {1} | timeout(seconds): {2}".
            format(arm, old_arm2, timeout)))


    def set_mode(self, mode, timeout):
        """mode: PX4 mode string, timeout(int): seconds"""
        rospy.loginfo("setting FCU mode: {0}".format(mode))
        old_mode = self.state.mode
        old_mode1 = self.state1.mode
        old_mode2 = self.state2.mode
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        mode_set = [False]*3
        for i in xrange(timeout * loop_freq):
            if self.state.mode == mode:
                mode_set[0] = True
                rospy.loginfo("set mode success vehicle 0 | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                #break
            else:
                try:
                    rospy.loginfo("In here 0")
                    res = self.set_mode_srv(0, mode)#0 is custom mode
                    if not res.mode_sent:
                        rospy.logerr("failed to send mode command vehicle 0")
                except rospy.ServiceException as e:
                    rospy.logerr(e)
            if self.state1.mode == mode:
                mode_set[1]= True
                rospy.loginfo("set mode success vehicle 1 | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                #break
            else:
                try:
                    rospy.loginfo("In here 1")
                    res1 = self.set_mode_srv1(0, mode)  # 0 is custom mode
                    if not res1.mode_sent:
                        rospy.logerr("failed to send mode command vehicle 1")
                except rospy.ServiceException as e:
                    rospy.logerr(e)
            if self.state2.mode == mode:
                mode_set[2]= True
                rospy.loginfo("set mode success vehicle 2 | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                #break
            else:
                try:
                    rospy.loginfo("In here 2")
                    res2 = self.set_mode_srv2(0, mode)  # 0 is custom mode
                    if not res2.mode_sent:
                        rospy.logerr("failed to send mode command vehicle 2")
                except rospy.ServiceException as e:
                    rospy.logerr(e)


            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(mode_set[0], (
            "failed to set mode vehicle 0 | new mode: {0}, old mode: {1} | timeout(seconds): {2}".
            format(mode, old_mode, timeout)))
        self.assertTrue(mode_set[1], (
            "failed to set mode vehicle 1 | new mode: {0}, old mode: {1} | timeout(seconds): {2}".
            format(mode, old_mode1, timeout)))
        self.assertTrue(mode_set[2], (
            "failed to set mode vehicle 2 | new mode: {0}, old mode: {1} | timeout(seconds): {2}".
            format(mode, old_mode2, timeout)))

    def wait_for_topics(self, timeout):
        """wait for simulation to be ready, make sure we're getting topic info
        from all topics by checking dictionary of flag values set in callbacks,
        timeout(int): seconds"""
        rospy.loginfo("waiting for subscribed topics to be ready")
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        simulation_ready = [False]*3
        for i in xrange(timeout * loop_freq):
            if all(value for value in self.sub_topics_ready.values()):
                simulation_ready[0] = True
                rospy.loginfo("simulation topics ready vehicle 0 | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
            if all(value for value in self.sub_topics_ready1.values()):
                simulation_ready[1] = True
                rospy.loginfo("simulation topics ready vehicle 1 | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
            if all(value for value in self.sub_topics_ready2.values()):
                simulation_ready[2] = True
                rospy.loginfo("simulation topics ready vehicle 2 | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
            
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(simulation_ready[0], (
            "failed to hear from all subscribed simulation topics | topic ready flags: {0} | timeout(seconds): {1}".
            format(self.sub_topics_ready, timeout)))
        self.assertTrue(simulation_ready[1], (
            "failed to hear from all subscribed simulation topics | topic ready flags: {0} | timeout(seconds): {1}".
            format(self.sub_topics_ready1, timeout)))
        self.assertTrue(simulation_ready[2], (
            "failed to hear from all subscribed simulation topics | topic ready flags: {0} | timeout(seconds): {1}".
            format(self.sub_topics_ready2, timeout)))

    def wait_for_landed_state(self, desired_landed_state, timeout, index):
        rospy.loginfo("waiting for landed state | state: {0}, index: {1}".
                      format(mavutil.mavlink.enums['MAV_LANDED_STATE'][
                          desired_landed_state].name, index))
        loop_freq = 10  # Hz
        rate = rospy.Rate(loop_freq)
        landed_state_confirmed = [False]*3
        for i in xrange(timeout * loop_freq):
            if self.extended_state.landed_state == desired_landed_state:
                landed_state_confirmed[0] = True
                rospy.loginfo("landed state confirmed vehicle 0 | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
            if self.extended_state1.landed_state == desired_landed_state:
                landed_state_confirmed[1] = True
                rospy.loginfo("landed state confirmed vehicle 1 | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
            if self.extended_state2.landed_state == desired_landed_state:
                landed_state_confirmed[2] = True
                rospy.loginfo("landed state confirmed vehicle 2 | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(landed_state_confirmed[0], (
            "landed state not detected vehicle 0| desired: {0}, current: {1} | index: {2}, timeout(seconds): {3}".
            format(mavutil.mavlink.enums['MAV_LANDED_STATE'][
                desired_landed_state].name, mavutil.mavlink.enums[
                    'MAV_LANDED_STATE'][self.extended_state.landed_state].name,
                   index, timeout)))
        self.assertTrue(landed_state_confirmed[1], (
            "landed state not detected vehicle 1| desired: {0}, current: {1} | index: {2}, timeout(seconds): {3}".
            format(mavutil.mavlink.enums['MAV_LANDED_STATE'][
                desired_landed_state].name, mavutil.mavlink.enums[
                    'MAV_LANDED_STATE'][self.extended_state1.landed_state].name,
                   index, timeout)))
        self.assertTrue(landed_state_confirmed[2], (
            "landed state not detected vehicle 2| desired: {0}, current: {1} | index: {2}, timeout(seconds): {3}".
            format(mavutil.mavlink.enums['MAV_LANDED_STATE'][
                desired_landed_state].name, mavutil.mavlink.enums[
                    'MAV_LANDED_STATE'][self.extended_state2.landed_state].name,
                   index, timeout)))

    def wait_for_vtol_state(self, transition, timeout, index):
        """Wait for VTOL transition, timeout(int): seconds"""
        rospy.loginfo(
            "waiting for VTOL transition | transition: {0}, index: {1}".format(
                mavutil.mavlink.enums['MAV_VTOL_STATE'][
                    transition].name, index))
        loop_freq = 10  # Hz
        rate = rospy.Rate(loop_freq)
        transitioned = [False]*3
        for i in xrange(timeout * loop_freq):
            if transition == self.extended_state.vtol_state:
                rospy.loginfo("transitioned vehicle 0 | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                transitioned[0] = True
            if transition == self.extended_state1.vtol_state:
                rospy.loginfo("transitioned vehicle 1| seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                transitioned[1] = True
            if transition == self.extended_state2.vtol_state:
                rospy.loginfo("transitioned vehicle 2| seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                transitioned[2] = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(transitioned[0], (
            "transition not detected vehicle 0| desired: {0}, current: {1} | index: {2} timeout(seconds): {3}".
            format(mavutil.mavlink.enums['MAV_VTOL_STATE'][transition].name,
                   mavutil.mavlink.enums['MAV_VTOL_STATE'][
                       self.extended_state.vtol_state].name, index, timeout)))
        self.assertTrue(transitioned[1], (
            "transition not detected vehicle 1| desired: {0}, current: {1} | index: {2} timeout(seconds): {3}".
            format(mavutil.mavlink.enums['MAV_VTOL_STATE'][transition].name,
                   mavutil.mavlink.enums['MAV_VTOL_STATE'][
                       self.extended_state1.vtol_state].name, index, timeout)))
        self.assertTrue(transitioned[2], (
            "transition not detected vehicle 2| desired: {0}, current: {1} | index: {2} timeout(seconds): {3}".
            format(mavutil.mavlink.enums['MAV_VTOL_STATE'][transition].name,
                   mavutil.mavlink.enums['MAV_VTOL_STATE'][
                       self.extended_state2.vtol_state].name, index, timeout)))


    def clear_wps(self, timeout):
        """timeout(int): seconds"""
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        wps_cleared =[ False]*3
        for i in xrange(timeout * loop_freq):
            if not self.mission_wp.waypoints:
                wps_cleared[0] = True
                rospy.loginfo("clear waypoints success vehicle 0| seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
            else:
                try:
                    res = self.wp_clear_srv()
                    if not res.success:
                        rospy.logerr("failed to send waypoint clear command vehicle 0")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)
            if not self.mission_wp1.waypoints:
                wps_cleared[1] = True
                rospy.loginfo("clear waypoints success vehicle 1| seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
            else:
                try:
                    res = self.wp_clear_srv1()
                    if not res.success:
                        rospy.logerr("failed to send waypoint clear command vehicle 1")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)
            if not self.mission_wp2.waypoints:
                wps_cleared[2] = True
                rospy.loginfo("clear waypoints success vehicle 2| seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.wp_clear_srv2()
                    if not res.success:
                        rospy.logerr("failed to send waypoint clear command vehicle 2")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(wps_cleared[0], (
            "failed to clear waypoints vehicle 0 | timeout(seconds): {0}".format(timeout)
        ))
        self.assertTrue(wps_cleared[1], (
            "failed to clear waypoints vehicle 1 | timeout(seconds): {0}".format(timeout)
        ))
        self.assertTrue(wps_cleared[2], (
            "failed to clear waypoints vehicle 2 | timeout(seconds): {0}".format(timeout)
        ))
        
        

    def send_wps(self, waypoints, timeout):
        """waypoints, timeout(int): seconds"""
        rospy.loginfo("sending mission waypoints")
        if self.mission_wp.waypoints:
            rospy.loginfo("FCU0 already has mission waypoints")
        if self.mission_wp1.waypoints:
            rospy.loginfo("FCU1 already has mission waypoints")
        if self.mission_wp2.waypoints:
            rospy.loginfo("FCU2 already has mission waypoints")



        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        wps_sent = [False]*3
        wps_verified = [False]*3
        for i in xrange(timeout * loop_freq):
            if not wps_sent[0]:
                try:
                    res = self.wp_push_srv(start_index=0, waypoints=waypoints)
                    wps_sent[0] = res.success
                    if wps_sent[0]:
                        rospy.loginfo("waypoints successfully transferred for vehicle 0")
                except rospy.ServiceException as e:
                    rospy.logerr(e)
            else:
                if len(waypoints) == len(self.mission_wp.waypoints):
                    rospy.loginfo("number of waypoints transferred: {0}".
                                  format(len(waypoints)))
                    wps_verified[0] = True

            if wps_sent and wps_verified:
                rospy.loginfo("send waypoints success | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue((
            wps_sent and wps_verified
        ), "mission could not be transferred and verified | timeout(seconds): {0}".
                        format(timeout))

    def wait_for_mav_type(self, timeout):
        """Wait for MAV_TYPE parameter, timeout(int): seconds"""
        rospy.loginfo("waiting for MAV_TYPE")
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        res = [False]*3
        for i in xrange(timeout * loop_freq):
            try:
                res[0]= self.get_param_srv('MAV_TYPE')
                res[1]= self.get_param_srv1('MAV_TYPE')
                res[2]= self.get_param_srv2('MAV_TYPE')
                if res[0].success:
                    self.mav_type = res.value.integer
                    rospy.loginfo(
                        "MAV_TYPE received vehicle 0| type: {0} | seconds: {1} of {2}".
                        format(mavutil.mavlink.enums['MAV_TYPE'][self.mav_type]
                               .name, i / loop_freq, timeout))
                if res[1].success:
                    self.mav_type1 = res.value.integer
                    rospy.loginfo(
                        "MAV_TYPE received vehicle 1| type: {0} | seconds: {1} of {2}".
                        format(mavutil.mavlink.enums['MAV_TYPE'][self.mav_type1]
                               .name, i / loop_freq, timeout))
                if res[2].success:
                    self.mav_type2 = res.value.integer
                    rospy.loginfo(
                        "MAV_TYPE received vehicle 2| type: {0} | seconds: {1} of {2}".
                        format(mavutil.mavlink.enums['MAV_TYPE'][self.mav_type2]
                               .name, i / loop_freq, timeout))
                    break
            except rospy.ServiceException as e:
                rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(res[0].success, (
            "MAV_TYPE param get failed vehicle 0 | timeout(seconds): {0}".format(timeout)
        ))
        self.assertTrue(res[1].success, (
            "MAV_TYPE param get failed vehicle 1 | timeout(seconds): {0}".format(timeout)
        ))
        self.assertTrue(res[2].success, (
            "MAV_TYPE param get failed vehicle 2 | timeout(seconds): {0}".format(timeout)
        ))

    def log_topic_vars(self):
        """log the state of topic variables"""
        rospy.loginfo("======== UAV0 =========")
        rospy.loginfo("========================")
        rospy.loginfo("===== topic values =====")
        rospy.loginfo("========================")
        rospy.loginfo("altitude:\n{}".format(self.altitude))
        rospy.loginfo("========================")
        rospy.loginfo("extended_state:\n{}".format(self.extended_state))
        rospy.loginfo("========================")
        rospy.loginfo("global_position:\n{}".format(self.global_position))
        rospy.loginfo("========================")
        rospy.loginfo("home_position:\n{}".format(self.home_position))
        rospy.loginfo("========================")
        rospy.loginfo("local_position:\n{}".format(self.local_position))
        rospy.loginfo("========================")
        rospy.loginfo("mission_wp:\n{}".format(self.mission_wp))
        rospy.loginfo("========================")
        rospy.loginfo("state:\n{}".format(self.state))
        rospy.loginfo("========================")
        rospy.loginfo("======== UAV1 =========")
        rospy.loginfo("========================")
        rospy.loginfo("===== topic values =====")
        rospy.loginfo("========================")
        rospy.loginfo("altitude:\n{}".format(self.altitude1))
        rospy.loginfo("========================")
        rospy.loginfo("extended_state:\n{}".format(self.extended_state1))
        rospy.loginfo("========================")
        rospy.loginfo("global_position:\n{}".format(self.global_position1))
        rospy.loginfo("========================")
        rospy.loginfo("home_position:\n{}".format(self.home_position1))
        rospy.loginfo("========================")
        rospy.loginfo("local_position:\n{}".format(self.local_position1))
        rospy.loginfo("========================")
        rospy.loginfo("mission_wp:\n{}".format(self.mission_wp1))
        rospy.loginfo("========================")
        rospy.loginfo("state:\n{}".format(self.state1))
        rospy.loginfo("========================")
        rospy.loginfo("======== UAV2 =========")
        rospy.loginfo("========================")
        rospy.loginfo("===== topic values =====")
        rospy.loginfo("========================")
        rospy.loginfo("altitude:\n{}".format(self.altitude2))
        rospy.loginfo("========================")
        rospy.loginfo("extended_state:\n{}".format(self.extended_state2))
        rospy.loginfo("========================")
        rospy.loginfo("global_position:\n{}".format(self.global_position2))
        rospy.loginfo("========================")
        rospy.loginfo("home_position:\n{}".format(self.home_position2))
        rospy.loginfo("========================")
        rospy.loginfo("local_position:\n{}".format(self.local_position2))
        rospy.loginfo("========================")
        rospy.loginfo("mission_wp:\n{}".format(self.mission_wp2))
        rospy.loginfo("========================")
        rospy.loginfo("state:\n{}".format(self.state2))
        rospy.loginfo("========================")

