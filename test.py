#!/usr/bin/env python
from __future__ import division
from pso import func1,Particle
PKG = 'px4'

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_test_common import MavrosTestCommon
from pymavlink import mavutil
from six.moves import xrange
from std_msgs.msg import Header
from threading import Thread
#import cv2,cv_bridge
from tf.transformations import quaternion_from_euler


class MavrosOffboardPosctlTest(MavrosTestCommon):
    """
    Tests flying a path in offboard control by sending position setpoints
    via MAVROS.
    For the test to be successful it needs to reach all setpoints in a certain time.
    FIXME: add flight path assertion (needs transformation from ROS frame to NED)
    """

    def setUp(self):
        super(MavrosOffboardPosctlTest, self).setUp()
        self.pos = PoseStamped()
        self.pos1 = PoseStamped()
        self.pos2 = PoseStamped()
        self.radius = 1

        self.pos_setpoint_pub = rospy.Publisher(
            'uav0/mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.pos_setpoint_pub1 = rospy.Publisher(
            'uav1/mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.pos_setpoint_pub2 = rospy.Publisher(
            'uav2/mavros/setpoint_position/local', PoseStamped, queue_size=1)

        # send setpoints in seperate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        rospy.loginfo("Before pos thread")
        self.pos_thread.start()
        rospy.loginfo("After pos thread start")

    def tearDown(self):
	    pass
        #super(MavrosOffboardPosctlTest, self).tearDown()

    #
    # Helper methods
    #
    def send_pos(self):
        rate = rospy.Rate(10)  # Hz
        self.pos.header = Header()
        self.pos1.header = Header()
        self.pos2.header = Header()
        self.pos.header.frame_id = "base_footprint"
        self.pos1.header.frame_id = "base_footprint"
        self.pos2.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            self.pos.header.stamp = rospy.Time.now()
            self.pos1.header.stamp = rospy.Time.now()
            self.pos2.header.stamp = rospy.Time.now()
            #rospy.loginfo("Before pos publish in send_pos")
            self.pos_setpoint_pub.publish(self.pos)
            self.pos_setpoint_pub1.publish(self.pos1)
            self.pos_setpoint_pub2.publish(self.pos2)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def is_at_position(self, x, y, z,uav,offset):
        """offset: meters"""
        desired = np.array((x, y, z))
        if uav == 0:
            rospy.logdebug(
                "current position vehicle 0| x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
                    self.local_position.pose.position.x, self.local_position.pose.
                    position.y, self.local_position.pose.position.z))
            pos = np.array((self.local_position.pose.position.x,
                            self.local_position.pose.position.y,
                            self.local_position.pose.position.z))
        elif uav == 1:
            rospy.logdebug(
                "current position vehicle 1| x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
                    self.local_position1.pose.position.x, self.local_position1.pose.
                    position.y, self.local_position1.pose.position.z))
            pos = np.array((self.local_position1.pose.position.x,
                            self.local_position1.pose.position.y,
                            self.local_position1.pose.position.z))
        else:
            rospy.logdebug(
                "current position vehicle 2| x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
                    self.local_position2.pose.position.x, self.local_position2.pose.
                    position.y, self.local_position2.pose.position.z))
            pos = np.array((self.local_position2.pose.position.x,
                            self.local_position2.pose.position.y,
                            self.local_position2.pose.position.z))


        return np.linalg.norm(desired - pos) < offset

    def reach_position(self, x, y, z,uav, timeout):
        """timeout(int): seconds"""
        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = 0  # North
        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        reached = False
        if uav == 0 :
            # set a position setpoint
            self.pos.pose.position.x = x
            self.pos.pose.position.y = y
            self.pos.pose.position.z = z
            rospy.loginfo(
                "attempting to reach position  vehicle 0| x: {0}, y: {1}, z: {2} | current position x: {3:.2f}, y: {4:.2f}, z: {5:.2f}".
                format(x, y, z, self.local_position.pose.position.x,
                    self.local_position.pose.position.y,
                    self.local_position.pose.position.z))
            self.pos.pose.orientation = Quaternion(*quaternion)

            for i in xrange(timeout * loop_freq):
                if self.is_at_position(self.pos.pose.position.x,
                                    self.pos.pose.position.y,
                                    self.pos.pose.position.z,0, self.radius):
                    rospy.loginfo("position reached vehicle 0| seconds: {0} of {1}".format(
                        i / loop_freq, timeout))
                    reached = True
                    break
                try:
                    rate.sleep()
                except rospy.ROSException as e:
                    self.fail(e)
        elif uav == 1:
            # set a position setpoint
            self.pos1.pose.position.x = x
            self.pos1.pose.position.y = y
            self.pos1.pose.position.z = z
            rospy.loginfo(
                "attempting to reach position  vehicle 1| x: {0}, y: {1}, z: {2} | current position x: {3:.2f}, y: {4:.2f}, z: {5:.2f}".
                format(x, y, z, self.local_position1.pose.position.x,
                    self.local_position1.pose.position.y,
                    self.local_position1.pose.position.z))
            self.pos1.pose.orientation = Quaternion(*quaternion)
            for i in xrange(timeout * loop_freq):
                if self.is_at_position(self.pos1.pose.position.x,
                                    self.pos1.pose.position.y,
                                    self.pos1.pose.position.z,1,self.radius):
                    rospy.loginfo("position reached  vehicle 1| seconds: {0} of {1}".format(
                        i / loop_freq, timeout))
                    reached = True
                    break
                try:
                    rate.sleep()
                except rospy.ROSException as e:
                    self.fail(e)
        else:
            # set a position setpoint
            rospy.loginfo("Heereeeee")
            self.pos2.pose.position.x = x
            self.pos2.pose.position.y = y
            self.pos2.pose.position.z = z
            rospy.loginfo(
                    "attempting to reach position  vehicle 2| x: {0}, y: {1}, z: {2} | current position x: {3:.2f}, y: {4:.2f}, z: {5:.2f}".
                    format(x, y, z, self.local_position2.pose.position.x,
                        self.local_position2.pose.position.y,
                        self.local_position2.pose.position.z))
            self.pos2.pose.orientation = Quaternion(*quaternion)
            for i in xrange(timeout * loop_freq):
                if self.is_at_position(self.pos2.pose.position.x,self.pos2.pose.position.y,self.pos2.pose.position.z,2, self.radius):
                    rospy.loginfo("position reached  vehicle 2| seconds: {0} of {1}".format(i / loop_freq, timeout))
                    reached = True
                    break
                try:
                    rate.sleep()
                except rospy.ROSException as e:
                    self.fail(e)
        if uav == 0 :
            self.assertTrue(reached, (
                "took too long to get to position vehicle 0| current position x: {0:.2f}, y: {1:.2f}, z: {2:.2f} | timeout(seconds): {3}".
                format(self.local_position.pose.position.x,
                    self.local_position.pose.position.y,
                    self.local_position.pose.position.z, timeout)))
        elif uav == 1:
            self.assertTrue(reached, (
                "took too long to get to position vehicle 1| current position x: {0:.2f}, y: {1:.2f}, z: {2:.2f} | timeout(seconds): {3}".
                format(self.local_position1.pose.position.x,
                    self.local_position1.pose.position.y,
                    self.local_position1.pose.position.z, timeout)))
        else:
            self.assertTrue(reached, (
                "took too long to get to position vehicle 2| current position x: {0:.2f}, y: {1:.2f}, z: {2:.2f} | timeout(seconds): {3}".
                format(self.local_position2.pose.position.x,
                    self.local_position2.pose.position.y,
                    self.local_position2.pose.position.z, timeout)))

    #
    # Test method
    #
    def test_posctl(self):
        """Test offboard position control"""

        # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   10, -1)

        #self.log_topic_vars()
        self.set_mode("OFFBOARD", 3)
        self.set_arm(True, 3)

        rospy.loginfo("run pso")
        #positions = ((0, 0, 0), (2, 2, 2), (2, -2, 2), (-2, -2, 2),(0, 0, 10))
        bounds=[(-10,10),(-10,10)]  # input bounds [(x1_min,x1_max),(x2_min,x2_max)...]
        #PSO(func1,initial,bounds,num_particles=10,maxiter=30)
        costFunc=func1
        num_particles=3
        maxiter=30
        global num_dimensions

        num_dimensions=2
        err_best_g=-1                   # best error for group
        pos_best_g=[]                   # best position for group

            # establish the swarm
        swarm=[]
        x0=[self.local_position.pose.position.x,self.local_position.pose.position.y]
        p = Particle(x0)
        swarm.append(p)
        x0=[self.local_position1.pose.position.x,self.local_position1.pose.position.y]
        p = Particle(x0)
        swarm.append(p)
        x0=[self.local_position2.pose.position.x,self.local_position2.pose.position.y]
        p = Particle(x0)
        swarm.append(p)
        self.reach_position(self.local_position.pose.position.x,self.local_position.pose.position.y,3,0,30)
        self.reach_position(self.local_position1.pose.position.x,self.local_position1.pose.position.y,3.3,1,30)
        self.reach_position(self.local_position2.pose.position.x,self.local_position2.pose.position.y,4,2,30)

            # begin optimization loop
        i=0
        while i < maxiter:
            for j in range(0, num_particles):
                swarm[j].evaluate(costFunc)
                rospy.loginfo(swarm[j].position_i)
                # determine if current particle is the best (globally)
                if swarm[j].err_i < err_best_g or err_best_g == -1:
                    pos_best_g = list(swarm[j].position_i)
                    err_best_g = float(swarm[j].err_i)
                    print("Best-position", pos_best_g)
                    print("Best-Err", err_best_g)

                # cycle through swarm and update velocities and position
            for j in range(0, num_particles):
                swarm[j].update_velocity(pos_best_g)
                swarm[j].update_position(bounds)
                if j == 0:
                    self.reach_position(swarm[j].position_i[0],swarm[j].position_i[1],self.local_position.pose.position.z,0,30)
                elif j == 1:
                    self.reach_position(swarm[j].position_i[0],swarm[j].position_i[1],self.local_position1.pose.position.z,1, 30)
                else:
                    rospy.loginfo("Setting pos 2")
                    self.reach_position(swarm[j].position_i[0],swarm[j].position_i[1],self.local_position2.pose.position.z,2, 30)

                print("UPDATE POS ------ "+str(swarm[j].position_i))
                print("UPDATE VEL ------ "+str(swarm[j].velocity_i))
                x0=[self.local_position2.pose.position.x,self.local_position2.pose.position.y]
                swarm[2].set_position(x0)
                x0=[self.local_position1.pose.position.x,self.local_position1.pose.position.y]
                swarm[1].set_position(x0)
                x0=[self.local_position.pose.position.x,self.local_position.pose.position.y]
                swarm[0].set_position(x0)
            i += 1

            # print final results
        print('FINAL:')
        print(pos_best_g)
        print(err_best_g)


        """
            for i in xrange(len(positions)):
                self.reach_position(positions[i][0], positions[i][1],
                                    positions[i][2], 30)
        """
        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,45, 0)
        self.set_arm(False, 5)

if __name__ == '__main__':
    import rostest
    rospy.init_node('test_nodass', anonymous=True)

    rostest.rosrun(PKG, 'mavros_offboard_posctl_test',
                   MavrosOffboardPosctlTest)

