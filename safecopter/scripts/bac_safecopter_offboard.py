PKG = 'px4'

import unittest
import rospy
import math
import rosbag
import time

from numpy import linalg
import numpy as np

from visualization_msgs.msg import Marker
from std_msgs.msg import Header, Float32, Bool
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from mavros_msgs.srv import CommandLong
from sensor_msgs.msg import NavSatFix
#from px4_test_helper import PX4TestHelper

class MavrosOffboardPosctlTest(unittest.TestCase):
    """
    Tests flying a path in offboard control by sending position setpoints
    via MAVROS.

    For the test to be successful it needs to reach all setpoints in a certain time.
    FIXME: add flight path assertion (needs transformation from ROS frame to NED)
    """

    def setUp(self):
        rospy.init_node('test_node', anonymous=True)
        # self.helper = PX4TestHelper("mavros_offboard_posctl_test")
        # self.helper.setUp()

        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.position_callback)
        rospy.Subscriber("mavros/global_position/global", NavSatFix, self.global_position_callback)
        rospy.Subscriber("/safecopter/collision_free_angle", Float32, self.safecopter_direction_callback)
        rospy.Subscriber("/safecopter/can_find_path", Bool, self.can_find_path_callback)
        rospy.Subscriber("/safecopter/will_collide", Bool, self.will_collide_callback)
        self.pub_location = rospy.Publisher("/safecopter/target_location", Marker, queue_size=100)
        self.pub_spt = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        rospy.wait_for_service('mavros/cmd/command', 30)
        self._srv_cmd_long = rospy.ServiceProxy('mavros/cmd/command', CommandLong, persistent=True)
        self.rate = rospy.Rate(10) # 10hz
        self.has_global_pos = False
        self.local_position = PoseStamped()
        self.armed = False

        self.direction_from_collision = 0
        self.can_find_path = False
        self.will_collide = False

    def tearDown(self):
        #self.helper.tearDown()
        pass

    #
    # General callback functions used in tests
    #
    def position_callback(self, data):
        self.local_position = data

    def global_position_callback(self, data):
        self.has_global_pos = True

    def safecopter_direction_callback(self, msg):
        self.safecopter_direction = msg.data

    def can_find_path_callback(self, msg):
        self.can_find_path = msg.data

    def will_collide_callback(self, msg):
        self.will_collide = msg.data

    #
    # Helper methods
    #
    def pub_target_location(self, x, y):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.id = 15
        marker.type = Marker.SPHERE
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.pose.orientation.w = 1
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0

        self.pub_location.publish(marker)

    def is_at_position(self, x, y, z, offset):
        rospy.logdebug("current position %f, %f, %f" %
                       (self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))

        desired = np.array((x, y, z))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))
        return linalg.norm(desired - pos) < offset

    def is_at_yaw(self, desired_yaw, offset_degrees):
        offset = math.radians(offset_degrees)
        quaternion = (
            self.local_position.pose.orientation.x,
            self.local_position.pose.orientation.y,
            self.local_position.pose.orientation.z,
            self.local_position.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        current_yaw = euler[2]
        return abs(desired_yaw - current_yaw) < offset

    def get_temp_pos(self, distance):
        pos = PoseStamped()
        pos.header = Header()
        pos.header.frame_id = "base_footprint"

        angle_degrees = self.direction_from_collision
        angle_radians = math.radians(angle_degrees)
        quaternion = (
            self.local_position.pose.orientation.x,
            self.local_position.pose.orientation.y,
            self.local_position.pose.orientation.z,
            self.local_position.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        quad_yaw = euler[2]
        angle_final = angle_radians + quad_yaw

        delta_x = distance * math.cos(angle_final)
        delta_y = distance * math.sin(angle_final)

        pos.pose.position.x = self.local_position.pose.position.x + delta_x
        pos.pose.position.y = self.local_position.pose.position.y + delta_y

        pos.pose.position.z = self.local_position.pose.position.z

        pos.pose.orientation = self.local_position.pose.orientation

        print("X: " + str(pos.pose.position.x) + " - Y: " + str(pos.pose.position.y))

        self.pub_target_location(delta_x, delta_y)

        return pos

    def reach_position(self, x, y, z, timeout):
        # set a position setpoint
        pos = PoseStamped()
        pos.header = Header()
        pos.header.frame_id = "base_footprint"
        pos.pose.position.x = x
        pos.pose.position.y = y
        pos.pose.position.z = z
        x_difference = pos.pose.position.x - self.local_position.pose.position.x
        y_difference = pos.pose.position.y - self.local_position.pose.position.y
        # yaw facing towards end target
        yaw = 0
        if y_difference != 0:
            if x_difference < 0:
                yaw = math.atan2(y_difference, x_difference)
            else:
                yaw = math.atan2(y_difference, x_difference)
        quaternion = quaternion_from_euler(0, 0, yaw)
        pos.pose.orientation = Quaternion(*quaternion)

        turned_local_pos = PoseStamped()
        turned_local_pos.header = Header()
        turned_local_pos.header.frame_id = "base_footprint"
        turned_local_pos.pose.position.x = self.local_position.pose.position.x
        turned_local_pos.pose.position.y = self.local_position.pose.position.y
        turned_local_pos.pose.position.z = 10
        turned_local_pos.pose.orientation = Quaternion(*quaternion)

        # does it reach the position in X seconds?
        count = 0
        while count < timeout:
            # update timestamp for each published SP
            pos.header.stamp = rospy.Time.now()
            if self.is_at_yaw(yaw, 10):
                print("Can find path: " + str(self.can_find_path) + " -- Will collide: " + str(self.will_collide))
                if self.local_position.pose.position.z > 2.5:
                    if self.can_find_path:
                        if self.will_collide:
                            self.pub_spt.publish(self.get_temp_pos(1))
                            # self.stop()
                        else:
                            self.pub_spt.publish(pos)
                        # else:
                            # FIXME: safe mode for when no path is found
                            # break
                else:
                    self.pub_spt.publish(pos)
            else:
                self.pub_spt.publish(turned_local_pos)

            #self.helper.bag_write('mavros/setpoint_position/local', pos)

            # FIXME: arm and switch to offboard
            # (need to wait the first few rounds until PX4 has the offboard stream)
            if not self.armed and count > 5:
                self._srv_cmd_long(False, 176, False,
                                   1, 6, 0, 0, 0, 0, 0)
                # make sure the first command doesn't get lost
                time.sleep(1)

                self._srv_cmd_long(False, 400, False,
                                   # arm
                                   1, 0, 0, 0, 0, 0, 0)

                self.armed = True

            if self.is_at_position(pos.pose.position.x, pos.pose.position.y, pos.pose.position.z, 1):
                break
            count += 1
            self.rate.sleep()

        self.assertTrue(count < timeout, "took too long to get to position")

    def test_posctl(self):
        """Test offboard position control"""

        # FIXME: hack to wait for simulation to be ready
        while not self.has_global_pos:
            self.rate.sleep()

        # positions = (
        #     (0, 0, 2),
        #     (2, 2, 2),
        #     (2, -2, 2),
        #     (-2, -2, 2),
        #     (2, 2, 2))

        positions = (
            (0, 0, 10),
            (10, 0, 10),
            (0, 0, 10))
        print(positions)

        for i in range(0, len(positions)):
            self.reach_position(positions[i][0], positions[i][1], positions[i][2], 1000)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'mavros_offboard_posctl_test', MavrosOffboardPosctlTest)
    #unittest.main()
