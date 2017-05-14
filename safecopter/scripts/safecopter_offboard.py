PKG = 'px4'

import rospy
import math
import rosbag
import time
import tf
import tf2_ros
import tf2_geometry_msgs

from numpy import linalg
import numpy as np

from visualization_msgs.msg import Marker
from std_msgs.msg import Header, Float32, Bool
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from mavros_msgs.srv import CommandLong
from sensor_msgs.msg import NavSatFix

class MavrosOffboardPosctlTest():
    """
    Tests flying a path in offboard control by sending position setpoints
    via MAVROS.

    For the test to be successful it needs to reach all setpoints in a certain time.
    FIXME: add flight path assertion (needs transformation from ROS frame to NED)
    """

    def __init__(self):
        rospy.init_node('safecopter_offboard')

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
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.height = 2
        self.target_pos = PoseStamped()

    #
    # General callback functions used in tests
    #
    def position_callback(self, data):
        self.local_position = data
        self.pub_position_tf()

    def global_position_callback(self, data):
        self.has_global_pos = True

    def safecopter_direction_callback(self, msg):
        self.direction_from_collision = msg.data

    def can_find_path_callback(self, msg):
        self.can_find_path = msg.data

    def will_collide_callback(self, msg):
        self.will_collide = msg.data

    #
    # Helper methods
    #
    def get_yaw(self):
        x_difference = self.target_pos.pose.position.x - self.local_position.pose.position.x
        y_difference = self.target_pos.pose.position.y - self.local_position.pose.position.y
        yaw = math.atan2(y_difference, x_difference)
        return float(yaw)

    def print_pos(self, s, pos):
        target_quaternion = (
            pos.pose.orientation.x,
            pos.pose.orientation.y,
            pos.pose.orientation.z,
            pos.pose.orientation.w)
        target_euler = euler_from_quaternion(target_quaternion)
        target_yaw = math.degrees(target_euler[2])

        current_quaternion = (
            self.local_position.pose.orientation.x,
            self.local_position.pose.orientation.y,
            self.local_position.pose.orientation.z,
            self.local_position.pose.orientation.w)
        current_euler = euler_from_quaternion(current_quaternion)
        current_yaw = math.degrees(current_euler[2])

        s = s + "(" + str(pos.pose.position.x)
        s = s + ", " + str(pos.pose.position.y)
        s = s + ", " + str(pos.pose.position.z) + ")"
        s = s + " Target Yaw: " + str(target_yaw)
        s = s + " Current Yaw: " + str(current_yaw)
        s = s + " Function Yaw: " + str(math.degrees(self.get_yaw()))
        print(s)

    def pub_pos(self, pos):
        self.pub_spt.publish(pos)

    def pub_position_tf(self):
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.local_position.pose.position.x
        t.transform.translation.y = self.local_position.pose.position.y
        t.transform.translation.z = self.local_position.pose.position.z
        t.transform.rotation.w = self.local_position.pose.orientation.w
        t.transform.rotation.x = self.local_position.pose.orientation.x
        t.transform.rotation.y = self.local_position.pose.orientation.y
        t.transform.rotation.z = self.local_position.pose.orientation.z

        br.sendTransform(t)

    def pub_target_marker(self, x, y):
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
        marker.lifetime = rospy.Duration(1)
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
        difference = abs(desired_yaw - current_yaw)
        if difference > math.pi:
            difference = (2 * math.pi) - difference
        return abs(difference) < offset

    def create_pose(self, x, y, z, roll, pitch, yaw, frame):
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        pose.header.frame_id = frame
        pose.header.stamp = rospy.Time.now()
        return pose

    def get_turned_local_pos(self, target_x, target_y):
        x = self.local_position.pose.position.x
        y = self.local_position.pose.position.y
        z = self.height
        x_difference = target_x - x
        y_difference = target_y - y
        # yaw facing towards end target
        yaw = 0
        if y_difference != 0:
            if x_difference < 0:
                yaw = math.atan2(y_difference, x_difference)
            else:
                yaw = math.atan2(y_difference, x_difference)
        pos = self.create_pose(x, y, z, 0, 0, self.get_yaw(), "base_link")
        return pos

    def get_temp_pos(self, target_x, target_y, distance):
        angle_degrees = self.direction_from_collision
        angle_radians = math.radians(angle_degrees)
        x = distance * math.cos(angle_radians)
        y = distance * math.sin(angle_radians)
        z = 0

        print("X: " + str(x) + " - Y: " + str(y))

        x_difference = target_x - self.local_position.pose.position.x
        y_difference = target_y - self.local_position.pose.position.y
        yaw = 0
        # if y_difference != 0:
        #     if x_difference < 0:
        #         yaw = math.atan2(y_difference, x_difference)
        #     else:
        #         yaw = math.atan2(y_difference, x_difference)
        yaw = math.atan2(y_difference, x_difference)

        base_pos = self.create_pose(x, y, z, 0, 0, self.get_yaw(), "base_link")

        pos = base_pos
        try:
            trans = self.tfBuffer.lookup_transform("odom", "base_link", rospy.Time(0))
            pos = tf2_geometry_msgs.do_transform_pose(pos, trans)
            self.pub_target_marker(x, y)
        except tf2_ros.LookupException as e:
            rospy.loginfo("Handling Lookup error: %s", e)
        except tf2_ros.ConnectivityException as e:
            rospy.loginfo("Handling run-time error: %s", e)
        except tf2_ros.ExtrapolationException as e:
            rospy.loginfo("Handling run-time error: %s", e)
        pos.pose.position.z = self.height

        x = pos.pose.position.x
        y = pos.pose.position.y
        z = self.height
        x_difference = x - self.local_position.pose.position.x
        y_difference = y - self.local_position.pose.position.y
        yaw = math.atan2(y_difference, x_difference)
        pos = self.create_pose(x, y, z, 0, 0, self.get_yaw(), "base_link")

        return pos

    def reach_position(self, x, y, z, timeout):
        # set a position setpoint
        x_difference = x - self.local_position.pose.position.x
        y_difference = y - self.local_position.pose.position.y
        # yaw facing towards end target
        yaw = 0
        if y_difference != 0:
            if x_difference < 0:
                yaw = math.atan2(y_difference, x_difference)
            else:
                yaw = math.atan2(y_difference, x_difference)
        pos = self.create_pose(x, y, z, 0, 0, yaw, "base_link")
        self.target_pos = pos

        # does it reach the position in X seconds?
        count = 0
        while count < timeout:
            # update timestamp for each published SP
            pos.header.stamp = rospy.Time.now()
            if self.is_at_yaw(self.get_yaw(), 15):
                # print("Can find path: " + str(self.can_find_path) + " -- Will collide: " + str(self.will_collide))
                if self.can_find_path:
                    if self.will_collide:
                        temp_pos = self.get_temp_pos(x, y, 1)
                        self.pub_pos(temp_pos)
                        self.print_pos("Printing temp_pos: ", temp_pos)
                        # self.stop()
                    else:
                        pos = self.create_pose(x, y, z, 0, 0, self.get_yaw(), "base_link")
                        self.pub_pos(pos)
                        self.print_pos("Printing pos: ", pos)
                # FIXME: safe mode for when no path is found
            else:
                turned_local_pos = self.get_turned_local_pos(x, y)
                self.pub_pos(turned_local_pos)
                self.print_pos("Printing turned_local_pos: ", turned_local_pos)

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

    def test_posctl(self):
        while not self.has_global_pos:
            self.rate.sleep()

        positions = (
            (9, -1, self.height),
            (-3, 0, self.height),
            (9, -1, self.height))
        print(positions)

        for i in range(0, len(positions)):
            self.reach_position(positions[i][0], positions[i][1], positions[i][2], 1000)


if __name__ == '__main__':
    try:
        obj = MavrosOffboardPosctlTest()
        obj.test_posctl()
    except rospy.ROSException:
        pass
