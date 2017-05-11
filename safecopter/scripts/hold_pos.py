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
        # return linalg.norm(desired - pos) < offset
        return False

    def reach_position(self, x, y, z, timeout):
        # set a position setpoint
        pos = PoseStamped()
        pos.header = Header()
        pos.header.frame_id = "base_footprint"
        pos.pose.position.x = x
        pos.pose.position.y = y
        pos.pose.position.z = z
        # yaw facing towards end target
        yaw = math.pi
        quaternion = quaternion_from_euler(0, 0, yaw)
        pos.pose.orientation = Quaternion(*quaternion)

        count = 0
        while True:
            # update timestamp for each published SP
            pos.header.stamp = rospy.Time.now()
            self.pub_spt.publish(pos)

            if not self.armed and count > 5:
                self._srv_cmd_long(False, 176, False,
                                   1, 6, 0, 0, 0, 0, 0)
                # make sure the first command doesn't get lost
                time.sleep(1)

                self._srv_cmd_long(False, 400, False,
                                   # arm
                                   1, 0, 0, 0, 0, 0, 0)

                self.armed = True

            count += 1
            self.rate.sleep()

    def test_posctl(self):
        while not self.has_global_pos:
            self.rate.sleep()

        positions = (9, 0, self.height)
        print(positions)

        for i in range(0, len(positions)):
            self.reach_position(positions[0], positions[1], positions[2], 1000)


if __name__ == '__main__':
    try:
        obj = MavrosOffboardPosctlTest()
        obj.test_posctl()
    except rospy.ROSException:
        pass
