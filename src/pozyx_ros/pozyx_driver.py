#!/usr/bin/env python3
import pypozyx
import rospy
import time
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu


class PozyxDriver:
    def __init__(self):

        self.serial_port = pypozyx.get_first_pozyx_serial_port()

        if self.serial_port is not None:
            self.pozyx = pypozyx.PozyxSerial(self.serial_port)
            self.uwb_settings = pypozyx.UWBSettings()
            self.pozyx.getUWBSettings(self.uwb_settings)
            rospy.loginfo("[POZYX]: " + str(self.uwb_settings))
        else:
            rospy.loginfo("[POZYX]: No Pozyx port was found")
            exit(0)
        self.pose_pub = rospy.Publisher(
            "pozyx_pose", PoseWithCovarianceStamped, queue_size=10)
        self.imu_pub = rospy.Publisher("pozyx_imu", Imu, queue_size=1)

        self.acceleration = pypozyx.Acceleration()
        self.angular_velocity = pypozyx.AngularVelocity()
        self.quaternion = pypozyx.Quaternion()
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = "pozyx"
        self.pos_msg = PoseWithCovarianceStamped()
        self.pos_msg.header.frame_id = "pozyx"

        self.__cov_index_pos_x = 0
        self.__cov_index_pos_y = 7
        self.__cov_index_pos_z = 14
        self.__cov_index_ang_x = 21
        self.__cov_index_ang_y = 28
        self.__cov_index_ang_z = 35

        self.pos_msg.pose.covariance[self.__cov_index_pos_z] = -1
        self.pos_msg.pose.pose.position.z = 0.0

        self.pos_msg.pose.covariance[self.__cov_index_ang_x] = 0.001
        self.pos_msg.pose.covariance[self.__cov_index_ang_y] = 0.001
        self.pos_msg.pose.covariance[self.__cov_index_ang_z] = 0.001

        self.imu_msg.orientation_covariance = [0.001, 0, 0,
                                               0, 0.001, 0,
                                               0, 0, 0.001]

        self.imu_msg.angular_velocity_covariance = [0.001, 0, 0,
                                                    0, 0.001, 0,
                                                    0, 0, 0.001]

        self.imu_msg.linear_acceleration_covariance = [0.001, 0, 0,
                                                       0, 0.001, 0,
                                                       0, 0, 0.001]

    def get_imu_covariance(self):
        pass

    def get_imu_data(self):
        self.pozyx.getAngularVelocity_dps(self.angular_velocity)
        self.pozyx.getAcceleration_mg(self.acceleration)
        self.pozyx.getQuaternion(self.quaternion)

        self.imu_msg.orientation.x = self.quaternion.x
        self.imu_msg.orientation.y = self.quaternion.y
        self.imu_msg.orientation.z = self.quaternion.z
        self.imu_msg.orientation.w = self.quaternion.w

        self.imu_msg.angular_velocity.x = self.angular_velocity.x * 0.0174532925
        self.imu_msg.angular_velocity.y = self.angular_velocity.y * 0.0174532925
        self.imu_msg.angular_velocity.z = self.angular_velocity.z * 0.0174532925

        self.imu_msg.linear_acceleration.x = self.acceleration.x * 0.00980665
        self.imu_msg.linear_acceleration.y = self.acceleration.y * 0.00980665
        self.imu_msg.linear_acceleration.z = self.acceleration.z * 0.00980665

        pass

    def start_localization(self):

        pos_error = pypozyx.PositionError()
        pos = pypozyx.Coordinates()
        rate_hz = rospy.get_param("~rate", 50)
        self.rate = rospy.Rate(rate_hz)
        # print( "{0}\t {1}\t {2}\t".format(acc.x * 0.00980665 ,acc.y * 0.00980665, acc.z * 0.00980665))
        # print( "{0}\t {1}\t {2}\t".format(ang.x*0.0174532925,ang.y*0.0174532925,ang.z*0.0174532925))

        while not rospy.is_shutdown():
            self.pozyx.doPositioning(pos)

            self.pos_msg.pose.pose.position.x = (pos.x/1000.0)  # [m]
            self.pos_msg.pose.pose.position.y = (pos.y/1000.0)  # [m]

            if self.pozyx.getPositionError(pos_error) == pypozyx.POZYX_SUCCESS:
                self.pos_msg.pose.covariance[self.__cov_index_pos_x] = (
                    pos_error.x/1000.0)  # [m]
                self.pos_msg.pose.covariance[self.__cov_index_pos_y] = (
                    pos_error.y/1000.0)  # [m]
            else:
                self.pos_msg.pose.covariance[self.__cov_index_pos_x] = 1  # [m]
                self.pos_msg.pose.covariance[self.__cov_index_pos_y] = 1  # [m]

            # self.pozyx.getQuaternion(self.quaternion)
            self.get_imu_data()

            self.pos_msg.pose.pose.orientation.x = self.quaternion.x
            self.pos_msg.pose.pose.orientation.y = self.quaternion.y
            self.pos_msg.pose.pose.orientation.z = self.quaternion.z
            self.pos_msg.pose.pose.orientation.w = self.quaternion.w
            self.pos_msg.header.stamp = rospy.Time.now()
            self.imu_msg.header.stamp = rospy.Time.now()

            self.pose_pub.publish(self.pos_msg)
            self.imu_pub.publish(self.imu_msg)

            # self.rate.sleep()

    def set_algorithm_configuration(self):
        alg = pypozyx.PozyxConstants.POSITIONING_ALGORITHM_TRACKING
        dim = pypozyx.PozyxConstants.DIMENSION_2D
        self.pozyx.setPositionAlgorithm(alg, dim)
        self.pozyx.savePositioningSettings()

    def set_anchor_configuration(self):
        tag_ids = [None]
        rospy.loginfo("Configuring anchors devices")
        anchors = []
        if not rospy.has_param("~anchors"):
            rospy.logwarn("Can't find anchors parameters.")
            rospy.logerr("Configuration aborted!")

            return
        else:
            for key, value in rospy.get_param('~anchors').items():
                # print(key, value["pos"])
                anchors.append(pypozyx.DeviceCoordinates(int(key, 16), 1, pypozyx.Coordinates(value["pos"][0],
                                                                                              value["pos"][1],
                                                                                              value["pos"][2])))

        settings_registers = [0x16, 0x17]  # POS ALG and NUM ANCHORS

        self.pozyx.doDiscovery(
            discovery_type=pypozyx.POZYX_DISCOVERY_TAGS_ONLY)
        device_list_size = pypozyx.SingleRegister()

        self.pozyx.getDeviceListSize(device_list_size)

        if device_list_size[0] > 0:
            device_list = pypozyx.DeviceList(list_size=device_list_size[0])
            self.pozyx.getDeviceIds(device_list)
            tag_ids += device_list.data

        for tag in tag_ids:
            for anchor in anchors:
                self.pozyx.addDevice(anchor, tag)
            if len(anchors) > 4:
                self.pozyx.setSelectionOfAnchors(pypozyx.POZYX_ANCHOR_SEL_AUTO,
                                                 len(anchors), remote_id=tag)
                self.pozyx.saveRegisters(settings_registers, remote_id=tag)
            self.pozyx.saveNetwork(remote_id=tag)

            if tag is None:
                rospy.loginfo("Local device configured")
            else:
                rospy.loginfo("Device with ID 0x%0.4x configured." % tag)
        rospy.loginfo("Configuration completed!")
