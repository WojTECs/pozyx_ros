#!/usr/bin/env python3
import pypozyx
import rospy
import time
from geometry_msgs.msg import PoseWithCovarianceStamped


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
            "poyzx_pose", PoseWithCovarianceStamped, queue_size=10)

    def get_imu_covariance(self):
        pass

    def start_localization(self):
        pos_msg = PoseWithCovarianceStamped()
        pos_msg.header.frame_id = "pozyx"

        cov_index_pos_x = 0
        cov_index_pos_y = 7
        cov_index_pos_z = 14

        cov_index_ang_x = 21
        cov_index_ang_y = 28
        cov_index_ang_z = 35

        pos_msg.pose.covariance[cov_index_pos_z] = -1
        pos_msg.pose.pose.position.z = 0.0

        pos_msg.pose.covariance[cov_index_ang_x] = 0.0001
        pos_msg.pose.covariance[cov_index_ang_y] = 0.0001
        pos_msg.pose.covariance[cov_index_ang_z] = 0.0001


        pos_error = pypozyx.PositionError()
        pos = pypozyx.Coordinates()
        quat = pypozyx.Quaternion()
        # print(
        while not rospy.is_shutdown():
            # print(pypozyx.PozyxRegisters.POSITIONING_ERROR_X)
            # pypozyx.PositioningData.

            self.pozyx.getPositionError(pos_error)
            self.pozyx.doPositioning(pos)
            self.pozyx.getQuaternion(quat)

            pos_msg.pose.pose.position.x = (pos.x/1000.0)  # [m]
            pos_msg.pose.pose.position.y = (pos.y/1000.0)  # [m]

            pos_msg.pose.covariance[cov_index_pos_x] = (pos_error.x/1000.0)  # [m]
            pos_msg.pose.covariance[cov_index_pos_y] = (pos_error.y/1000.0)  # [m]

            pos_msg.pose.pose.orientation.x = quat.x
            pos_msg.pose.pose.orientation.y = quat.y
            pos_msg.pose.pose.orientation.z = quat.z
            pos_msg.pose.pose.orientation.w = quat.w
            pos_msg.header.stamp = rospy.Time.now()

            self.pose_pub.publish(pos_msg)

            time.sleep(0.02)

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
