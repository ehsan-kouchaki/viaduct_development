#!/usr/bin/env python
# ROS1 / rospy ArUco detector for the Intel RealSense T265 Fisheye 1.
#
# Processing pipeline:
#   raw fisheye image -> full-image fisheye rectification
#   -> ArUco detection -> pose estimation in rectified pinhole geometry
#
# Published topics:
#   /aruco/image_rect : rectified image without annotations (mono8)
#   /aruco/image_raw  : rectified image with marker outlines and axes (bgr8)
#   /aruco_pose       : PoseStamped for each detected marker (last marker wins,
#                       matching the behaviour of the previous node)
#   TF               : camera_fisheye1_optical_frame -> aruco_marker_<id>
#
# The node uses the equidistant calibration from the T265 CameraInfo message.
# It does not recalibrate the camera.

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from cv_bridge import CvBridge
import tf2_ros
import tf.transformations as tf_trans


class ArucoDetector:

    def __init__(self):
        self.bridge = CvBridge()

        # Parameters
        self.image_topic = rospy.get_param("~image_topic")
        self.camera_info_topic = rospy.get_param("~camera_info_topic")
        self.camera_frame = rospy.get_param("~camera_frame")
        self.marker_size = float(rospy.get_param("~marker_size"))
        self.marker_prefix = rospy.get_param("~marker_prefix")
        dictionary_name = rospy.get_param("~aruco_dictionary")

        # ROI rectangle parameters (pixels)
        self.roi_center_x = int(rospy.get_param("~roi_center_x", -1))
        self.roi_center_y = int(rospy.get_param("~roi_center_y", -1))

        self.roi_width = int(rospy.get_param("~roi_width", 120))
        self.roi_height = int(rospy.get_param("~roi_height", 120))

        # Rectification parameters
        # This implementation uses K itself as the rectified camera matrix.
        # The rectified field of view is therefore determined by the factory K.

        # Publishers
        self.pose_pub = rospy.Publisher(
            "/aruco_pose", PoseStamped, queue_size=10
        )
        self.rectified_image_pub = rospy.Publisher(
            "/aruco/image_rect", Image, queue_size=1
        )
        self.annotated_image_pub = rospy.Publisher(
            "/aruco/image_raw", Image, queue_size=1
        )

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Calibration and rectification state
        self.camera_matrix = None
        self.dist_coeffs = None
        self.calibration_width = None
        self.calibration_height = None

        self.map1 = None
        self.map2 = None
        self.new_camera_matrix = None
        self.zero_dist_coeffs = np.zeros((4, 1), dtype=np.float64)
        self.rectification_size = None
        self.calibration_error_reported = False

        # ArUco dictionary
        dictionary_map = {
            "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
            "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
            "DICT_6X6_50": cv2.aruco.DICT_6X6_50
        }

        if dictionary_name not in dictionary_map:
            raise rospy.ROSException(
                "Unsupported ArUco dictionary: " + str(dictionary_name)
            )

        # Keep the API used by the existing ROS1 node / installed OpenCV.
        self.aruco_dict = cv2.aruco.Dictionary_get(
            dictionary_map[dictionary_name]
        )
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.parameters.cornerRefinementMethod = (
            cv2.aruco.CORNER_REFINE_SUBPIX
        )

        # Subscribe to calibration before image processing.
        rospy.Subscriber(
            self.camera_info_topic,
            CameraInfo,
            self.camera_info_callback,
            queue_size=1
        )
        rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1
        )

        rospy.loginfo("T265 fisheye ArUco detector started.")
        rospy.loginfo("Raw image topic: %s", self.image_topic)
        rospy.loginfo("CameraInfo topic: %s", self.camera_info_topic)
        rospy.loginfo("Rectified output: /aruco/image_rect")
        rospy.loginfo("Annotated output: /aruco/image_raw")

    # ----------------------------------------------------------------------

    def camera_info_callback(self, msg):
        if self.camera_matrix is not None:
            return

        if msg.distortion_model != "equidistant":
            rospy.logerr(
                "Expected T265 fisheye distortion_model='equidistant', "
                "but CameraInfo reports '%s'. Pose processing is disabled.",
                msg.distortion_model
            )
            self.calibration_error_reported = True
            return

        if len(msg.D) != 4:
            rospy.logerr(
                "Expected 4 equidistant fisheye distortion coefficients; "
                "received %d. Pose processing is disabled.",
                len(msg.D)
            )
            self.calibration_error_reported = True
            return

        self.camera_matrix = np.asarray(
            msg.K, dtype=np.float64
        ).reshape((3, 3))

        self.dist_coeffs = np.asarray(
            msg.D, dtype=np.float64
        ).reshape((4, 1))

        self.calibration_width = int(msg.width)
        self.calibration_height = int(msg.height)

        rospy.loginfo(
            "Received T265 fisheye calibration: %dx%d, model=%s",
            self.calibration_width,
            self.calibration_height,
            msg.distortion_model
        )
        rospy.loginfo("Camera matrix K:\n%s", self.camera_matrix)
        rospy.loginfo("Fisheye distortion D:\n%s", self.dist_coeffs)

    # ----------------------------------------------------------------------

    def build_rectification_maps(self, width, height):
        if self.camera_matrix is None or self.dist_coeffs is None:
            return False

        # Do not silently use calibration for a different image resolution.
        if (self.calibration_width > 0 and self.calibration_height > 0 and
                (width != self.calibration_width or
                 height != self.calibration_height)):
            rospy.logerr(
                "Image size is %dx%d, but CameraInfo calibration is %dx%d. "
                "The node will not apply mismatched calibration.",
                width, height,
                self.calibration_width, self.calibration_height
            )
            self.calibration_error_reported = True
            return False

        image_size = (width, height)
        identity = np.eye(3, dtype=np.float64)

        try:
            # Use the factory intrinsic matrix as the rectified pinhole
            # camera matrix. This avoids relying on
            # estimateNewCameraMatrixForUndistortRectify(), whose Python
            # binding can produce invalid results with some OpenCV builds.
            #
            # K is a valid choice for P: the rectified image uses the same
            # focal lengths and principal point, while the fisheye D
            # coefficients are removed by the remapping operation.
            self.new_camera_matrix = self.camera_matrix.copy()

            # Build maps once; remap() then uses these maps for every frame.
            self.map1, self.map2 = (
                cv2.fisheye.initUndistortRectifyMap(
                    self.camera_matrix,
                    self.dist_coeffs,
                    identity,
                    self.new_camera_matrix,
                    image_size,
                    cv2.CV_16SC2
                )
            )

            self.rectification_size = image_size

        except Exception as exc:
            rospy.logerr(
                "Could not create T265 fisheye rectification maps: %s",
                str(exc)
            )
            self.map1 = None
            self.map2 = None
            self.new_camera_matrix = None
            return False

        rospy.loginfo(
            "Fisheye rectification maps created for %dx%d.",
            width, height
        )
        rospy.loginfo(
            "Rectified camera matrix:\n%s",
            self.new_camera_matrix
        )
        rospy.loginfo(
            "Rectified camera matrix uses the factory K matrix."
        )

        return True

    # ----------------------------------------------------------------------

    def image_callback(self, msg):
        if self.camera_matrix is None:
            if not self.calibration_error_reported:
                rospy.logwarn_throttle(
                    5.0, "Waiting for valid T265 fisheye CameraInfo..."
                )
            return

        try:
            # T265 Fisheye 1 is normally monochrome.
            frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="mono8"
            )
        except Exception as exc:
            rospy.logerr_throttle(
                5.0, "Could not convert camera image: %s", str(exc)
            )
            return

        height, width = frame.shape[:2]
        current_size = (width, height)

        # Build maps on the first image. Rebuild only if image dimensions change.
        if self.rectification_size != current_size:
            if not self.build_rectification_maps(width, height):
                return

        # Full-image fisheye correction.
        rectified = cv2.remap(
            frame,
            self.map1,
            self.map2,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT
        )

        # Publish the clean rectified image for rqt_image_view.
        rectified_msg = self.bridge.cv2_to_imgmsg(
            rectified, encoding="mono8"
        )
        rectified_msg.header = msg.header
        self.rectified_image_pub.publish(rectified_msg)

        # Detect markers in the rectified image.
        annotated = cv2.cvtColor(rectified, cv2.COLOR_GRAY2BGR)

        # --------------------------------------------------
        # Draw a fixed ROI rectangle on the rectified image
        # --------------------------------------------------
        # Default centre: camera optical axis (principal point)
        if self.roi_center_x < 0:
            center_x = int(round(self.new_camera_matrix[0, 2]))
        else:
            center_x = self.roi_center_x

        if self.roi_center_y < 0:
            center_y = int(round(self.new_camera_matrix[1, 2]))
        else:
            center_y = self.roi_center_y

        # Calculate rectangle corners
        x1 = center_x - self.roi_width // 2
        y1 = center_y - self.roi_height // 2

        x2 = center_x + self.roi_width // 2
        y2 = center_y + self.roi_height // 2

        # Draw green rectangle
        cv2.rectangle(
            annotated,
            (x1, y1),
            (x2, y2),
            (0, 255, 0),
            2
        )

        # Draw a small cross at the rectangle centre
        cv2.drawMarker(
            annotated,
            (center_x, center_y),
            (0, 0, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=15,
            thickness=2
        )

        corners, ids, rejected = cv2.aruco.detectMarkers(
            rectified,
            self.aruco_dict,
            parameters=self.parameters
        )

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(annotated, corners, ids)

            # The rectified image is pinhole-projected. Therefore use the
            # rectified camera matrix and zero distortion coefficients.
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners,
                self.marker_size,
                self.new_camera_matrix,
                self.zero_dist_coeffs
            )

            for i in range(len(ids)):
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]

                cv2.drawFrameAxes(
                    annotated,
                    self.new_camera_matrix,
                    self.zero_dist_coeffs,
                    rvec,
                    tvec,
                    self.marker_size / 2.0
                )

                self.publish_pose(
                    ids[i][0],
                    rvec,
                    tvec,
                    msg.header.stamp
                )

        # Preserve the existing annotated-image topic name.
        annotated_msg = self.bridge.cv2_to_imgmsg(
            annotated, encoding="bgr8"
        )
        annotated_msg.header = msg.header
        self.annotated_image_pub.publish(annotated_msg)

    # ----------------------------------------------------------------------

    def publish_pose(self, marker_id, rvec, tvec, stamp):
        pose = PoseStamped()
        pose.header.stamp = stamp

        # Rectification changes the pixel projection, not the physical
        # camera optical coordinate frame. Keep the T265 optical frame.
        pose.header.frame_id = self.camera_frame

        pose.pose.position.x = float(tvec[0])
        pose.pose.position.y = float(tvec[1])
        pose.pose.position.z = float(tvec[2])

        rotation_matrix, _ = cv2.Rodrigues(rvec)

        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = rotation_matrix

        quaternion = tf_trans.quaternion_from_matrix(transform_matrix)

        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        self.pose_pub.publish(pose)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.camera_frame
        tf_msg.child_frame_id = self.marker_prefix + str(marker_id)

        tf_msg.transform.translation.x = float(tvec[0])
        tf_msg.transform.translation.y = float(tvec[1])
        tf_msg.transform.translation.z = float(tvec[2])

        tf_msg.transform.rotation.x = quaternion[0]
        tf_msg.transform.rotation.y = quaternion[1]
        tf_msg.transform.rotation.z = quaternion[2]
        tf_msg.transform.rotation.w = quaternion[3]

        self.tf_broadcaster.sendTransform(tf_msg)


# --------------------------------------------------------------------------

if __name__ == "__main__":
    rospy.init_node("aruco_detector")

    try:
        detector = ArucoDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass