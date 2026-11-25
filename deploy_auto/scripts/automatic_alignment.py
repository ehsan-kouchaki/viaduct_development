#!/usr/bin/env python
# ROS1 Melodic / Python 2.7 compatible.
# Four-stage continuous-feedback ArUco alignment for the ceiling crawler.
# Uses the existing /end_effector/command interface; it does not open serial.

import math
import time

import rospy
import tf.transformations as tft

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String


def finite(value):
    return not (math.isnan(value) or math.isinf(value))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class AutomaticAlignment(object):
    def __init__(self):
        self.pose_topic = rospy.get_param('/deploy_auto/aruco_pose_topic', '/aruco_pose')
        self.yaw_source = str(rospy.get_param('/deploy_auto/yaw_source', 'aruco')).strip().lower()
        self.vehicle_pose_topic = rospy.get_param(
            '/deploy_auto/vehicle_pose_topic', '/mavros/local_position/pose')
        self.command_topic = rospy.get_param(
            '/deploy_auto/motor_command_topic', '/end_effector/command')
        self.emergency_topic = rospy.get_param(
            '/deploy_auto/emergency_stop_topic', '/deploy_auto/emergency_stop')
        self.start_topic = rospy.get_param(
            '/deploy_auto/start_topic', '/deploy_auto/start')
        self.expected_frame_id = rospy.get_param('/deploy_auto/expected_frame_id', '')

        self.enable_motors = bool(rospy.get_param('/deploy_auto/enable_motors', False))
        self.pose_timeout_s = float(rospy.get_param('/deploy_auto/pose_timeout_s', 0.60))
        self.vehicle_pose_timeout_s = float(
            rospy.get_param('/deploy_auto/vehicle_pose_timeout_s', 0.60))
        self.controller_wait_timeout_s = float(
            rospy.get_param('/deploy_auto/controller_wait_timeout_s', 60.0))
        self.max_alignment_time_s = float(
            rospy.get_param('/deploy_auto/max_alignment_time_s', 180.0))

        # Marker geometry. At camera_forward_axis_deg=0: forward=y, lateral=x.
        self.camera_forward_axis_deg = float(
            rospy.get_param('/deploy_auto/camera_forward_axis_deg', 0.0))
        self.max_bearing_deg = float(
            rospy.get_param('/deploy_auto/max_bearing_deg', 85.0))
        self.bearing_tolerance_deg = float(
            rospy.get_param('/deploy_auto/bearing_tolerance_deg', 3.0))
        self.position_tolerance_m = float(
            rospy.get_param('/deploy_auto/position_tolerance_m', 0.01))
        self.max_forward_bearing_deg = float(
            rospy.get_param('/deploy_auto/max_forward_bearing_deg', 10.0))

        # Independent safety limits for the vehicle yaw relative to its start.
        self.max_vehicle_yaw_deviation_deg = float(
            rospy.get_param('/deploy_auto/max_vehicle_yaw_deviation_deg', 85.0))
        self.yaw_tolerance_deg = float(
            rospy.get_param('/deploy_auto/yaw_tolerance_deg', 2.0))

        self.backward_stage_timeout_s = float(
            rospy.get_param('/deploy_auto/backward_stage_timeout_s', 20.0))
        self.rotation_stage_timeout_s = float(
            rospy.get_param('/deploy_auto/rotation_stage_timeout_s', 30.0))
        self.forward_stage_timeout_s = float(
            rospy.get_param('/deploy_auto/forward_stage_timeout_s', 60.0))
        self.yaw_restore_stage_timeout_s = float(
            rospy.get_param('/deploy_auto/yaw_restore_stage_timeout_s', 30.0))

        # Re-send the active command as a heartbeat. Arduino firmware must have
        # the matching track-command watchdog enabled (see supplied .ino).
        self.command_heartbeat_s = float(
            rospy.get_param('/deploy_auto/command_heartbeat_s', 0.15))
        self.stop_repeats = int(rospy.get_param('/deploy_auto/stop_repeats', 3))
        self.stop_repeat_interval_s = float(
            rospy.get_param('/deploy_auto/stop_repeat_interval_s', 0.03))

        self.forward_command = rospy.get_param('/deploy_auto/forward_command', 'a')
        self.reverse_command = rospy.get_param('/deploy_auto/reverse_command', 'b')
        self.clockwise_command = rospy.get_param('/deploy_auto/clockwise_command', 'c')
        self.counterclockwise_command = rospy.get_param(
            '/deploy_auto/counterclockwise_command', 'd')
        self.stop_command = rospy.get_param('/deploy_auto/stop_tracks_command', 'e')

        self.latest_pose = None
        self.pose_received_at = 0.0
        self.pose_sequence = 0
        self.vehicle_yaw = None
        self.vehicle_pose_received_at = 0.0
        self.abort_requested = False
        self.start_requested = False
        self.finished = False
        self.alignment_start_time = None

        self.pose_sub = rospy.Subscriber(
            self.pose_topic, PoseStamped, self.pose_callback, queue_size=1)
        self.vehicle_pose_sub = None
        if self.yaw_source == 'mavros':
            self.vehicle_pose_sub = rospy.Subscriber(
                self.vehicle_pose_topic, PoseStamped, self.vehicle_pose_callback, queue_size=1)
        self.command_pub = rospy.Publisher(self.command_topic, String, queue_size=10)
        self.emergency_sub = rospy.Subscriber(
            self.emergency_topic, Bool, self.emergency_callback, queue_size=1)
        self.start_sub = rospy.Subscriber(
            self.start_topic, Bool, self.start_callback, queue_size=1)

        self.validate_parameters()
        rospy.on_shutdown(self.stop_tracks)

        rospy.loginfo('deploy_auto four-stage continuous-feedback controller started.')
        rospy.loginfo('ArUco pose topic: %s', self.pose_topic)
        rospy.loginfo('Yaw feedback source: %s', self.yaw_source)
        if self.yaw_source == 'mavros':
            rospy.loginfo('MAVROS vehicle pose topic: %s', self.vehicle_pose_topic)
        else:
            rospy.loginfo('Office mode: using ArUco-relative yaw; MAVROS is not required.')
        rospy.loginfo('Motor enable: %s', str(self.enable_motors))
        rospy.loginfo('Command mapping: a=forward, b=backward, c=CW, d=CCW, e=stop')
        rospy.loginfo('Press f in the end-effector keyboard terminal to start alignment.')

    def validate_parameters(self):
        if self.yaw_source not in ('aruco', 'mavros'):
            raise rospy.ROSException("yaw_source must be either 'aruco' or 'mavros'")
        positive = [
            self.pose_timeout_s, self.vehicle_pose_timeout_s,
            self.controller_wait_timeout_s, self.max_alignment_time_s,
            self.max_bearing_deg, self.bearing_tolerance_deg,
            self.position_tolerance_m, self.max_forward_bearing_deg,
            self.max_vehicle_yaw_deviation_deg, self.yaw_tolerance_deg,
            self.backward_stage_timeout_s, self.rotation_stage_timeout_s,
            self.forward_stage_timeout_s, self.yaw_restore_stage_timeout_s,
            self.command_heartbeat_s, self.stop_repeat_interval_s]
        if any(value <= 0.0 for value in positive):
            raise rospy.ROSException('All timeout, tolerance, and heartbeat parameters must be positive')
        if self.max_bearing_deg >= 90.0:
            raise rospy.ROSException('max_bearing_deg must be less than 90 degrees')
        if self.bearing_tolerance_deg >= self.max_bearing_deg:
            raise rospy.ROSException('bearing_tolerance_deg must be smaller than max_bearing_deg')
        if self.stop_repeats < 1:
            raise rospy.ROSException('stop_repeats must be at least 1')
        commands = [self.forward_command, self.reverse_command,
                    self.clockwise_command, self.counterclockwise_command,
                    self.stop_command]
        if any((not isinstance(command, str) or len(command) != 1)
               for command in commands):
            raise rospy.ROSException('Every Arduino command must be one character')
        if len(set(commands)) != len(commands):
            raise rospy.ROSException('Forward/reverse/turn/stop commands must be distinct')

    def pose_callback(self, msg):
        self.latest_pose = msg
        self.pose_received_at = time.time()
        self.pose_sequence += 1

    def vehicle_pose_callback(self, msg):
        q = msg.pose.orientation
        values = [q.x, q.y, q.z, q.w]
        if not all(finite(v) for v in values):
            return
        q_norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if q_norm < 1.0e-6:
            return
        quat = [q.x/q_norm, q.y/q_norm, q.z/q_norm, q.w/q_norm]
        unused_roll, unused_pitch, yaw = tft.euler_from_quaternion(quat)
        self.vehicle_yaw = float(yaw)
        self.vehicle_pose_received_at = time.time()

    def start_callback(self, msg):
        if msg.data and not self.finished and not self.abort_requested:
            self.start_requested = True
            rospy.loginfo('Alignment start request received (keyboard key f).')

    def emergency_callback(self, msg):
        if msg.data:
            rospy.logerr('Emergency-stop topic activated.')
            self.abort_requested = True
            self.stop_tracks()

    def get_vehicle_yaw(self):
        if self.vehicle_yaw is None:
            return None
        if time.time() - self.vehicle_pose_received_at > self.vehicle_pose_timeout_s:
            return None
        return self.vehicle_yaw

    def get_alignment_yaw(self):
        # ArUco mode measures marker orientation relative to the camera. For a
        # stationary marker, this changes with the opposite sign to crawler yaw.
        if self.yaw_source == 'aruco':
            pose = self.get_pose()
            return None if pose is None else pose['aruco_yaw']
        return self.get_vehicle_yaw()

    def get_alignment_yaw_description(self):
        if self.yaw_source == 'aruco':
            return 'ArUco-relative yaw'
        return 'MAVROS vehicle yaw'

    def get_pose(self):
        if self.latest_pose is None:
            return None
        age = time.time() - self.pose_received_at
        if age > self.pose_timeout_s:
            return None
        if self.expected_frame_id and self.latest_pose.header.frame_id != self.expected_frame_id:
            return None

        p = self.latest_pose.pose.position
        q = self.latest_pose.pose.orientation
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if not all(finite(v) for v in values) or p.z <= 0.0:
            return None
        q_norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if q_norm < 1.0e-6:
            return None
        quat = [q.x/q_norm, q.y/q_norm, q.z/q_norm, q.w/q_norm]
        unused_roll, unused_pitch, yaw = tft.euler_from_quaternion(quat)
        return {'x': float(p.x), 'y': float(p.y), 'z': float(p.z),
                'aruco_yaw': float(yaw), 'age': float(age)}

    def camera_to_crawler(self, x, y):
        alpha = math.radians(self.camera_forward_axis_deg)
        forward = x * math.sin(alpha) + y * math.cos(alpha)
        lateral = x * math.cos(alpha) - y * math.sin(alpha)
        return forward, lateral

    def marker_geometry(self, pose):
        forward, lateral = self.camera_to_crawler(pose['x'], pose['y'])
        theta = math.atan2(lateral, forward)
        distance = math.sqrt(forward * forward + lateral * lateral)
        return forward, lateral, theta, distance

    def controller_connected(self):
        return self.command_pub.get_num_connections() > 0

    def publish_command(self, command):
        if not self.controller_connected():
            rospy.logerr('Arduino command subscriber disconnected; cannot safely command tracks.')
            return False
        self.command_pub.publish(String(data=command))
        return True

    def stop_tracks(self):
        # Repeated stop messages improve delivery, but the firmware watchdog is
        # the protection if ROS/serial fails before these messages arrive.
        for unused in range(self.stop_repeats):
            try:
                if self.command_pub.get_num_connections() > 0:
                    self.command_pub.publish(String(data=self.stop_command))
                    time.sleep(self.stop_repeat_interval_s)
            except Exception as exc:
                rospy.logerr('Could not publish track stop command: %s', str(exc))

    def wait_for_ready(self):
        start = time.time()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.abort_requested:
            if self.controller_connected() and self.get_pose() is not None:
                if self.get_alignment_yaw() is not None:
                    rospy.loginfo('Controller, fresh ArUco pose, and %s are available.',
                                  self.get_alignment_yaw_description())
                    return True
            if time.time() - start >= self.controller_wait_timeout_s:
                rospy.logerr('Startup timeout: controller, fresh ArUco pose, or selected yaw feedback unavailable (%s).',
                             self.yaw_source)
                return False
            rate.sleep()
        return False

    def check_global_timeout(self):
        if self.alignment_start_time is None:
            return False
        if time.time() - self.alignment_start_time > self.max_alignment_time_s:
            rospy.logerr('Maximum total alignment time reached.')
            return True
        return False

    def check_yaw_envelope(self, initial_yaw):
        current_yaw = self.get_alignment_yaw()
        if current_yaw is None:
            return False, '%s is stale or unavailable' % self.get_alignment_yaw_description()
        delta_deg = math.degrees(wrap_angle(current_yaw - initial_yaw))
        if abs(delta_deg) > self.max_vehicle_yaw_deviation_deg:
            return False, ('%s changed by %.2f deg from its initial value; limit is %.2f deg' %
                           (self.get_alignment_yaw_description(), delta_deg,
                            self.max_vehicle_yaw_deviation_deg))
        return True, delta_deg

    def run_feedback_stage(self, name, timeout_s, evaluator,
                           initial_yaw, require_pose=True, enforce_yaw_limit=True):
        rospy.loginfo('Starting stage: %s', name)
        stage_start = time.time()
        last_heartbeat = 0.0
        active_command = None
        rate = rospy.Rate(20)

        while not rospy.is_shutdown() and not self.abort_requested:
            if self.check_global_timeout():
                self.stop_tracks()
                return False
            if time.time() - stage_start > timeout_s:
                rospy.logerr('Stage "%s" exceeded its %.1f s timeout.', name, timeout_s)
                self.stop_tracks()
                return False
            if not self.controller_connected():
                rospy.logerr('Arduino command subscriber lost during stage "%s".', name)
                self.stop_tracks()
                return False

            pose = self.get_pose() if require_pose else None
            if require_pose and pose is None:
                rospy.logerr('ArUco pose lost/stale during stage "%s". Stopping immediately.', name)
                self.stop_tracks()
                return False

            if enforce_yaw_limit:
                yaw_ok, yaw_result = self.check_yaw_envelope(initial_yaw)
                if not yaw_ok:
                    rospy.logerr('Yaw safety limit in stage "%s": %s', name, yaw_result)
                    self.stop_tracks()
                    return False

            done, desired_command, detail, error = evaluator(pose)
            if error:
                rospy.logerr('Stage "%s" aborted: %s', name, error)
                self.stop_tracks()
                return False
            if done:
                self.stop_tracks()
                rospy.loginfo('Stage completed: %s (%s)', name, detail)
                return True

            now = time.time()
            if desired_command != active_command:
                if not self.publish_command(desired_command):
                    self.stop_tracks()
                    return False
                active_command = desired_command
                last_heartbeat = now
                rospy.loginfo('Stage %s: command=%s; %s', name, active_command, detail)
            elif now - last_heartbeat >= self.command_heartbeat_s:
                # Refresh the Arduino watchdog without interrupting motion.
                if not self.publish_command(active_command):
                    self.stop_tracks()
                    return False
                last_heartbeat = now
            rate.sleep()

        self.stop_tracks()
        return False

    def run_alignment(self):
        if not self.enable_motors:
            rospy.logerr('Motor interlock is OFF. Relaunch with enable_motors:=true after safety checks.')
            return False
        if not self.wait_for_ready():
            return False

        initial_pose = self.get_pose()
        initial_yaw = self.get_alignment_yaw()
        if initial_pose is None or initial_yaw is None:
            rospy.logerr('Initial ArUco pose or selected yaw feedback became stale. No movement.')
            return False

        self.alignment_start_time = time.time()
        fwd, lat, theta, distance = self.marker_geometry(initial_pose)
        rospy.loginfo('Initial marker: x=%+.4f, y=%+.4f, forward=%+.4f, lateral=%+.4f, theta=%+.2f deg, r=%.4f m',
                      initial_pose['x'], initial_pose['y'], fwd, lat,
                      math.degrees(theta), distance)
        rospy.loginfo('Initial %s recorded: %.2f deg', self.get_alignment_yaw_description(),
                      math.degrees(initial_yaw))

        # STAGE 1: back away until the marker is in front and within the
        # permitted approach cone. This corresponds to Q1 or Q2 and |theta|<=50.
        def backward_condition(pose):
            forward, lateral, angle, radius = self.marker_geometry(pose)
            angle_deg = math.degrees(angle)
            if forward > 0.0 and abs(angle_deg) <= self.max_bearing_deg:
                return True, None, 'marker is in front and within %.1f deg' % self.max_bearing_deg, None
            return False, self.reverse_command, 'forward=%.3f m, theta=%+.2f deg; backing up' % (
                forward, angle_deg), None

        if not self.run_feedback_stage('BACKWARD APPROACH', self.backward_stage_timeout_s,
                                       backward_condition, initial_yaw):
            return False

        # STAGE 2: positive theta (Q1) means marker is to the right -> CW (c).
        # Negative theta (Q2) means marker is to the left -> CCW (d).
        def rotation_condition(pose):
            forward, lateral, angle, radius = self.marker_geometry(pose)
            angle_deg = math.degrees(angle)
            if forward <= 0.0:
                return False, None, '', 'marker moved behind the crawler during rotation'
            if abs(angle_deg) <= self.bearing_tolerance_deg:
                return True, None, 'theta=%+.2f deg' % angle_deg, None
            command = self.clockwise_command if angle_deg > 0.0 else self.counterclockwise_command
            return False, command, 'theta=%+.2f deg; rotating to zero' % angle_deg, None

        if not self.run_feedback_stage('ROTATE TO MARKER', self.rotation_stage_timeout_s,
                                       rotation_condition, initial_yaw):
            return False

        # STAGE 3: approach straight ahead. Stop if the marker drifts too far
        # laterally; do not silently continue with a poor heading.
        distance_increase_count = [0]
        previous_distance = [None]

        def forward_condition(pose):
            forward, lateral, angle, radius = self.marker_geometry(pose)
            angle_deg = math.degrees(angle)
            if abs(pose['x']) <= self.position_tolerance_m and abs(pose['y']) <= self.position_tolerance_m:
                return True, None, 'x=%.4f m, y=%.4f m' % (pose['x'], pose['y']), None
            if forward <= 0.0:
                return False, None, '', 'marker is no longer in front during forward approach'
            if abs(angle_deg) > self.max_forward_bearing_deg:
                return False, None, '', ('bearing drifted to %.2f deg during forward approach (limit %.2f deg)' %
                                         (angle_deg, self.max_forward_bearing_deg))
            if previous_distance[0] is not None and radius > previous_distance[0] + 0.02:
                distance_increase_count[0] += 1
            else:
                distance_increase_count[0] = 0
            previous_distance[0] = radius
            if distance_increase_count[0] >= 5:
                return False, None, '', 'ArUco distance increased repeatedly during forward movement'
            return False, self.forward_command, 'r=%.3f m, theta=%+.2f deg' % (radius, angle_deg), None

        if not self.run_feedback_stage('FORWARD APPROACH', self.forward_stage_timeout_s,
                                       forward_condition, initial_yaw):
            return False

        # STAGE 4: restore the initial heading using the selected feedback.
        # MAVROS yaw follows the vehicle convention. ArUco-relative yaw has the
        # opposite sign for a stationary marker, so its correction is reversed.
        def restore_yaw_condition(unused_pose):
            current_yaw = self.get_alignment_yaw()
            if current_yaw is None:
                return False, None, '', '%s became stale during yaw restoration' % self.get_alignment_yaw_description()
            error_deg = math.degrees(wrap_angle(current_yaw - initial_yaw))
            if abs(error_deg) <= self.yaw_tolerance_deg:
                return True, None, 'yaw error=%+.2f deg' % error_deg, None
            if self.yaw_source == 'aruco':
                command = self.counterclockwise_command if error_deg > 0.0 else self.clockwise_command
            else:
                command = self.clockwise_command if error_deg > 0.0 else self.counterclockwise_command
            return False, command, 'yaw error=%+.2f deg; restoring initial heading' % error_deg, None

        if not self.run_feedback_stage('RESTORE INITIAL YAW', self.yaw_restore_stage_timeout_s,
                                       restore_yaw_condition, initial_yaw,
                                       require_pose=(self.yaw_source == 'aruco'),
                                       enforce_yaw_limit=False):
            return False

        self.stop_tracks()
        rospy.loginfo('SUCCESS: marker alignment completed and initial vehicle yaw restored.')
        return True

    def run(self):
        if not self.enable_motors:
            rospy.logerr('Motor interlock is OFF. Set enable_motors:=true in deploy_auto.launch.')
            self.finished = True
            self.stop_tracks()
            return

        rospy.loginfo('Automatic alignment is READY and waiting for the f key.')
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.abort_requested and not self.start_requested:
            rate.sleep()

        if rospy.is_shutdown() or self.abort_requested:
            self.stop_tracks()
            return

        success = self.run_alignment()
        self.finished = True
        self.stop_tracks()
        if not success:
            rospy.logerr('Automatic alignment failed or was interrupted. Tracks commanded to stop.')


def main():
    rospy.init_node('automatic_alignment')
    controller = AutomaticAlignment()
    controller.run()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass