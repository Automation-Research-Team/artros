import os, sys, rospkg, rospy, tf
import std_msgs.msg
import ur_msgs.msg

######################################################################
#  class URScriptPublisher                                           #
######################################################################
class URScriptPublisher(object):
    def __init__(self, robot_name):
        super(URScriptPublisher, self).__init__()
        self._robot_name = robot_name
        self._listener   = tf.TransformListener()
        self._publisher  = rospy.Publisher("/" + robot_name +
                                           "_controller/ur_driver/URScript",
                                           std_msgs.msg.String, queue_size=1)
        self._rospack = rospkg.RosPack()
        self._move_j_template \
            = self._read_template("move_j.script")
        self._move_lin_template \
            = self._read_template("move_to_pose_lin.script")
        self._move_lin_rel_template \
            = self._read_template("move_lin_rel.script")
        self._linear_push_template \
            = self._read_template("linear_search_short.script")
        self._spiral_motion_template \
            = self._read_template("spiral_motion.script")
        self._insertion_template \
            = self._read_template("peginholespiral_imp_osx.script")
        self._horizontal_insertion_template \
            = self._read_template("peginholespiral_imp_osx_y_negative.script")

    # Publish various programs
    def wait(self, timeout_duration=rospy.Duration.from_sec(20.0)):
        rospy.logdebug("Waiting for UR program to finish.")
        # Only run this after sending custom URScripts and not the regular
        # motion commands, or this call will not terminate before the timeout.
        rospy.sleep(1.0)
        t_start     = rospy.Time.now()
        time_passed = rospy.Time.now() - t_start
        while self._is_program_running():
            rospy.sleep(.05)
            time_passed = rospy.Time.now() - t_start
            if time_passed > timeout_duration:
                rospy.loginfo("Timeout reached.")
                return False
        rospy.logdebug("UR Program has terminated.")
        return True

    def move_j(self, joint_positions, acceleration=0.5, velocity=0.5,
               wait=False):
        if not len(joint_positions) == 6:
            rospy.logwarn("Joint pose vector not of the correct length")
            return False
        return self._publish_program(self._move_j_template, wait,
                                     joint_positions, acceleration, velocity)

    def lin_move(self, target_pose, acceleration=0.5, velocity=0.03,
                 wait=False):
        rospy.logdebug("UR script lin move uses the ee_link of the robot, not the EE of the move group.")
        rospy.logdebug("original pose:")
        rospy.logdebug(target_pose)

        transform_success = False
        counter = 50
        while not transform_success:
            try:
                counter += 1
                robot_pose = self._listener.transformPose(
                                self._robot_name + "_base", target_pose)
                transform_success = True
            except:
                rospy.logdebug("Failed to transform from frame "
                               + target_pose.header.frame_id
                               + ". Waiting for .1 seconds")
                rospy.sleep(.1)

        xyz = [robot_pose.pose.position.x,
               robot_pose.pose.position.y, robot_pose.pose.position.z]
        q   = [robot_pose.pose.orientation.x, robot_pose.pose.orientation.y,
               robot_pose.pose.orientation.z, robot_pose.pose.orientation.w]
        rpy = tf.transformations.euler_from_quaternion(q)
        # rpy needs to be in axis-angle representation
        # http://www.zacobria.com/universal-robots-knowledge-base-tech-support-forum-hints-tips/python-code-example-of-converting-rpyeuler-angles-to-rotation-vectorangle-axis-for-universal-robots/
        rospy.logdebug("q in robot base:")
        rospy.logdebug(q)

        # This seems to work, but it uses the ee_link TCP of the robot.
        return self._publish_program(self._lin_move_template, wait,
                                     rpy, xyz, acceleration, velocity)

    def lin_mov_rel(self, translation, acceleration=0.5, velocity=0.03,
                    wait=False):
        return self._publish_program(self._lin_mov_rel_template, wait,
                                     translation.x, translation.y,
                                     translation.z, acceleration, velockty)

    def linear_push(self, max_force=10.0, force_direction="Z+",
                    max_approach_distance=0.1, forward_speed=0.02, wait=True):
        return self._publish_program(self._linear_push_template, wait,
                                     force_direction, max_force, forward_speed,
                                     max_approach_distance)

    def spiral_motion(self, acceleration=0.1, velocity=0.03,
                      max_radius=0.0065, radius_increment=0.002,
                      theta_increment=30, spiral_axis="Z", wait=False):
        if (radius_increment < 0.0001) or (radius_increment > 0.005):
            rospy.logerr("radius_incr needs to be between 0.0001 and 0.005 but is " + str(radius_increment))
            return False

        return self._publish_program(self._spiral_motion_template, wait,
                                     max_radius, radius_increment,
                                     velocity, acceleration,
                                     spiral_axis, theta_increment)

    def insertion(self, max_force=10.0, force_direction="Z+",
                  forward_speed=0.02, max_approach_distance=0.1,
                  max_radius=0.004, radius_increment=0.0003,
                  max_insertion_distance=0.035, impedance_mass=10,
                  peck_mode=False, wait=False):
        ### Function definitions, for reference:
        ### rq_linear_search(direction="Z+",force = 10, speed = 0.004, max_distance = 0.02 )
        ### rq_spiral_search_new(max_insertion_distance, force_threshold = 3, max_radius = 5.0, radius_incr=0.3, peck_mode = False):
        return self._publish_program(self._insertion_template, wait,
                                     force_direction, max_force, forward_speed,
                                     max_approach_distance,
                                     max_insertion_distance,
                                     max_radius, radius_increment, peck_mode,
                                     impedance_mass)

    def horizontal_insertion(self, max_force=10.0, force_direction="Y-",
                             forward_speed=0.02, max_approach_distance=0.1,
                             max_radius=0.007, radius_increment=0.0003,
                             max_insertion_distance=0.035, impedance_mass=10,
                             peck_mode=False, wait=False):
        ### Function definitions, for reference:
        ### rq_linear_search(direction="Z+",force = 10, speed = 0.004, max_distance = 0.02 )
        ### rq_spiral_search_new(max_insertion_distance, force_threshold = 3, max_radius = 5.0, radius_incr=0.3, peck_mode = False):
        return self._publish_program(self._horizontal_insertion_template, wait,
                                     force_direction, max_force, forward_speed,
                                     max_approach_distance,
                                     max_insertion_distance,
                                     max_radius, radius_increment, peck_mode,
                                     impedance_mass)

    # Private functions
    def _publish_program(self, template, wait, *args):
        rospy.loginfo("Sending UR robot program ")
        # rospy.logdebug("Program is:")
        # rospy.logdebug(program)
        program_msg = std_msgs.msg.String()
        program_msg.data = template.format(*args)
        print(program_msg.data)
        self._publisher.publish(program_msg)
        if wait:
            return self.wait(rospy.Duration.from_sec(30.0))
        return True

    def _read_template(self, filename):
        with open(os.path.join(self._rospack.get_path("aist_routines"),
                               "src/aist_routines/urscript", filename),
                  'rb') as file:
            template = ""
            program_line = file.read(1024)
            while program_line:
                template += program_line
                program_line = file.read(1024)
            return template

    def _is_program_running(self):
        msg = rospy.wait_for_message("/" + self._robot_name +
                                     "_controller/ur_driver/robot_mode_state",
                                     ur_msgs.msg.RobotModeDataMsg)
        if msg:
            return msg.is_program_running
        else:
            rospy.logerr("No message received from the robot. Is everything running? Is the namespace entered correctly with a leading slash?")
            return False
