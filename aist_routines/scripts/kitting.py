#!/usr/bin/env python

import os, csv, copy, time, datetime, re, collections
import rospy, rospkg

from aist_routines.ur import URRoutines
from aist_msgs        import msg as amsg

######################################################################
#  global variables                                                  #
######################################################################
rp = rospkg.RosPack()

ts = time.time()
start_date_time = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H-%M-%S')
number_of_attempted = 1

######################################################################
#  class KittingRoutines                                             #
######################################################################
class KittingRoutines(URRoutines):
    """Implements kitting routines for aist robot system."""
    PartProps = collections.namedtuple(
        "PartProps", "robot_name, camera_name, destination, approach_offset, grasp_offset, place_offset, speed_slow, use_normals")
    _part_props = {
        4  : PartProps("b_bot", "a_phoxi_m_camera", "tray_1_partition_4",
                       0.15, -0.002, 0.05, 0.04, True),
        5  : PartProps("b_bot", "a_phoxi_m_camera", "tray_2_partition_6",
                       0.15, -0.002, 0.03, 1.0,  True),
        6  : PartProps("b_bot", "a_phoxi_m_camera", "tray_1_partition_3",
                       0.15,  0.0,   0.05, 1.0,  False),
        7  : PartProps("b_bot", "a_phoxi_m_camera", "tray_1_partition_2",
                       0.15, -0.002, 0.05, 0.04, True),
        8  : PartProps("b_bot", "a_phoxi_m_camera", "tray_2_partition_1",
                       0.15, -0.004, 0.03, 0.1,  True),
        9  : PartProps("a_bot", "a_phoxi_m_camera", "tray_2_partition_4",
                       0.15, -0.002, 0.05, 1.0,  False),
        10 : PartProps("a_bot", "a_phoxi_m_camera", "tray_2_partition_7",
                       0.15, -0.002, 0.05, 1.0,  False),
        11 : PartProps("b_bot", "a_phoxi_m_camera", "tray_1_partition_1",
                       0.15,  0.0,   0.05, 0.04, True),
        12 : PartProps("b_bot", "a_phoxi_m_camera", "tray_2_partition_3",
                       0.15,  0.0,   0.05, 0.04, False),
        13 : PartProps("b_bot", "a_phoxi_m_camera", "tray_1_partition_5",
                       0.15,  0.0,   0.05, 0.04, True),
        14 : PartProps("a_bot", "a_phoxi_m_camera", "tray_2_partition_2",
                       0.15, -0.002, 0.05, 1.0,  False),
        15 : PartProps("a_bot", "a_phoxi_m_camera", "tray_2_partition_5",
                       0.15, -0.002, 0.05, 1.0,  False),
        16 : PartProps("a_bot", "a_phoxi_m_camera", "tray_2_partition_8",
                       0.15, -0.002, 0.05, 1.0,  False),
        17 : PartProps("a_bot", "a_phoxi_m_camera", "skrewholder_1",
                       0.15, -0.002, 0.05, 1.0,  False),
        18 : PartProps("a_bot", "a_phoxi_m_camera", "skrewholder_2",
                       0.15, -0.002, 0.05, 1.0,  False),
    }
    BinProps = collections.namedtuple("BinProps",
                                      "bin_id, part_id, part_props")

    def __init__(self):
        super(KittingRoutines, self).__init__()

        self._bins = [
            "bin_1_part_5",
            "bin_1_part_16",
            "bin_1_part_17",
            "bin_2_part_4",
            "bin_2_part_7",
            "bin_2_part_8",
        ]

        # Assign part information to each bin.
        self._items = {}
        for bin_id, bin in enumerate(self._bins, 1):
            part_id = int(re.search("[0-9]+$", bin).group())
            self._items[bin] = KittingRoutines.BinProps(
                bin_id, part_id, KittingRoutines._part_props[part_id])

        self._former_robot_name = None
        self._fail_poses = []
        #self.go_to_named_pose("home", "all_bots")

    @property
    def nbins(self):
        return len(self._bins)

    @property
    def former_robot_name(self):
        return self._former_robot_name

    def item(self, bin):
        return self._items[bin]

    ###----- main procedure
    def run(self):
        self.go_to_named_pose("back", "all_bots")
        for bin in self._bins:
            if rospy.is_shutdown():
                break
            self.attempt_bin(bin, 1)
        self.go_to_named_pose("home", "all_bots")

    def attempt_bin(self, bin, max_attempts=5, marker_lifetime=0):
        item  = self._items[bin]
        props = item.part_props

        # If using a different robot from the former, move it back to home.
        if self._former_robot_name != None and \
           self._former_robot_name != props.robot_name:
            self.go_to_named_pose("back", self._former_robot_name)
        self._former_robot_name = props.robot_name

        # Move to 0.15m above the bin if the camera is mounted on the robot.
        if self._is_eye_on_hand(props.robot_name, props.camera_name):
            self.go_to_frame(props.robot_name, bin, (0, 0, 0.15))

        # Search for graspabilities.
        self.graspability_send_goal(props.robot_name, props.camera_name,
                                    item.part_id, item.bin_id,
                                    props.use_normals)
        (pick_poses, _, success) = self.graspability_wait_for_result(
                                        props.camera_name, marker_lifetime)
        if not success:
            return False

        # Attempt to pick the item.
        nattempts = 0
        for pose in pick_poses:
            if nattempts == max_attempts:
                break

            if self._is_close_to_fail_poses(pose):
                continue

            result = self.pick(props.robot_name, pose,
                               speed_slow=props.speed_slow,
                               grasp_offset=props.grasp_offset,
                               approach_offset=props.approach_offset)
            if result == amsg.pickOrPlaceResult.SUCCESS:
                result = self.place_at_frame(
                                props.robot_name, props.destination,
                                speed_slow=props.speed_slow,
                                place_offset=props.place_offset,
                                approach_offset=props.approach_offset)
                return result == amsg.pickOrPlaceResult.SUCCESS
            elif result == amsg.pickOrPlaceResult.MOVE_FAILURE or \
                 result == amsg.pickOrPlaceResult.APPROACH_FAILURE:
                self._fail_poses.append(pose)
            elif result == amsg.pickOrPlaceResult.DEPARTURE_FAILURE:
                self.release(props.robot_name)
                raise RuntimeError("Failed to depart from pick/place pose")
            elif result == amsg.pickOrPlaceResult.PICK_FAILURE:
                self._fail_poses.append(pose)
                nattempts += 1

            self.release(props.robot_name)

        return False

    def clear_fail_poses(self):
        self._fail_poses = []

    def _is_eye_on_hand(self, robot_name, camera_name):
        return camera_name == robot_name + "_camera"

    def _is_close_to_fail_poses(self, pose):
        for fail_pose in self._fail_poses:
            if self._all_close(pose, fail_pose, 0.005):
                return True
        return False


if __name__ == '__main__':
    with KittingRoutines() as kitting:
        while not rospy.is_shutdown():
            print("============ Kitting procedures ============ ")
            print("  b: Create a backgroud image")
            print("  m: Create a mask image")
            print("  s: Search graspabilities")
            print("  a: Attempt to pick and place")
            print("  A: Repeat attempts to pick and place")
            print("  k: Do kitting task")
            print("  H: Move all robots to home")
            print("  B: Move all robots to back")
            print("  q: Quit")

            try:
                key = raw_input(">> ")
                if key == 'q':
                    break
                elif key == 'H':
                    kitting.go_to_named_pose("home", "all_bots")
                elif key == 'B':
                    kitting.go_to_named_pose("back", "all_bots")
                elif key == 'b':
                    kitting.create_background_image("a_phoxi_m_camera")
                elif key == 'm':
                    kitting.create_mask_image("a_phoxi_m_camera",
                                              kitting.nbins)
                elif key == 's':
                    item  = kitting.item(raw_input("  bin name? "))
                    props = item.part_props
                    # kitting.search_graspability(props.robot_name,
                    #                             props.camera_name,
                    #                             item.part_id, item.bin_id,
                    #                             marker_lifetime=0)
                    kitting.graspability_send_goal(props.robot_name,
                                                   props.camera_name,
                                                   item.part_id, item.bin_id,
                                                   props.use_normals)
                    kitting.graspability_wait_for_result(props.camera_name,
                                                         marker_lifetime=0)
                elif key == 'a':
                    bin = raw_input("  bin name? ")
                    kitting.clear_fail_poses()
                    kitting.attempt_bin(bin, 5, 0)
                    kitting.go_to_named_pose("home", kitting.former_robot_name)
                elif key == 'A':
                    bin = raw_input("  bin name? ")
                    kitting.clear_fail_poses()
                    while kitting.attempt_bin(bin, 5, 0):
                        pass
                    kitting.go_to_named_pose("home", kitting.former_robot_name)
                elif key == 'k':
                    kitting.run()
            except Exception as e:
                print(e.message)
