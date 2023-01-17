#!/usr/bin/env python
import rospy
import rosnode
import dynamic_reconfigure.client

class SetParams():

    def __init__(self):
        self.params = ["/move_base_node/global_costmap/obstacle_layer/base_scan_filtered/topic",
                       "/move_base_node/local_costmap/obstacle_layer/base_scan_filtered/topic",
                       "/safe_teleop_base/local_costmap/obstacle_layer/base_scan_filtered/topic",
                       "/move_base_node/global_costmap/obstacle_layer/tilt_scan_filtered/topic",
                       "/move_base_node/local_costmap/obstacle_layer/tilt_scan_filtered/topic",
                       "/safe_teleop_base/local_costmap/obstacle_layer/tilt_scan_filtered/topic"]
        self.mux_topics = ["/base_scan_filtered_mux", "/base_scan_filtered_mux", "/base_scan_filtered_mux",
                       "/tilt_scan_filtered_mux", "/tilt_scan_filtered_mux", "/tilt_scan_filtered_mux"]

        self.footprint_params = ["/move_base_node/global_costmap/footprint_topic",
                       "/move_base_node/local_costmap/footprint_topic",
                       "/safe_teleop_base/local_costmap/footprint_topic"]
        self.footprint_topics = ["/global_costmap/dynamic_footprint",
                                 "/local_costmap/dynamic_footprint",
                                 "/local_costmap/dynamic_footprint"]

        # self.set_default_params()
        rospy.on_shutdown(self.restore_params)
        self.store_params()
        self.clients = [dynamic_reconfigure.client.Client("/move_base_node/global_costmap/"),
                        dynamic_reconfigure.client.Client("/move_base_node/local_costmap/"),
                        dynamic_reconfigure.client.Client("/safe_teleop_base/local_costmap/")]
        self.set_params()

    def wait_node_startup(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            is_alive = rosnode.rosnode_ping("/move_base_node", max_count=1, verbose=False)
            if is_alive is True:
                break
            rospy.logwarn("waiting move_base_node ...")
            rate.sleep()

    def set_default_params(self):
        params = [("/move_base_node/global_costmap/obstacle_layer/base_scan_filtered/topic", "/base_scan_filtered"),
                  ("/move_base_node/local_costmap/obstacle_layer/base_scan_filtered/topic", "/base_scan_filtered"),
                  ("/safe_teleop_base/local_costmap/obstacle_layer/base_scan_filtered/topic", "/base_scan_filtered"),
                  ("/move_base_node/global_costmap/obstacle_layer/tilt_scan_filtered/topic", "/tilt_scan_filtered/navigation"),
                  ("/move_base_node/local_costmap/obstacle_layer/tilt_scan_filtered/topic", "/tilt_scan_filtered/navigation"),
                  ("/safe_teleop_base/local_costmap/obstacle_layer/tilt_scan_filtered/topic", "/tilt_scan_filtered/navigation")]
        for param, topic in params:
            rospy.set_param(param, topic)
        self.print_params()

    def print_params(self):
        for param in self.params:
            print("{} {}".format(param, rospy.get_param(param)))
        for param in self.footprint_params:
            print("{} {}".format(param, rospy.get_param(param)))

    def store_params(self):
        self.move_base_topics = []
        for param in self.params:
            self.move_base_topics.append(rospy.get_param(param))

    def set_params(self):
        rospy.loginfo("Set move_base parameters")
        for param, topic in zip(self.params, self.mux_topics):
            rospy.set_param(param, topic)
        for param, topic in zip(self.footprint_params, self.footprint_topics):
            rospy.set_param(param, topic)
        self.kill_move_base()
        
    def restore_params(self):
        rospy.loginfo("Restore move_base parameters")
        for param, topic in zip(self.params, self.move_base_topics):
            rospy.set_param(param, topic)
        self.delete_params()
        self.kill_move_base()

    def delete_params(self):
        rospy.loginfo("Delete footprint parameters")
        for param in self.footprint_params:
            if rospy.has_param(param):
                rospy.delete_param(param)

    def kill_move_base(self):
        rosnode.kill_nodes(["/move_base_node"])
        self.wait_node_startup()
        rospy.loginfo("move_base_node respawn")
        self.print_params()

if __name__ == '__main__':
    rospy.init_node("set_params")
    sp = SetParams()
    rospy.spin()
