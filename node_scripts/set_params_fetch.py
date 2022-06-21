#!/usr/bin/env python
import rospy
import rosnode
import dynamic_reconfigure.client

class SetParams():

    def __init__(self):
        # self.set_default_params()
        self.params = ["/move_base/global_costmap/obstacles/base_scan/topic",
                       "/move_base/local_costmap/obstacles/base_scan/topic",
                       "/safe_teleop_base/local_costmap/obstacles/base_scan/topic"]
        self.mux_topics = ["/base_scan_mux", "/base_scan_mux", "/base_scan_mux"]
        rospy.on_shutdown(self.restore_params)
        self.store_params()
        self.clients = [dynamic_reconfigure.client.Client("/move_base/global_costmap/"),
                        dynamic_reconfigure.client.Client("/move_base/local_costmap/"),
                        dynamic_reconfigure.client.Client("/safe_teleop_base/local_costmap/")]
        self.set_params()

    def wait_node_startup(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            is_alive = rosnode.rosnode_ping("/move_base", max_count=1, verbose=False)
            if is_alive is True:
                break
            rospy.logwarn("waiting move_base_node ...")
            rate.sleep()

    def set_default_params(self):
        params = [("/move_base/global_costmap/obstacles/base_scan/topic", "/base_scan"),
                  ("/move_base/local_costmap/obstacles/base_scan/topic", "/base_scan"),
                  ("/safe_teleop_base/local_costmap/obstacles/base_scan/topic", "/base_scan")]
        for param, topic in params:
            rospy.set_param(param, topic)
        self.print_params()

    def print_params(self):
        params = [("/move_base/global_costmap/obstacles/base_scan/topic", "/base_scan"),
                  ("/move_base/local_costmap/obstacles/base_scan/topic", "/base_scan"),
                  ("/safe_teleop_base/local_costmap/obstacles/base_scan/topic", "/base_scan")]
        for param, _ in params:
            print("{} {}".format(param, rospy.get_param(param)))

    def store_params(self):
        self.move_base_topics = []
        for param in self.params:
            self.move_base_topics.append(rospy.get_param(param))

        self.footprints = [rospy.get_param("/move_base/global_costmap/footprint"),
                           rospy.get_param("/move_base/local_costmap/footprint"),
                           rospy.get_param("/safe_teleop_base/local_costmap/footprint")]    

    def set_params(self):
        rospy.loginfo("Set move_base parameters")
        for param, topic in zip(self.params, self.mux_topics):
            rospy.set_param(param, topic)
        rosnode.kill_nodes(["/move_base"])
        self.wait_node_startup()
        self.print_params()
        
    def restore_params(self):
        rospy.loginfo("Restore move_base parameters")
        for client, footprint in zip(self.clients, self.footprints):
            client.update_configuration({"footprint": footprint})        
        for param, topic in zip(self.params, self.move_base_topics):
            rospy.set_param(param, topic)
        rosnode.kill_nodes(["/move_base"])
        self.wait_node_startup()
        self.print_params()

if __name__ == '__main__':
    rospy.init_node("set_params")
    sp = SetParams()
    rospy.spin()
