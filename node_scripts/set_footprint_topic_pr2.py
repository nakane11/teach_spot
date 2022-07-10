#!/usr/bin/env python
import rospy
import rosnode

class SetFootprintTopic():

    def __init__(self):
        self.set_default_params()
        self.topic_params = ["/move_base_node/global_costmap/footprint_topic",
                       "/move_base_node/local_costmap/footprint_topic",
                       "/safe_teleop_base/local_costmap/footprint_topic"]
        self.topic_name = "/dynamic_footprint"
        rospy.on_shutdown(self.delete_params)
        self.set_params()

    def wait_node_startup(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            is_alive = rosnode.rosnode_ping("/move_base_node", max_count=1, verbose=False)
            if is_alive is True:
                break
            rospy.logwarn("waiting move_base_node ...")
            rate.sleep()

    def print_params(self):
        for topic_param in self.topic_params:
            print("{} {}".format(topic_param, rospy.get_param(topic_param)))

    def set_params(self):
        rospy.loginfo("Set footprint parameters")
        for topic_param in self.topic_params:
            rospy.set_param(topic_param, self.topic_name)
        rosnode.kill_nodes(["/move_base_node"])
        self.wait_node_startup()
        self.print_params()
        
    def delete_params(self):
        rospy.loginfo("Delete footprint parameters")
        for topic_param in self.topic_params:
            if rospy.has_param(topic_param):
                rospy.delete_param(topic_param)
        rosnode.kill_nodes(["/move_base_node"])
        self.wait_node_startup()
        rospy.loginfo("move_base_node respawn")

if __name__ == '__main__':
    rospy.init_node("set_footprint_topic")
    sft = SetFootprintTopic()
    rospy.spin()
