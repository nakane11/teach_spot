#!/usr/bin/env python
import rospy
import dynamic_reconfigure.client

class SetParams():

    def __init__(self):
        rospy.on_shutdown(self.restore_params)
        self.store_params()
        self.clients = [dynamic_reconfigure.client.Client("/move_base_node/global_costmap/"),
                        dynamic_reconfigure.client.Client("/move_base_node/local_costmap/"),
                        dynamic_reconfigure.client.Client("/safe_teleop_base/local_costmap/")]
        # self.set_params()
        
    def store_params(self):
        # self.move_base_node_global_costmap_base_scan = rospy.get_param("/move_base_node/global_costmap/obstacle_layer/base_scan_filtered/topic")
        # self.move_base_node_local_costmap_base_scan = rospy.get_param("/move_base_node/local_costmap/obstacle_layer/base_scan_filtered/topic")
        # self.safe_teleop_base_local_costmap_base_scan = rospy.get_param("/safe_teleop_base/local_costmap/obstacle_layer/base_scan_filtered/topic")
        # self.move_base_node_global_costmap_tilt_scan = rospy.get_param("/move_base_node/global_costmap/obstacle_layer/tilt_scan_filtered/topic")
        # self.move_base_node_local_costmap_tilt_scan = rospy.get_param("/move_base_node/local_costmap/obstacle_layer/tilt_scan_filtered/topic")
        # self.safe_teleop_base_local_costmap_tilt_scan = rospy.get_param("/safe_teleop_base/local_costmap/obstacle_layer/tilt_scan_filtered/topic")
        self.footprints = [rospy.get_param("/move_base_node/global_costmap/footprint"),
                           rospy.get_param("/move_base_node/local_costmap/footprint"),
                           rospy.get_param("/safe_teleop_base/local_costmap/footprint")]    

    def set_params(self):
        rospy.set_param("/move_base_node/global_costmap/obstacle_layer/base_scan_filtered/topic", "/base_scan_mux")
        rospy.set_param("/move_base_node/local_costmap/obstacle_layer/base_scan_filtered/topic", "/base_scan_mux")
        rospy.set_param("/safe_teleop_base/local_costmap/obstacle_layer/base_scan_filtered/topic", "/base_scan_mux")
        rospy.set_param("/move_base_node/global_costmap/obstacle_layer/tilt_scan_filtered/topic", "/tilt_scan_mux")
        rospy.set_param("/move_base_node/local_costmap/obstacle_layer/tilt_scan_filtered/topic", "/tilt_scan_mux")
        rospy.set_param("/safe_teleop_base/local_costmap/obstacle_layer/tilt_scan_filtered/topic", "/tilt_scan_mux")
    
    def restore_params(self):
        # rospy.set_param("/move_base_node/global_costmap/obstacle_layer/base_scan_filtered/topic", self.move_base_node_global_costmap_base_scan)
        # rospy.set_param("/move_base_node/local_costmap/obstacle_layer/base_scan_filtered/topic", self.move_base_node_local_costmap_base_scan)
        # rospy.set_param("/safe_teleop_base/local_costmap/obstacle_layer/base_scan_filtered/topic", self.safe_teleop_base_local_costmap_base_scan)
        # rospy.set_param("/move_base_node/global_costmap/obstacle_layer/tilt_scan_filtered/topic", self.move_base_node_global_costmap_tilt_scan)
        # rospy.set_param("/move_base_node/local_costmap/obstacle_layer/tilt_scan_filtered/topic", self.move_base_node_local_costmap_tilt_scan)
        # rospy.set_param("/safe_teleop_base/local_costmap/obstacle_layer/tilt_scan_filtered/topic", self.safe_teleop_base_local_costmap_tilt_scan)
        for client, footprint in zip(self.clients, self.footprints):
            client.update_configuration({"footprint": footprint})        

if __name__ == '__main__':
    rospy.init_node("set_params")
    SetParams()
    rospy.spin()
