#!/usr/bin/env python
import rospy

class SetParams():

    def __init__(self):
        rospy.on_shutdown(self.restore_params)
        self.store_params()
        # self.set_params()
        
    def store_params(self):
        # self.move_base_global_costmap_base_scan = rospy.get_param("/move_base/global_costmap/obstacles/base_scan/topic")
        # self.move_base_local_costmap_base_scan = rospy.get_param("/move_base/local_costmap/obstacles/base_scan/topic")
        # self.safe_teleop_base_local_costmap_base_scan = rospy.get_param("/safe_teleop_base/local_costmap/obstacles/base_scan/topic")
        self.move_base_global_costmap_footprint = rospy.get_param("/move_base/global_costmap/footprint")
        self.move_base_local_costmap_footprint = rospy.get_param("/move_base/local_costmap/footprint")
        self.safe_teleop_base_local_costmap_footprint = rospy.get_param("/safe_teleop_base/local_costmap/footprint")
    

    def set_params(self):
        rospy.set_param("/move_base/global_costmap/obstacles/base_scan/topic", "/base_scan_mux")
        rospy.set_param("/move_base/local_costmap/obstacles/base_scan/topic", "/base_scan_mux")
        rospy.set_param("/safe_teleop_base/local_costmap/obstacles/base_scan/topic", "/base_scan_mux")
    
    def restore_params(self):
        # rospy.set_param("/move_base/global_costmap/obstacles/base_scan/topic", self.move_base_global_costmap_base_scan)
        # rospy.set_param("/move_base/local_costmap/obstacles/base_scan/topic", self.move_base_local_costmap_base_scan)
        # rospy.set_param("/safe_teleop_base/local_costmap/obstacles/base_scan/topic", self.safe_teleop_base_local_costmap_base_scan)
        rospy.set_param("/move_base/global_costmap/footprint", self.move_base_global_costmap_footprint)
        rospy.set_param("/move_base/local_costmap/footprint", self.move_base_local_costmap_footprint)
        rospy.set_param("/safe_teleop_base/local_costmap/footprint", self.safe_teleop_base_local_costmap_footprint)

if __name__ == '__main__':
    rospy.init_node("set_params")
    SetParams()
    rospy.spin()
