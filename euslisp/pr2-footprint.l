(load "package://teach_spot/euslisp/robot-footprint.l")
(ros::roseus "change_footprint")

(setq *arm* (ros::get-param "~arm"))

(defclass pr2-footprint
    :super robot-footprint)

(defmethod pr2-footprint
  (:init ()
    (send-super :init :target-arm *arm*
                :forward-fp "[0.34,0.35],[0.4,0],[0.34,-0.35]"
                :backward-fp "[-0.34,-0.35],[-0.34,0.35]"
                :footprint-ns (list "/move_base_node/global_costmap"
                                    "/move_base_node/local_costmap"
                                    "/safe_teleop_base/local_costmap"))
    (ros::ros::wait-for-service "/move_base_node/global_costmap/set_parameters")
    (ros::ros::wait-for-service "/move_base_node/local_costmap/set_parameters")
    (ros::subscribe "/person_pose_array_to_boxes/output" jsk_recognition_msgs::BoundingBoxArray #'send self :callback)))

(instance pr2-footprint :init)
(ros::spin)
