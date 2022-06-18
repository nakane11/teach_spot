(load "package://teach_spot/euslisp/robot-footprint.l")
(ros::roseus "change_footprint")

(setq *arm* (ros::get-param "~arm"))

(defclass fetch-footprint
    :super robot-footprint)

(defmethod fetch-footprint
  (:init ()
    (send-super :init :target-arm *arm*
                 :forward-fp "[0.115,0.277],[0.212,0.212],[0.277,0.115],[0.3,0],[0.277,-0.115],[0.212,-0.212],[0.115,-0.277]"
                 :backward-fp "[0,-0.3],[-0.115,-0.277],[-0.212,-0.212],[-0.277,-0.115],[-0.3,0],[-0.277,0.115],[-0.212,0.212],[-0.115,0.277],[0,0.3]"
                 :footprint-ns (list "/move_base/global_costmap"
                                     "/move_base/local_costmap"
                                     "/safe_teleop_base/local_costmap"))
    (ros::subscribe "/person_pose_array_to_boxes/output" jsk_recognition_msgs::BoundingBoxArray #'send self :callback)))

(instance fetch-footprint :init)
(ros::spin)

