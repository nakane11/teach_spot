(ros::roseus-add-msgs "geometry_msgs")
(load "package://teach_spot/euslisp/marker.l")

(ros::roseus "human_pin")
(setq *tfl* (instance ros::transform-listener :init))
;; (when (boundp '*tfl*)
;;   (setq *tfl* (instance ros::transform-listener :init)))

(defun callback (msg)
  (let* (spots (stamp (ros::time-now)) (header (instance std_msgs::header :init :stamp stamp :frame_id "/base_link")) (poses (send msg :poses)))
    (setq base-to-laser-coords (send *tfl* :lookup-transform "/base_link" "/laser_link" stamp))
    (when base-to-laser-coords
      (dolist (pose poses)
        (send (send pose :orientation) :w 1.0)
        (let (coords)
          (setq laser-coords (ros::tf-pose->coords pose))
          (setq coords (send (send base-to-laser-coords :copy-worldcoords) :transform laser-coords))
          (setq coords (make-coords :pos (send coords :worldpos)))
          (push coords spots)
          ))
      (publish-human-marker-array spots header :lifetime 1))))

(ros::subscribe "/dr_spaam_lidar_person_detection/output" geometry_msgs::PoseArray #'callback)

(do-until-key
 (ros::spin-once)
 (ros::sleep))

