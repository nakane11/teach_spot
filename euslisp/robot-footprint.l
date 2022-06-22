(ros::roseus-add-msgs "jsk_recognition_msgs")

(defclass robot-footprint
  :slots (arm forward-fp backward-fp footprint-ns service-list))

(defmethod robot-footprint
  (:init (&key target-arm ((:forward-fp for)) ((:backward-fp back)) ((:footprint-ns ns)) ((:service-list service)))
    (setq arm target-arm)
    (setq forward-fp for)
    (setq backward-fp back)
    (setq footprint-ns ns)
    (setq service-list service))

  (:callback (msg)
      (let ((boxes (send msg :boxes)) human-fp footprint)
        (unless (null boxes)
          (let* ((box (elt boxes 0)) (position (send (send box :pose) :position)))
            (if (string= arm "larm")
                (progn
                  (setq human-fp (send self :left-footprint position))
                  (setq footprint (format nil "[~A,~A,~A]" forward-fp backward-fp human-fp)))
                (progn
                  (setq human-fp (send self :right-footprint position))
                  (setq footprint (format nil "[~A,~A,~A]" forward-fp human-fp backward-fp))))))
        (when (null boxes)
          (setq footprint (format nil "[~A,~A]" forward-fp backward-fp)))
        (dolist (s service-list)
          (ros::wait-for-service s))
        (dolist (ns footprint-ns)
          (ros::set-dynamic-reconfigure-param ns "footprint" :string footprint))))

  (:left-footprint  (position)
    (let (footprint (width 0.35))
      (setq footprint (format nil "[~A,~A],[~A,~A],[~A,~A],[~A,~A]"
                                  (- (send position :x) (/ width 2.0)) (- (send position :y) (/ width 2.0))
                                  (- (send position :x) (/ width 2.0)) (+ (send position :y) (/ width 2.0))
                                  (+ (send position :x) (/ width 2.0)) (+ (send position :y) (/ width 2.0))
                                  (+ (send position :x) (/ width 2.0)) (- (send position :y) (/ width 2.0))))
      footprint))

  (:right-footprint  (position)
    (let (footprint (width 0.5))
      (setq footprint (format nil "[~A,~A],[~A,~A],[~A,~A],[~A,~A]"
                                  (+ (send position :x) (/ width 2.0)) (+ (send position :y) (/ width 2.0))
                                  (+ (send position :x) (/ width 2.0)) (- (send position :y) (/ width 2.0))
                                  (- (send position :x) (/ width 2.0)) (- (send position :y) (/ width 2.0))
                                  (- (send position :x) (/ width 2.0)) (+ (send position :y) (/ width 2.0))))
      footprint)))


                      
          

  

              
