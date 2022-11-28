(defvar *pub-spots-marker-topic* "/navigation_pr2/spots_marker_array")
(defvar *pub-arcs-marker-topic* "/navigation_pr2/arcs_marker_array")
(defvar *pub-human-marker-topic* "/human_marker_array")

(defun cube->marker-msg (cb header
                            &key ((:color col) (float-vector 1.0 0 0))
                            ((:alpha a) 1.0)
                            ((:id idx) 0)
                            ((:action ac) visualization_msgs::Marker::*ADD*)
                            ns lifetime)
  "Convert cube object to visualization_msgs::Marker"
  (send cb :worldcoords) ;; make sure to update vertices
  (let ((msg (instance visualization_msgs::Marker :init
                       :type visualization_msgs::Marker::*CUBE*
                       :header header
                       :id idx
                       :action ac))
        (cent-coords (send cb :copy-worldcoords))
        (cx (x-of-cube cb))
        (cy (y-of-cube cb))
        (cz (z-of-cube cb)))
    (send msg :pose (ros::coords->tf-pose cent-coords))
    (send msg :scale (ros::pos->tf-translation
                      (float-vector cx cy cz)))
    (send msg :color (vector->rgba col a))
    (if ns (send msg :ns ns))
    (if lifetime (send msg :lifetime (ros::time lifetime)))
    msg))

(defun cylinder->marker-msg (cyl header
				 &key ((:color col) (float-vector 1.0 0 0))
				      ((:alpha a) 1.0)
                      ((:id idx) 0)
                      ((:action ac) visualization_msgs::Marker::*ADD*)
                      ns lifetime)
  "Convert cylinder object to visualization_msgs::Marker"
  (send cyl :worldcoords) ;; make sure to update vertices
  (let ((msg (instance visualization_msgs::Marker :init
                       :type visualization_msgs::Marker::*CYLINDER*
                       :header header
                       :id idx
                       :action ac))
        (cent-coords (send cyl :copy-worldcoords))
        (height (height-of-cylinder cyl))
        (radius (radius-of-cylinder cyl)))
    (send cent-coords :translate (float-vector 0 0 (/ height 2)))
    (send msg :pose (ros::coords->tf-pose cent-coords))
    (send msg :scale (ros::pos->tf-translation
                      (float-vector (* 2 radius) (* 2 radius) height)))
    (send msg :color (vector->rgba col a))
    (if ns (send msg :ns ns))
    (if lifetime (send msg :lifetime (ros::time lifetime)))
    msg))

(defun text->marker-msg (str c header
			   &key
			   ((:color col) (float-vector 1 1 1))
			   ((:alpha a) 1.0)
			   ((:id idx) 0)
               ((:action ac) visualization_msgs::Marker::*ADD*)
			   ((:scale sc) 100.0) ns lifetime)
  "Convert text to visualization_msgs::Marker"
  (let ((msg (instance visualization_msgs::Marker :init
                       :type visualization_msgs::Marker::*TEXT_VIEW_FACING*
                       :header header
                       :id idx
                       :action ac)))
    (send msg :pose (ros::coords->tf-pose c))
    (send msg :scale (ros::pos->tf-translation (float-vector sc sc sc)))
    (send msg :color (vector->rgba col a))
    (send msg :text str)
    (if ns (send msg :ns ns))
    (if lifetime (send msg :lifetime (ros::time lifetime)))
    msg))

(defun line-list->marker-msg (li header
                            &key ((:color col) (float-vector 1 0 0))
                                 ((:alpha a) 1.0)
                                 ((:id idx) 0)
                                 ((:action ac) visualization_msgs::Marker::*ADD*)
                                 ((:scale sc) 10.0) ns lifetime)
  "Convert list of line object to visualization_msgs::Marker"
  (let ((msg (instance visualization_msgs::Marker :init
                       :type visualization_msgs::Marker::*LINE_LIST*
                       :header header
                       :id idx
                       :action ac)))
    (send msg :points (mapcar #'(lambda (l) (ros::pos->tf-point l)) li))
    (send msg :scale (ros::pos->tf-translation (float-vector sc sc sc)))
    (send msg :color (vector->rgba col a))
    (if ns (send msg :ns ns))
    (if lifetime (send msg :lifetime (ros::time lifetime)))
    msg))

(defun make-pin-marker (name coords header id
                        &key (height 1800) (radius 100) (alpha 1.0) (lifetime 15) ((:action ac) 0) color)
  "Make pin marker messages for coordinates.
   if :color is set, pin color is set, otherwise random color is set automatically."
  (let ((cyl (make-cylinder radius (* radius 0.7) :segments 9 :pos (float-vector 0 0 (/ height 3))))
        (cub (make-cube (/ radius 3) (/ radius 3) (/ height 2.5) :pos (float-vector 0 0 (/ height 6))))
        (cds (send coords :copy-worldcoords)))

    ;; transform pin
    (send cyl :rotate -pi/2 :y)
    (send cyl :transform cds :world)
    (send cub :transform cds :world)

    ;; generate random color
    (unless color
      (setq color (scale (/ 1.0 255.0) (apply #'float-vector (color-category20 id)))))

    ;; make pin
    (list
     (cylinder->marker-msg cyl header :color color :ns "pin_head" :lifetime lifetime :id id :alpha alpha :action ac)
     (cube->marker-msg cub header :color color :ns "pin_body" :lifetime lifetime :id id :alpha alpha :action ac)
     (text->marker-msg name (send cds :translate (float-vector 0 0 (/ height 6)))
                       header :ns "pin_label" :lifetime lifetime :id id :action ac))))

(defun publish-pin-marker-array (spots header &key (alpha 0.8) (lifetime 15) ((:action ac) 0))
  "Publish marker array from spots.
   'spots' is list of cons (name . coords) for each spots."
  (unless (ros::get-topic-publisher *pub-spots-marker-topic*)
    (ros::advertise *pub-spots-marker-topic* visualization_msgs::MarkerArray 5)
    (unix:sleep 1))
  (let ((id 0))
    (ros::publish
     *pub-spots-marker-topic*
     (instance visualization_msgs::MarkerArray :init
               :markers
               (flatten
                (mapcar #'(lambda (spot)
                            (make-pin-marker (car spot) (cdr spot)
                                             header (incf id)
                                             :alpha alpha
                                             :lifetime lifetime
                                             :action ac)) spots))))))

(defun make-line-list-marker (coords1 coords2 header id
                        &key (alpha 1.0) (lifetime 15) ((:action ac) 0) color)
  (let ((li (list (send coords1 :pos) (send coords2 :pos))))

    ;; generate random color
    (unless color
      (setq color (scale (/ 1.0 255.0) (apply #'float-vector (color-category20 id)))))

    ;; make line-list
    (line-list->marker-msg li header :color color :ns "line_list" :lifetime lifetime :id id :alpha alpha :action ac)))

(defun publish-arcs-marker-array (nodes header &key (alpha 0.8) (lifetime 15) ((:action ac) 0))
  (unless (ros::get-topic-publisher *pub-arcs-marker-topic*)
    (ros::advertise *pub-arcs-marker-topic* visualization_msgs::MarkerArray 5)
    (unix:sleep 1))
  (let (markers (id 0))
    (dolist (n nodes)
      (push (mapcar #'(lambda (i) (make-line-list-marker (send n :get :coords) (send (cdr i) :get :coords) header (incf id) :color #f(0 0 1) :action ac)) (send n :successors)) markers))
    (ros::publish
     *pub-arcs-marker-topic*
     (instance visualization_msgs::MarkerArray :init
               :markers
               (flatten markers)))))

(defun make-human-marker (coords header id
                        &key (height 800) (radius 100) (alpha 1.0) (lifetime 15) color)
  "Make pin marker messages for coordinates.
   if :color is set, pin color is set, otherwise random color is set automatically."
  (let ((cyl (make-cylinder radius height :segments 20))
        (sphe (make-sphere (* radius 1.6) :pos (float-vector 0 0 (* height 1.1))))
        (cds (send coords :copy-worldcoords)))

    (send cyl :transform cds :world)
    (send sphe :transform cds :world)

    ;; generate random color
    (unless color
      (setq color (scale (/ 1.0 255.0) (apply #'float-vector (color-category20 id)))))

    ;; make pin
    (list
     (cylinder->marker-msg cyl header :color color :ns "human_body" :lifetime lifetime :id id :alpha alpha)
     (sphere->marker-msg sphe header :color color :ns "human_head" :lifetime lifetime :id id :alpha alpha))))

(defun publish-human-marker-array (spots header &key (alpha 0.8) (lifetime 15))
  "Publish marker array from spots.
   'spots' is list of cons (name . coords) for each spots."
  (unless (ros::get-topic-publisher *pub-human-marker-topic*)
    (ros::advertise *pub-human-marker-topic* visualization_msgs::MarkerArray 5)
    (unix:sleep 1))
  (let ((id 0))
    (ros::publish
     *pub-human-marker-topic*
     (instance visualization_msgs::MarkerArray :init
               :markers
               (flatten
                (mapcar #'(lambda (spot)
                            (make-human-marker spot
                                             header (incf id)
                                             :alpha alpha
                                             :lifetime lifetime)) spots))))))
