#!/usr/bin/env roseus
(require :pr2-interface "package://pr2eus/pr2-interface.l")

(ros::roseus "pr2_realtime_ik")

(pr2-init)
(setq *robot* *pr2*)
(setq larm-tgt (make-coords :pos #f(500 300 1000)))
(setq rarm-tgt (make-coords :pos #f(500 -300 1000)))

(objects (list *robot* larm-tgt rarm-tgt))

(defclass realtime-ik-class
  :super propertied-object
  :slots ())
(defmethod realtime-ik-class
  (:init ()
	 (ros::subscribe "master_larm_pose" geometry_msgs::PoseStamped #'send self :larm-cb)
	 (ros::subscribe "master_rarm_pose" geometry_msgs::PoseStamped #'send self :rarm-cb)
	 )
  (:larm-cb (msg)
	    (setq raw-coords (ros::tf-pose->coords (send msg :pose)))
	    (setq llim #f( 500 -200  600))
	    (setq ulim #f(1200  500 1200))
	    (setq safe-pos (vmin (vmax (send raw-coords :pos) llim) ulim))
	    (setq larm-tgt (make-coords :pos safe-pos :rpy (float-vector 0 0 (/ pi 2) ))))
  (:rarm-cb (msg)
	    (setq raw-coords (ros::tf-pose->coords (send msg :pose)))
	    (setq llim #f( 500 -500  600))
	    (setq ulim #f(1200  200 1200))
	    (setq safe-pos (vmin (vmax (send raw-coords :pos) llim) ulim))
	    (setq rarm-tgt (make-coords :pos safe-pos :rot (send raw-coords :rot))))
  )


(setq m (instance realtime-ik-class :init))

(defun run ()
  (send *pr2* :reset-pose)
  (send *pr2* :head :neck-p :joint-angle 60)
  (send *pr2* :torso_lift_joint :joint-angle 300)
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (ros::rate 10)
  (while (ros::ok)
    (send *robot* :inverse-kinematics (list larm-tgt rarm-tgt)
	  :move-target (list (send *robot* :larm :end-coords) (send *robot* :rarm :end-coords))
	  :translation-axis (list t t)
	  :rotation-axis (list t t)
	  ;; :rotation-axis (list :z :z)
	  :use-torso nil
	  :stop 2
	  :revert-if-fail nil :debug-view nil
	  )
    (send *ri* :angle-vector (send *robot* :angle-vector) 100)
    (send *irtviewer* :draw-objects :flush nil)
    (dolist (tgt (list larm-tgt rarm-tgt)) (send tgt :draw-on :flush t))
    (x::window-main-one)
    (ros::spin-once)
    (ros::sleep)
    )
  )

(run)


