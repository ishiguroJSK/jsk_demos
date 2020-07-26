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
              (setq larm-tgt (make-cascoords
  		       :pos (float-vector
  			     (* (send msg :pose :position :x) 1000)
  			     (* (send msg :pose :position :y) 1000)
  			     (* (send msg :pose :position :z) 1000))
  		       :rpy (float-vector 0 0 0)))
	      )
  (:rarm-cb (msg)
              (setq rarm-tgt (make-cascoords
		       :pos (float-vector
			     (* (send msg :pose :position :x) 1000)
			     (* (send msg :pose :position :y) 1000)
			     (* (send msg :pose :position :z) 1000))
		       :rpy (float-vector 0 0 0)))
	      )
  )


(setq m (instance realtime-ik-class :init))

(defun run ()
  (send *pr2* :head :neck-p :joint-angle 45)
  (send *pr2* :torso_lift_joint :joint-angle 300)

  (do-until-key
   (send *robot* :inverse-kinematics (list larm-tgt rarm-tgt)
	 :move-target (list (send *robot* :larm :end-coords) (send *robot* :rarm :end-coords))
	 :translation-axis (list t t)
	 :rotation-axis (list :z :z)
	 :use-torso nil
	 :stop 2
	 :revert-if-fail nil :debug-view nil
	 )
   (send *ri* :angle-vector (send *robot* :angle-vector) 100)
   (unix:usleep 100000)
   (ros::spin-once)
   )
  )

(run)


