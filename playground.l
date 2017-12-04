(load "package://dxl_armed_turtlebot/euslisp/dxl-armed-turtlebot-interface.l")


(load "package://roseus/test/joy.l")
(init-ps3joy) ;; *joy* という大域変数のセット







(defun circle ()
  (defvar *arm-dof* 7)
  (dxl-armed-turtlebot-init)
  ; (send *dxl-armed-turtlebot* :init-pose)
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (unix::usleep (* 2000 1000))
  (dotimes (i 10000)

        (send *dxl-armed-turtlebot* :inverse-kinematics
              (make-coords :pos (float-vector (+ 300 (* 30 (sin (deg2rad (* 200 i))))) (+ 150 (* 30 (cos (deg2rad (* 200 i))))) 150))
              :rotation-axis nil)
        (send *irtviewer* :draw-objects)
        (send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 3)
        (send *ri* :wait-interpolation)

        ))



(defun listen ()
  (progn
   (ros::rate 10)
   (do-until-key
       (ros::spin-once)
       (ros::sleep)
       (print (send *joy* :ljoy-lr))
      ;  1 ~ -1 is left ~ right
       (print (send *joy* :ljoy-fb))
 )))

(defun pickup ()
  (defvar *arm-dof* 7)
  (dxl-armed-turtlebot-init)
  (send *dxl-armed-turtlebot* :init-pose)
  ; (setq reset-coords (send *dxl-armed-turtlebot* :end-coords))
  (setq reset-coords (make-coords :pos (float-vector 300 150 150)))
  (send *irtviewer* :draw-objects)
  (send *dxl-armed-turtlebot* :inverse-kinematics
        reset-coords
        :rotation-axis nil)
  (send *ri* :wait-interpolation)

  (ros::rate 10)
  (setq state 1)
  (setq hand-coords (make-coords :pos (float-vector 300 150 300)))
  (setq open 1)
  (do-until-key
   (ros::spin-once)
   (ros::sleep)
   (if (= (send *joy* :circle-button) 1) (cond
                                           ((= state 1) (setq state 2))
                                           ((= state 2) (setq state 1))))
   (if (= (send *joy* :start-button) 1)
   (setq hand-coords (make-coords :pos (float-vector 300 150 150))))
   (cond
     ((= state 1)
      ; search for TRASH
      (setq x (- (send *joy* :arrow-r) (send *joy* :arrow-l)))
      (setq y (- (send *joy* :arrow-f) (send *joy* :arrow-b)))
      (setq z (- (send *joy* :triangle-button) (send *joy* :cross-button)))
      (send hand-coords :translate (float-vector x y z))
      )
     ((= state 2)
      (print "state 2")
      (if
        (= open 1)
        (send *dxl-armed-turtlebot* :gripper :arm :angle-vector #f(40))
        (send *dxl-armed-turtlebot* :gripper :arm :angle-vector #f(-30))
        )
      (send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 1000 :gripper-controller)
      (send *ri* :wait-interpolation :gripper-controller)
      (setq open (- 1 open))
      (setq state 1)
      (unix::usleep (* 2000 1000))
      )
     )
   (send *dxl-armed-turtlebot* :inverse-kinematics hand-coords :rotation-axis nil)
   (send *irtviewer* :draw-objects)
   (send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 1)
   (send *ri* :wait-interpolation)
   )
  )
