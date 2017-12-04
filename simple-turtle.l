#!/usr/bin/env roseus
(load "/home/mech-user/catkin_ws/src/robot-programming/turtleboteus/euslisp/turtlebot-interface.l")
(turtlebot-init)
(send *ri* :state :bumper-vector)
;;(defun wait () (nil))
(defun wait () (print "hoge"))
;;(setq x 0)
;;(setq y 0)
;;(setq r 0)
          (progn
          (let ((x 0)
          (y 0)
          (r 0))
          (do-until-key
          (cond
          ((or (= (elt (send *ri* :state :bumper-vector) 1) 1) (= (elt (send *ri* :state :cliff-vector) 1) 1))
          ;; center bumper/cliff sensor
          (setq x -0.05)
          (setq r 20)
          )
          ((or (= (elt (send *ri* :state :bumper-vector) 2) 1) (= (elt (send *ri* :state :cliff-vector) 2) 1))
          ;; right sensor
          (setq x -0.05)
          (setq r 20)
          )
          ((or (= (elt (send *ri* :state :bumper-vector) 0) 1) (= (elt (send *ri* :state :cliff-vector) 0) 1))
          ; left sensor
          (setq x -0.05)
          (setq r -20)
          )
          (t
          (setq x 0.02)
          (setq r 0)))
          (send *ri* :go-velocity x y r)
          )
          (send *ri* :go-velocity 0 0 0)
          ))
