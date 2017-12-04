#!/usr/bin/env roseus

;; robotの初期化
(load "package://dxl_armed_turtlebot/euslisp/dxl-armed-turtlebot-interface.l")
(defvar *arm-dof* 7)
(dxl-armed-turtlebot-init)

;; ObjectDetection型トピックを使うため
(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "roseus")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "image_view2")
(ros::roseus-add-msgs "posedetection_msgs")

;;; 表示モデルなど
(load "models/chessboard-30-7x5-object.l")
(if (not (boundp '*irtviewer*)) (make-irtviewer))
(setq *target-object* (chessboard-30-7x5 :name "/test_object"))
(setq *target-rel-coords* (make-coords :pos (float-vector 0 0 1000)))
(objects (list *target-object* *dxl-armed-turtlebot*))

(defvar *root-frame-id* "camera_rgb_frame")
(setq *tfb* (instance ros::transform-broadcaster :init))

(ros::roseus "objectdetection_client")

;; ObjectDetection用コールバック関数定義
(defun objectdetection-cb (msg)
  (let ((mrk (instance image_view2::ImageMarker2 :init)) frame-id type ret)
    ;; 物体モデルを配置
    (setq frame-id (concatenate string "/" (send msg :header :frame_id)))
    (mapcar #'(lambda (obj-pose)
                (let* (;; (1) カメラ相対の座標系は、geometry_msgs/Poseという型で得られるので、Euslispのcoordsに変換する
                       (cam->obj-coords (ros::tf-pose->coords (send obj-pose :pose)))
                       ;; (2) *dxl-armed-turtlebot*モデルがカメラの座標系をもってるので、取得する
                       (cam-coords (send (send *dxl-armed-turtlebot* :camera_rgb_optical_frame_lk) :copy-worldcoords)))
                  ;; (3) Euslisp内部でのworld座標系の値にして、そこにcheckerboardモデルを配置する
                  (setq *target-rel-coords* cam->obj-coords)
                  (send *target-object* :newcoords (send cam-coords :transform cam->obj-coords))
                  ; (print (send cam-coords :transform cam->obj-coords))
                  ))
            (send msg :objects))
    ;; image_markerを出力
    (dolist (obj-pose (send msg :objects))
      (setq type (send obj-pose :type))
      (unless (eq (char type 0) #\/) (setq type (concatenate string "/" type)))
      (setq ret (ros::tf-pose->coords (send obj-pose :pose)))
      (send mrk :type image_view2::ImageMarker2::*FRAMES*)
      (send mrk :frames (list type))
      (send *tfb* :send-transform ret frame-id type)
      (ros::ros-info "~A ~A ~A" ret frame-id type)
      (ros::publish "image_marker" mrk))
    ))

(ros::advertise "image_marker" image_view2::ImageMarker2 1)
(ros::subscribe "/camera/rgb/ObjectDetection" posedetection_msgs::ObjectDetection #'objectdetection-cb)

(send *dxl-armed-turtlebot* :gripper :arm :angle-vector #f(40))
(send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 1000 :gripper-controller)
(send *ri* :wait-interpolation :gripper-controller)

(setq isUsingCamera 1)
(setq hideArm 1)
(do-until-key

  (ros::spin-once)

  (x::window-main-one)

  ; turn to direction of board
  (print *target-rel-coords*)
  (setq x (elt (send *target-rel-coords* :pos) 0))
  (setq z (elt (send *target-rel-coords* :pos) 2))
  (setq theta (* (/ 180 3.14) (- 0 (atan (/ x z)))))
  (cond
    ((> theta 20)
     (if (= isUsingCamera 0)
       (setq hideArm 1))
     (send *ri* :go-velocity 0 0 10)
       (print "move left")
     )
    ((< theta -20)
      (if (= isUsingCamera 0)
      (setq hideArm 1))
      (send *ri* :go-velocity 0 0 -10)
      (print "move right")
      )
    ((> z 600)
      (if (= isUsingCamera 0)
        (setq hideArm 1))
         (send *ri* :go-velocity 0.05 0 0)
      (print "move forward")
     )
    ((= isUsingCamera 0)
     (setq hideArm 1)
     (setq *target-rel-coords* (make-coords :pos (float-vector 0 0 1000))))
    (t
     (setq isUsingCamera 0)
     (send *dxl-armed-turtlebot* :inverse-kinematics (send *target-object* :coords) :rotation-axis nil :revert-if-fail nil)
     (send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 2000)
     (send *ri* :wait-interpolation)
     (print "move to position")
    ;  (unix::usleep (* 1000 1000))

     (send *dxl-armed-turtlebot* :gripper :arm :angle-vector #f(-45))
     (send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 1000 :gripper-controller)
     (send *ri* :wait-interpolation :gripper-controller)
     (print "close hand")
    ;  (unix::usleep (* 1000 1000))

     (send *dxl-armed-turtlebot* :reset-pose)
     (send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 1000)
     (send *ri* :wait-interpolation)
     (print "move to reset-pose")
    ;  (unix::usleep (* 1000 1000))

     (send *dxl-armed-turtlebot* :gripper :arm :angle-vector #f(40))
     (send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 1000 :gripper-controller)
     (send *ri* :wait-interpolation :gripper-controller)
     (print "open hand")
     (unix::usleep (* 1000 1000))
     )
    )
  (cond ((= hideArm 1)
         (send *dxl-armed-turtlebot* :tuckarm-pose3)
         (send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 1000)
         (send *ri* :wait-interpolation)
         (setq isUsingCamera 1)
         (setq hideArm 0)
         (print "hidden arm"))
    )
  (send *irtviewer* :draw-objects)
  (ros::rate 10)
  (ros::sleep)
  )
