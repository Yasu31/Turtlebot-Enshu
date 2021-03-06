#!/usr/bin/env roseus

; to move hand in a circular manner.
(setq demo 1)

; to move real robot
(setq real 0)

(setq *checker-coords* (make-coords :pos (float-vector 0 0 0)))
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

(if (not (boundp '*irtviewer*)) (make-irtviewer))
(setq cube (make-cube 300 10 10))
(setq tsuba1 (make-cylinder 30 10))
(send tsuba1 :move-to (make-coords :pos #f(-90 0 0) :rpy (float-vector 0 (deg2rad 90) 0)))
(setq cube (body+ cube tsuba1))
(send cube :set-color :red)

(setq sword-cube (make-cube 10 10 300))
(setq tsuba2 (make-cylinder 30 10))
(send tsuba2 :move-to (make-coords :pos #f(0 0 -90)))
(setq sword-cube (body+ sword-cube tsuba2))
(send sword-cube :set-color :blue)
(objects (list *dxl-armed-turtlebot* cube sword-cube))
(defvar *root-frame-id* "camera_rgb_frame")
(setq *tfb* (instance ros::transform-broadcaster :init))

(send *dxl-armed-turtlebot* :gripper :arm :angle-vector #f(-45))
(if (= real 1)
  (progn
   (send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 10 :gripper-controller)
   (send *ri* :wait-interpolation :gripper-controller)))


(ros::roseus "objectdetection_client")

(defun objectdetection-cb (msg)
  (let ((mrk (instance image_view2::ImageMarker2 :init)) frame-id type ret)
    (setq frame-id (concatenate string "/" (send msg :header :frame_id)))
    (mapcar #'(lambda (obj-pose)
                (let* (;; (1) カメラ相対の座標系は、geometry_msgs/Poseという型で得られるので、Euslispのcoordsに変換する
                       (cam->obj-coords (ros::tf-pose->coords (send obj-pose :pose)))
                       ;; (2) *dxl-armed-turtlebot*モデルがカメラの座標系をもってるので、取得する
                       (cam-coords (send (send *dxl-armed-turtlebot* :camera_rgb_optical_frame_lk) :copy-worldcoords)))
                  ;; (3) Euslisp内部でのworld座標系の値にして、そこにcheckerboardモデルを配置する
                  (setq *target-rel-coords* cam->obj-coords)
                  (send cam-coords :transform cam->obj-coords)
                  (print (send cam-coords :transform cam->obj-coords))
                  (setq *checker-coords* (send cam-coords :transform cam->obj-coords))
                  ))
            (send msg :objects))
    ))

(ros::subscribe "/camera/rgb/ObjectDetection" posedetection_msgs::ObjectDetection #'objectdetection-cb)

(ros::rate 100)

; returns a vector orthogonal to any random vector you throw at it.
; input&output are both float-vector
; http://www.cg.info.hiroshima-cu.ac.jp/~miyazaki/knowledge/tech68.html
(defun get-orthogonal (vector)
  (let ((x (elt vector 0))
        (y (elt vector 1))
        (z (elt vector 2))
        (return-vector (float-vector 0 0 0)))
    (cond
      ((and (< (abs x) (abs y)) (< (abs x) (abs z)))
       (setq return-vector (float-vector 0 z (- 0 y))))
      ((and (< (abs y) (abs x)) (< (abs y) (abs z)))
       (setq return-vector (float-vector (- 0 z) 0 x)))
      (t
       (setq return-vector (float-vector y (- 0 x) 0)))
      )
    return-vector
    )
  )

; return a 3x3 matrix with the x axis being the vector you threw at it.
; http://euslisp.github.io/jskeus/jmanual-node111.html
; use :rot to set rotation matrix
(defun get-matrix (vector)
  (let ((x-vector (float-vector 0 0 0))
        (y-vector (float-vector 0 0 0))
        (z-vector (float-vector 0 0 0)))
    (setq x-vector (normalize-vector vector))
    (setq y-vector (normalize-vector (get-orthogonal x-vector)))
    (setq z-vector (normalize-vector (v* x-vector y-vector)))
    (setq return-matrix (make-matrix 3 3 (list
                                          y-vector
                                          z-vector
                                          x-vector)))
    ; this was the key!!!(or more like, my 過失 for not realizing it was caused by this)
    ; I'm not sure why it worked without the transposition in the first place, but I'm not complaining
    (setq return-matrix (transpose return-matrix))
    return-matrix))

; clash-point: position of swords' clash point
; blade-vector : vector from my hand to my clash point
; computes the destination coordinates for robot arm, of which the position is where the hand should be based on
; the clash-point and blade-vector, and its x axis being parallel to the blade-vector.
(defun get-hand-coordinates (clash-point blade-vector)
  (setq destination (make-coords :pos (v- clash-point blade-vector) :rot (get-matrix blade-vector)))
  destination
  )

(defun parry (their-pos their-vector)
  (let ((my-vector (v* their-vector (float-vector 1 0 0)))
        )
        (setq my-vector (normalize-vector my-vector))
    (setq len 200)
    (setq my-vector (float-vector
                     (* len (elt my-vector 0))
                     (* len (elt my-vector 1))
                     (* len (elt my-vector 2))))
    ; invert my-vector if necessary
    (setq hand-c (get-hand-coordinates their-pos my-vector))
    (send *dxl-armed-turtlebot* :inverse-kinematics hand-c :rotation-axis :z :revert-if-fail nil)


    ))

; states
; 0: default
; 1: parry
; 2: attack!
(setq state 1)

(setq clash-coords (make-coords :pos (float-vector 800 100 700)))
(setq blade-vector (float-vector 400 0 400))

; (defun cc (x y z)
;   (setq clash-coords (make-coords :pos (v+ (send clash-coords :pos) (float-vector x y z)))))
; (defun cb (x y z)
;   (setq blade-vector (v+ blade-vector (float-vector x y z))))

(setq r 50)
(setq h (sqrt (- (* 250 250) (* r r))))
(setq i 0)

(while (ros::ok)
  (setq i (+ i 1))
 (ros::spin-once)
 (x::window-main-one)
 (if
   (= demo 1)
   (progn
    (setq *checker-coords* (make-coords :pos (float-vector (+ (* 150 (cos (deg2rad (* 5 i)))) 450) (+ (* 50 (cos (deg2rad (* 8 i)))) 100) (+ (* 100 (cos (deg2rad (* 7 i)))) 400))
                                        :Euler (list (* 0.7 (cos (deg2rad (* 4 i)))) (/ -3.14 2) (* 0.7 (cos (deg2rad (* 3 i))))))
          )))

  (setq their-vector (transform (send *checker-coords* :rot) (float-vector 300 0 0)))
  (setq their-pos (v+ (send *checker-coords* :pos) their-vector))

 (cond
   ((< (elt their-pos 0) 570)
    (setq state 1))
   (t
    (setq state 0)))
 (cond
   ((= state 0)
    ; go forward
    ; (mostly copied from follow-checker.l)

    (setq x (elt their-pos 0))
    (setq y (elt their-pos 1))
    (setq theta (* (/ 180 3.14) (- 0 (atan (/ y x)))))

    (if (= real 1)
      (cond
      ((> theta 20)
       (send *ri* :go-velocity 0 0 -10)
         (print "move left")
       )
      ((< theta -20)
        (send *ri* :go-velocity 0 0 10)
        (print "move right")
        )
      (t
        (send *ri* :go-velocity 0.05 0 0)
        (print "move forward")
       )
     )))
   ((= state 1)
    ; parry

    (parry their-pos their-vector)


    )
   ((= state 2)
    ; attack

    )
   (t
    ; nothing
    ))
 ; (send cube :move-to )

 ; draw sword
 (send sword-cube :move-to (make-coords :pos (send *dxl-armed-turtlebot* :arm :end-coords :worldpos) :rot (send *dxl-armed-turtlebot* :arm :end-coords :worldrot)) :world)
 (send sword-cube :translate #f(0 0 150))
 (send cube :move-to *checker-coords* :world)
 (send cube :translate #f(150 0 0))

 (send *irtviewer* :draw-objects)
 (if (= real 1)
   (progn
    (send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 100)
    (send *ri* :wait-interpolation-smooth 100)
    ))
 (ros::sleep)
 )
