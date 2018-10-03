;; Auto-generated. Do not edit!


(when (boundp 'bebop_brain_drone_race::Error)
  (if (not (find-package "BEBOP_BRAIN_DRONE_RACE"))
    (make-package "BEBOP_BRAIN_DRONE_RACE"))
  (shadow 'Error (find-package "BEBOP_BRAIN_DRONE_RACE")))
(unless (find-package "BEBOP_BRAIN_DRONE_RACE::ERROR")
  (make-package "BEBOP_BRAIN_DRONE_RACE::ERROR"))

(in-package "ROS")
;;//! \htmlinclude Error.msg.html


(defclass bebop_brain_drone_race::Error
  :super ros::object
  :slots (_y _z ))

(defmethod bebop_brain_drone_race::Error
  (:init
   (&key
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    )
   (send-super :init)
   (setq _y (float __y))
   (setq _z (float __z))
   self)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:serialization-length
   ()
   (+
    ;; float32 _y
    4
    ;; float32 _z
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z
       (sys::poke _z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z
     (setq _z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get bebop_brain_drone_race::Error :md5sum-) "620052061599731a9bd2b8c4e407c781")
(setf (get bebop_brain_drone_race::Error :datatype-) "bebop_brain_drone_race/Error")
(setf (get bebop_brain_drone_race::Error :definition-)
      "float32 y
float32 z

")



(provide :bebop_brain_drone_race/Error "620052061599731a9bd2b8c4e407c781")


