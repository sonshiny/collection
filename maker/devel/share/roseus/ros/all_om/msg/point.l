;; Auto-generated. Do not edit!


(when (boundp 'all_om::point)
  (if (not (find-package "ALL_OM"))
    (make-package "ALL_OM"))
  (shadow 'point (find-package "ALL_OM")))
(unless (find-package "ALL_OM::POINT")
  (make-package "ALL_OM::POINT"))

(in-package "ROS")
;;//! \htmlinclude point.msg.html


(defclass all_om::point
  :super ros::object
  :slots (_index _x _y _z _distance _degree _x_size _y_size _object ))

(defmethod all_om::point
  (:init
   (&key
    ((:index __index) 0)
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:distance __distance) 0.0)
    ((:degree __degree) 0.0)
    ((:x_size __x_size) 0)
    ((:y_size __y_size) 0)
    ((:object __object) "")
    )
   (send-super :init)
   (setq _index (round __index))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _distance (float __distance))
   (setq _degree (float __degree))
   (setq _x_size (round __x_size))
   (setq _y_size (round __y_size))
   (setq _object (string __object))
   self)
  (:index
   (&optional __index)
   (if __index (setq _index __index)) _index)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:degree
   (&optional __degree)
   (if __degree (setq _degree __degree)) _degree)
  (:x_size
   (&optional __x_size)
   (if __x_size (setq _x_size __x_size)) _x_size)
  (:y_size
   (&optional __y_size)
   (if __y_size (setq _y_size __y_size)) _y_size)
  (:object
   (&optional __object)
   (if __object (setq _object __object)) _object)
  (:serialization-length
   ()
   (+
    ;; int32 _index
    4
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _z
    4
    ;; float32 _distance
    4
    ;; float32 _degree
    4
    ;; int32 _x_size
    4
    ;; int32 _y_size
    4
    ;; string _object
    4 (length _object)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _index
       (write-long _index s)
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z
       (sys::poke _z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _distance
       (sys::poke _distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _degree
       (sys::poke _degree (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _x_size
       (write-long _x_size s)
     ;; int32 _y_size
       (write-long _y_size s)
     ;; string _object
       (write-long (length _object) s) (princ _object s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _index
     (setq _index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z
     (setq _z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _distance
     (setq _distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _degree
     (setq _degree (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _x_size
     (setq _x_size (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _y_size
     (setq _y_size (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _object
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _object (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get all_om::point :md5sum-) "0af1c687bcb24ea84d8c64d9f6633331")
(setf (get all_om::point :datatype-) "all_om/point")
(setf (get all_om::point :definition-)
      "int32 index
float32 x
float32 y
float32 z
float32 distance
float32 degree
int32 x_size
int32 y_size
string object

")



(provide :all_om/point "0af1c687bcb24ea84d8c64d9f6633331")


