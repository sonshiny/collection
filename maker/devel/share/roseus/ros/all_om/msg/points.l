;; Auto-generated. Do not edit!


(when (boundp 'all_om::points)
  (if (not (find-package "ALL_OM"))
    (make-package "ALL_OM"))
  (shadow 'points (find-package "ALL_OM")))
(unless (find-package "ALL_OM::POINTS")
  (make-package "ALL_OM::POINTS"))

(in-package "ROS")
;;//! \htmlinclude points.msg.html


(defclass all_om::points
  :super ros::object
  :slots (_num _s _data ))

(defmethod all_om::points
  (:init
   (&key
    ((:num __num) "")
    ((:s __s) 0)
    ((:data __data) ())
    )
   (send-super :init)
   (setq _num (string __num))
   (setq _s (round __s))
   (setq _data __data)
   self)
  (:num
   (&optional __num)
   (if __num (setq _num __num)) _num)
  (:s
   (&optional __s)
   (if __s (setq _s __s)) _s)
  (:data
   (&rest __data)
   (if (keywordp (car __data))
       (send* _data __data)
     (progn
       (if __data (setq _data (car __data)))
       _data)))
  (:serialization-length
   ()
   (+
    ;; string _num
    4 (length _num)
    ;; int32 _s
    4
    ;; all_om/point[] _data
    (apply #'+ (send-all _data :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _num
       (write-long (length _num) s) (princ _num s)
     ;; int32 _s
       (write-long _s s)
     ;; all_om/point[] _data
     (write-long (length _data) s)
     (dolist (elem _data)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _num
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _num (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _s
     (setq _s (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; all_om/point[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (let (r) (dotimes (i n) (push (instance all_om::point :init) r)) r))
     (dolist (elem- _data)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get all_om::points :md5sum-) "bc1d3270b5a8918bda079385939e1a51")
(setf (get all_om::points :datatype-) "all_om/points")
(setf (get all_om::points :definition-)
      "string num
int32 s
point[] data

================================================================================
MSG: all_om/point
int32 index
float32 x
float32 y
float32 z
float32 distance
float32 degree
int32 x_size
int32 y_size
string object

")



(provide :all_om/points "bc1d3270b5a8918bda079385939e1a51")


