; Auto-generated. Do not edit!


(cl:in-package all_om-msg)


;//! \htmlinclude points.msg.html

(cl:defclass <points> (roslisp-msg-protocol:ros-message)
  ((num
    :reader num
    :initarg :num
    :type cl:string
    :initform "")
   (s
    :reader s
    :initarg :s
    :type cl:integer
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type (cl:vector all_om-msg:point)
   :initform (cl:make-array 0 :element-type 'all_om-msg:point :initial-element (cl:make-instance 'all_om-msg:point))))
)

(cl:defclass points (<points>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <points>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'points)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name all_om-msg:<points> is deprecated: use all_om-msg:points instead.")))

(cl:ensure-generic-function 'num-val :lambda-list '(m))
(cl:defmethod num-val ((m <points>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader all_om-msg:num-val is deprecated.  Use all_om-msg:num instead.")
  (num m))

(cl:ensure-generic-function 's-val :lambda-list '(m))
(cl:defmethod s-val ((m <points>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader all_om-msg:s-val is deprecated.  Use all_om-msg:s instead.")
  (s m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <points>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader all_om-msg:data-val is deprecated.  Use all_om-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <points>) ostream)
  "Serializes a message object of type '<points>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'num))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'num))
  (cl:let* ((signed (cl:slot-value msg 's)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <points>) istream)
  "Deserializes a message object of type '<points>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'num) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 's) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'all_om-msg:point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<points>)))
  "Returns string type for a message object of type '<points>"
  "all_om/points")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'points)))
  "Returns string type for a message object of type 'points"
  "all_om/points")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<points>)))
  "Returns md5sum for a message object of type '<points>"
  "bc1d3270b5a8918bda079385939e1a51")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'points)))
  "Returns md5sum for a message object of type 'points"
  "bc1d3270b5a8918bda079385939e1a51")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<points>)))
  "Returns full string definition for message of type '<points>"
  (cl:format cl:nil "string num~%int32 s~%point[] data~%~%================================================================================~%MSG: all_om/point~%int32 index~%float32 x~%float32 y~%float32 z~%float32 distance~%float32 degree~%int32 x_size~%int32 y_size~%string object~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'points)))
  "Returns full string definition for message of type 'points"
  (cl:format cl:nil "string num~%int32 s~%point[] data~%~%================================================================================~%MSG: all_om/point~%int32 index~%float32 x~%float32 y~%float32 z~%float32 distance~%float32 degree~%int32 x_size~%int32 y_size~%string object~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <points>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'num))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <points>))
  "Converts a ROS message object to a list"
  (cl:list 'points
    (cl:cons ':num (num msg))
    (cl:cons ':s (s msg))
    (cl:cons ':data (data msg))
))
