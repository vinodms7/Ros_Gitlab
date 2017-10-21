; Auto-generated. Do not edit!


(cl:in-package ros_ran_num_msg-msg)


;//! \htmlinclude rand_num.msg.html

(cl:defclass <rand_num> (roslisp-msg-protocol:ros-message)
  ((number1
    :reader number1
    :initarg :number1
    :type cl:integer
    :initform 0)
   (number2
    :reader number2
    :initarg :number2
    :type cl:integer
    :initform 0))
)

(cl:defclass rand_num (<rand_num>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <rand_num>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'rand_num)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_ran_num_msg-msg:<rand_num> is deprecated: use ros_ran_num_msg-msg:rand_num instead.")))

(cl:ensure-generic-function 'number1-val :lambda-list '(m))
(cl:defmethod number1-val ((m <rand_num>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_ran_num_msg-msg:number1-val is deprecated.  Use ros_ran_num_msg-msg:number1 instead.")
  (number1 m))

(cl:ensure-generic-function 'number2-val :lambda-list '(m))
(cl:defmethod number2-val ((m <rand_num>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_ran_num_msg-msg:number2-val is deprecated.  Use ros_ran_num_msg-msg:number2 instead.")
  (number2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <rand_num>) ostream)
  "Serializes a message object of type '<rand_num>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'number1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'number1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'number1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'number1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'number2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'number2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'number2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'number2)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <rand_num>) istream)
  "Deserializes a message object of type '<rand_num>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'number1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'number1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'number1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'number1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'number2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'number2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'number2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'number2)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<rand_num>)))
  "Returns string type for a message object of type '<rand_num>"
  "ros_ran_num_msg/rand_num")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rand_num)))
  "Returns string type for a message object of type 'rand_num"
  "ros_ran_num_msg/rand_num")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<rand_num>)))
  "Returns md5sum for a message object of type '<rand_num>"
  "20e27c3e6eee8dfccd76499464b39a05")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'rand_num)))
  "Returns md5sum for a message object of type 'rand_num"
  "20e27c3e6eee8dfccd76499464b39a05")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<rand_num>)))
  "Returns full string definition for message of type '<rand_num>"
  (cl:format cl:nil "uint32 number1~%uint32 number2~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'rand_num)))
  "Returns full string definition for message of type 'rand_num"
  (cl:format cl:nil "uint32 number1~%uint32 number2~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <rand_num>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <rand_num>))
  "Converts a ROS message object to a list"
  (cl:list 'rand_num
    (cl:cons ':number1 (number1 msg))
    (cl:cons ':number2 (number2 msg))
))
