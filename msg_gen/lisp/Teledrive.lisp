; Auto-generated. Do not edit!


(cl:in-package teledrive-msg)


;//! \htmlinclude Teledrive.msg.html

(cl:defclass <Teledrive> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (twist
    :reader twist
    :initarg :twist
    :type geometry_msgs-msg:Twist
    :initform (cl:make-instance 'geometry_msgs-msg:Twist))
   (op_linear
    :reader op_linear
    :initarg :op_linear
    :type cl:float
    :initform 0.0)
   (op_angular
    :reader op_angular
    :initarg :op_angular
    :type cl:float
    :initform 0.0))
)

(cl:defclass Teledrive (<Teledrive>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Teledrive>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Teledrive)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name teledrive-msg:<Teledrive> is deprecated: use teledrive-msg:Teledrive instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Teledrive>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader teledrive-msg:header-val is deprecated.  Use teledrive-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'twist-val :lambda-list '(m))
(cl:defmethod twist-val ((m <Teledrive>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader teledrive-msg:twist-val is deprecated.  Use teledrive-msg:twist instead.")
  (twist m))

(cl:ensure-generic-function 'op_linear-val :lambda-list '(m))
(cl:defmethod op_linear-val ((m <Teledrive>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader teledrive-msg:op_linear-val is deprecated.  Use teledrive-msg:op_linear instead.")
  (op_linear m))

(cl:ensure-generic-function 'op_angular-val :lambda-list '(m))
(cl:defmethod op_angular-val ((m <Teledrive>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader teledrive-msg:op_angular-val is deprecated.  Use teledrive-msg:op_angular instead.")
  (op_angular m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Teledrive>) ostream)
  "Serializes a message object of type '<Teledrive>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'twist) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'op_linear))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'op_angular))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Teledrive>) istream)
  "Deserializes a message object of type '<Teledrive>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'twist) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'op_linear) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'op_angular) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Teledrive>)))
  "Returns string type for a message object of type '<Teledrive>"
  "teledrive/Teledrive")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Teledrive)))
  "Returns string type for a message object of type 'Teledrive"
  "teledrive/Teledrive")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Teledrive>)))
  "Returns md5sum for a message object of type '<Teledrive>"
  "b30409c617b49806737c187ab173e37a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Teledrive)))
  "Returns md5sum for a message object of type 'Teledrive"
  "b30409c617b49806737c187ab173e37a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Teledrive>)))
  "Returns full string definition for message of type '<Teledrive>"
  (cl:format cl:nil "Header header~%~%geometry_msgs/Twist twist~%float32 op_linear~%float32 op_angular~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Teledrive)))
  "Returns full string definition for message of type 'Teledrive"
  (cl:format cl:nil "Header header~%~%geometry_msgs/Twist twist~%float32 op_linear~%float32 op_angular~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Teledrive>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'twist))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Teledrive>))
  "Converts a ROS message object to a list"
  (cl:list 'Teledrive
    (cl:cons ':header (header msg))
    (cl:cons ':twist (twist msg))
    (cl:cons ':op_linear (op_linear msg))
    (cl:cons ':op_angular (op_angular msg))
))
