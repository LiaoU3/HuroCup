; Auto-generated. Do not edit!


(cl:in-package robot_msgs-msg)


;//! \htmlinclude HeadStatus.msg.html

(cl:defclass <HeadStatus> (roslisp-msg-protocol:ros-message)
  ((pan
    :reader pan
    :initarg :pan
    :type cl:float
    :initform 0.0)
   (tilt
    :reader tilt
    :initarg :tilt
    :type cl:float
    :initform 0.0))
)

(cl:defclass HeadStatus (<HeadStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HeadStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HeadStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_msgs-msg:<HeadStatus> is deprecated: use robot_msgs-msg:HeadStatus instead.")))

(cl:ensure-generic-function 'pan-val :lambda-list '(m))
(cl:defmethod pan-val ((m <HeadStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:pan-val is deprecated.  Use robot_msgs-msg:pan instead.")
  (pan m))

(cl:ensure-generic-function 'tilt-val :lambda-list '(m))
(cl:defmethod tilt-val ((m <HeadStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msgs-msg:tilt-val is deprecated.  Use robot_msgs-msg:tilt instead.")
  (tilt m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HeadStatus>) ostream)
  "Serializes a message object of type '<HeadStatus>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pan))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tilt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HeadStatus>) istream)
  "Deserializes a message object of type '<HeadStatus>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pan) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tilt) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HeadStatus>)))
  "Returns string type for a message object of type '<HeadStatus>"
  "robot_msgs/HeadStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HeadStatus)))
  "Returns string type for a message object of type 'HeadStatus"
  "robot_msgs/HeadStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HeadStatus>)))
  "Returns md5sum for a message object of type '<HeadStatus>"
  "938e11f380abc0513a5b7367d0f157bf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HeadStatus)))
  "Returns md5sum for a message object of type 'HeadStatus"
  "938e11f380abc0513a5b7367d0f157bf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HeadStatus>)))
  "Returns full string definition for message of type '<HeadStatus>"
  (cl:format cl:nil "float32 pan~%float32 tilt~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HeadStatus)))
  "Returns full string definition for message of type 'HeadStatus"
  (cl:format cl:nil "float32 pan~%float32 tilt~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HeadStatus>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HeadStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'HeadStatus
    (cl:cons ':pan (pan msg))
    (cl:cons ':tilt (tilt msg))
))
