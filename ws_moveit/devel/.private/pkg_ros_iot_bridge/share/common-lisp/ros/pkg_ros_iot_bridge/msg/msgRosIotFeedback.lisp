; Auto-generated. Do not edit!


(cl:in-package pkg_ros_iot_bridge-msg)


;//! \htmlinclude msgRosIotFeedback.msg.html

(cl:defclass <msgRosIotFeedback> (roslisp-msg-protocol:ros-message)
  ((percentage_complete
    :reader percentage_complete
    :initarg :percentage_complete
    :type cl:fixnum
    :initform 0))
)

(cl:defclass msgRosIotFeedback (<msgRosIotFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <msgRosIotFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'msgRosIotFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pkg_ros_iot_bridge-msg:<msgRosIotFeedback> is deprecated: use pkg_ros_iot_bridge-msg:msgRosIotFeedback instead.")))

(cl:ensure-generic-function 'percentage_complete-val :lambda-list '(m))
(cl:defmethod percentage_complete-val ((m <msgRosIotFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pkg_ros_iot_bridge-msg:percentage_complete-val is deprecated.  Use pkg_ros_iot_bridge-msg:percentage_complete instead.")
  (percentage_complete m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <msgRosIotFeedback>) ostream)
  "Serializes a message object of type '<msgRosIotFeedback>"
  (cl:let* ((signed (cl:slot-value msg 'percentage_complete)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <msgRosIotFeedback>) istream)
  "Deserializes a message object of type '<msgRosIotFeedback>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'percentage_complete) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<msgRosIotFeedback>)))
  "Returns string type for a message object of type '<msgRosIotFeedback>"
  "pkg_ros_iot_bridge/msgRosIotFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'msgRosIotFeedback)))
  "Returns string type for a message object of type 'msgRosIotFeedback"
  "pkg_ros_iot_bridge/msgRosIotFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<msgRosIotFeedback>)))
  "Returns md5sum for a message object of type '<msgRosIotFeedback>"
  "d5151973e38c593f60bed311537d61df")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'msgRosIotFeedback)))
  "Returns md5sum for a message object of type 'msgRosIotFeedback"
  "d5151973e38c593f60bed311537d61df")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<msgRosIotFeedback>)))
  "Returns full string definition for message of type '<msgRosIotFeedback>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#feedback~%int8 percentage_complete~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'msgRosIotFeedback)))
  "Returns full string definition for message of type 'msgRosIotFeedback"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#feedback~%int8 percentage_complete~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <msgRosIotFeedback>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <msgRosIotFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'msgRosIotFeedback
    (cl:cons ':percentage_complete (percentage_complete msg))
))
