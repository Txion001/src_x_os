; Auto-generated. Do not edit!


(cl:in-package x_msgs-msg)


;//! \htmlinclude X_Status.msg.html

(cl:defclass <X_Status> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0))
)

(cl:defclass X_Status (<X_Status>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <X_Status>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'X_Status)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name x_msgs-msg:<X_Status> is deprecated: use x_msgs-msg:X_Status instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <X_Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader x_msgs-msg:status-val is deprecated.  Use x_msgs-msg:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <X_Status>) ostream)
  "Serializes a message object of type '<X_Status>"
  (cl:let* ((signed (cl:slot-value msg 'status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <X_Status>) istream)
  "Deserializes a message object of type '<X_Status>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<X_Status>)))
  "Returns string type for a message object of type '<X_Status>"
  "x_msgs/X_Status")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'X_Status)))
  "Returns string type for a message object of type 'X_Status"
  "x_msgs/X_Status")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<X_Status>)))
  "Returns md5sum for a message object of type '<X_Status>"
  "581cc55c12abfc219e446416014f6d0e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'X_Status)))
  "Returns md5sum for a message object of type 'X_Status"
  "581cc55c12abfc219e446416014f6d0e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<X_Status>)))
  "Returns full string definition for message of type '<X_Status>"
  (cl:format cl:nil "int8 status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'X_Status)))
  "Returns full string definition for message of type 'X_Status"
  (cl:format cl:nil "int8 status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <X_Status>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <X_Status>))
  "Converts a ROS message object to a list"
  (cl:list 'X_Status
    (cl:cons ':status (status msg))
))
