; Auto-generated. Do not edit!


(cl:in-package x_msgs-srv)


;//! \htmlinclude X_Command-request.msg.html

(cl:defclass <X_Command-request> (roslisp-msg-protocol:ros-message)
  ((command
    :reader command
    :initarg :command
    :type cl:fixnum
    :initform 0))
)

(cl:defclass X_Command-request (<X_Command-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <X_Command-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'X_Command-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name x_msgs-srv:<X_Command-request> is deprecated: use x_msgs-srv:X_Command-request instead.")))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <X_Command-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader x_msgs-srv:command-val is deprecated.  Use x_msgs-srv:command instead.")
  (command m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <X_Command-request>) ostream)
  "Serializes a message object of type '<X_Command-request>"
  (cl:let* ((signed (cl:slot-value msg 'command)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <X_Command-request>) istream)
  "Deserializes a message object of type '<X_Command-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'command) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<X_Command-request>)))
  "Returns string type for a service object of type '<X_Command-request>"
  "x_msgs/X_CommandRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'X_Command-request)))
  "Returns string type for a service object of type 'X_Command-request"
  "x_msgs/X_CommandRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<X_Command-request>)))
  "Returns md5sum for a message object of type '<X_Command-request>"
  "28a37da2e074d60a86413b1466cbc6e6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'X_Command-request)))
  "Returns md5sum for a message object of type 'X_Command-request"
  "28a37da2e074d60a86413b1466cbc6e6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<X_Command-request>)))
  "Returns full string definition for message of type '<X_Command-request>"
  (cl:format cl:nil "int8 command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'X_Command-request)))
  "Returns full string definition for message of type 'X_Command-request"
  (cl:format cl:nil "int8 command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <X_Command-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <X_Command-request>))
  "Converts a ROS message object to a list"
  (cl:list 'X_Command-request
    (cl:cons ':command (command msg))
))
;//! \htmlinclude X_Command-response.msg.html

(cl:defclass <X_Command-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0))
)

(cl:defclass X_Command-response (<X_Command-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <X_Command-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'X_Command-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name x_msgs-srv:<X_Command-response> is deprecated: use x_msgs-srv:X_Command-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <X_Command-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader x_msgs-srv:status-val is deprecated.  Use x_msgs-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <X_Command-response>) ostream)
  "Serializes a message object of type '<X_Command-response>"
  (cl:let* ((signed (cl:slot-value msg 'status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <X_Command-response>) istream)
  "Deserializes a message object of type '<X_Command-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<X_Command-response>)))
  "Returns string type for a service object of type '<X_Command-response>"
  "x_msgs/X_CommandResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'X_Command-response)))
  "Returns string type for a service object of type 'X_Command-response"
  "x_msgs/X_CommandResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<X_Command-response>)))
  "Returns md5sum for a message object of type '<X_Command-response>"
  "28a37da2e074d60a86413b1466cbc6e6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'X_Command-response)))
  "Returns md5sum for a message object of type 'X_Command-response"
  "28a37da2e074d60a86413b1466cbc6e6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<X_Command-response>)))
  "Returns full string definition for message of type '<X_Command-response>"
  (cl:format cl:nil "int8 status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'X_Command-response)))
  "Returns full string definition for message of type 'X_Command-response"
  (cl:format cl:nil "int8 status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <X_Command-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <X_Command-response>))
  "Converts a ROS message object to a list"
  (cl:list 'X_Command-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'X_Command)))
  'X_Command-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'X_Command)))
  'X_Command-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'X_Command)))
  "Returns string type for a service object of type '<X_Command>"
  "x_msgs/X_Command")