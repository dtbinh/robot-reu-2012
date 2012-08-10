; Auto-generated. Do not edit!


(cl:in-package irobot_nav-srv)


;//! \htmlinclude Status-request.msg.html

(cl:defclass <Status-request> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:string
    :initform ""))
)

(cl:defclass Status-request (<Status-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Status-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Status-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name irobot_nav-srv:<Status-request> is deprecated: use irobot_nav-srv:Status-request instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <Status-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader irobot_nav-srv:status-val is deprecated.  Use irobot_nav-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Status-request>) ostream)
  "Serializes a message object of type '<Status-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Status-request>) istream)
  "Deserializes a message object of type '<Status-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'status) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Status-request>)))
  "Returns string type for a service object of type '<Status-request>"
  "irobot_nav/StatusRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Status-request)))
  "Returns string type for a service object of type 'Status-request"
  "irobot_nav/StatusRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Status-request>)))
  "Returns md5sum for a message object of type '<Status-request>"
  "ec61c102682182b7da93e14e53536e5d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Status-request)))
  "Returns md5sum for a message object of type 'Status-request"
  "ec61c102682182b7da93e14e53536e5d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Status-request>)))
  "Returns full string definition for message of type '<Status-request>"
  (cl:format cl:nil "string status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Status-request)))
  "Returns full string definition for message of type 'Status-request"
  (cl:format cl:nil "string status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Status-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'status))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Status-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Status-request
    (cl:cons ':status (status msg))
))
;//! \htmlinclude Status-response.msg.html

(cl:defclass <Status-response> (roslisp-msg-protocol:ros-message)
  ((acknowledgment
    :reader acknowledgment
    :initarg :acknowledgment
    :type cl:string
    :initform ""))
)

(cl:defclass Status-response (<Status-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Status-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Status-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name irobot_nav-srv:<Status-response> is deprecated: use irobot_nav-srv:Status-response instead.")))

(cl:ensure-generic-function 'acknowledgment-val :lambda-list '(m))
(cl:defmethod acknowledgment-val ((m <Status-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader irobot_nav-srv:acknowledgment-val is deprecated.  Use irobot_nav-srv:acknowledgment instead.")
  (acknowledgment m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Status-response>) ostream)
  "Serializes a message object of type '<Status-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'acknowledgment))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'acknowledgment))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Status-response>) istream)
  "Deserializes a message object of type '<Status-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'acknowledgment) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'acknowledgment) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Status-response>)))
  "Returns string type for a service object of type '<Status-response>"
  "irobot_nav/StatusResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Status-response)))
  "Returns string type for a service object of type 'Status-response"
  "irobot_nav/StatusResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Status-response>)))
  "Returns md5sum for a message object of type '<Status-response>"
  "ec61c102682182b7da93e14e53536e5d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Status-response)))
  "Returns md5sum for a message object of type 'Status-response"
  "ec61c102682182b7da93e14e53536e5d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Status-response>)))
  "Returns full string definition for message of type '<Status-response>"
  (cl:format cl:nil "string acknowledgment~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Status-response)))
  "Returns full string definition for message of type 'Status-response"
  (cl:format cl:nil "string acknowledgment~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Status-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'acknowledgment))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Status-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Status-response
    (cl:cons ':acknowledgment (acknowledgment msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Status)))
  'Status-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Status)))
  'Status-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Status)))
  "Returns string type for a service object of type '<Status>"
  "irobot_nav/Status")