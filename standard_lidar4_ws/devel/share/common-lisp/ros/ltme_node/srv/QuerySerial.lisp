; Auto-generated. Do not edit!


(cl:in-package ltme_node-srv)


;//! \htmlinclude QuerySerial-request.msg.html

(cl:defclass <QuerySerial-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass QuerySerial-request (<QuerySerial-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <QuerySerial-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'QuerySerial-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ltme_node-srv:<QuerySerial-request> is deprecated: use ltme_node-srv:QuerySerial-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <QuerySerial-request>) ostream)
  "Serializes a message object of type '<QuerySerial-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <QuerySerial-request>) istream)
  "Deserializes a message object of type '<QuerySerial-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<QuerySerial-request>)))
  "Returns string type for a service object of type '<QuerySerial-request>"
  "ltme_node/QuerySerialRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'QuerySerial-request)))
  "Returns string type for a service object of type 'QuerySerial-request"
  "ltme_node/QuerySerialRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<QuerySerial-request>)))
  "Returns md5sum for a message object of type '<QuerySerial-request>"
  "fca40cf463282a80db4e2037c8a61741")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'QuerySerial-request)))
  "Returns md5sum for a message object of type 'QuerySerial-request"
  "fca40cf463282a80db4e2037c8a61741")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<QuerySerial-request>)))
  "Returns full string definition for message of type '<QuerySerial-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'QuerySerial-request)))
  "Returns full string definition for message of type 'QuerySerial-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <QuerySerial-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <QuerySerial-request>))
  "Converts a ROS message object to a list"
  (cl:list 'QuerySerial-request
))
;//! \htmlinclude QuerySerial-response.msg.html

(cl:defclass <QuerySerial-response> (roslisp-msg-protocol:ros-message)
  ((serial
    :reader serial
    :initarg :serial
    :type cl:string
    :initform ""))
)

(cl:defclass QuerySerial-response (<QuerySerial-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <QuerySerial-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'QuerySerial-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ltme_node-srv:<QuerySerial-response> is deprecated: use ltme_node-srv:QuerySerial-response instead.")))

(cl:ensure-generic-function 'serial-val :lambda-list '(m))
(cl:defmethod serial-val ((m <QuerySerial-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ltme_node-srv:serial-val is deprecated.  Use ltme_node-srv:serial instead.")
  (serial m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <QuerySerial-response>) ostream)
  "Serializes a message object of type '<QuerySerial-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'serial))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'serial))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <QuerySerial-response>) istream)
  "Deserializes a message object of type '<QuerySerial-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'serial) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'serial) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<QuerySerial-response>)))
  "Returns string type for a service object of type '<QuerySerial-response>"
  "ltme_node/QuerySerialResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'QuerySerial-response)))
  "Returns string type for a service object of type 'QuerySerial-response"
  "ltme_node/QuerySerialResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<QuerySerial-response>)))
  "Returns md5sum for a message object of type '<QuerySerial-response>"
  "fca40cf463282a80db4e2037c8a61741")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'QuerySerial-response)))
  "Returns md5sum for a message object of type 'QuerySerial-response"
  "fca40cf463282a80db4e2037c8a61741")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<QuerySerial-response>)))
  "Returns full string definition for message of type '<QuerySerial-response>"
  (cl:format cl:nil "string serial~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'QuerySerial-response)))
  "Returns full string definition for message of type 'QuerySerial-response"
  (cl:format cl:nil "string serial~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <QuerySerial-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'serial))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <QuerySerial-response>))
  "Converts a ROS message object to a list"
  (cl:list 'QuerySerial-response
    (cl:cons ':serial (serial msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'QuerySerial)))
  'QuerySerial-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'QuerySerial)))
  'QuerySerial-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'QuerySerial)))
  "Returns string type for a service object of type '<QuerySerial>"
  "ltme_node/QuerySerial")