; Auto-generated. Do not edit!


(cl:in-package ltme_node-srv)


;//! \htmlinclude QueryHardwareVersion-request.msg.html

(cl:defclass <QueryHardwareVersion-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass QueryHardwareVersion-request (<QueryHardwareVersion-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <QueryHardwareVersion-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'QueryHardwareVersion-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ltme_node-srv:<QueryHardwareVersion-request> is deprecated: use ltme_node-srv:QueryHardwareVersion-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <QueryHardwareVersion-request>) ostream)
  "Serializes a message object of type '<QueryHardwareVersion-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <QueryHardwareVersion-request>) istream)
  "Deserializes a message object of type '<QueryHardwareVersion-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<QueryHardwareVersion-request>)))
  "Returns string type for a service object of type '<QueryHardwareVersion-request>"
  "ltme_node/QueryHardwareVersionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'QueryHardwareVersion-request)))
  "Returns string type for a service object of type 'QueryHardwareVersion-request"
  "ltme_node/QueryHardwareVersionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<QueryHardwareVersion-request>)))
  "Returns md5sum for a message object of type '<QueryHardwareVersion-request>"
  "77f2d4a0768554de8b833ceceda9c80a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'QueryHardwareVersion-request)))
  "Returns md5sum for a message object of type 'QueryHardwareVersion-request"
  "77f2d4a0768554de8b833ceceda9c80a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<QueryHardwareVersion-request>)))
  "Returns full string definition for message of type '<QueryHardwareVersion-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'QueryHardwareVersion-request)))
  "Returns full string definition for message of type 'QueryHardwareVersion-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <QueryHardwareVersion-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <QueryHardwareVersion-request>))
  "Converts a ROS message object to a list"
  (cl:list 'QueryHardwareVersion-request
))
;//! \htmlinclude QueryHardwareVersion-response.msg.html

(cl:defclass <QueryHardwareVersion-response> (roslisp-msg-protocol:ros-message)
  ((hardware_version
    :reader hardware_version
    :initarg :hardware_version
    :type cl:string
    :initform ""))
)

(cl:defclass QueryHardwareVersion-response (<QueryHardwareVersion-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <QueryHardwareVersion-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'QueryHardwareVersion-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ltme_node-srv:<QueryHardwareVersion-response> is deprecated: use ltme_node-srv:QueryHardwareVersion-response instead.")))

(cl:ensure-generic-function 'hardware_version-val :lambda-list '(m))
(cl:defmethod hardware_version-val ((m <QueryHardwareVersion-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ltme_node-srv:hardware_version-val is deprecated.  Use ltme_node-srv:hardware_version instead.")
  (hardware_version m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <QueryHardwareVersion-response>) ostream)
  "Serializes a message object of type '<QueryHardwareVersion-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'hardware_version))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'hardware_version))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <QueryHardwareVersion-response>) istream)
  "Deserializes a message object of type '<QueryHardwareVersion-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'hardware_version) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'hardware_version) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<QueryHardwareVersion-response>)))
  "Returns string type for a service object of type '<QueryHardwareVersion-response>"
  "ltme_node/QueryHardwareVersionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'QueryHardwareVersion-response)))
  "Returns string type for a service object of type 'QueryHardwareVersion-response"
  "ltme_node/QueryHardwareVersionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<QueryHardwareVersion-response>)))
  "Returns md5sum for a message object of type '<QueryHardwareVersion-response>"
  "77f2d4a0768554de8b833ceceda9c80a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'QueryHardwareVersion-response)))
  "Returns md5sum for a message object of type 'QueryHardwareVersion-response"
  "77f2d4a0768554de8b833ceceda9c80a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<QueryHardwareVersion-response>)))
  "Returns full string definition for message of type '<QueryHardwareVersion-response>"
  (cl:format cl:nil "string hardware_version~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'QueryHardwareVersion-response)))
  "Returns full string definition for message of type 'QueryHardwareVersion-response"
  (cl:format cl:nil "string hardware_version~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <QueryHardwareVersion-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'hardware_version))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <QueryHardwareVersion-response>))
  "Converts a ROS message object to a list"
  (cl:list 'QueryHardwareVersion-response
    (cl:cons ':hardware_version (hardware_version msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'QueryHardwareVersion)))
  'QueryHardwareVersion-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'QueryHardwareVersion)))
  'QueryHardwareVersion-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'QueryHardwareVersion)))
  "Returns string type for a service object of type '<QueryHardwareVersion>"
  "ltme_node/QueryHardwareVersion")