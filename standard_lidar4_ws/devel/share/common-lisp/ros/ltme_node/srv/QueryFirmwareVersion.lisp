; Auto-generated. Do not edit!


(cl:in-package ltme_node-srv)


;//! \htmlinclude QueryFirmwareVersion-request.msg.html

(cl:defclass <QueryFirmwareVersion-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass QueryFirmwareVersion-request (<QueryFirmwareVersion-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <QueryFirmwareVersion-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'QueryFirmwareVersion-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ltme_node-srv:<QueryFirmwareVersion-request> is deprecated: use ltme_node-srv:QueryFirmwareVersion-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <QueryFirmwareVersion-request>) ostream)
  "Serializes a message object of type '<QueryFirmwareVersion-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <QueryFirmwareVersion-request>) istream)
  "Deserializes a message object of type '<QueryFirmwareVersion-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<QueryFirmwareVersion-request>)))
  "Returns string type for a service object of type '<QueryFirmwareVersion-request>"
  "ltme_node/QueryFirmwareVersionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'QueryFirmwareVersion-request)))
  "Returns string type for a service object of type 'QueryFirmwareVersion-request"
  "ltme_node/QueryFirmwareVersionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<QueryFirmwareVersion-request>)))
  "Returns md5sum for a message object of type '<QueryFirmwareVersion-request>"
  "968367e081bb6dba33b3daf3e01dab62")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'QueryFirmwareVersion-request)))
  "Returns md5sum for a message object of type 'QueryFirmwareVersion-request"
  "968367e081bb6dba33b3daf3e01dab62")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<QueryFirmwareVersion-request>)))
  "Returns full string definition for message of type '<QueryFirmwareVersion-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'QueryFirmwareVersion-request)))
  "Returns full string definition for message of type 'QueryFirmwareVersion-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <QueryFirmwareVersion-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <QueryFirmwareVersion-request>))
  "Converts a ROS message object to a list"
  (cl:list 'QueryFirmwareVersion-request
))
;//! \htmlinclude QueryFirmwareVersion-response.msg.html

(cl:defclass <QueryFirmwareVersion-response> (roslisp-msg-protocol:ros-message)
  ((firmware_version
    :reader firmware_version
    :initarg :firmware_version
    :type cl:string
    :initform ""))
)

(cl:defclass QueryFirmwareVersion-response (<QueryFirmwareVersion-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <QueryFirmwareVersion-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'QueryFirmwareVersion-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ltme_node-srv:<QueryFirmwareVersion-response> is deprecated: use ltme_node-srv:QueryFirmwareVersion-response instead.")))

(cl:ensure-generic-function 'firmware_version-val :lambda-list '(m))
(cl:defmethod firmware_version-val ((m <QueryFirmwareVersion-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ltme_node-srv:firmware_version-val is deprecated.  Use ltme_node-srv:firmware_version instead.")
  (firmware_version m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <QueryFirmwareVersion-response>) ostream)
  "Serializes a message object of type '<QueryFirmwareVersion-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'firmware_version))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'firmware_version))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <QueryFirmwareVersion-response>) istream)
  "Deserializes a message object of type '<QueryFirmwareVersion-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'firmware_version) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'firmware_version) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<QueryFirmwareVersion-response>)))
  "Returns string type for a service object of type '<QueryFirmwareVersion-response>"
  "ltme_node/QueryFirmwareVersionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'QueryFirmwareVersion-response)))
  "Returns string type for a service object of type 'QueryFirmwareVersion-response"
  "ltme_node/QueryFirmwareVersionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<QueryFirmwareVersion-response>)))
  "Returns md5sum for a message object of type '<QueryFirmwareVersion-response>"
  "968367e081bb6dba33b3daf3e01dab62")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'QueryFirmwareVersion-response)))
  "Returns md5sum for a message object of type 'QueryFirmwareVersion-response"
  "968367e081bb6dba33b3daf3e01dab62")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<QueryFirmwareVersion-response>)))
  "Returns full string definition for message of type '<QueryFirmwareVersion-response>"
  (cl:format cl:nil "string firmware_version~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'QueryFirmwareVersion-response)))
  "Returns full string definition for message of type 'QueryFirmwareVersion-response"
  (cl:format cl:nil "string firmware_version~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <QueryFirmwareVersion-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'firmware_version))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <QueryFirmwareVersion-response>))
  "Converts a ROS message object to a list"
  (cl:list 'QueryFirmwareVersion-response
    (cl:cons ':firmware_version (firmware_version msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'QueryFirmwareVersion)))
  'QueryFirmwareVersion-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'QueryFirmwareVersion)))
  'QueryFirmwareVersion-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'QueryFirmwareVersion)))
  "Returns string type for a service object of type '<QueryFirmwareVersion>"
  "ltme_node/QueryFirmwareVersion")