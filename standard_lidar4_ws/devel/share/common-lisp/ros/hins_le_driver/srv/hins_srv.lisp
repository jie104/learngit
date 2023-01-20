; Auto-generated. Do not edit!


(cl:in-package hins_le_driver-srv)


;//! \htmlinclude hins_srv-request.msg.html

(cl:defclass <hins_srv-request> (roslisp-msg-protocol:ros-message)
  ((channel
    :reader channel
    :initarg :channel
    :type cl:integer
    :initform 0))
)

(cl:defclass hins_srv-request (<hins_srv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <hins_srv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'hins_srv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hins_le_driver-srv:<hins_srv-request> is deprecated: use hins_le_driver-srv:hins_srv-request instead.")))

(cl:ensure-generic-function 'channel-val :lambda-list '(m))
(cl:defmethod channel-val ((m <hins_srv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hins_le_driver-srv:channel-val is deprecated.  Use hins_le_driver-srv:channel instead.")
  (channel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <hins_srv-request>) ostream)
  "Serializes a message object of type '<hins_srv-request>"
  (cl:let* ((signed (cl:slot-value msg 'channel)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <hins_srv-request>) istream)
  "Deserializes a message object of type '<hins_srv-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'channel) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<hins_srv-request>)))
  "Returns string type for a service object of type '<hins_srv-request>"
  "hins_le_driver/hins_srvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'hins_srv-request)))
  "Returns string type for a service object of type 'hins_srv-request"
  "hins_le_driver/hins_srvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<hins_srv-request>)))
  "Returns md5sum for a message object of type '<hins_srv-request>"
  "1b7a8b465a26f7d507ff8e5984967a01")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'hins_srv-request)))
  "Returns md5sum for a message object of type 'hins_srv-request"
  "1b7a8b465a26f7d507ff8e5984967a01")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<hins_srv-request>)))
  "Returns full string definition for message of type '<hins_srv-request>"
  (cl:format cl:nil "int64 channel~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'hins_srv-request)))
  "Returns full string definition for message of type 'hins_srv-request"
  (cl:format cl:nil "int64 channel~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <hins_srv-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <hins_srv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'hins_srv-request
    (cl:cons ':channel (channel msg))
))
;//! \htmlinclude hins_srv-response.msg.html

(cl:defclass <hins_srv-response> (roslisp-msg-protocol:ros-message)
  ((area1
    :reader area1
    :initarg :area1
    :type cl:boolean
    :initform cl:nil)
   (area2
    :reader area2
    :initarg :area2
    :type cl:boolean
    :initform cl:nil)
   (area3
    :reader area3
    :initarg :area3
    :type cl:boolean
    :initform cl:nil)
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass hins_srv-response (<hins_srv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <hins_srv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'hins_srv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hins_le_driver-srv:<hins_srv-response> is deprecated: use hins_le_driver-srv:hins_srv-response instead.")))

(cl:ensure-generic-function 'area1-val :lambda-list '(m))
(cl:defmethod area1-val ((m <hins_srv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hins_le_driver-srv:area1-val is deprecated.  Use hins_le_driver-srv:area1 instead.")
  (area1 m))

(cl:ensure-generic-function 'area2-val :lambda-list '(m))
(cl:defmethod area2-val ((m <hins_srv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hins_le_driver-srv:area2-val is deprecated.  Use hins_le_driver-srv:area2 instead.")
  (area2 m))

(cl:ensure-generic-function 'area3-val :lambda-list '(m))
(cl:defmethod area3-val ((m <hins_srv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hins_le_driver-srv:area3-val is deprecated.  Use hins_le_driver-srv:area3 instead.")
  (area3 m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <hins_srv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hins_le_driver-srv:success-val is deprecated.  Use hins_le_driver-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <hins_srv-response>) ostream)
  "Serializes a message object of type '<hins_srv-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'area1) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'area2) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'area3) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <hins_srv-response>) istream)
  "Deserializes a message object of type '<hins_srv-response>"
    (cl:setf (cl:slot-value msg 'area1) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'area2) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'area3) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<hins_srv-response>)))
  "Returns string type for a service object of type '<hins_srv-response>"
  "hins_le_driver/hins_srvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'hins_srv-response)))
  "Returns string type for a service object of type 'hins_srv-response"
  "hins_le_driver/hins_srvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<hins_srv-response>)))
  "Returns md5sum for a message object of type '<hins_srv-response>"
  "1b7a8b465a26f7d507ff8e5984967a01")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'hins_srv-response)))
  "Returns md5sum for a message object of type 'hins_srv-response"
  "1b7a8b465a26f7d507ff8e5984967a01")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<hins_srv-response>)))
  "Returns full string definition for message of type '<hins_srv-response>"
  (cl:format cl:nil "bool area1~%bool area2~%bool area3~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'hins_srv-response)))
  "Returns full string definition for message of type 'hins_srv-response"
  (cl:format cl:nil "bool area1~%bool area2~%bool area3~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <hins_srv-response>))
  (cl:+ 0
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <hins_srv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'hins_srv-response
    (cl:cons ':area1 (area1 msg))
    (cl:cons ':area2 (area2 msg))
    (cl:cons ':area3 (area3 msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'hins_srv)))
  'hins_srv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'hins_srv)))
  'hins_srv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'hins_srv)))
  "Returns string type for a service object of type '<hins_srv>"
  "hins_le_driver/hins_srv")