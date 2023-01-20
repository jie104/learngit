;; Auto-generated. Do not edit!


(when (boundp 'ltme_node::QueryFirmwareVersion)
  (if (not (find-package "LTME_NODE"))
    (make-package "LTME_NODE"))
  (shadow 'QueryFirmwareVersion (find-package "LTME_NODE")))
(unless (find-package "LTME_NODE::QUERYFIRMWAREVERSION")
  (make-package "LTME_NODE::QUERYFIRMWAREVERSION"))
(unless (find-package "LTME_NODE::QUERYFIRMWAREVERSIONREQUEST")
  (make-package "LTME_NODE::QUERYFIRMWAREVERSIONREQUEST"))
(unless (find-package "LTME_NODE::QUERYFIRMWAREVERSIONRESPONSE")
  (make-package "LTME_NODE::QUERYFIRMWAREVERSIONRESPONSE"))

(in-package "ROS")





(defclass ltme_node::QueryFirmwareVersionRequest
  :super ros::object
  :slots ())

(defmethod ltme_node::QueryFirmwareVersionRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass ltme_node::QueryFirmwareVersionResponse
  :super ros::object
  :slots (_firmware_version ))

(defmethod ltme_node::QueryFirmwareVersionResponse
  (:init
   (&key
    ((:firmware_version __firmware_version) "")
    )
   (send-super :init)
   (setq _firmware_version (string __firmware_version))
   self)
  (:firmware_version
   (&optional __firmware_version)
   (if __firmware_version (setq _firmware_version __firmware_version)) _firmware_version)
  (:serialization-length
   ()
   (+
    ;; string _firmware_version
    4 (length _firmware_version)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _firmware_version
       (write-long (length _firmware_version) s) (princ _firmware_version s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _firmware_version
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _firmware_version (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass ltme_node::QueryFirmwareVersion
  :super ros::object
  :slots ())

(setf (get ltme_node::QueryFirmwareVersion :md5sum-) "968367e081bb6dba33b3daf3e01dab62")
(setf (get ltme_node::QueryFirmwareVersion :datatype-) "ltme_node/QueryFirmwareVersion")
(setf (get ltme_node::QueryFirmwareVersion :request) ltme_node::QueryFirmwareVersionRequest)
(setf (get ltme_node::QueryFirmwareVersion :response) ltme_node::QueryFirmwareVersionResponse)

(defmethod ltme_node::QueryFirmwareVersionRequest
  (:response () (instance ltme_node::QueryFirmwareVersionResponse :init)))

(setf (get ltme_node::QueryFirmwareVersionRequest :md5sum-) "968367e081bb6dba33b3daf3e01dab62")
(setf (get ltme_node::QueryFirmwareVersionRequest :datatype-) "ltme_node/QueryFirmwareVersionRequest")
(setf (get ltme_node::QueryFirmwareVersionRequest :definition-)
      "---
string firmware_version

")

(setf (get ltme_node::QueryFirmwareVersionResponse :md5sum-) "968367e081bb6dba33b3daf3e01dab62")
(setf (get ltme_node::QueryFirmwareVersionResponse :datatype-) "ltme_node/QueryFirmwareVersionResponse")
(setf (get ltme_node::QueryFirmwareVersionResponse :definition-)
      "---
string firmware_version

")



(provide :ltme_node/QueryFirmwareVersion "968367e081bb6dba33b3daf3e01dab62")


