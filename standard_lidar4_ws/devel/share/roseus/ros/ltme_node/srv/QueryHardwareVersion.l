;; Auto-generated. Do not edit!


(when (boundp 'ltme_node::QueryHardwareVersion)
  (if (not (find-package "LTME_NODE"))
    (make-package "LTME_NODE"))
  (shadow 'QueryHardwareVersion (find-package "LTME_NODE")))
(unless (find-package "LTME_NODE::QUERYHARDWAREVERSION")
  (make-package "LTME_NODE::QUERYHARDWAREVERSION"))
(unless (find-package "LTME_NODE::QUERYHARDWAREVERSIONREQUEST")
  (make-package "LTME_NODE::QUERYHARDWAREVERSIONREQUEST"))
(unless (find-package "LTME_NODE::QUERYHARDWAREVERSIONRESPONSE")
  (make-package "LTME_NODE::QUERYHARDWAREVERSIONRESPONSE"))

(in-package "ROS")





(defclass ltme_node::QueryHardwareVersionRequest
  :super ros::object
  :slots ())

(defmethod ltme_node::QueryHardwareVersionRequest
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

(defclass ltme_node::QueryHardwareVersionResponse
  :super ros::object
  :slots (_hardware_version ))

(defmethod ltme_node::QueryHardwareVersionResponse
  (:init
   (&key
    ((:hardware_version __hardware_version) "")
    )
   (send-super :init)
   (setq _hardware_version (string __hardware_version))
   self)
  (:hardware_version
   (&optional __hardware_version)
   (if __hardware_version (setq _hardware_version __hardware_version)) _hardware_version)
  (:serialization-length
   ()
   (+
    ;; string _hardware_version
    4 (length _hardware_version)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _hardware_version
       (write-long (length _hardware_version) s) (princ _hardware_version s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _hardware_version
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _hardware_version (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass ltme_node::QueryHardwareVersion
  :super ros::object
  :slots ())

(setf (get ltme_node::QueryHardwareVersion :md5sum-) "77f2d4a0768554de8b833ceceda9c80a")
(setf (get ltme_node::QueryHardwareVersion :datatype-) "ltme_node/QueryHardwareVersion")
(setf (get ltme_node::QueryHardwareVersion :request) ltme_node::QueryHardwareVersionRequest)
(setf (get ltme_node::QueryHardwareVersion :response) ltme_node::QueryHardwareVersionResponse)

(defmethod ltme_node::QueryHardwareVersionRequest
  (:response () (instance ltme_node::QueryHardwareVersionResponse :init)))

(setf (get ltme_node::QueryHardwareVersionRequest :md5sum-) "77f2d4a0768554de8b833ceceda9c80a")
(setf (get ltme_node::QueryHardwareVersionRequest :datatype-) "ltme_node/QueryHardwareVersionRequest")
(setf (get ltme_node::QueryHardwareVersionRequest :definition-)
      "---
string hardware_version

")

(setf (get ltme_node::QueryHardwareVersionResponse :md5sum-) "77f2d4a0768554de8b833ceceda9c80a")
(setf (get ltme_node::QueryHardwareVersionResponse :datatype-) "ltme_node/QueryHardwareVersionResponse")
(setf (get ltme_node::QueryHardwareVersionResponse :definition-)
      "---
string hardware_version

")



(provide :ltme_node/QueryHardwareVersion "77f2d4a0768554de8b833ceceda9c80a")


