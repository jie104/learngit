;; Auto-generated. Do not edit!


(when (boundp 'ltme_node::QuerySerial)
  (if (not (find-package "LTME_NODE"))
    (make-package "LTME_NODE"))
  (shadow 'QuerySerial (find-package "LTME_NODE")))
(unless (find-package "LTME_NODE::QUERYSERIAL")
  (make-package "LTME_NODE::QUERYSERIAL"))
(unless (find-package "LTME_NODE::QUERYSERIALREQUEST")
  (make-package "LTME_NODE::QUERYSERIALREQUEST"))
(unless (find-package "LTME_NODE::QUERYSERIALRESPONSE")
  (make-package "LTME_NODE::QUERYSERIALRESPONSE"))

(in-package "ROS")





(defclass ltme_node::QuerySerialRequest
  :super ros::object
  :slots ())

(defmethod ltme_node::QuerySerialRequest
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

(defclass ltme_node::QuerySerialResponse
  :super ros::object
  :slots (_serial ))

(defmethod ltme_node::QuerySerialResponse
  (:init
   (&key
    ((:serial __serial) "")
    )
   (send-super :init)
   (setq _serial (string __serial))
   self)
  (:serial
   (&optional __serial)
   (if __serial (setq _serial __serial)) _serial)
  (:serialization-length
   ()
   (+
    ;; string _serial
    4 (length _serial)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _serial
       (write-long (length _serial) s) (princ _serial s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _serial
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _serial (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass ltme_node::QuerySerial
  :super ros::object
  :slots ())

(setf (get ltme_node::QuerySerial :md5sum-) "fca40cf463282a80db4e2037c8a61741")
(setf (get ltme_node::QuerySerial :datatype-) "ltme_node/QuerySerial")
(setf (get ltme_node::QuerySerial :request) ltme_node::QuerySerialRequest)
(setf (get ltme_node::QuerySerial :response) ltme_node::QuerySerialResponse)

(defmethod ltme_node::QuerySerialRequest
  (:response () (instance ltme_node::QuerySerialResponse :init)))

(setf (get ltme_node::QuerySerialRequest :md5sum-) "fca40cf463282a80db4e2037c8a61741")
(setf (get ltme_node::QuerySerialRequest :datatype-) "ltme_node/QuerySerialRequest")
(setf (get ltme_node::QuerySerialRequest :definition-)
      "---
string serial

")

(setf (get ltme_node::QuerySerialResponse :md5sum-) "fca40cf463282a80db4e2037c8a61741")
(setf (get ltme_node::QuerySerialResponse :datatype-) "ltme_node/QuerySerialResponse")
(setf (get ltme_node::QuerySerialResponse :definition-)
      "---
string serial

")



(provide :ltme_node/QuerySerial "fca40cf463282a80db4e2037c8a61741")


