;; Auto-generated. Do not edit!


(when (boundp 'hins_le_driver::hins_srv)
  (if (not (find-package "HINS_LE_DRIVER"))
    (make-package "HINS_LE_DRIVER"))
  (shadow 'hins_srv (find-package "HINS_LE_DRIVER")))
(unless (find-package "HINS_LE_DRIVER::HINS_SRV")
  (make-package "HINS_LE_DRIVER::HINS_SRV"))
(unless (find-package "HINS_LE_DRIVER::HINS_SRVREQUEST")
  (make-package "HINS_LE_DRIVER::HINS_SRVREQUEST"))
(unless (find-package "HINS_LE_DRIVER::HINS_SRVRESPONSE")
  (make-package "HINS_LE_DRIVER::HINS_SRVRESPONSE"))

(in-package "ROS")





(defclass hins_le_driver::hins_srvRequest
  :super ros::object
  :slots (_channel ))

(defmethod hins_le_driver::hins_srvRequest
  (:init
   (&key
    ((:channel __channel) 0)
    )
   (send-super :init)
   (setq _channel (round __channel))
   self)
  (:channel
   (&optional __channel)
   (if __channel (setq _channel __channel)) _channel)
  (:serialization-length
   ()
   (+
    ;; int64 _channel
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _channel
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _channel (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _channel) (= (length (_channel . bv)) 2)) ;; bignum
              (write-long (ash (elt (_channel . bv) 0) 0) s)
              (write-long (ash (elt (_channel . bv) 1) -1) s))
             ((and (class _channel) (= (length (_channel . bv)) 1)) ;; big1
              (write-long (elt (_channel . bv) 0) s)
              (write-long (if (>= _channel 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _channel s)(write-long (if (>= _channel 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _channel
#+(or :alpha :irix6 :x86_64)
      (setf _channel (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _channel (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass hins_le_driver::hins_srvResponse
  :super ros::object
  :slots (_area1 _area2 _area3 _success ))

(defmethod hins_le_driver::hins_srvResponse
  (:init
   (&key
    ((:area1 __area1) nil)
    ((:area2 __area2) nil)
    ((:area3 __area3) nil)
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _area1 __area1)
   (setq _area2 __area2)
   (setq _area3 __area3)
   (setq _success __success)
   self)
  (:area1
   (&optional (__area1 :null))
   (if (not (eq __area1 :null)) (setq _area1 __area1)) _area1)
  (:area2
   (&optional (__area2 :null))
   (if (not (eq __area2 :null)) (setq _area2 __area2)) _area2)
  (:area3
   (&optional (__area3 :null))
   (if (not (eq __area3 :null)) (setq _area3 __area3)) _area3)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _area1
    1
    ;; bool _area2
    1
    ;; bool _area3
    1
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _area1
       (if _area1 (write-byte -1 s) (write-byte 0 s))
     ;; bool _area2
       (if _area2 (write-byte -1 s) (write-byte 0 s))
     ;; bool _area3
       (if _area3 (write-byte -1 s) (write-byte 0 s))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _area1
     (setq _area1 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _area2
     (setq _area2 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _area3
     (setq _area3 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass hins_le_driver::hins_srv
  :super ros::object
  :slots ())

(setf (get hins_le_driver::hins_srv :md5sum-) "1b7a8b465a26f7d507ff8e5984967a01")
(setf (get hins_le_driver::hins_srv :datatype-) "hins_le_driver/hins_srv")
(setf (get hins_le_driver::hins_srv :request) hins_le_driver::hins_srvRequest)
(setf (get hins_le_driver::hins_srv :response) hins_le_driver::hins_srvResponse)

(defmethod hins_le_driver::hins_srvRequest
  (:response () (instance hins_le_driver::hins_srvResponse :init)))

(setf (get hins_le_driver::hins_srvRequest :md5sum-) "1b7a8b465a26f7d507ff8e5984967a01")
(setf (get hins_le_driver::hins_srvRequest :datatype-) "hins_le_driver/hins_srvRequest")
(setf (get hins_le_driver::hins_srvRequest :definition-)
      "int64 channel
---
bool area1
bool area2
bool area3
bool success

")

(setf (get hins_le_driver::hins_srvResponse :md5sum-) "1b7a8b465a26f7d507ff8e5984967a01")
(setf (get hins_le_driver::hins_srvResponse :datatype-) "hins_le_driver/hins_srvResponse")
(setf (get hins_le_driver::hins_srvResponse :definition-)
      "int64 channel
---
bool area1
bool area2
bool area3
bool success

")



(provide :hins_le_driver/hins_srv "1b7a8b465a26f7d507ff8e5984967a01")


