
(cl:in-package :asdf)

(defsystem "ltme_node-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "QueryFirmwareVersion" :depends-on ("_package_QueryFirmwareVersion"))
    (:file "_package_QueryFirmwareVersion" :depends-on ("_package"))
    (:file "QueryHardwareVersion" :depends-on ("_package_QueryHardwareVersion"))
    (:file "_package_QueryHardwareVersion" :depends-on ("_package"))
    (:file "QuerySerial" :depends-on ("_package_QuerySerial"))
    (:file "_package_QuerySerial" :depends-on ("_package"))
  ))