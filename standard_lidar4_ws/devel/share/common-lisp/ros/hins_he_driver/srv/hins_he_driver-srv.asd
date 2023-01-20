
(cl:in-package :asdf)

(defsystem "hins_he_driver-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "hins_srv" :depends-on ("_package_hins_srv"))
    (:file "_package_hins_srv" :depends-on ("_package"))
  ))