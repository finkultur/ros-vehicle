
(cl:in-package :asdf)

(defsystem "read_discovery_data-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "IMUData" :depends-on ("_package_IMUData"))
    (:file "_package_IMUData" :depends-on ("_package"))
  ))