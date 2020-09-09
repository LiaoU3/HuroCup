
(cl:in-package :asdf)

(defsystem "robot_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "HeadStatus" :depends-on ("_package_HeadStatus"))
    (:file "_package_HeadStatus" :depends-on ("_package"))
  ))