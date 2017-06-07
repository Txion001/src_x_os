
(cl:in-package :asdf)

(defsystem "x_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "X_Command" :depends-on ("_package_X_Command"))
    (:file "_package_X_Command" :depends-on ("_package"))
  ))