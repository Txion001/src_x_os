
(cl:in-package :asdf)

(defsystem "x_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "X_Status" :depends-on ("_package_X_Status"))
    (:file "_package_X_Status" :depends-on ("_package"))
  ))