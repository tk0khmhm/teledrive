
(cl:in-package :asdf)

(defsystem "teledrive-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Teledrive" :depends-on ("_package_Teledrive"))
    (:file "_package_Teledrive" :depends-on ("_package"))
  ))