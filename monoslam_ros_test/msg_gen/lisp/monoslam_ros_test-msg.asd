
(cl:in-package :asdf)

(defsystem "monoslam_ros_test-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "filter_state" :depends-on ("_package_filter_state"))
    (:file "_package_filter_state" :depends-on ("_package"))
  ))