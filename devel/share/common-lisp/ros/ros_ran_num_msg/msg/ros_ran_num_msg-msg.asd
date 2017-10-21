
(cl:in-package :asdf)

(defsystem "ros_ran_num_msg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "rand_num" :depends-on ("_package_rand_num"))
    (:file "_package_rand_num" :depends-on ("_package"))
  ))