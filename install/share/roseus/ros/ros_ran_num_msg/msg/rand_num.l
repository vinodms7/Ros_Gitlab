;; Auto-generated. Do not edit!


(when (boundp 'ros_ran_num_msg::rand_num)
  (if (not (find-package "ROS_RAN_NUM_MSG"))
    (make-package "ROS_RAN_NUM_MSG"))
  (shadow 'rand_num (find-package "ROS_RAN_NUM_MSG")))
(unless (find-package "ROS_RAN_NUM_MSG::RAND_NUM")
  (make-package "ROS_RAN_NUM_MSG::RAND_NUM"))

(in-package "ROS")
;;//! \htmlinclude rand_num.msg.html


(defclass ros_ran_num_msg::rand_num
  :super ros::object
  :slots (_number1 _number2 ))

(defmethod ros_ran_num_msg::rand_num
  (:init
   (&key
    ((:number1 __number1) 0)
    ((:number2 __number2) 0)
    )
   (send-super :init)
   (setq _number1 (round __number1))
   (setq _number2 (round __number2))
   self)
  (:number1
   (&optional __number1)
   (if __number1 (setq _number1 __number1)) _number1)
  (:number2
   (&optional __number2)
   (if __number2 (setq _number2 __number2)) _number2)
  (:serialization-length
   ()
   (+
    ;; uint32 _number1
    4
    ;; uint32 _number2
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _number1
       (write-long _number1 s)
     ;; uint32 _number2
       (write-long _number2 s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _number1
     (setq _number1 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _number2
     (setq _number2 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ros_ran_num_msg::rand_num :md5sum-) "20e27c3e6eee8dfccd76499464b39a05")
(setf (get ros_ran_num_msg::rand_num :datatype-) "ros_ran_num_msg/rand_num")
(setf (get ros_ran_num_msg::rand_num :definition-)
      "uint32 number1
uint32 number2


")



(provide :ros_ran_num_msg/rand_num "20e27c3e6eee8dfccd76499464b39a05")


