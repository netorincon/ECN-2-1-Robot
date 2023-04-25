;; Auto-generated. Do not edit!


(when (boundp 'dynamixel_sdk_examples::SetVelocity)
  (if (not (find-package "DYNAMIXEL_SDK_EXAMPLES"))
    (make-package "DYNAMIXEL_SDK_EXAMPLES"))
  (shadow 'SetVelocity (find-package "DYNAMIXEL_SDK_EXAMPLES")))
(unless (find-package "DYNAMIXEL_SDK_EXAMPLES::SETVELOCITY")
  (make-package "DYNAMIXEL_SDK_EXAMPLES::SETVELOCITY"))

(in-package "ROS")
;;//! \htmlinclude SetVelocity.msg.html


(defclass dynamixel_sdk_examples::SetVelocity
  :super ros::object
  :slots (_id _velocity ))

(defmethod dynamixel_sdk_examples::SetVelocity
  (:init
   (&key
    ((:id __id) 0)
    ((:velocity __velocity) 0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _velocity (round __velocity))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:serialization-length
   ()
   (+
    ;; uint8 _id
    1
    ;; int32 _velocity
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _id
       (write-byte _id s)
     ;; int32 _velocity
       (write-long _velocity s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; int32 _velocity
     (setq _velocity (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get dynamixel_sdk_examples::SetVelocity :md5sum-) "32d435fbc97d5b2eb83a691dcedaaf9b")
(setf (get dynamixel_sdk_examples::SetVelocity :datatype-) "dynamixel_sdk_examples/SetVelocity")
(setf (get dynamixel_sdk_examples::SetVelocity :definition-)
      "uint8 id
int32 velocity

")



(provide :dynamixel_sdk_examples/SetVelocity "32d435fbc97d5b2eb83a691dcedaaf9b")


