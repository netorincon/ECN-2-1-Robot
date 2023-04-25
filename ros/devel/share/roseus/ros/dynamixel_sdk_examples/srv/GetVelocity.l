;; Auto-generated. Do not edit!


(when (boundp 'dynamixel_sdk_examples::GetVelocity)
  (if (not (find-package "DYNAMIXEL_SDK_EXAMPLES"))
    (make-package "DYNAMIXEL_SDK_EXAMPLES"))
  (shadow 'GetVelocity (find-package "DYNAMIXEL_SDK_EXAMPLES")))
(unless (find-package "DYNAMIXEL_SDK_EXAMPLES::GETVELOCITY")
  (make-package "DYNAMIXEL_SDK_EXAMPLES::GETVELOCITY"))
(unless (find-package "DYNAMIXEL_SDK_EXAMPLES::GETVELOCITYREQUEST")
  (make-package "DYNAMIXEL_SDK_EXAMPLES::GETVELOCITYREQUEST"))
(unless (find-package "DYNAMIXEL_SDK_EXAMPLES::GETVELOCITYRESPONSE")
  (make-package "DYNAMIXEL_SDK_EXAMPLES::GETVELOCITYRESPONSE"))

(in-package "ROS")





(defclass dynamixel_sdk_examples::GetVelocityRequest
  :super ros::object
  :slots (_id ))

(defmethod dynamixel_sdk_examples::GetVelocityRequest
  (:init
   (&key
    ((:id __id) 0)
    )
   (send-super :init)
   (setq _id (round __id))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:serialization-length
   ()
   (+
    ;; uint8 _id
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _id
       (write-byte _id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass dynamixel_sdk_examples::GetVelocityResponse
  :super ros::object
  :slots (_velocity ))

(defmethod dynamixel_sdk_examples::GetVelocityResponse
  (:init
   (&key
    ((:velocity __velocity) 0)
    )
   (send-super :init)
   (setq _velocity (round __velocity))
   self)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:serialization-length
   ()
   (+
    ;; int32 _velocity
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _velocity
       (write-long _velocity s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _velocity
     (setq _velocity (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass dynamixel_sdk_examples::GetVelocity
  :super ros::object
  :slots ())

(setf (get dynamixel_sdk_examples::GetVelocity :md5sum-) "93bf375ee98b0b0eda0baf02f10b2e49")
(setf (get dynamixel_sdk_examples::GetVelocity :datatype-) "dynamixel_sdk_examples/GetVelocity")
(setf (get dynamixel_sdk_examples::GetVelocity :request) dynamixel_sdk_examples::GetVelocityRequest)
(setf (get dynamixel_sdk_examples::GetVelocity :response) dynamixel_sdk_examples::GetVelocityResponse)

(defmethod dynamixel_sdk_examples::GetVelocityRequest
  (:response () (instance dynamixel_sdk_examples::GetVelocityResponse :init)))

(setf (get dynamixel_sdk_examples::GetVelocityRequest :md5sum-) "93bf375ee98b0b0eda0baf02f10b2e49")
(setf (get dynamixel_sdk_examples::GetVelocityRequest :datatype-) "dynamixel_sdk_examples/GetVelocityRequest")
(setf (get dynamixel_sdk_examples::GetVelocityRequest :definition-)
      "uint8 id
---
int32 velocity

")

(setf (get dynamixel_sdk_examples::GetVelocityResponse :md5sum-) "93bf375ee98b0b0eda0baf02f10b2e49")
(setf (get dynamixel_sdk_examples::GetVelocityResponse :datatype-) "dynamixel_sdk_examples/GetVelocityResponse")
(setf (get dynamixel_sdk_examples::GetVelocityResponse :definition-)
      "uint8 id
---
int32 velocity

")



(provide :dynamixel_sdk_examples/GetVelocity "93bf375ee98b0b0eda0baf02f10b2e49")


