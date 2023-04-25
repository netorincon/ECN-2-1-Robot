;; Auto-generated. Do not edit!


(when (boundp 'dynamixel_sdk_examples::Position)
  (if (not (find-package "DYNAMIXEL_SDK_EXAMPLES"))
    (make-package "DYNAMIXEL_SDK_EXAMPLES"))
  (shadow 'Position (find-package "DYNAMIXEL_SDK_EXAMPLES")))
(unless (find-package "DYNAMIXEL_SDK_EXAMPLES::POSITION")
  (make-package "DYNAMIXEL_SDK_EXAMPLES::POSITION"))

(in-package "ROS")
;;//! \htmlinclude Position.msg.html


(defclass dynamixel_sdk_examples::Position
  :super ros::object
  :slots (_id _position ))

(defmethod dynamixel_sdk_examples::Position
  (:init
   (&key
    ((:id __id) (make-array 0 :initial-element 0 :element-type :char))
    ((:position __position) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _id __id)
   (setq _position __position)
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:serialization-length
   ()
   (+
    ;; uint8[] _id
    (* 1    (length _id)) 4
    ;; int32[] _position
    (* 4    (length _position)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8[] _id
     (write-long (length _id) s)
     (princ _id s)
     ;; int32[] _position
     (write-long (length _position) s)
     (dotimes (i (length _position))
       (write-long (elt _position i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8[] _id
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _id (make-array n :element-type :char))
     (replace _id buf :start2 ptr-) (incf ptr- n))
   ;; int32[] _position
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _position (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _position i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get dynamixel_sdk_examples::Position :md5sum-) "8d1d43c65af5558fb9b71be9651e4d8b")
(setf (get dynamixel_sdk_examples::Position :datatype-) "dynamixel_sdk_examples/Position")
(setf (get dynamixel_sdk_examples::Position :definition-)
      "uint8[] id
int32[] position

")



(provide :dynamixel_sdk_examples/Position "8d1d43c65af5558fb9b71be9651e4d8b")


