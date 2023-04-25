; Auto-generated. Do not edit!


(cl:in-package dynamixel_sdk_examples-msg)


;//! \htmlinclude SetVelocity.msg.html

(cl:defclass <SetVelocity> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0)
   (velocity
    :reader velocity
    :initarg :velocity
    :type cl:integer
    :initform 0))
)

(cl:defclass SetVelocity (<SetVelocity>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetVelocity>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetVelocity)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamixel_sdk_examples-msg:<SetVelocity> is deprecated: use dynamixel_sdk_examples-msg:SetVelocity instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <SetVelocity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamixel_sdk_examples-msg:id-val is deprecated.  Use dynamixel_sdk_examples-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <SetVelocity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamixel_sdk_examples-msg:velocity-val is deprecated.  Use dynamixel_sdk_examples-msg:velocity instead.")
  (velocity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetVelocity>) ostream)
  "Serializes a message object of type '<SetVelocity>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'velocity)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetVelocity>) istream)
  "Deserializes a message object of type '<SetVelocity>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'velocity) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetVelocity>)))
  "Returns string type for a message object of type '<SetVelocity>"
  "dynamixel_sdk_examples/SetVelocity")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetVelocity)))
  "Returns string type for a message object of type 'SetVelocity"
  "dynamixel_sdk_examples/SetVelocity")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetVelocity>)))
  "Returns md5sum for a message object of type '<SetVelocity>"
  "32d435fbc97d5b2eb83a691dcedaaf9b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetVelocity)))
  "Returns md5sum for a message object of type 'SetVelocity"
  "32d435fbc97d5b2eb83a691dcedaaf9b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetVelocity>)))
  "Returns full string definition for message of type '<SetVelocity>"
  (cl:format cl:nil "uint8 id~%int32 velocity~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetVelocity)))
  "Returns full string definition for message of type 'SetVelocity"
  (cl:format cl:nil "uint8 id~%int32 velocity~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetVelocity>))
  (cl:+ 0
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetVelocity>))
  "Converts a ROS message object to a list"
  (cl:list 'SetVelocity
    (cl:cons ':id (id msg))
    (cl:cons ':velocity (velocity msg))
))
