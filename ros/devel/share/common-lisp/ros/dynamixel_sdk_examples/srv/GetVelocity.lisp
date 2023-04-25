; Auto-generated. Do not edit!


(cl:in-package dynamixel_sdk_examples-srv)


;//! \htmlinclude GetVelocity-request.msg.html

(cl:defclass <GetVelocity-request> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetVelocity-request (<GetVelocity-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetVelocity-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetVelocity-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamixel_sdk_examples-srv:<GetVelocity-request> is deprecated: use dynamixel_sdk_examples-srv:GetVelocity-request instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <GetVelocity-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamixel_sdk_examples-srv:id-val is deprecated.  Use dynamixel_sdk_examples-srv:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetVelocity-request>) ostream)
  "Serializes a message object of type '<GetVelocity-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetVelocity-request>) istream)
  "Deserializes a message object of type '<GetVelocity-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetVelocity-request>)))
  "Returns string type for a service object of type '<GetVelocity-request>"
  "dynamixel_sdk_examples/GetVelocityRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetVelocity-request)))
  "Returns string type for a service object of type 'GetVelocity-request"
  "dynamixel_sdk_examples/GetVelocityRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetVelocity-request>)))
  "Returns md5sum for a message object of type '<GetVelocity-request>"
  "93bf375ee98b0b0eda0baf02f10b2e49")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetVelocity-request)))
  "Returns md5sum for a message object of type 'GetVelocity-request"
  "93bf375ee98b0b0eda0baf02f10b2e49")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetVelocity-request>)))
  "Returns full string definition for message of type '<GetVelocity-request>"
  (cl:format cl:nil "uint8 id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetVelocity-request)))
  "Returns full string definition for message of type 'GetVelocity-request"
  (cl:format cl:nil "uint8 id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetVelocity-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetVelocity-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetVelocity-request
    (cl:cons ':id (id msg))
))
;//! \htmlinclude GetVelocity-response.msg.html

(cl:defclass <GetVelocity-response> (roslisp-msg-protocol:ros-message)
  ((velocity
    :reader velocity
    :initarg :velocity
    :type cl:integer
    :initform 0))
)

(cl:defclass GetVelocity-response (<GetVelocity-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetVelocity-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetVelocity-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamixel_sdk_examples-srv:<GetVelocity-response> is deprecated: use dynamixel_sdk_examples-srv:GetVelocity-response instead.")))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <GetVelocity-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamixel_sdk_examples-srv:velocity-val is deprecated.  Use dynamixel_sdk_examples-srv:velocity instead.")
  (velocity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetVelocity-response>) ostream)
  "Serializes a message object of type '<GetVelocity-response>"
  (cl:let* ((signed (cl:slot-value msg 'velocity)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetVelocity-response>) istream)
  "Deserializes a message object of type '<GetVelocity-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'velocity) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetVelocity-response>)))
  "Returns string type for a service object of type '<GetVelocity-response>"
  "dynamixel_sdk_examples/GetVelocityResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetVelocity-response)))
  "Returns string type for a service object of type 'GetVelocity-response"
  "dynamixel_sdk_examples/GetVelocityResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetVelocity-response>)))
  "Returns md5sum for a message object of type '<GetVelocity-response>"
  "93bf375ee98b0b0eda0baf02f10b2e49")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetVelocity-response)))
  "Returns md5sum for a message object of type 'GetVelocity-response"
  "93bf375ee98b0b0eda0baf02f10b2e49")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetVelocity-response>)))
  "Returns full string definition for message of type '<GetVelocity-response>"
  (cl:format cl:nil "int32 velocity~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetVelocity-response)))
  "Returns full string definition for message of type 'GetVelocity-response"
  (cl:format cl:nil "int32 velocity~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetVelocity-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetVelocity-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetVelocity-response
    (cl:cons ':velocity (velocity msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetVelocity)))
  'GetVelocity-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetVelocity)))
  'GetVelocity-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetVelocity)))
  "Returns string type for a service object of type '<GetVelocity>"
  "dynamixel_sdk_examples/GetVelocity")