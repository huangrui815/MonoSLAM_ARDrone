; Auto-generated. Do not edit!


(cl:in-package monoslam_ros_test-msg)


;//! \htmlinclude filter_state.msg.html

(cl:defclass <filter_state> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0)
   (dx
    :reader dx
    :initarg :dx
    :type cl:float
    :initform 0.0)
   (dy
    :reader dy
    :initarg :dy
    :type cl:float
    :initform 0.0)
   (dz
    :reader dz
    :initarg :dz
    :type cl:float
    :initform 0.0)
   (roll
    :reader roll
    :initarg :roll
    :type cl:float
    :initform 0.0)
   (pitch
    :reader pitch
    :initarg :pitch
    :type cl:float
    :initform 0.0)
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0)
   (dyaw
    :reader dyaw
    :initarg :dyaw
    :type cl:float
    :initform 0.0)
   (scale
    :reader scale
    :initarg :scale
    :type cl:float
    :initform 0.0)
   (ptamState
    :reader ptamState
    :initarg :ptamState
    :type cl:integer
    :initform 0)
   (scaleAccuracy
    :reader scaleAccuracy
    :initarg :scaleAccuracy
    :type cl:float
    :initform 0.0)
   (droneState
    :reader droneState
    :initarg :droneState
    :type cl:integer
    :initform 0)
   (batteryPercent
    :reader batteryPercent
    :initarg :batteryPercent
    :type cl:float
    :initform 0.0))
)

(cl:defclass filter_state (<filter_state>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <filter_state>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'filter_state)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name monoslam_ros_test-msg:<filter_state> is deprecated: use monoslam_ros_test-msg:filter_state instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <filter_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader monoslam_ros_test-msg:header-val is deprecated.  Use monoslam_ros_test-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <filter_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader monoslam_ros_test-msg:x-val is deprecated.  Use monoslam_ros_test-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <filter_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader monoslam_ros_test-msg:y-val is deprecated.  Use monoslam_ros_test-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <filter_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader monoslam_ros_test-msg:z-val is deprecated.  Use monoslam_ros_test-msg:z instead.")
  (z m))

(cl:ensure-generic-function 'dx-val :lambda-list '(m))
(cl:defmethod dx-val ((m <filter_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader monoslam_ros_test-msg:dx-val is deprecated.  Use monoslam_ros_test-msg:dx instead.")
  (dx m))

(cl:ensure-generic-function 'dy-val :lambda-list '(m))
(cl:defmethod dy-val ((m <filter_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader monoslam_ros_test-msg:dy-val is deprecated.  Use monoslam_ros_test-msg:dy instead.")
  (dy m))

(cl:ensure-generic-function 'dz-val :lambda-list '(m))
(cl:defmethod dz-val ((m <filter_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader monoslam_ros_test-msg:dz-val is deprecated.  Use monoslam_ros_test-msg:dz instead.")
  (dz m))

(cl:ensure-generic-function 'roll-val :lambda-list '(m))
(cl:defmethod roll-val ((m <filter_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader monoslam_ros_test-msg:roll-val is deprecated.  Use monoslam_ros_test-msg:roll instead.")
  (roll m))

(cl:ensure-generic-function 'pitch-val :lambda-list '(m))
(cl:defmethod pitch-val ((m <filter_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader monoslam_ros_test-msg:pitch-val is deprecated.  Use monoslam_ros_test-msg:pitch instead.")
  (pitch m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <filter_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader monoslam_ros_test-msg:yaw-val is deprecated.  Use monoslam_ros_test-msg:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'dyaw-val :lambda-list '(m))
(cl:defmethod dyaw-val ((m <filter_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader monoslam_ros_test-msg:dyaw-val is deprecated.  Use monoslam_ros_test-msg:dyaw instead.")
  (dyaw m))

(cl:ensure-generic-function 'scale-val :lambda-list '(m))
(cl:defmethod scale-val ((m <filter_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader monoslam_ros_test-msg:scale-val is deprecated.  Use monoslam_ros_test-msg:scale instead.")
  (scale m))

(cl:ensure-generic-function 'ptamState-val :lambda-list '(m))
(cl:defmethod ptamState-val ((m <filter_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader monoslam_ros_test-msg:ptamState-val is deprecated.  Use monoslam_ros_test-msg:ptamState instead.")
  (ptamState m))

(cl:ensure-generic-function 'scaleAccuracy-val :lambda-list '(m))
(cl:defmethod scaleAccuracy-val ((m <filter_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader monoslam_ros_test-msg:scaleAccuracy-val is deprecated.  Use monoslam_ros_test-msg:scaleAccuracy instead.")
  (scaleAccuracy m))

(cl:ensure-generic-function 'droneState-val :lambda-list '(m))
(cl:defmethod droneState-val ((m <filter_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader monoslam_ros_test-msg:droneState-val is deprecated.  Use monoslam_ros_test-msg:droneState instead.")
  (droneState m))

(cl:ensure-generic-function 'batteryPercent-val :lambda-list '(m))
(cl:defmethod batteryPercent-val ((m <filter_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader monoslam_ros_test-msg:batteryPercent-val is deprecated.  Use monoslam_ros_test-msg:batteryPercent instead.")
  (batteryPercent m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<filter_state>)))
    "Constants for message type '<filter_state>"
  '((:PTAM_IDLE . 0)
    (:PTAM_INITIALIZING . 1)
    (:PTAM_LOST . 2)
    (:PTAM_GOOD . 3)
    (:PTAM_BEST . 4)
    (:PTAM_TOOKKF . 5)
    (:PTAM_FALSEPOSITIVE . 6))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'filter_state)))
    "Constants for message type 'filter_state"
  '((:PTAM_IDLE . 0)
    (:PTAM_INITIALIZING . 1)
    (:PTAM_LOST . 2)
    (:PTAM_GOOD . 3)
    (:PTAM_BEST . 4)
    (:PTAM_TOOKKF . 5)
    (:PTAM_FALSEPOSITIVE . 6))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <filter_state>) ostream)
  "Serializes a message object of type '<filter_state>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'roll))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dyaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'scale))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ptamState)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ptamState)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'ptamState)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'ptamState)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'scaleAccuracy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'droneState)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'droneState)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'droneState)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'droneState)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'batteryPercent))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <filter_state>) istream)
  "Deserializes a message object of type '<filter_state>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dy) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dz) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'roll) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dyaw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'scale) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ptamState)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ptamState)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'ptamState)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'ptamState)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'scaleAccuracy) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'droneState)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'droneState)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'droneState)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'droneState)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'batteryPercent) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<filter_state>)))
  "Returns string type for a message object of type '<filter_state>"
  "monoslam_ros_test/filter_state")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'filter_state)))
  "Returns string type for a message object of type 'filter_state"
  "monoslam_ros_test/filter_state")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<filter_state>)))
  "Returns md5sum for a message object of type '<filter_state>"
  "33f8050d082c4ebadff26def43dcfd15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'filter_state)))
  "Returns md5sum for a message object of type 'filter_state"
  "33f8050d082c4ebadff26def43dcfd15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<filter_state>)))
  "Returns full string definition for message of type '<filter_state>"
  (cl:format cl:nil "# constants~%uint32 PTAM_IDLE = 0           # PTAM not running.~%uint32 PTAM_INITIALIZING = 1   # initialization (trails)~%uint32 PTAM_LOST = 2           # ptam is running, but lost~%uint32 PTAM_GOOD = 3           # tracking quality OK~%uint32 PTAM_BEST = 4           # tracking quality best~%uint32 PTAM_TOOKKF = 5         # just took a new KF (equivalent to PTAM_BEST)~%uint32 PTAM_FALSEPOSITIVE = 6  # ptam thinks it is good, but its estimate is rejected.~%~%# header~%Header      header~%~%# ----------------- raw 10d filter state ----------------------------~%float32     x~%float32     y~%float32     z~%float32     dx~%float32     dy~%float32     dz~%float32     roll~%float32     pitch~%float32     yaw~%float32     dyaw~%~%~%~%# --------------------- other values ---------------------------~%float32     scale         # ptam scale factor (PTAMpos * scale = WORLDpos).~%uint32      ptamState~%float32     scaleAccuracy # if scale is very inaccurate, this is about 0.5, and grows up to 1 (=very accurate).~%~%~%# ----------------- propagated from drone messages: -----------------~%# 0: Unknown, 1: Init, 2: Landed, 3: Flying, 4: Hovering, 5: Test~%# 6: Taking off, 7: Goto Fix Point, 8: Landing, 9: Looping~%# Note: 3,7 seems to discriminate type of flying (isFly = 3 | 7)~%uint32       droneState~%float32      batteryPercent    # 0 means no battery, 100 means full battery~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'filter_state)))
  "Returns full string definition for message of type 'filter_state"
  (cl:format cl:nil "# constants~%uint32 PTAM_IDLE = 0           # PTAM not running.~%uint32 PTAM_INITIALIZING = 1   # initialization (trails)~%uint32 PTAM_LOST = 2           # ptam is running, but lost~%uint32 PTAM_GOOD = 3           # tracking quality OK~%uint32 PTAM_BEST = 4           # tracking quality best~%uint32 PTAM_TOOKKF = 5         # just took a new KF (equivalent to PTAM_BEST)~%uint32 PTAM_FALSEPOSITIVE = 6  # ptam thinks it is good, but its estimate is rejected.~%~%# header~%Header      header~%~%# ----------------- raw 10d filter state ----------------------------~%float32     x~%float32     y~%float32     z~%float32     dx~%float32     dy~%float32     dz~%float32     roll~%float32     pitch~%float32     yaw~%float32     dyaw~%~%~%~%# --------------------- other values ---------------------------~%float32     scale         # ptam scale factor (PTAMpos * scale = WORLDpos).~%uint32      ptamState~%float32     scaleAccuracy # if scale is very inaccurate, this is about 0.5, and grows up to 1 (=very accurate).~%~%~%# ----------------- propagated from drone messages: -----------------~%# 0: Unknown, 1: Init, 2: Landed, 3: Flying, 4: Hovering, 5: Test~%# 6: Taking off, 7: Goto Fix Point, 8: Landing, 9: Looping~%# Note: 3,7 seems to discriminate type of flying (isFly = 3 | 7)~%uint32       droneState~%float32      batteryPercent    # 0 means no battery, 100 means full battery~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <filter_state>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <filter_state>))
  "Converts a ROS message object to a list"
  (cl:list 'filter_state
    (cl:cons ':header (header msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
    (cl:cons ':dx (dx msg))
    (cl:cons ':dy (dy msg))
    (cl:cons ':dz (dz msg))
    (cl:cons ':roll (roll msg))
    (cl:cons ':pitch (pitch msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':dyaw (dyaw msg))
    (cl:cons ':scale (scale msg))
    (cl:cons ':ptamState (ptamState msg))
    (cl:cons ':scaleAccuracy (scaleAccuracy msg))
    (cl:cons ':droneState (droneState msg))
    (cl:cons ':batteryPercent (batteryPercent msg))
))
