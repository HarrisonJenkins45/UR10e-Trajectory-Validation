#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "ur10e_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ur10e_interfaces__srv__ValidateTrajectory_Request() -> *const std::ffi::c_void;
}

#[link(name = "ur10e_interfaces__rosidl_generator_c")]
extern "C" {
    fn ur10e_interfaces__srv__ValidateTrajectory_Request__init(msg: *mut ValidateTrajectory_Request) -> bool;
    fn ur10e_interfaces__srv__ValidateTrajectory_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ValidateTrajectory_Request>, size: usize) -> bool;
    fn ur10e_interfaces__srv__ValidateTrajectory_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ValidateTrajectory_Request>);
    fn ur10e_interfaces__srv__ValidateTrajectory_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ValidateTrajectory_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ValidateTrajectory_Request>) -> bool;
}

// Corresponds to ur10e_interfaces__srv__ValidateTrajectory_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ValidateTrajectory_Request {
    /// 1. REQUEST (Input sent to the service)
    /// Desired EE trajectory path (flattened matrix or list of poses)
    pub ee_positions_x: rosidl_runtime_rs::Sequence<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub ee_positions_y: rosidl_runtime_rs::Sequence<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub ee_positions_z: rosidl_runtime_rs::Sequence<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub ee_quat: rosidl_runtime_rs::Sequence<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub sim_time: rosidl_runtime_rs::Sequence<f64>,

}



impl Default for ValidateTrajectory_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ur10e_interfaces__srv__ValidateTrajectory_Request__init(&mut msg as *mut _) {
        panic!("Call to ur10e_interfaces__srv__ValidateTrajectory_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ValidateTrajectory_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ur10e_interfaces__srv__ValidateTrajectory_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ur10e_interfaces__srv__ValidateTrajectory_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ur10e_interfaces__srv__ValidateTrajectory_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ValidateTrajectory_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ValidateTrajectory_Request where Self: Sized {
  const TYPE_NAME: &'static str = "ur10e_interfaces/srv/ValidateTrajectory_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ur10e_interfaces__srv__ValidateTrajectory_Request() }
  }
}


#[link(name = "ur10e_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ur10e_interfaces__srv__ValidateTrajectory_Response() -> *const std::ffi::c_void;
}

#[link(name = "ur10e_interfaces__rosidl_generator_c")]
extern "C" {
    fn ur10e_interfaces__srv__ValidateTrajectory_Response__init(msg: *mut ValidateTrajectory_Response) -> bool;
    fn ur10e_interfaces__srv__ValidateTrajectory_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ValidateTrajectory_Response>, size: usize) -> bool;
    fn ur10e_interfaces__srv__ValidateTrajectory_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ValidateTrajectory_Response>);
    fn ur10e_interfaces__srv__ValidateTrajectory_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ValidateTrajectory_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ValidateTrajectory_Response>) -> bool;
}

// Corresponds to ur10e_interfaces__srv__ValidateTrajectory_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ValidateTrajectory_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,

    /// Approved velocity commands (flattened)
    pub joint_velocities: rosidl_runtime_rs::Sequence<f64>,

}



impl Default for ValidateTrajectory_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ur10e_interfaces__srv__ValidateTrajectory_Response__init(&mut msg as *mut _) {
        panic!("Call to ur10e_interfaces__srv__ValidateTrajectory_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ValidateTrajectory_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ur10e_interfaces__srv__ValidateTrajectory_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ur10e_interfaces__srv__ValidateTrajectory_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ur10e_interfaces__srv__ValidateTrajectory_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ValidateTrajectory_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ValidateTrajectory_Response where Self: Sized {
  const TYPE_NAME: &'static str = "ur10e_interfaces/srv/ValidateTrajectory_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ur10e_interfaces__srv__ValidateTrajectory_Response() }
  }
}






#[link(name = "ur10e_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__ur10e_interfaces__srv__ValidateTrajectory() -> *const std::ffi::c_void;
}

// Corresponds to ur10e_interfaces__srv__ValidateTrajectory
#[allow(missing_docs, non_camel_case_types)]
pub struct ValidateTrajectory;

impl rosidl_runtime_rs::Service for ValidateTrajectory {
    type Request = ValidateTrajectory_Request;
    type Response = ValidateTrajectory_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__ur10e_interfaces__srv__ValidateTrajectory() }
    }
}


