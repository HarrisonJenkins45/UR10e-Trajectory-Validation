#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to ur10e_interfaces__srv__ValidateTrajectory_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ValidateTrajectory_Request {
    /// 1. REQUEST (Input sent to the service)
    /// Desired EE trajectory path (flattened matrix or list of poses)
    pub ee_positions_x: Vec<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub ee_positions_y: Vec<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub ee_positions_z: Vec<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub ee_quat: Vec<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub sim_time: Vec<f64>,

}



impl Default for ValidateTrajectory_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ValidateTrajectory_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ValidateTrajectory_Request {
  type RmwMsg = super::srv::rmw::ValidateTrajectory_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        ee_positions_x: msg.ee_positions_x.into(),
        ee_positions_y: msg.ee_positions_y.into(),
        ee_positions_z: msg.ee_positions_z.into(),
        ee_quat: msg.ee_quat.into(),
        sim_time: msg.sim_time.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        ee_positions_x: msg.ee_positions_x.as_slice().into(),
        ee_positions_y: msg.ee_positions_y.as_slice().into(),
        ee_positions_z: msg.ee_positions_z.as_slice().into(),
        ee_quat: msg.ee_quat.as_slice().into(),
        sim_time: msg.sim_time.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      ee_positions_x: msg.ee_positions_x
          .into_iter()
          .collect(),
      ee_positions_y: msg.ee_positions_y
          .into_iter()
          .collect(),
      ee_positions_z: msg.ee_positions_z
          .into_iter()
          .collect(),
      ee_quat: msg.ee_quat
          .into_iter()
          .collect(),
      sim_time: msg.sim_time
          .into_iter()
          .collect(),
    }
  }
}


// Corresponds to ur10e_interfaces__srv__ValidateTrajectory_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ValidateTrajectory_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,

    /// Approved velocity commands (flattened)
    pub joint_velocities: Vec<f64>,

}



impl Default for ValidateTrajectory_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ValidateTrajectory_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ValidateTrajectory_Response {
  type RmwMsg = super::srv::rmw::ValidateTrajectory_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
        message: msg.message.as_str().into(),
        joint_velocities: msg.joint_velocities.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
        message: msg.message.as_str().into(),
        joint_velocities: msg.joint_velocities.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
      message: msg.message.to_string(),
      joint_velocities: msg.joint_velocities
          .into_iter()
          .collect(),
    }
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


