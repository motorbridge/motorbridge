use crate::model::{ActiveCommand, ControllerHandle, MotorHandle, Target};

mod connect;
mod runtime;

pub(crate) struct SessionCtx {
    pub(crate) target: Target,
    pub(crate) controller: Option<ControllerHandle>,
    pub(crate) motor: Option<MotorHandle>,
    pub(crate) active: Option<ActiveCommand>,
    pub(crate) robstride_mit_gains: Option<(f32, f32)>,
}

pub(crate) fn myactuator_feedback_default(motor_id: u16) -> u16 {
    0x240u16.saturating_add(motor_id)
}

impl SessionCtx {
    pub(crate) fn model_is_auto(model: &str) -> bool {
        let m = model.trim().to_ascii_lowercase();
        m.is_empty() || m == "auto" || m == "all" || m == "*"
    }

    pub(crate) fn new(target: Target) -> Self {
        Self {
            target,
            controller: None,
            motor: None,
            active: None,
            robstride_mit_gains: None,
        }
    }

    pub(crate) fn retarget_from_request_if_present(
        &mut self,
        vendor: Option<crate::model::Vendor>,
        model: Option<&str>,
        motor_id: Option<u16>,
        feedback_id: Option<u16>,
    ) -> Result<bool, String> {
        let Some(motor_id) = motor_id else {
            return Ok(false);
        };
        let mut next = self.target.clone();
        if let Some(vendor) = vendor {
            next.vendor = vendor;
        }
        if let Some(model) = model {
            if !model.trim().is_empty() {
                next.model = model.to_string();
            }
        }
        next.motor_id = motor_id;
        if let Some(feedback_id) = feedback_id {
            next.feedback_id = feedback_id;
        }
        if next.vendor == self.target.vendor
            && next.model == self.target.model
            && next.motor_id == self.target.motor_id
            && next.feedback_id == self.target.feedback_id
        {
            return Ok(false);
        }
        self.disconnect(false);
        self.target = next;
        self.ensure_connected()?;
        Ok(true)
    }
}
