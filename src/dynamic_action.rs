use ai_behavior::Status;
use std::rc::Rc;

pub struct DynamicAction<S> {
    pub action: Rc<dyn ActionTrait<S>>,
}

impl<S> Clone for DynamicAction<S> {
    fn clone(&self) -> Self {
        Self {
            action: self.action.clone(),
        }
    }
}

pub trait ActionTrait<S> {
    fn handle(&self, state: &mut S) -> Status;
}

impl<F, S> ActionTrait<S> for F
where
    F: Fn(&mut S) -> Status,
{
    fn handle(&self, state: &mut S) -> Status {
        (self)(state)
    }
}

impl<S> DynamicAction<S> {
    pub fn new(action: impl ActionTrait<S> + 'static) -> Self {
        Self {
            action: Rc::new(action),
        }
    }
}
