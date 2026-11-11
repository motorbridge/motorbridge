use crate::{set_last_error, MotorHandle, MotorHandleInner};

pub(crate) fn ffi_get<T, F>(motor: *mut MotorHandle, out_value: *mut T, f: F) -> i32
where
    F: FnOnce(&MotorHandleInner) -> Result<T, String>,
{
    if motor.is_null() || out_value.is_null() {
        set_last_error("motor or out_value is null");
        return -1;
    }
    let motor = unsafe { &*motor };
    let inner = motor.inner.lock();
    let out = unsafe { &mut *out_value };
    match f(&inner) {
        Ok(v) => {
            *out = v;
            0
        }
        Err(e) => {
            set_last_error(e);
            -1
        }
    }
}

pub(crate) fn ffi_run<F>(motor: *mut MotorHandle, f: F) -> i32
where
    F: FnOnce(&MotorHandleInner) -> Result<(), String>,
{
    if motor.is_null() {
        set_last_error("motor is null");
        return -1;
    }
    let motor = unsafe { &*motor };
    let inner = motor.inner.lock();
    match f(&inner) {
        Ok(()) => 0,
        Err(e) => {
            set_last_error(e);
            -1
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{ffi_rc, FairMutex, HightorqueController};
    use motor_core::test_support::MockBus;
    use std::sync::{mpsc, Arc, Barrier, Mutex};
    use std::time::Duration;

    fn control(motor: *mut MotorHandle, f: impl FnOnce() -> Result<(), String>) -> i32 {
        ffi_wrap_motor!(motor, |_inner: &MotorHandleInner| f())
    }

    #[test]
    fn queued_control_and_parameter_calls_are_not_starved_by_reads() {
        let controller = HightorqueController::new(Arc::new(MockBus::new()));
        let handle = Arc::new(MotorHandle {
            inner: FairMutex::new(MotorHandleInner::Hightorque(
                controller.add_motor(1, 0x101, "ht").unwrap(),
            )),
        });
        let order = Arc::new(Mutex::new(Vec::new()));
        let (entered, first_read) = mpsc::channel();
        let (release, resume) = mpsc::channel();
        let reader = {
            let handle = Arc::clone(&handle);
            let order = Arc::clone(&order);
            std::thread::spawn(move || {
                for index in 0..100 {
                    let mut out = 0;
                    assert_eq!(
                        ffi_get(Arc::as_ptr(&handle).cast_mut(), &mut out, |_| {
                            if index == 0 {
                                entered.send(()).unwrap();
                                resume.recv_timeout(Duration::from_secs(2)).unwrap();
                            }
                            order.lock().unwrap().push("read");
                            Ok(42)
                        }),
                        0
                    );
                    assert_eq!(out, 42);
                }
            })
        };
        first_read.recv_timeout(Duration::from_secs(2)).unwrap();
        let start = Arc::new(Barrier::new(3));
        let waiters: Vec<_> = [false, true]
            .into_iter()
            .map(|is_control| {
                let handle = Arc::clone(&handle);
                let order = Arc::clone(&order);
                let start = Arc::clone(&start);
                std::thread::spawn(move || {
                    start.wait();
                    let run = || {
                        order.lock().unwrap().push(if is_control {
                            "control"
                        } else {
                            "parameter"
                        });
                        Ok(())
                    };
                    let ptr = Arc::as_ptr(&handle).cast_mut();
                    let rc = if is_control {
                        control(ptr, run)
                    } else {
                        ffi_run(ptr, |_| run())
                    };
                    assert_eq!(rc, 0);
                })
            })
            .collect();
        start.wait();
        // The first read models a blocking device response while the other calls queue.
        std::thread::sleep(Duration::from_millis(20));
        assert!(order.lock().unwrap().is_empty());
        release.send(()).unwrap();
        reader.join().unwrap();
        for waiter in waiters {
            waiter.join().unwrap();
        }
        let order = order.lock().unwrap();
        assert_eq!(order.len(), 102);
        assert_eq!(order[0], "read");
        assert!(order[1..3].contains(&"control"), "{order:?}");
        assert!(order[1..3].contains(&"parameter"), "{order:?}");
        assert!(order[3..].iter().all(|entry| *entry == "read"));
    }
}
