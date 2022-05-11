//! Stress test the grabber mechanism by executing the following sequence in a loop:
//!
//! 1. Lower arm
//! 2. Start suction pump
//! 3. Contact card
//! 4. "Grab" card (using suction)
//! 5. Lift card
//! 6. Reverse suction pump, release card
//!
//! This allows us to monitor the machine and confirm that this mechanism is working as expected.

use std::{env, io, thread};
use std::io::Write;
use std::time::{Duration, Instant};
use ai_behavior::{Behavior, State, Status};
use anyhow::anyhow;
use input::{Event, UpdateArgs};
use ev3_shuffl3bot::grabber_bt::{GrabberAction, GrabberBehaviourTreeFactory, GrabberState};
use ev3_shuffl3bot::grabber_hal::GrabberHal;
use ev3_shuffl3bot::grabber_hal_factory::GrabberHalFactory;

const TICK_INTERVAL: Duration = Duration::from_millis(20);

fn main() -> anyhow::Result<()> {
    let args: Vec<_> = env::args().collect();
    let num_runs_str = args.get(1).cloned().unwrap_or_else(|| "50".to_owned());
    let num_runs: u32 = num_runs_str.parse()?;
    let mut hal = GrabberHalFactory.create_hal()?;
    hal.calibrate()?;
    let bt = GrabberBehaviourTreeFactory.create_bt();

    for i in 0..num_runs {
        println!("Starting run #{i}...");
        hal = run_machine(bt.clone(), hal)?;
    }
    println!("Successful stress test!");
    Ok(())
}

fn run_machine(behaviour: Behavior<GrabberAction>, hal: Box<dyn GrabberHal>) -> anyhow::Result<Box<dyn GrabberHal>> {
    let mut machine: State<GrabberAction, ()> = State::new(behaviour);
    let mut state = GrabberState::new(hal);

    let mut dt = 0.0;
    let mut ticks = 0;
    let result = loop {
        let start = Instant::now();
        let e: Event = UpdateArgs { dt }.into();
        let (status, _) = machine.event(&e, &mut |args| {
            (args.action.action.handle(&mut state), args.dt)
        });

        // TODO: This of course should be a proper RT interval!
        thread::sleep(TICK_INTERVAL);
        dt = start.elapsed().as_secs_f64();
        ticks += 1;

        match status {
            Status::Success => break Ok(()),
            Status::Failure => break Err(anyhow!("Unknown state failure!")),
            Status::Running => {
                // Print that a tick happened but keep going...
                if ticks % 60 != 0 {
                    print!(".");
                    io::stdout().flush()?;
                } else {
                    println!();
                }
            }
        }
    };
    println!();

    // Oh hell, the ownership of the HAL isn't great but it's not worth fixing right now for this
    // tool.
    result.map(|_| state.into_hal())
}
