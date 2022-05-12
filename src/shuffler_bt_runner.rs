use std::{io, thread};
use std::io::Write;
use std::time::{Duration, Instant};

use ai_behavior::{State, Status};
use anyhow::anyhow;
use input::{Event, UpdateArgs};

use crate::shuffler_bt_library::ShufflerAction;
use crate::shuffler_bt_resolver::ShufflerTreeHolder;

const TICK_INTERVAL: Duration = Duration::from_millis(20);

pub struct ShufflerBehaviourTreeRunner {
    holder: ShufflerTreeHolder,
}

impl ShufflerBehaviourTreeRunner {
    pub fn new(holder: ShufflerTreeHolder) -> Self {
        Self { holder }
    }

    pub fn run(mut self) -> anyhow::Result<()> {
        let mut machine: State<ShufflerAction, ()> = State::new(self.holder.bt);

        let mut dt = 0.0;
        let mut ticks = 0;
        let result = loop {
            let start = Instant::now();
            let e: Event = UpdateArgs { dt }.into();
            let (status, _) = machine.event(&e, &mut |args| {
                (args.action.action.handle(&mut self.holder.state), args.dt)
            });

            // TODO: This of course should be a proper RT interval!
            thread::sleep(TICK_INTERVAL);
            dt = start.elapsed().as_secs_f64();
            ticks += 1;

            match status {
                Status::Success => break Ok(()),
                Status::Failure if self.holder.state.is_really_success() => break Ok(()),
                Status::Failure => {
                    self.holder.state.dump();
                    break Err(anyhow!("Unknown state failure!"))
                },
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

        result
    }
}
