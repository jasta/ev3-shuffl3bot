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

use std::{io, thread};
use std::collections::VecDeque;
use std::io::Write;
use std::time::{Duration, Instant};

use ai_behavior::{Behavior, State, Status};
use anyhow::anyhow;
use clap::Parser;
use input::{Event, UpdateArgs};

use ev3_shuffl3bot::shuffler_bt_library::{ShufflerAction, ShufflerState};
use ev3_shuffl3bot::shuffler_hal::ShufflerHal;
use ev3_shuffl3bot::shuffler_hal_factory::ShufflerHalFactory;
use ev3_shuffl3bot::shuffle_solver::{CardMove, ShuffleSolver, ShuffleSolverOptions};
use ev3_shuffl3bot::shuffler_bt_resolver::{Options, ShufflerBehaviourTreeResolver, ShufflerTreeHolder};
use ev3_shuffl3bot::shuffler_bt_runner::ShufflerBehaviourTreeRunner;

#[derive(Parser, Debug)]
#[clap(name = "shuffler")]
struct Opts {
    #[clap(short = 'n', long, default_value = "40")]
    deck_size: usize,

    #[clap(long)]
    skip_moves: bool,

    #[clap(long)]
    fake_hw: bool,

    #[clap(long)]
    calibrate_only: bool,
}

fn main() -> anyhow::Result<()> {
    let opts: Opts = Opts::parse();

    let mut hal = ShufflerHalFactory::new_maybe_mock(opts.fake_hw).create_hal()?;
    hal.calibrate_grabber()?;
    if !opts.skip_moves {
        hal.calibrate_gantry()?;
    }
    if opts.calibrate_only {
        return Ok(());
    }
    let bt_factory = ShufflerBehaviourTreeResolver::new(Options {
        skip_moves: opts.skip_moves,
        fake_hw: opts.fake_hw,
    });

    let holder = bt_factory.resolve(hal, opts.deck_size);
    ShufflerBehaviourTreeRunner::new(holder).run()?;
    println!("Success!");
    Ok(())
}
