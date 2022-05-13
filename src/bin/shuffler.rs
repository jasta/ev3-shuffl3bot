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
use clap::{ArgEnum, Parser};
use input::{Event, UpdateArgs};
use log::{info, LevelFilter};

use ev3_shuffl3bot::shuffler_bt_library::{ShufflerAction, ShufflerState};
use ev3_shuffl3bot::shuffler_hal::ShufflerHal;
use ev3_shuffl3bot::shuffler_hal_factory::ShufflerHalFactory;
use ev3_shuffl3bot::shuffle_solver::{CardMove, ShuffleSolver, ShuffleSolverOptions};
use ev3_shuffl3bot::shuffler_bt_resolver::{Options, ResolverMode, ShufflerBehaviourTreeResolver, ShufflerTreeHolder};
use ev3_shuffl3bot::shuffler_bt_runner::ShufflerBehaviourTreeRunner;
use crate::CliMode::Calibrate;

#[derive(Parser, Debug)]
#[clap(name = "shuffler")]
struct Opts {
    #[clap(short = 'n', long, default_value = "40")]
    deck_size: usize,

    #[clap(long)]
    skip_moves: bool,

    #[clap(long)]
    fake_hw: bool,

    #[clap(short = 'm', long, arg_enum, default_value = "shuffle")]
    mode: CliMode,
}

#[derive(PartialEq, Eq, Debug, Clone, ArgEnum)]
pub enum CliMode {
    Shuffle,
    Cleanup,
    Calibrate,
}

fn main() -> anyhow::Result<()> {
    let opts: Opts = Opts::parse();

    env_logger::builder()
        .filter_level(LevelFilter::Debug)
        .parse_default_env()
        .init();

    let mut hal = ShufflerHalFactory::new_maybe_mock(opts.fake_hw).create_hal()?;
    hal.calibrate_grabber()?;
    if !opts.skip_moves {
        hal.calibrate_gantry()?;
    }
    let bt_mode = match opts.mode {
        Calibrate => return Ok(()),
        CliMode::Shuffle => ResolverMode::Shuffle,
        CliMode::Cleanup => ResolverMode::CleanupStacks,
    };
    let bt_factory = ShufflerBehaviourTreeResolver::new(Options {
        skip_moves: opts.skip_moves,
        fake_hw: opts.fake_hw,
        mode: bt_mode,
    });

    let holder = bt_factory.resolve(hal, opts.deck_size);
    ShufflerBehaviourTreeRunner::new(holder).run()?;
    info!("Success!");
    Ok(())
}
