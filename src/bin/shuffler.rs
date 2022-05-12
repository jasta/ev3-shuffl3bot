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
use std::collections::VecDeque;
use std::io::Write;
use std::time::{Duration, Instant};

use ai_behavior::{Behavior, State, Status};
use anyhow::anyhow;
use clap::Parser;
use input::{Event, UpdateArgs};

use ev3_shuffl3bot::shuffler_bt::{ShufflerAction, ShufflerBehaviourTreeFactory, ShufflerState, Options};
use ev3_shuffl3bot::shuffler_hal::ShufflerHal;
use ev3_shuffl3bot::shuffler_hal_factory::ShufflerHalFactory;
use ev3_shuffl3bot::shuffle_solver::{CardMove, ShuffleSolver, ShuffleSolverOptions};

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

const TICK_INTERVAL: Duration = Duration::from_millis(20);
const NUM_ROWS: usize = 3;
const NUM_COLS: usize = 3;
const INPUT_STACK: usize = 0;
const OUTPUT_STACK: usize = 6;

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
    let bt_factory = ShufflerBehaviourTreeFactory::new(Options {
        skip_moves: opts.skip_moves,
        fake_hw: opts.fake_hw,
    });
    let bt = bt_factory.create_bt();

    let moves = if opts.skip_moves {
        let dummy_move = CardMove { src_stack: 0, dst_stack: 0 };
        VecDeque::from(vec![dummy_move; opts.deck_size])
    } else {
        let solution = ShuffleSolver::solve(ShuffleSolverOptions {
            deck_size: opts.deck_size,
            num_stacks: NUM_ROWS * NUM_COLS,
            input_stack: INPUT_STACK,
            output_stack: OUTPUT_STACK,
        });
        VecDeque::from(solution.required_moves)
    };
    println!("Generated {} moves, let's do this...", moves.len());
    run_machine(bt, hal, moves)?;
    println!("Success!");
    Ok(())
}

fn run_machine(
    behaviour: Behavior<ShufflerAction>,
    hal: Box<dyn ShufflerHal>,
    moves: VecDeque<CardMove>,
) -> anyhow::Result<Box<dyn ShufflerHal>> {
    let mut machine: State<ShufflerAction, ()> = State::new(behaviour);
    let mut state = ShufflerState::new(hal, moves, NUM_ROWS);

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
            Status::Failure if state.is_really_success() => break Ok(()),
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
