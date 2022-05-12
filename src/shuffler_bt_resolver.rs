use std::collections::VecDeque;

use ai_behavior::Behavior;

use crate::shuffle_solver::{CardMove, ShuffleSolver, ShuffleSolverOptions};
use crate::shuffler_bt_library::{ShufflerAction, ShufflerBehaviourTreeLibrary, ShufflerState, ShuffleStateArgs};
use crate::shuffler_hal::ShufflerHal;

const NUM_ROWS: usize = 3;
const NUM_COLS: usize = 3;
const INPUT_STACK: usize = 0;
const OUTPUT_STACK: usize = 6;

#[derive(Default)]
pub struct ShufflerBehaviourTreeResolver {
    options: Options,
}

#[derive(Default)]
pub struct Options {
    pub skip_moves: bool,
    pub fake_hw: bool,
}

impl ShufflerBehaviourTreeResolver {
    pub fn new(options: Options) -> Self {
        Self { options }
    }

    pub fn resolve(&self, hal: Box<dyn ShufflerHal>, deck_size: usize) -> ShufflerTreeHolder {
        let moves = if self.options.skip_moves {
            let dummy_move = CardMove { src_stack: 0, dst_stack: 0 };
            VecDeque::from(vec![dummy_move; deck_size])
        } else {
            let solution = ShuffleSolver::solve(ShuffleSolverOptions {
                deck_size,
                num_stacks: NUM_ROWS * NUM_COLS,
                input_stack: INPUT_STACK,
                output_stack: OUTPUT_STACK,
            });
            VecDeque::from(solution.required_moves)
        };
        println!("Generated {} moves, let's do this...", moves.len());

        let bt = ShufflerBehaviourTreeLibrary.shuffle_or_grabber_stress_bt(self.options.skip_moves, self.options.fake_hw);
        let state = ShufflerState::new(ShuffleStateArgs {
            hal,
            num_rows: NUM_ROWS,
            moves_queue: Some(moves),
            cleanup_stacks_queue: None,
        });
        ShufflerTreeHolder { bt, state }
    }
}

pub struct ShufflerTreeHolder {
    pub bt: Behavior<ShufflerAction>,
    pub state: ShufflerState,
}
