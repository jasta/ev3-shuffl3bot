use std::collections::VecDeque;
use std::path::{Path, PathBuf};

use ai_behavior::Behavior;
use log::info;

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
    pub mode: ResolverMode,
    pub deck_size: usize,
    pub state_in: Option<PathBuf>,
}

pub enum ResolverMode {
    Shuffle,
    CleanupStacks,
}

impl Default for ResolverMode {
    fn default() -> Self {
        ResolverMode::Shuffle
    }
}

impl ShufflerBehaviourTreeResolver {
    pub fn new(options: Options) -> Self {
        Self { options }
    }

    pub fn resolve(&self, hal: Box<dyn ShufflerHal>) -> anyhow::Result<ShufflerTreeHolder> {
        let library = ShufflerBehaviourTreeLibrary;
        match self.options.mode {
            ResolverMode::Shuffle => {
                let bt = library.MoveCards(self.options.skip_moves, self.options.fake_hw);

                let state = match self.options.state_in {
                    Some(ref path) => {
                        ShufflerState::from_save_state(hal, path)?
                    }
                    None => {
                        let moves = if self.options.skip_moves {
                            let dummy_move = CardMove { src_stack: 0, dst_stack: 0 };
                            VecDeque::from(vec![dummy_move; self.options.deck_size])
                        } else {
                            let solution = ShuffleSolver::solve(ShuffleSolverOptions {
                                deck_size: self.options.deck_size,
                                num_stacks: NUM_ROWS * NUM_COLS,
                                input_stack: INPUT_STACK,
                                output_stack: OUTPUT_STACK,
                            });
                            VecDeque::from(solution.required_moves)
                        };
                        info!("Generated {} moves, let's do this...", moves.len());

                        ShufflerState::new(ShuffleStateArgs {
                            hal,
                            num_rows: NUM_ROWS,
                            moves_queue: Some(moves),
                            cleanup_stacks_queue: None,
                        })
                    }
                };
                Ok(ShufflerTreeHolder { bt, state })
            },
            ResolverMode::CleanupStacks => {
                let bt = library.MoveAllCardsToStack(INPUT_STACK, self.options.fake_hw);

                let state = match self.options.state_in {
                    Some(ref path) => {
                        ShufflerState::from_save_state(hal, path)?
                    }
                    None => {
                        let mut cleanup_stacks_queue: Vec<_> = (0..(NUM_ROWS * NUM_COLS)).collect();
                        cleanup_stacks_queue.retain(|&stack| stack != INPUT_STACK);
                        ShufflerState::new(ShuffleStateArgs {
                            hal,
                            num_rows: NUM_ROWS,
                            moves_queue: None,
                            cleanup_stacks_queue: Some(VecDeque::from(cleanup_stacks_queue)),
                        })
                    }
                };
                Ok(ShufflerTreeHolder { bt, state })
            },
        }
    }
}

pub struct ShufflerTreeHolder {
    pub bt: Behavior<ShufflerAction>,
    pub state: ShufflerState,
}
