use clap::Parser;

use ev3_shuffl3bot::shuffle_solver::{CardMove, ShuffleSolution, ShuffleSolver, ShuffleSolverOptions};

#[derive(Parser, Debug)]
#[clap(name = "moves_generator")]
struct Opts {
  #[clap(short = 'n', long, default_value = "40")]
  deck_size: u32,

  #[clap(short = 's', long, default_value = "9")]
  num_stacks: u32,

  #[clap(short, long, default_value = "0")]
  input_stack: u32,

  #[clap(short, long, default_value = "2")]
  output_stack: u32,
}

fn main() {
  let opts: Opts = Opts::parse();

  let solution = ShuffleSolver::solve(ShuffleSolverOptions {
    deck_size: opts.deck_size as usize,
    num_stacks: opts.num_stacks as usize,
    input_stack: opts.input_stack as usize,
    output_stack: opts.output_stack as usize
  });

  for card_move in solution.required_moves {
    println!("{} {}", card_move.src_stack, card_move.dst_stack);
  }
}
