use rand::prelude::SliceRandom;
use rand::thread_rng;

pub struct ShuffleSolverOptions {
  pub deck_size: usize,
  pub num_stacks: usize,
  pub input_stack: usize,
  pub output_stack: usize,
}

pub struct ShuffleSolution {
  pub required_moves: Vec<CardMove>
}

pub struct CardMove {
  pub src_stack: usize,
  pub dst_stack: usize,
}

pub struct ShuffleSolver {
  options: ShuffleSolverOptions,

  stacks: Vec<Vec<usize>>,
  desired_draw_order: Vec<usize>,

  required_moves: Vec<CardMove>,
}

impl ShuffleSolver {
  pub fn solve(options: ShuffleSolverOptions) -> ShuffleSolution {
    let mut solver = ShuffleSolver::init_internal(options);
    solver.solve_internal();
    solver.validate_solution();
    return ShuffleSolution { required_moves: solver.required_moves };
  }

  fn init_internal(options: ShuffleSolverOptions) -> ShuffleSolver {
    let mut stacks: Vec<Vec<usize>> = Vec::with_capacity(options.num_stacks);
    for _ in 0..options.num_stacks {
      stacks.push(Vec::new());
    }

    // Arrange the input such that the top-most card (i.e. the last one, the one you get with
    // pop() is the 0th.
    let mut input_deck: Vec<usize> = (0 .. options.deck_size).rev().collect();
    let mut desired_draw_order = input_deck.to_vec();
    desired_draw_order.shuffle(&mut thread_rng());

    stacks[options.input_stack].append(&mut input_deck);

    return ShuffleSolver { options, stacks, desired_draw_order, required_moves: Vec::new() };
  }

  fn solve_internal(&mut self) {
    // Loop backwards so that the top of the output stack (i.e. the last one we physically
    // move) is the 0th in the desired draw order.
    for next_desired_output_index in (0 .. self.desired_draw_order.len()).rev() {
      let next_desired_output = self.desired_draw_order[next_desired_output_index];
      let which_stack_index = self.find_stack_index_with_card(next_desired_output).unwrap();

      while self.stacks[which_stack_index].last().copied().unwrap() != next_desired_output {
        let dst_stack_index = self.select_target_stack_naively(which_stack_index).unwrap();
        self.move_card(which_stack_index, dst_stack_index);
      }
      self.move_card(which_stack_index, self.options.output_stack);
    }
  }

  fn validate_solution(&self) {
    let mut other = ShuffleSolver::init_internal(ShuffleSolverOptions {
      ..self.options
    });
    for card_move in &self.required_moves {
      other.move_card(card_move.src_stack, card_move.dst_stack);
    }
    assert_eq!(self.stacks, other.stacks);
  }

  fn select_target_stack_naively(&self, src_stack_index: usize) -> Option<usize> {
    let mut smallest_stack_size = usize::MAX;
    let mut smallest_stack_index = None;

    for index in 0 .. self.stacks.len() {
      let stack_size = self.stacks[index].len();
      if index != src_stack_index &&
          index != self.options.output_stack &&
          stack_size < smallest_stack_size {
        smallest_stack_index = Some(index);
        smallest_stack_size = stack_size;
      }
    }

    return smallest_stack_index;
  }

  fn find_stack_index_with_card(&self, card: usize) -> Option<usize> {
    return self.stacks.iter().position(|v| v.contains(&card));
  }

  fn move_card(&mut self, src_stack_index: usize, dst_stack_index: usize) {
    if src_stack_index == dst_stack_index {
      panic!("Illegal move from and to {}", src_stack_index);
    }

    let card = self.stacks[src_stack_index].pop().unwrap();
    self.stacks[dst_stack_index].push(card);
    self.required_moves.push(CardMove { src_stack: src_stack_index, dst_stack:
    dst_stack_index });
  }
}
