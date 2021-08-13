#!/usr/bin/env python3

import argparse
import random
import sys

class ShuffleSolver:
  def __init__(self, deck_size: int, num_stacks: int, input_stack_index: int, output_stack_index: int):
    if deck_size <= 0:
      raise Exception('Invalid deck_size={}'.format(deck_size))

    if input_stack_index >= num_stacks:
      raise Exception('Invalid input_stack_index={}'.format(input_stack_index))

    if output_stack_index >= num_stacks:
      raise Exception('Invalid output_stack_index={}'.format(output_stack_index))

    self.deck_size = deck_size
    self.num_stacks = num_stacks
    self.input_stack_index = input_stack_index
    self.output_stack_index = output_stack_index

    stacks = []
    for i in range(num_stacks):
      stacks.append([])

    # Arrange the input such that the top-most card (i.e. the last one, the one you get with pop()) is the 0th.
    input_deck = [*range(deck_size - 1, -1, -1)]
    
    stacks[input_stack_index] += input_deck
    self.stacks = stacks

    output_deck = input_deck.copy()
    random.shuffle(output_deck)
    self.desired_draw_order = output_deck

    self.required_moves = []

  def solve(self):
    # Loop backwards so that the top of the output stack (i.e. the last one we physically move) is the 0th in
    # the desired draw order.
    for next_desired_output_index in range(len(self.desired_draw_order) - 1, -1, -1):
      next_desired_output = self.desired_draw_order[next_desired_output_index]
      which_stack_index = self._find_stack_index_with_card(next_desired_output)
      if which_stack_index < 0:
        raise Exception('Could not find: {}'.format(next_desired_output))

      which_stack = self.stacks[which_stack_index]

      while which_stack[-1] != next_desired_output:
        dst_stack_index = self._select_target_stack_naively(which_stack_index)
        self._move_card(which_stack_index, dst_stack_index)
      self._move_card(which_stack_index, self.output_stack_index)

    self._validate_result()

    return self.required_moves

  # Apply the moves then draw the output deck and verify it's in the intended draw order.
  def _validate_result(self):
    copy = ShuffleSolver(self.deck_size, self.num_stacks, self.input_stack_index, self.output_stack_index)
    for move in self.required_moves:
      card = copy.stacks[move[0]].pop()
      copy.stacks[move[1]].append(card)

    output = copy.stacks[copy.output_stack_index]
    actual_draw_order = []
    while len(output) > 0:
      actual_draw_order.append(output.pop())

    if actual_draw_order != self.desired_draw_order:
      raise Exception('Something went wrong, attach a debugger to find out what')

  def _find_stack_index_with_card(self, card: int) -> int:
    for index in range(len(self.stacks)):
      if card in self.stacks[index]:
        return index
    return -1

  def _select_target_stack_naively(self, src_stack_index) -> int:
    smallest_stack_size = 999999
    smallest_stack_index = -1

    for index in range(len(self.stacks)):
      stack_size = len(self.stacks[index])
      if index != src_stack_index and index != self.output_stack_index and stack_size < smallest_stack_size:
        smallest_stack_index = index
        smallest_stack_size = stack_size

    return smallest_stack_index

  def _move_card(self, src_stack_index, dst_stack_index): 
    if src_stack_index == dst_stack_index:
      raise Exception('Illegal move from and to {}'.format(src_stack_index))

    card = self.stacks[src_stack_index].pop()
    self.stacks[dst_stack_index].append(card)
    self.required_moves.append((src_stack_index, dst_stack_index))

def main():
  parser = argparse.ArgumentParser(description = "Generate moves for shuffl3bot")
  parser.add_argument('-n', '--deck-size', type=int, required = True, help = 'Number of cards in deck')
  parser.add_argument('-s', '--num-stacks', type=int, required = True, help = 'Total number of stacks')
  parser.add_argument('-i', '--input-stack', type=int, required = True, help = 'Input stack index')
  parser.add_argument('-o', '--output-stack', type=int, required = True, help = 'Output stack index')

  try:
      args = parser.parse_args()
  except:
      parser.print_help()
      sys.exit(0)

  solver = ShuffleSolver(
      deck_size = args.deck_size,
      num_stacks = args.num_stacks, 
      input_stack_index = args.input_stack, 
      output_stack_index = args.output_stack)
  moves = solver.solve()

  for move in moves:
    print('{} {}'.format(move[0], move[1]))

if __name__ == "__main__":
  main()