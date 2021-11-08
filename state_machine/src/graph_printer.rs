use std::any::TypeId;

use crate::graph::StateGraph;

pub struct StateGraphPrinter;

impl StateGraphPrinter {
  /// # Example output:
  ///
  /// ```text
  /// StateGraph {
  ///   Uninitialized => Initializing
  ///   Initializing => [Ready, Uninitialized]
  ///   Initialized { => Uninitialized
  ///     Ready => Working
  ///     Working => Ready
  ///   }
  /// }
  /// ```
  pub fn pretty_print<C, E>(states: &StateGraph<C, E>) {
    println!("StateGraph {{");
    StateGraphPrinter::print_internal(states, "StateGraph", None, 1);
    println!("}}");
  }

  fn print_internal<C, E>(
    states: &StateGraph<C, E>,
    parent_name: &'static str,
    parent_type_id: Option<TypeId>,
    indent: usize) {
    let children = states.nodes.values().filter(|s| {
      s.parent_type_id == parent_type_id
    });

    // Stupid inefficient iteration but since this is just for pretty printing who really cares.
    // I really don't know what the proper idiomatic way to do this in Rust would even be either...
    for &ordered_type_id in &states.insertion_order {
      let found = children.clone().find(|s| {
        s.type_id == ordered_type_id
      });
      if let Some(node) = found {
        for _ in 0..indent {
          print!("  ");
        }

        let node_name = states.nodes.get(&node.type_id).unwrap().debug_name;
        print!("{}", node_name);

        if node.is_parent {
          print!(" {{");
        }

        let num_transitions = node.transitions_to.len();
        if num_transitions > 0 {
          print!(" => ");
          let transition_suffix = if num_transitions > 1 {
            print!("[");
            "]"
          } else {
            ""
          };
          let names: Vec<&str> = node.transitions_to.iter().map(|t| {
            states.nodes.get(t).unwrap().debug_name
          }).collect();
          print!("{}", names.join(", "));
          print!("{}", transition_suffix);
        }
        println!();

        if node.is_parent {
          StateGraphPrinter::print_internal(states, node_name, Some(node.type_id), indent + 1);
          for _ in 0..indent {
            print!("  ");
          }
          println!("}}");
        }
      }
    }
  }
}
