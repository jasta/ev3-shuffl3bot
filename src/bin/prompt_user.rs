use cursive::event::Event::Key;
use cursive::event::Key::Enter;
use cursive::views::{Dialog, TextView};

fn main() {
  let mut siv = cursive::default();
  siv.add_global_callback(Key(Enter), |s| s.quit());
  siv.add_layer(
      TextView::new("Hello!"));
  siv.run();
}
