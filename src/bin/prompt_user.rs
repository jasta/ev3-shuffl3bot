use std::error::Error;
use std::process::Command;
use anyhow::anyhow;

use cursive::event::Event::Key;
use cursive::event::Key::Enter;
use cursive::theme::{BorderStyle, Color, Palette, PaletteColor, Theme};
use cursive::views::{Dialog, LinearLayout, NamedView, SliderView, TextView};
use cursive::With;

fn main() -> anyhow::Result<()> {
  set_terminal_font();

  let mut siv = cursive::default();
  siv.set_theme(load_theme());
  siv.add_global_callback(Key(Enter), |s| s.quit());
  let default_value = 40;
  siv.add_layer(
    LinearLayout::vertical()
        .child(
          NamedView::new("deck_size_label", TextView::new(default_value.to_string())))
        .child(
          NamedView::new(
            "deck_size",
            SliderView::horizontal(60)
                .value(default_value)
                .on_change(|s, deck_size| {
                  s.call_on_name("deck_size_label", |v: &mut TextView| {
                    v.set_content(deck_size.to_string());
                  });
                }))));
  siv.run();

  let mut deck_size = siv.find_name::<SliderView>("deck_size").unwrap();
  println!("You selected: {}", deck_size.get_value());

  Ok(())
}

fn set_terminal_font() -> anyhow::Result<()> {
  let setfont_result = Command::new("setfont").arg("Lat15-TerminusBold24x12").status()?;
  if !setfont_result.success() {
    return Err(anyhow!("setfont failed: {}, proceeding anyway...", setfont_result.code()
      .unwrap()));
  }
  Ok(())
}

fn load_theme() -> Theme {
  let mut palette = Palette::default();
  palette.set_basic_color("Background", Color::TerminalDefault)
      .expect("Background not legal color??");
  return cursive::theme::Theme {
    shadow: false,
    borders: BorderStyle::None,
    palette,
  };
}