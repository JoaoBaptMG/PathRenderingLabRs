//--------------------------------------------------------------------
// pause.rs
//--------------------------------------------------------------------
// Utility function to pause the cursos
//--------------------------------------------------------------------

// Copied from https://users.rust-lang.org/t/rusts-equivalent-of-cs-system-pause/4494/4
use std::io;
use std::io::prelude::*;

// This is used in a feature, so we supress the warning
#[allow(dead_code)]
pub fn pause() {
    let stdin = io::stdin();
    let mut stdout = io::stdout();

    // We want the cursor to stay at the end of the line, so we print without a newline and flush manually.
    write!(stdout, "Press enter to continue...").unwrap();
    stdout.flush().unwrap();

    // Read a single line and discard
    let mut st = String::new();
    stdin.read_line(&mut st).unwrap();
}