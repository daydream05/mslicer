use anyhow::Result;
use clap::Parser;

use reslicer::{cli::Cli, run};

fn main() -> Result<()> {
    run(Cli::parse())
}
