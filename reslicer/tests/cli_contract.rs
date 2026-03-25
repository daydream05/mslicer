use std::{
    fs,
    path::PathBuf,
    time::{SystemTime, UNIX_EPOCH},
};

use clap::Parser;
use common::slice::Format;
use reslicer::{cli::Cli, printer::PrinterId, run};

#[test]
fn parses_target_command_contract() {
    let cli = Cli::parse_from([
        "reslicer",
        "input.stl",
        "--height",
        "4in",
        "--printer",
        "saturn3",
        "-o",
        "ready.ctb",
    ]);

    assert_eq!(cli.inputs, vec![PathBuf::from("input.stl")]);
    assert_eq!(cli.height.unwrap().as_millimeters(), 101.6);
    assert_eq!(cli.printer, Some(PrinterId::Saturn3));
    assert_eq!(cli.output, PathBuf::from("ready.ctb"));
}

#[test]
fn slices_stl_to_ctb_with_height_and_printer_profile() {
    let temp_root = std::env::temp_dir().join(format!(
        "reslicer-test-{}",
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    fs::create_dir_all(&temp_root).unwrap();

    let input = temp_root.join("input.stl");
    let output = temp_root.join("ready.ctb");
    fs::write(&input, ascii_wedge_stl()).unwrap();

    let cli = Cli::parse_from([
        "reslicer",
        input.to_str().unwrap(),
        "--height",
        "4in",
        "--printer",
        "saturn3",
        "-o",
        output.to_str().unwrap(),
    ]);

    run(cli).unwrap();

    assert!(output.exists());
    assert!(fs::metadata(&output).unwrap().len() > 0);
    assert_eq!(Format::from_extension("ctb"), Some(Format::Ctb));

    let _ = fs::remove_dir_all(temp_root);
}

fn ascii_wedge_stl() -> &'static str {
    r#"solid wedge
facet normal 0 0 -1
  outer loop
    vertex 0 0 0
    vertex 20 0 0
    vertex 0 20 0
  endloop
endfacet
facet normal 0 0 1
  outer loop
    vertex 0 0 10
    vertex 0 20 0
    vertex 20 0 0
  endloop
endfacet
facet normal 0 -1 0
  outer loop
    vertex 0 0 0
    vertex 0 0 10
    vertex 20 0 0
  endloop
endfacet
facet normal -1 0 0
  outer loop
    vertex 0 0 0
    vertex 0 20 0
    vertex 0 0 10
  endloop
endfacet
facet normal 0.577 0.577 0.577
  outer loop
    vertex 20 0 0
    vertex 0 0 10
    vertex 0 20 0
  endloop
endfacet
endsolid wedge
"#
}
