# Resin Slicer — CLAUDE.md

## Project Context
This is a fork of [mslicer](https://github.com/connorslade/mslicer), an open source Rust MSLA/DLP resin slicer.

## Goal
Add headless CLI mode + automatic support generation to make this a fully automated STL → print-ready pipeline.

## Key Files
- `PRD.md` — Full product requirements
- `RESEARCH-CLAUDE.md` — Deep technical research (algorithms, formats, architecture)
- `RESEARCH-CODEX.md` — Additional research (slicing, supports, file formats)
- `slicer/` — Core slicing engine
- `format/` — Printer file format encoders
- `mslicer/` — GUI application (egui + wgpu)
- `common/` — Shared types

## Build
```bash
cargo build
cargo run  # launches GUI
```

## Architecture
The slicer library (`slicer/`) is separate from the GUI (`mslicer/`). 
We want to add a CLI binary that uses the slicer library directly.

## Priority
1. Add CLI binary (`cli/`) using clap
2. Add support generation module (`slicer/src/supports/`)
3. Add auto-orientation
4. Add scale-to-height
