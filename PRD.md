# Resin Slicer — Product Requirements Document

## Vision

An open source, headless-first resin slicer that automates the entire STL → print-ready pipeline. Fork of [mslicer](https://github.com/connorslade/mslicer) (Rust, GPL-3.0) with added support generation, auto-orientation, auto-layout, and hollowing.

**One command, zero interaction:**
```bash
reslicer chibi.stl --height 4in --printer saturn3 -o ready.ctb
```

**Full pipeline integration:**
```
Photo → Grok Imagine → Tripo3D → reslicer → USB → Print
```

## Product Modes

### 1. CLI / Headless (Primary)
- Fully automated: STL in → .ctb out
- Scriptable, pipeable, composable
- Zero GUI dependency
- Perfect for automation pipelines, web APIs, CI/CD

### 2. GUI / Desktop (Secondary)
- Tauri + egui (mslicer already has egui + wgpu)
- 3D viewport with orbit controls
- Visual support editing (add/remove/move supports)
- Real-time slice preview
- For when you need manual control or want to inspect before printing

Both modes share the same core engine. GUI is a thin layer on top.

## Core Features

### P0 — Must Have (MVP)

#### Auto-Support Generation
The single most important missing feature in mslicer.
- **Overhang detection**: Identify faces below critical angle (default 45°)
- **Island detection**: Find floating geometry with no path to build plate
- **Pillar supports**: Vertical columns from build plate to overhang points
- **Contact tips**: Small tapered contact points (0.3-0.5mm) for clean removal
- **Raft generation**: Flat base for build plate adhesion
- Algorithm reference: PrusaSlicer SLA code (`src/libslic3r/SLA/`) + Vanek et al. "Clever Support" paper

#### Auto-Orientation
- Brute-force angle search (sample rotations across 2 axes, 5-10° increments)
- Score each orientation by: overhang area, support volume, max cross-section, detail preservation
- Pick optimal angle automatically
- CLI flag: `--auto-orient` (default on)

#### Scale to Target Size
- `--height 4in` or `--height 100mm`
- Calculate bounding box, apply uniform scale factor
- Preserve aspect ratio

#### Printer Profiles
- Built-in profiles for common printers:
  - Anycubic Photon M5s, Photon Mono M7, Photon Ultra
  - Elegoo Saturn 3, Saturn 3 Ultra, Mars 5 Ultra
  - Creality Halot series
  - Phrozen Sonic series
- Profile includes: build plate dimensions, pixel resolution, default exposure times
- Custom profiles via JSON/TOML

#### File Format Output
- .ctb v4 (most universal — covers Anycubic + Elegoo + most ChiTu-based printers)
- .goo (Elegoo native — mslicer already supports)
- Keep mslicer's existing format support

### P1 — Should Have

#### Auto-Layout / Multi-Model
- `reslicer a.stl b.stl c.stl --height 4in --printer saturn3 -o batch.ctb`
- 2D bin packing of model footprints on build plate
- Configurable spacing between models (default 5mm)
- Automatic arrangement to maximize plate usage

#### Hollowing
- Shell thickness parameter (default 2mm)
- Automatic drain hole placement (2 holes, bottom/side)
- Configurable hole diameter (default 3mm)
- Saves 40-60% resin on solid models

#### Anti-Aliasing
- Layer edge smoothing for smoother surface finish
- 4x or 8x anti-aliasing on layer bitmaps
- mslicer may already handle this

### P2 — Nice to Have

#### Tree Supports
- Branching support structures that merge as they reach the build plate
- Less resin than pillar supports, fewer contact points
- Algorithm: Vanek et al. tree support, or PrusaSlicer's branching approach

#### Web API Mode
- `reslicer serve --port 8080`
- REST API: POST STL → GET print-ready file
- For SaaS integration (another-dev-swag: user buys figure → auto-slice → ship)

#### Smart Orientation
- ML-based orientation scoring (train on successful/failed prints)
- Community-contributed orientation presets for common model types (figurines, miniatures, jewelry)

#### Print Time Estimation
- Calculate estimated print time based on layer count + exposure settings
- Resin volume estimation (model volume + supports)

#### Failure Prediction
- Detect high-risk areas (large flat overhangs, thin pillars, heavy unsupported mass)
- Warning system before committing to print

## Technical Architecture

```
┌──────────────────────────────────────────────┐
│                   reslicer                    │
├──────────────────────────────────────────────┤
│                                              │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  │
│  │   CLI    │  │   GUI    │  │ Web API  │  │
│  │ (clap)   │  │ (egui)   │  │ (axum)   │  │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  │
│       │              │              │        │
│  ┌────┴──────────────┴──────────────┴────┐  │
│  │           Core Engine (lib)            │  │
│  ├────────────────────────────────────────┤  │
│  │  ┌──────────┐  ┌──────────────────┐   │  │
│  │  │  Mesh    │  │  Support Engine  │   │  │
│  │  │  Loader  │  │  ┌────────────┐  │   │  │
│  │  │ STL/GLB  │  │  │ Overhang   │  │   │  │
│  │  │ OBJ/3MF  │  │  │ Detection  │  │   │  │
│  │  └──────────┘  │  ├────────────┤  │   │  │
│  │                │  │ Island     │  │   │  │
│  │  ┌──────────┐  │  │ Detection  │  │   │  │
│  │  │  Orient  │  │  ├────────────┤  │   │  │
│  │  │  + Scale │  │  │ Pillar Gen │  │   │  │
│  │  │  + Layout│  │  ├────────────┤  │   │  │
│  │  └──────────┘  │  │ Tree Gen   │  │   │  │
│  │                │  ├────────────┤  │   │  │
│  │  ┌──────────┐  │  │ Raft Gen   │  │   │  │
│  │  │ Hollower │  │  └────────────┘  │   │  │
│  │  └──────────┘  └──────────────────┘   │  │
│  │                                        │  │
│  │  ┌──────────────────────────────────┐  │  │
│  │  │        Slicer (existing)         │  │  │
│  │  │  Plane intersection + scanline   │  │  │
│  │  │  + RLE compression + AA          │  │  │
│  │  └──────────────────────────────────┘  │  │
│  │                                        │  │
│  │  ┌──────────────────────────────────┐  │  │
│  │  │     Format Encoders (existing)   │  │  │
│  │  │  CTB v4 │ GOO │ NanoDLP │ ...    │  │  │
│  │  └──────────────────────────────────┘  │  │
│  └────────────────────────────────────────┘  │
└──────────────────────────────────────────────┘
```

## Tech Stack

| Component | Technology | Notes |
|-----------|-----------|-------|
| Language | Rust | Already mslicer's language |
| CLI | clap | Standard Rust CLI framework |
| GUI | egui + wgpu | Already in mslicer |
| Desktop wrapper | Tauri v2 | Optional, for packaged desktop app |
| Web API | axum | If/when web mode is needed |
| Mesh processing | parry3d, nalgebra | 3D math |
| Parallelism | rayon | Layer slicing is embarrassingly parallel |
| GPU compute | wgpu | Optional acceleration for rasterization |
| Image | image crate | Bitmap generation |
| Mesh I/O | stl_io, gltf crate | STL + GLB loading |

## CLI Interface

```bash
# Basic usage
reslicer input.stl -o output.ctb

# Full options
reslicer input.stl \
  --height 4in \                    # Scale to 4 inches tall
  --printer saturn3 \               # Printer profile
  --layer-height 0.05 \             # 50 micron layers
  --exposure 2.5 \                  # Normal layer exposure (seconds)
  --bottom-exposure 30 \            # Bottom layer exposure
  --bottom-layers 5 \               # Number of bottom layers
  --supports auto \                 # auto | none | tree
  --orient auto \                   # auto | none
  --hollow 2mm \                    # Shell thickness (omit for solid)
  --drain-holes 2 \                 # Number of drain holes
  -o output.ctb

# Multi-model batch
reslicer a.stl b.stl c.stl \
  --height 4in \
  --printer saturn3 \
  --layout auto \
  --spacing 5mm \
  -o batch.ctb

# GUI mode
reslicer --gui

# Web API mode
reslicer serve --port 8080
```

## Success Metrics

- **MVP**: Single STL → supported + sliced .ctb in < 60 seconds
- **Quality**: Auto-supports produce successful prints ≥ 90% of the time on standard figurine models
- **Speed**: Slicing 100k poly mesh into 2000 layers in < 5 seconds (mslicer already achieves ~2s)
- **Adoption**: 100+ GitHub stars in first month (resin community is hungry for this)

## Risks

| Risk | Severity | Mitigation |
|------|----------|------------|
| Support generation quality | High | Start with simple pillars, iterate. Use PrusaSlicer SLA code as reference. |
| GPL-3.0 license from mslicer | Medium | Fine for open source. If SaaS needed, support engine could be a separate module. |
| Printer compatibility | Medium | Start with CTB v4 (covers 90% of printers). Add formats incrementally. |
| Auto-orientation accuracy | Low | Brute-force works. Can improve later with ML. |

## Timeline Estimate

| Phase | Scope | Est. Time |
|-------|-------|-----------|
| Phase 1 | Fork mslicer, add CLI mode, scale-to-height, printer profiles | 1-2 days |
| Phase 2 | Pillar support generation + raft | 3-5 days |
| Phase 3 | Auto-orientation | 1-2 days |
| Phase 4 | Auto-layout (multi-model) | 1-2 days |
| Phase 5 | Hollowing + drain holes | 2-3 days |
| Phase 6 | Tree supports | 3-5 days |
| Phase 7 | Polish, testing, printer profile library | 2-3 days |

## References

- mslicer: https://github.com/connorslade/mslicer
- UVtools (file formats): https://github.com/sn4k3/UVtools
- PrusaSlicer SLA supports: `src/libslic3r/SLA/SupportTreeBuilder.cpp`
- Vanek et al. "Clever Support" (2014): Tree support algorithm
- Research docs: `~/clawd/research/resin-slicer-research-claude.md` + `resin-slicer-research-codex.md`
