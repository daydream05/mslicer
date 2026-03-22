# Open Source Resin (MSLA/DLP) Slicer — Deep Research

> Comprehensive research for building a Rust + Tauri desktop slicer to replace Chitubox/Lychee.
> Compiled: 2026-03-21

---

## Table of Contents

1. [Cross-Section Slicing Math](#1-cross-section-slicing-math)
2. [Support Generation](#2-support-generation--the-hard-problem)
3. [Printer File Formats](#3-printer-file-format-deep-dive)
4. [Build Plate Auto-Layout](#4-build-plate-auto-layout)
5. [Performance Considerations](#5-performance-considerations)
6. [Competitive Analysis](#6-competitive-analysis)
7. [Rust Ecosystem](#7-rust-ecosystem)
8. [Proposed Architecture](#8-proposed-architecture)
9. [Risk Assessment](#9-risk-assessment)
10. [References](#10-references)

---

## 1. Cross-Section Slicing Math

### Core Algorithm: Plane-Mesh Intersection

Given a triangle mesh and a Z height, compute the 2D polygon cross-section:

```
ALGORITHM: SliceAtHeight(mesh, z_height)
─────────────────────────────────────────
INPUT:  Triangle mesh M, slice height Z
OUTPUT: Set of 2D line segments forming closed contours

1. For each triangle T = (v0, v1, v2) in active set:
   a. Compute signed distances to plane:
      d0 = v0.z - Z
      d1 = v1.z - Z
      d2 = v2.z - Z
   
   b. For each edge (vi, vj) where sign(di) ≠ sign(dj):
      t = di / (di - dj)
      intersection = vi + t * (vj - vi)
      // Project to 2D: keep only (x, y)
   
   c. Each intersected triangle produces exactly 0 or 2 points
      → yields one line segment per triangle

2. Collect all segments → chain into closed contours
   (head-to-tail matching with epsilon tolerance)

3. Return contour set (may include holes as inner contours)
```

**Key optimization — Active Triangle Set (Plane Sweep):**

```
OPTIMIZATION: IncrementalSlicing
──────────────────────────────────
1. Sort all triangles by z_min
2. Maintain active set A
3. For each slice plane z_i (ascending):
   a. ADD triangles where z_min ≤ z_i (newly entering)
   b. REMOVE triangles where z_max < z_i (fully passed)
   c. Only test triangles in A
   
Complexity: O(n + k + m) when pre-sorted
  vs naive O(n*k) where n=triangles, k=layers
```

**Practical note from mslicer (connorslade):** BVH acceleration doesn't help for plane intersections because planes extend infinitely — too many bounding volumes get hit. A simple Z-segment bucketing approach (divide Z range into N buckets, assign each triangle to its buckets) works far better.

### Contour Reconstruction

Line segments from triangle intersections must be chained into closed polygons:
- **Simple approach:** Hash endpoints, walk chains until closure
- **Robust approach:** Use epsilon-based spatial hashing (merge points within tolerance)
- **Edge cases:** Non-manifold meshes can produce open contours or dangling segments

### Scanline Rasterization (Polygon → Bitmap)

For each layer, convert 2D contours to a monochrome bitmap at printer resolution (e.g., 11520×5120 for Elegoo Saturn 3 Ultra 12K):

```
ALGORITHM: ScanlineFill(contours, resolution)
──────────────────────────────────────────────
INPUT:  2D line segments with face normals, pixel resolution (W, H)
OUTPUT: RLE-encoded monochrome bitmap

For each row y in 0..H:
  1. Find all segment-row intersections:
     For each segment (a, b, facing):
       if (a.y > y) XOR (b.y > y):
         t = (y - a.y) / (b.y - a.y)
         x_intersect = a.x + t * (b.x - a.x)
         → collect (x_intersect, facing)
  
  2. Sort intersections by x position
  
  3. Depth tracking (handles overlapping/self-intersecting meshes):
     depth = 0
     filtered = []
     For each (x, direction) in sorted intersections:
       prev_depth = depth
       depth += (direction ? +1 : -1)
       if (depth == 0) XOR (prev_depth == 0):
         filtered.push(x)
  
  4. Encode runs:
     For each pair (x_start, x_end) in filtered:
       emit gap run (black pixels before x_start)
       emit fill run (white pixels from x_start to x_end)
```

**Depth tracking is critical** — it's what makes overlapping meshes and boolean operations work correctly. The `facing` direction (from triangle normal) determines whether you're entering (+1) or exiting (-1) a solid volume.

### Handling Holes and Nested Contours

The depth-tracking scanline approach handles nested contours automatically:
- **Outer contour:** depth goes 0→1 (start filling)
- **Hole contour:** depth goes 1→0 (stop filling)
- **Nested solid:** depth goes 0→1 again (resume filling)

This is equivalent to the **even-odd rule** but generalized with winding numbers via face normals.

### Non-Manifold Geometry

Common in user-submitted STL files. Strategies:
- **Detection:** Edges shared by >2 faces, vertices with disjoint face fans
- **Repair:** Delete excess faces, duplicate vertices per fan (MeshLib approach)
- **Tolerance:** The depth-tracking algorithm is somewhat resilient — incorrect winding just produces visual artifacts rather than crashes

### Relevant Rust Libraries for Mesh I/O & Math

| Crate | Purpose | Notes |
|-------|---------|-------|
| `nalgebra` | Linear algebra (vectors, matrices, transforms) | Foundation for all 3D math |
| `parry3d` | Collision detection, geometry queries | Mesh-plane intersection helpers possible |
| `stl_io` | Lightweight STL reading/writing | Simple, well-tested |
| `gltf` | glTF 2.0 file format | For glTF/GLB import |
| `baby_shark` | STL/OBJ/PLY I/O + mesh simplification | Auto-format detection, edge decimation |
| `mesh-repair` | Triangle mesh repair, shell generation | Watertight preparation |
| `ordered-float` | Sortable floats (for scanline sorting) | Essential utility |

---

## 2. Support Generation — The Hard Problem

### Overhang Detection

```
ALGORITHM: DetectOverhangs(mesh, critical_angle=45°)
────────────────────────────────────────────────────
For each triangle face F:
  angle = acos(F.normal · UP_VECTOR)
  if angle > (180° - critical_angle):  // face points downward
    classify as OVERHANG
    
  overhang_severity = angle - (180° - critical_angle)
  // Higher severity = more support needed
```

**Practical thresholds:**
- **<30° from horizontal:** Definitely needs support
- **30-45°:** Usually needs support depending on resin/exposure
- **>45° from horizontal:** Typically self-supporting
- Resin printing is less forgiving than FDM — even 40° overhangs often fail

### Island Detection

Islands are disconnected geometry regions that have no path to the build plate at a given layer:

```
ALGORITHM: DetectIslands(layer_contours, previous_layer_contours)
────────────────────────────────────────────────────────────────
For each connected region R in current layer:
  if R has NO overlap with any region in previous layer:
    → R is an ISLAND (floating in mid-resin)
    → MUST have support or will fail

Accuracy levels:
  - Fast: Check every Nth layer, large bounding box overlap
  - Detailed: Per-layer pixel-level connectivity analysis
  - Real: Sub-pixel with area threshold filtering
```

**Important nuance:** Islands smaller than the light support tip diameter (~0.3mm) often don't need support — adding support can cause more surface damage than the small unsupported island.

### Pillar/Column Support Placement

The simplest support type — vertical cylinders from overhang points to build plate:

```
ALGORITHM: PlaceColumnSupports(overhang_points, config)
───────────────────────────────────────────────────────
INPUT: Set of overhang sample points, support parameters

1. Sample overhang surfaces at regular grid spacing
   (e.g., every 2-5mm depending on density setting)

2. For each sample point P:
   a. Cast ray downward from P
   b. If ray hits build plate: place full column
   c. If ray hits model surface: 
      - If surface is self-supporting: place column to surface
      - Else: extend through to build plate
   
3. Generate column geometry:
   - Tip: small sphere/cone contact point (0.3-0.6mm)
   - Shaft: cylinder (0.4-1.2mm diameter)
   - Base: wider cylinder on build plate (2-4mm)
   
4. Collision check: ensure columns don't intersect model
```

### Tree Support Algorithm

The most material-efficient approach. Based on Steiner tree minimization:

```
ALGORITHM: GenerateTreeSupports(overhang_points)
─────────────────────────────────────────────────
Based on "Clever Support" (Vanek et al., 2014)

1. COLLECT support contact points from overhang detection
   
2. For each layer (top-down):
   a. Project contact points downward
   b. MERGE nearby branches:
      - If two branch tips are within merge_distance:
        Create junction point at midpoint
        Connect both tips to junction
   c. Apply angle constraints:
      - Branch angle from vertical ≤ max_branch_angle (~30°)
      - This ensures branch is printable without self-support

3. Continue merging + growing downward until:
   - Branch reaches build plate → anchor with base pad
   - Branch merges into existing branch → junction node

4. Generate geometry for each branch segment:
   - Tip: light contact sphere (minimize surface marks)
   - Branch: tapered cylinder (thicker toward base)
   - Junction: smooth blend between merged branches
   - Base: wider pad on build plate

OPTIMIZATIONS:
  - Use Particle Swarm Optimization (PSO) to optimize 
    junction placement for minimum total volume
  - Greedy linking: connect each leaf to nearest existing 
    branch when possible
```

### Light Support Tips

Critical for surface quality — the contact point between support and model:

```
Contact point anatomy:
  ┌─── Model surface
  ▼
  ·  ← Contact point (0.2-0.4mm sphere)
  │  ← Penetration depth (0.1-0.3mm into model)
  │
  ╲  ← Tip taper (cone from 0.3mm to shaft diameter)
  │
  │  ← Support shaft
```

**Key parameters:**
- **Contact diameter:** 0.2-0.6mm (smaller = less scarring, weaker hold)
- **Penetration depth:** How far tip enters model surface (too little = detachment)
- **Taper angle:** Gradual widening from contact to shaft

### Raft/Base Generation

```
ALGORITHM: GenerateRaft(support_bases, config)
──────────────────────────────────────────────
1. Compute convex hull of all support base positions
2. Offset hull outward by raft_margin (2-5mm)
3. Generate raft layers:
   - Bottom layers: thicker (0.1mm), longer exposure
   - Transition layers: gradual taper to normal
4. Connect support bases to raft surface
```

### Key References for Support Generation

| Source | Type | Value |
|--------|------|-------|
| **PrusaSlicer SLA** (`src/libslic3r/SLA/`) | C++ open source | Best open-source SLA support implementation; tree supports, branching, contact optimization |
| **"Clever Support" (Vanek et al., 2014)** | Academic paper | Tree support generation algorithm; Steiner tree minimization |
| **"Local Barycenter Based Tree Support" (LBTS)** | Academic paper | Efficient tree generation using local barycenters |
| **UVtools** (github.com/sn4k3/UVtools) | C# open source | Post-processing, island detection, support analysis |
| **Runebrace** (tarabella.it/Runebrace) | Closed source | Recommended by mslicer author; dedicated support placement tool |
| **PrusaSlicer v2.6+** branching supports | C++ open source | Experimental tree supports for SLA, inspired by "Clever Support" |
| **PrusaSlicer v2.9.1** | C++ open source | Completely new non-randomized SLA support spot generator |

**No standalone open-source support generation library exists in any language.** PrusaSlicer's implementation is the closest, but deeply embedded in their codebase. This is the single hardest feature to implement.

---

## 3. Printer File Format Deep Dive

### Format Landscape

| Format | Extension | Printers | Importance |
|--------|-----------|----------|------------|
| **CTB v3/v4** | `.ctb` | Chitu-based (90% of consumer printers) | ★★★★★ Critical |
| **GOO** | `.goo` | Elegoo Mars 4+, Saturn 3+ | ★★★★ Important |
| **CBDDLP** | `.cbddlp` | Older Elegoo Mars, Creality | ★★★ Legacy |
| **PHZ** | `.phz` | Phrozen printers | ★★★ Important |
| **PWS/PW0** | `.pws` | Anycubic printers | ★★★ Important |
| **NanoDLP** | `.nanodlp` | NanoDLP-based printers | ★★ Niche |

### CTB v4 Format (Most Critical)

**Reverse-engineered primarily by UVtools (sn4k3) and catibo (cbiffle).**

```
CTB FILE STRUCTURE (Binary, Little-Endian)
══════════════════════════════════════════

HEADER (offset 0):
┌──────────────────────────────────────────┐
│ Magic Number    : u32                    │
│   0x12FD0106 = CTB v4 (unencrypted)     │
│   0x12FD0107 = CTB v4 (encrypted)       │
│   0x12FD0086 = Elegoo Mars 3 variant    │
│ Version         : u32                    │
│ Bed Size X (mm) : f32                    │
│ Bed Size Y (mm) : f32                    │
│ Bed Size Z (mm) : f32                    │
│ Unknown/Reserved: 12 bytes               │
│ Layer Height(mm) : f32                   │
│ Exposure Time(s) : f32                   │
│ Bottom Exp Time : f32                    │
│ Bottom Layers   : u32                    │
│ Resolution X    : u32                    │
│ Resolution Y    : u32                    │
│ Large Preview Offset : u32              │
│ Layers Table Offset  : u32              │
│ Layer Count     : u32                    │
│ Small Preview Offset : u32              │
│ Print Time (s)  : u32                    │
│ Projection Type : u32 (0=normal, 1=LCD) │
│ ... (additional parameters)              │
│                                          │
│ CTB v4 additions:                        │
│ TSMC (Two-Stage Motion Control) params   │
│ Lift height 1/2, lift speed 1/2          │
│ Retract height 1/2, retract speed 1/2   │
│ Rest time after lift/retract             │
└──────────────────────────────────────────┘

PREVIEW IMAGES (at preview offsets):
┌──────────────────────────────────────────┐
│ Resolution X : u32                       │
│ Resolution Y : u32                       │
│ Data Length  : u32                        │
│ Image Data   : RGB565 encoded pixels     │
│   (typically 400×300 and 800×480)        │
└──────────────────────────────────────────┘

LAYER TABLE (at layers offset):
For each layer i in 0..layer_count:
┌──────────────────────────────────────────┐
│ Layer Position Z (mm) : f32              │
│ Data Offset    : u32                     │
│ Data Length    : u32                      │
│ Exposure Time  : f32                     │
│ Off Time       : f32                     │
│ ... (per-layer overrides)                │
└──────────────────────────────────────────┘

LAYER IMAGE DATA (at each layer's data offset):
┌──────────────────────────────────────────┐
│ RLE-encoded monochrome bitmap            │
│ Encoding: Run-Length Encoding            │
│   Each run: (count, value)               │
│   value: 0x00=off, 0xFF=on              │
│   With anti-aliasing: grayscale 0-255    │
│                                          │
│ For encrypted CTB v4:                    │
│   AES encryption with keys extracted     │
│   from Chitubox binary (plaintext XOR)   │
└──────────────────────────────────────────┘
```

### CTB RLE Encoding Detail

```
ENCODING: CTB Run-Length Encoding
─────────────────────────────────
Each byte encodes a run:
  Bit 7: pixel value (0=dark, 1=light)  
  Bits 0-6: run length (1-127)
  
  If bit 6 is set: extended length
    Next byte provides additional length bits
    Total length = (byte1 & 0x3F) << 8 | byte2

For anti-aliased CTB:
  Multiple "sub-layers" per physical layer
  Each sub-layer has different threshold
  Final exposure = weighted average
```

### GOO Format (Elegoo)

Better documented than CTB — Elegoo released an [official spec](https://github.com/elegooofficial/GOO):

```
GOO FILE STRUCTURE (Binary, Big-Endian!)
════════════════════════════════════════

HEADER:
  - Magic: "GOO" + version bytes
  - Printer name, resin name
  - X/Y resolution (e.g., 11520 × 5120 for 12K)
  - Build volume dimensions (mm)
  - Layer height, exposure times
  - Bottom layer count + exposure
  - Lift/retract parameters
  - Preview image (embedded)
  - Total layers, total print time

PER-LAYER DATA:
  - Layer index
  - Exposure time
  - Off time
  - Z position
  - RLE-encoded image data
  - Checksum (negated byte sum of payload)

RLE ENCODING (GOO-specific):
  Variable-length runs (1-4 bytes for count):
  - Encodes grayscale value (0-255) + run length
  - Smaller numbers use fewer bytes
  - Every pixel in resolution must be defined
    (trailing zeros required — the "undefined memory" bug)
```

**Important lesson from mslicer:** Always pad RLE output to fill the entire pixel count. Printers don't zero-initialize their frame buffers — undefined pixels will cure random resin.

### Rust Crates for File Formats

| Crate | Formats | Status |
|-------|---------|--------|
| **catibo** (cbiffle/catibo) | CTB, CBDDLP, PHZ | Mature read/write; encryption support; BSD-2-Clause; last updated 2020 |
| **goo** (connorslade/goo) | GOO (Elegoo) | Active; extracted from mslicer; encoding/decoding with ImHex patterns |
| **msla_format** (connorslade) | Multiple | Part of mslicer workspace; newer |

### UVtools as Reference Implementation

**UVtools** (github.com/sn4k3/UVtools) is the definitive reference for resin printer file formats:
- Written in C# / .NET
- `UVtools.Core/FileFormats/` contains implementations for 30+ formats
- `ChituboxFile.cs` — CTB/CBDDLP parsing with full header structs
- Handles encryption/decryption (AES with XOR-extracted keys from Chitubox binary)
- Supports reading, writing, conversion, and repair
- Active development (v5.2.1+)

---

## 4. Build Plate Auto-Layout

### 2D Nesting/Bin-Packing

The auto-layout problem is NP-hard (variant of 2D irregular strip packing):

```
ALGORITHM: AutoLayout(models, plate_size)
──────────────────────────────────────────
INPUT: List of 3D models, build plate dimensions (W × H mm)
OUTPUT: (x, y, rotation) for each model

1. COMPUTE FOOTPRINTS:
   For each model M:
     Project onto XY plane → 2D polygon
     Options: convex hull (fast), actual projected outline (accurate)
     Add clearance margin (1-3mm between models)

2. SORT by area (largest first — First Fit Decreasing)

3. PLACE using Bottom-Left heuristic:
   For each model (sorted):
     For each candidate rotation (0°, 90°, 180°, 270°, or finer):
       Try placing at lowest available Y, then leftmost X
       Check no overlap with placed models
       Check within plate bounds
     Select rotation + position with best packing

4. OPTIONAL: Local search refinement
   Randomly swap/rotate pairs, accept if improves density

Target: 85-90% plate utilization (manual typically hits 60%)
```

**Advanced: PAMPA (Pixel-based AM Packing Algorithm)**
- Rasterize footprints to pixel grid
- Bottom-Left placement with pixel-level precision
- Handles arbitrary irregular shapes with holes
- Rotation selection by packing density score

### Auto-Orientation

```
ALGORITHM: OptimalOrientation(mesh)
────────────────────────────────────
Goal: Find rotation that minimizes support requirements

1. Generate candidate orientations:
   - Sample 100-1000 rotations (fibonacci sphere sampling)
   - Or: use face normals as candidates (place-on-face)

2. For each candidate rotation R:
   Score(R) = w1 * overhang_area(R) 
            + w2 * support_volume(R)
            + w3 * z_height(R)        // shorter = faster print
            + w4 * cross_section(R)   // smaller = less peel force
            + w5 * island_count(R)

3. Select R with minimum Score

Weights (resin-specific):
  - Heavily penalize suction cup geometry (trapped resin)
  - Prefer ~45° tilts (balances support area vs peel force)
  - Minimize large flat horizontal areas (peel force spikes)
```

**Carbon3D's "Easier to Support" mode** targets surfaces exceeding self-supporting angles (>30° from platform).

### Hollowing

```
ALGORITHM: HollowMesh(mesh, wall_thickness, drain_holes)
─────────────────────────────────────────────────────────
1. OFFSET SURFACE:
   Generate inner shell by offsetting each vertex inward 
   by wall_thickness along vertex normal
   
   Methods:
   a. Vertex normal offset (fast, issues at sharp edges)
   b. Level-set / signed distance field (robust)
      - Voxelize mesh → compute SDF → extract isosurface 
        at -wall_thickness → marching cubes
   c. OpenVDB-style (best quality, complex)

2. FLIP inner shell normals (now points inward)

3. COMBINE outer + inner shell → hollow mesh

4. ADD DRAIN HOLES:
   For each hole:
     - Select position on lowest surface 
     - Boolean subtract cylinder (2-5mm diameter)
     - Place at least 2 holes for airflow

Wall thickness guidelines:
  - Minimum: 1.0-1.5mm (structural minimum)
  - Recommended: 2.0mm (good balance)
  - Thick: 3.0mm+ (for functional parts)

Resin savings: typically 60-80% reduction
```

---

## 5. Performance Considerations

### Scale of the Problem

For a Saturn 3 Ultra (12K resolution):
- **Resolution:** 11,520 × 5,120 pixels per layer
- **Uncompressed layer:** 58.98 MB (monochrome)
- **Typical layers:** 500-5,000+ per print
- **Triangle count:** 50K-500K typical, some models 1M+
- **Total uncompressed:** 29-295 GB

### Parallel Slicing with Rayon

Each layer is independent → embarrassingly parallel:

```rust
use rayon::prelude::*;

let layers: Vec<EncodedLayer> = (0..layer_count)
    .into_par_iter()
    .map(|i| {
        let z = i as f32 * layer_height;
        let segments = mesh.intersect_plane(z);      // Step 1
        let rle = scanline_fill(segments, resolution); // Step 2
        encode_layer(rle, format)                      // Step 3
    })
    .collect();
```

**mslicer achieves 20×-120× faster slicing than competitors** using this approach with Rust.

### GPU Compute with wgpu

Two GPU-acceleratable stages:

**1. Polygon Rasterization (highest impact):**
```wgsl
// WGSL compute shader for scanline fill
@compute @workgroup_size(64)
fn scanline_fill(@builtin(global_invocation_id) id: vec3<u32>) {
    let y = id.x;
    if (y >= resolution_y) { return; }
    
    // For each segment, test intersection with this row
    // Sort intersections, apply depth tracking
    // Write directly to RLE buffer or bitmap
}
```

**2. Mesh-Plane Intersection (moderate impact):**
- Upload triangle data to GPU buffer
- Each thread tests one triangle against one plane
- Dispatch: `(triangle_count / 64, layer_count, 1)`

### Memory Management Strategy

```
STRATEGY: Streaming Layer Pipeline
───────────────────────────────────
Don't hold all layers in memory simultaneously!

Pipeline stages (bounded buffer of N layers):
  [Intersect] → [Rasterize] → [Encode] → [Write to file]
       ↓              ↓            ↓            ↓
    ~1KB/layer    ~60MB/layer   ~50KB/layer   disk
    (segments)    (bitmap)      (RLE)

Keep at most 4-8 uncompressed layers in flight (rayon threads).
After RLE encoding, each layer shrinks to ~20-200KB.
Total memory: ~500MB peak for 4K, ~2GB for 12K resolution.
```

### Benchmark Reference (mslicer)

From Connor Slade's benchmarks:
- **mslicer (Rust):** 1-5 seconds for typical models
- **Chitubox:** 20-60 seconds
- **Lychee:** 30-120+ seconds
- **Factor:** 20×-120× faster

This is mostly from:
1. Rust's zero-cost abstractions vs. C#/Electron overhead
2. Parallel slicing (rayon)
3. Direct RLE encoding during scanline (no intermediate bitmap)

---

## 6. Competitive Analysis

### Chitubox Pro ($169/year)

**What you're paying for:**
- Advanced auto-support with island detection
- "ChituAction" automation for production workflows  
- Suction Cup Detection (Pro-exclusive) — detects trapped air/resin
- Two-Stage Motion Control (TSMC) parameter support
- Advanced mesh repair tools
- Text labeling on models
- Multi-parameter editing per layer
- Claims 88× faster processing (than what baseline?)

**What users hate (Reddit r/resinprinting, r/AnycubicPhoton):**
- **Crashes constantly** — especially on large models, Windows 11, hollowing operations
- **Auto-supports are mediocre** — "ChiTuBox supports are trash" is a common sentiment
- **Anti-aliasing preview lies** — preview doesn't match actual print output
- **File export bugs** — .ctb files incompatible with newer printer firmware
- **Slow printer profile updates** — new printers wait months for official profiles
- **Missing island detection in V2** (only added in V2+ Pro)
- **No variable layer heights** (repeatedly requested)
- **Poor batch nesting** — only basic arranging, no smart packing
- **Hollowing doesn't auto-place drain holes reliably**
- **Ghost support/company** — official channels rarely respond to bug reports

### Lychee Slicer Pro (€99.90/year)

**Unique features:**
- **Best auto-support system** — 90% of "switch from Chitubox" threads recommend Lychee for supports
- **Voxel-based 3D hollowing** — more robust than offset approach
- **Support painting** — manually brush where supports go
- **Proximity detection** — avoids support-model collisions
- **One-click "Magic" button** — auto-orients, supports, slices
- **Cross-platform** (Windows, Mac, Linux)
- **Unified resin + FDM** in one app
- **750+ printer profiles**
- **Model library** (2,500+ pre-supported models in top tier)

**What users hate:**
- **Slow startup** (reported 10+ minutes on first launch!)
- **Requires account creation + has ads** (even in paid version historically)
- **Slicing speed is mediocre** — noticeably slower than Chitubox
- **Crashes/instability** on complex scenes
- **Raft warping** issues
- **Doesn't remember settings** between sessions

### VoxelDance Tango

- **$16/month** with limited slice operations per month (!)
- Good quality but restrictive pricing kills adoption
- Users who tried it liked the UX

### What's Missing From ALL Current Slicers

| Missing Feature | Impact | Difficulty |
|----------------|--------|------------|
| **Open source / community-driven** | Huge community desire | Medium |
| **Variable/adaptive layer heights** | Faster prints, better detail | Medium |
| **Real-time GPU-accelerated slicing** | Instant feedback | Hard |
| **Smart batch nesting with rotation** | Production efficiency | Medium |
| **Resin usage estimation with cost** | Budget planning | Easy |
| **Print failure prediction** | Save time/resin | Very Hard |
| **Cross-printer profile sharing** | Community knowledge | Easy |
| **Version control for slice files** | Reproducibility | Easy |
| **CLI/headless slicing** | Automation, CI/CD for print farms | Medium |
| **Plugin/extension system** | Community features | Medium |
| **Integrated printer control** | Send-to-print workflow | Medium |
| **Multi-exposure per layer** (bloom/bleed control) | Better dimensional accuracy | Hard |

### Market Opportunity

The resin printing community is **desperate** for a good open-source slicer:
- Chitubox is buggy and unresponsive to users
- Lychee is slow and ad-supported
- VoxelDance prices out hobbyists
- **No open-source option has supports yet** (mslicer is closest but explicitly lacks them)
- Print farms need CLI/automation support badly

---

## 7. Rust Ecosystem

### Core Crates

| Crate | Version | Purpose | Maturity |
|-------|---------|---------|----------|
| **nalgebra** | 0.33+ | Linear algebra, transforms | ★★★★★ Production |
| **parry3d** | 0.17+ | Computational geometry, collision | ★★★★ Mature |
| **rayon** | 1.10+ | Data parallelism | ★★★★★ Production |
| **wgpu** | 24.0+ | GPU compute + rendering | ★★★★ Mature |
| **image** | 0.25+ | Bitmap generation, PNG encoding | ★★★★★ Production |
| **stl_io** | 0.7+ | STL file I/O | ★★★★ Stable |
| **gltf** | 1.4+ | glTF 2.0 I/O | ★★★★ Stable |
| **three-d** | 0.17+ | 3D rendering (higher-level) | ★★★ Good |
| **egui** | 0.30+ | Immediate-mode GUI | ★★★★ Active |
| **ordered-float** | 4.0+ | Sortable floats | ★★★★★ Essential |
| **tauri** | 2.x | Desktop app shell | ★★★★ Production |
| **serde** | 1.0 | Serialization | ★★★★★ Essential |

### File Format Crates

| Crate | Formats | Notes |
|-------|---------|-------|
| **catibo** | CTB, CBDDLP, PHZ | Read/write/convert; encryption support; BSD-2 |
| **goo** | GOO (Elegoo) | Encode/decode; from mslicer; includes ImHex pattern |
| **msla_format** | Multiple | From mslicer workspace; newer |
| **baby_shark** | STL, OBJ, PLY | Mesh I/O with format auto-detection |

### Existing Rust 3D Printing Projects

| Project | URL | Description |
|---------|-----|-------------|
| **mslicer** | github.com/connorslade/mslicer | Most advanced Rust MSLA slicer; CTB+GOO+NanoDLP; 20-120× faster than competitors; no support generation yet; GPL-3.0 |
| **catibo** | github.com/cbiffle/catibo | CTB/CBDDLP/PHZ format library; reverse-engineered encryption; BSD-2 |
| **disco** | github.com/KeKsBoTer/disco | General 3D printing slicer in Rust; not resin-specific |
| **turbo-resin** | (GitHub) | Open-source firmware for SLA printers; Rust-based; targets Anycubic Photon Mono 4K |
| **rust-irmf-slicer** | (crates.io) | IRMF volumetric slicer for 3D printers |
| **cassini** | github.com/vvuk/cassini | Elegoo printer control protocol (discovery + MQTT + file upload) |

### Tauri v2 for Desktop Shell

**Why Tauri over Electron:**
- **Binary size:** ~5-15MB vs Electron's ~150-300MB
- **Memory:** Uses system WebView (WebKit/WebView2), not bundled Chromium
- **Performance:** Rust backend with async Tokio runtime
- **Cross-platform:** Windows, macOS, Linux, iOS, Android
- **Security:** Content security policies by default, IPC surface minimization

**Architecture with Tauri:**
```
┌─────────────────────────────────┐
│         Frontend (WebView)       │
│   React/Vue/Svelte + Three.js   │
│   - 3D viewport (WebGL/WebGPU)  │
│   - UI controls                 │
│   - Layer preview               │
└──────────┬──────────────────────┘
           │ IPC (Tauri commands)
┌──────────┴──────────────────────┐
│         Rust Backend             │
│   - Mesh loading & processing   │
│   - Slicing engine (rayon)      │
│   - Support generation          │
│   - File format encoding        │
│   - GPU compute (wgpu)          │
│   - Printer communication       │
└─────────────────────────────────┘
```

**Alternative: egui + wgpu (mslicer's approach)**
- Pure Rust, no web tech overhead
- Direct GPU access for viewport
- Smaller binary, faster startup
- Less polished UI toolkit (but rapidly improving)
- No web developer ecosystem

### Recommendation: Hybrid Approach

Use **egui + wgpu** for the core app (matching mslicer's proven approach) but consider Tauri only if a web-based UI is strongly preferred. The 3D viewport and compute-heavy operations all happen in Rust either way.

---

## 8. Proposed Architecture

```
ARCHITECTURE: ResinSlicer
═══════════════════════════

┌─────────────────────────────────────────────────────┐
│                    APPLICATION LAYER                  │
│                                                       │
│  ┌──────────┐  ┌──────────┐  ┌───────────────────┐  │
│  │  egui    │  │  wgpu    │  │  Printer Control  │  │
│  │  UI      │  │ Viewport │  │  (UDP/MQTT/HTTP)  │  │
│  │ Panels   │  │ Renderer │  │  (Cassini proto)  │  │
│  └────┬─────┘  └────┬─────┘  └────────┬──────────┘  │
│       └──────────────┼─────────────────┘              │
│                      │                                │
├──────────────────────┼────────────────────────────────┤
│               ORCHESTRATION LAYER                     │
│                      │                                │
│  ┌───────────────────┴──────────────────────────┐    │
│  │              SliceJob Pipeline                │    │
│  │  load → validate → orient → support →        │    │
│  │  hollow → slice → encode → write             │    │
│  └──────────────────────────────────────────────┘    │
│                                                       │
├───────────────────────────────────────────────────────┤
│                   CORE ENGINE LAYER                    │
│                                                       │
│  ┌─────────────┐  ┌──────────────┐  ┌────────────┐  │
│  │ Mesh Engine │  │ Slice Engine │  │  Support   │  │
│  │             │  │              │  │  Engine    │  │
│  │ - Load STL  │  │ - Plane      │  │            │  │
│  │ - Load OBJ  │  │   intersect  │  │ - Overhang │  │
│  │ - Load 3MF  │  │ - Scanline   │  │   detect   │  │
│  │ - Repair    │  │   rasterize  │  │ - Island   │  │
│  │ - Transform │  │ - RLE encode │  │   detect   │  │
│  │ - Hollowing │  │ - AA (gray)  │  │ - Column   │  │
│  │ - Boolean   │  │              │  │ - Tree     │  │
│  │             │  │ GPU path:    │  │ - Raft     │  │
│  │             │  │ - wgpu       │  │ - Tips     │  │
│  │             │  │   compute    │  │            │  │
│  └─────────────┘  └──────────────┘  └────────────┘  │
│                                                       │
│  ┌──────────────┐  ┌─────────────┐  ┌────────────┐  │
│  │ Layout Engine│  │Format Engine│  │   Compute  │  │
│  │              │  │             │  │   Backend  │  │
│  │ - Nesting    │  │ - CTB v3/v4 │  │            │  │
│  │ - Bin-pack   │  │ - GOO       │  │ - rayon    │  │
│  │ - Auto-orient│  │ - CBDDLP    │  │   (CPU)    │  │
│  │ - Collision  │  │ - PHZ       │  │ - wgpu     │  │
│  │              │  │ - PWS       │  │   (GPU)    │  │
│  └──────────────┘  └─────────────┘  └────────────┘  │
│                                                       │
├───────────────────────────────────────────────────────┤
│                    CRATE STRUCTURE                     │
│                                                       │
│  Cargo workspace:                                     │
│  ├── slicer-core/      # Mesh, slicing, supports     │
│  ├── slicer-formats/   # File format encode/decode   │
│  ├── slicer-gpu/       # wgpu compute shaders        │
│  ├── slicer-layout/    # Nesting, orientation        │
│  ├── slicer-remote/    # Printer discovery + control │
│  ├── slicer-gui/       # egui UI + wgpu viewport    │
│  └── slicer-cli/       # Headless CLI for farms      │
└───────────────────────────────────────────────────────┘
```

### Data Flow

```
STL/OBJ/3MF → [Mesh Load] → [Repair] → [Transform/Orient]
                                              │
                    ┌─────────────────────────┤
                    ↓                         ↓
              [Hollowing]              [Support Gen]
                    │                         │
                    └──────────┬──────────────┘
                               ↓
                        [Slice Engine]
                    (parallel per layer)
                               │
                    ┌──────────┴──────────┐
                    ↓                     ↓
            [CPU: rayon]           [GPU: wgpu]
            (intersect +           (rasterize)
             scanline)
                    │                     │
                    └──────────┬──────────┘
                               ↓
                        [RLE Encode]
                               ↓
                      [Format Write]
                     (CTB/GOO/etc.)
                               ↓
                      [Upload to Printer]
                      (optional, MQTT)
```

---

## 9. Risk Assessment

### Easy (< 1 month of effort)

| Component | Why Easy | Risk |
|-----------|----------|------|
| **STL/OBJ loading** | Well-understood, good crates exist | Low |
| **Basic plane-mesh intersection** | Simple math, proven algorithms | Low |
| **Scanline rasterization** | Straightforward scanline fill | Low |
| **RLE encoding** | Simple byte-level encoding | Low |
| **GOO format output** | Official spec exists, goo crate works | Low |
| **Basic transforms** (move, rotate, scale) | nalgebra handles it | Low |
| **Parallel slicing** (rayon) | Embarrassingly parallel | Low |
| **CLI interface** | Just wire up the engine | Low |

### Medium (1-3 months)

| Component | Why Medium | Risk |
|-----------|-----------|------|
| **CTB v4 format output** | Reverse-engineered, encryption needed; catibo helps but is dated | Medium |
| **Anti-aliasing** | Grayscale edge pixels, multiple sub-exposure layers | Medium |
| **Basic auto-layout** (bounding box) | Simple bin-packing, well-studied | Low-Medium |
| **Basic column supports** | Vertical pillars with tips, not too complex geometrically | Medium |
| **Mesh repair** (basic) | Close holes, fix normals; reference implementations exist | Medium |
| **3D viewport** (egui + wgpu) | Complex but proven (mslicer exists) | Medium |
| **Island detection** | Connected component analysis per layer | Medium |
| **Hollowing** (offset method) | Vertex normal offset, add drain holes | Medium |
| **Printer communication** | Cassini protocol documented; needs MQTT broker | Medium |

### Hard (3-6+ months)

| Component | Why Hard | Risk |
|-----------|----------|------|
| **Tree support generation** | No open-source library; complex geometry + optimization | **HIGH** |
| **Overhang detection + smart support placement** | Needs robust normal analysis, density control | High |
| **Auto-orientation optimization** | Multi-objective optimization, many candidate evaluations | Medium-High |
| **Advanced hollowing** (level-set / voxel) | Requires SDF computation, marching cubes | High |
| **GPU compute slicing** | wgpu compute shader development, debugging GPU code | Medium-High |
| **Irregular 2D nesting** | NP-hard problem, heuristic quality matters | High |
| **Support tip optimization** | Contact point engineering, resin-specific tuning | High |
| **Print failure prediction** | AI/ML territory, needs training data | **VERY HIGH** |

### Unknown/Research Needed

| Component | What's Unknown |
|-----------|---------------|
| **CTB v4 encryption details** | Keys extracted from Chitubox binary — legal grey area? |
| **Anti-aliasing quality** | How do different AA approaches compare in print quality? |
| **Support placement heuristics** | What density/pattern produces least failures? Needs print testing |
| **Performance ceiling** | Is GPU compute worth the complexity for typical meshes? mslicer is already 120× faster on CPU alone |
| **Format compatibility** | Do printers accept files from non-official slicers reliably? Edge cases? |
| **Community adoption** | Will users switch from Chitubox/Lychee to an open-source tool without supports? |

### Recommended MVP Scope

**Phase 1 — Core Slicer (months 1-2):**
- STL/OBJ/3MF loading
- Basic mesh transforms
- Plane-mesh intersection with Z-bucket acceleration
- Scanline rasterization with depth tracking
- RLE encoding → GOO format output
- Parallel slicing with rayon
- Basic egui + wgpu GUI (viewport, settings panel)
- CLI mode for headless slicing

**Phase 2 — Essential Features (months 3-4):**
- CTB v3/v4 output (leverage catibo)
- Basic column support generation
- Island detection + visualization
- Auto-layout (bounding box, simple nesting)
- Anti-aliasing (grayscale edge pixels)
- Printer communication (discovery, upload, start)

**Phase 3 — Advanced Features (months 5-8):**
- Tree support generation (the big one)
- Auto-orientation optimization
- Basic hollowing with drain holes
- Advanced nesting with rotation
- GPU compute shader acceleration (wgpu)
- Multiple printer profiles

**Phase 4 — Polish + Community (ongoing):**
- Advanced hollowing (voxel/SDF)
- Support painting/editing
- Plugin system
- Community printer profiles
- Print farm automation
- Resin profiles with exposure recommendations

### Strategic Consideration: Fork vs. Build

**Option A: Build from scratch** — Full control, clean architecture, but years of work for feature parity.

**Option B: Contribute to mslicer** — It's already GPL-3.0, has the core slicing working at incredible speed, has CTB+GOO+NanoDLP support, and the author explicitly notes he still needs support generation and island detection. Contributing the hard features (supports, layout) to an existing project with proven slicing would be significantly faster than starting from zero.

**Recommendation:** Seriously consider contributing to or forking mslicer. The slicing engine is already 20-120× faster than competitors. What's missing is exactly what this research covers — supports, layout, hollowing. The architecture is clean Rust with egui + wgpu. Adding Tauri on top would be optional.

---

## 10. References

### Source Code & Tools

- **mslicer** — https://github.com/connorslade/mslicer (Rust MSLA slicer, GPL-3.0)
- **mslicer writeup** — https://connorslade.com/projects/mslicer (excellent technical deep-dive)
- **catibo** — https://github.com/cbiffle/catibo (Rust CTB/CBDDLP/PHZ, BSD-2)
- **goo crate** — https://github.com/connorslade/goo (Rust GOO format)
- **UVtools** — https://github.com/sn4k3/UVtools (C# format reference, 30+ formats)
- **PrusaSlicer SLA** — https://github.com/prusa3d/PrusaSlicer (`src/libslic3r/SLA/`)
- **Elegoo GOO spec** — https://github.com/elegooofficial/GOO
- **Cassini** — https://github.com/vvuk/cassini (Elegoo printer protocol)
- **Turbo Resin** — Open-source SLA printer firmware in Rust

### Academic Papers

- **"Clever Support: Efficient Support Structure Generation for Digital Fabrication"** — Vanek, Galicia, Benes (2014). Tree support generation; 29.4% material savings vs columnar.
- **"Local Barycenter Based Tree Support (LBTS)"** — Efficient tree algorithm using barycenters.
- **"Particle Swarm Optimization with Greedy Strategy for Tree Support"** — PSO-based optimization for lightweight internal supports.
- **"Physical Constraint Formulas for Stable Tree-Support Growth"** — Experimental branch stability formulas.

### Community & Discussion

- **r/resinprinting** — Primary community; Chitubox complaints, Lychee recommendations
- **r/AnycubicPhoton** — Printer-specific issues
- **r/ElegooMars** / **r/ElegooSaturn** — Elegoo-specific
- **UVtools GitHub Discussions** — Discussion #750 for CTB reverse engineering details
- **Chitubox forums** — forums.chitubox.com (official but sparse responses)

### Rust Ecosystem

- **nalgebra** — https://docs.rs/nalgebra
- **parry3d** — https://docs.rs/parry3d
- **rayon** — https://docs.rs/rayon
- **wgpu** — https://wgpu.rs
- **egui** — https://github.com/emilk/egui
- **tauri** — https://v2.tauri.app
- **image** — https://docs.rs/image
- **stl_io** — https://docs.rs/stl_io
- **baby_shark** — https://docs.rs/baby_shark
- **ordered-float** — https://docs.rs/ordered-float
