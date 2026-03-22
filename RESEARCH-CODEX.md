# Open Source Resin 3D Print Slicer — Technical Research Codex

> Research compiled 2026-03-21. Covers algorithms, file formats, existing codebases, and recommended architecture for building an open source MSLA/DLP resin slicer from scratch.

---

## Table of Contents

1. [Mesh → Layer Slicing Algorithms](#1-mesh--layer-slicing-algorithms)
2. [Support Generation Algorithms](#2-support-generation-algorithms)
3. [Printer File Formats](#3-printer-file-formats)
4. [Auto-Layout / Bin Packing](#4-auto-layout--bin-packing)
5. [Existing Open Source Code](#5-existing-open-source-code)
6. [Tech Stack Recommendation](#6-tech-stack-recommendation)
7. [Architecture Blueprint](#7-architecture-blueprint)

---

## 1. Mesh → Layer Slicing Algorithms

### 1.1 The Two Approaches

There are fundamentally two ways to slice a 3D mesh into 2D layer images:

#### Approach A: CPU Geometric Slicing (Plane-Mesh Intersection)

The classic computational geometry approach:

1. For each layer height `z`, define a cutting plane `P(z)`
2. Find all triangles that intersect plane `P(z)`
3. Compute line segments where each triangle edge crosses the plane
4. Connect line segments into closed contour polygons
5. Fill polygons using scanline/even-odd rule to produce a raster image

**Pseudocode:**
```
function slice_mesh(mesh, layer_height, resolution):
    layers = []
    for z in range(0, mesh.max_z, layer_height):
        contours = []
        for triangle in mesh.triangles:
            if triangle.min_z <= z <= triangle.max_z:
                segment = intersect_triangle_plane(triangle, z)
                if segment:
                    contours.append(segment)

        # Connect segments into closed polygons
        polygons = connect_segments(contours)

        # Rasterize polygons to image
        image = rasterize(polygons, resolution)
        layers.append(image)
    return layers

function intersect_triangle_plane(tri, z):
    # For each edge of the triangle, check if it crosses z
    # If two edges cross z, compute intersection points
    # Return the line segment between them
    points = []
    for edge in tri.edges():
        v0, v1 = edge
        if (v0.z - z) * (v1.z - z) < 0:  # opposite sides of plane
            t = (z - v0.z) / (v1.z - v0.z)
            x = v0.x + t * (v1.x - v0.x)
            y = v0.y + t * (v1.y - v0.y)
            points.append((x, y))
    if len(points) == 2:
        return Segment(points[0], points[1])
```

**Performance:** CPU approach processes triangles serially. For a 50k triangle mesh with 2000 layers, you're doing ~100M triangle-plane checks. Optimizations:
- Sort triangles by Z range, skip non-intersecting ones
- Use spatial indexing (interval tree on Z ranges)
- Parallelise across layers (each layer is independent)

**Reference paper:** "An Optimal Algorithm for 3D Triangle Mesh Slicing" (2016) — achieves O(n log n + k) where n = triangles, k = output segments.

#### Approach B: GPU Rendering Slicing (The Orthographic Projection Trick)

The much faster approach used by Formware, Microtome, and mslicer:

1. Load mesh onto GPU as vertex buffer
2. Set up orthographic camera looking down Z axis
3. For each layer, render the mesh with Z-clipping at the layer height
4. Use stencil buffer to determine inside/outside

**The Three-Pass Stencil Algorithm:**

```glsl
// Vertex shader — normalize Z to clipping range
void main() {
    vec4 pos = model_matrix * vec4(position, 1.0);
    pos.z -= z_min;                           // Bottom at z=0
    pos.z *= -2.0 / (z_max - z_min);         // Scale to [-1, +1]
    pos.z -= layer_fraction;                   // Offset to current layer
    gl_Position = projection * pos;
}
```

**Pass 1:** Render front faces, increment stencil buffer. Depth test OFF.
**Pass 2:** Render back faces, decrement stencil buffer. Depth test OFF.
**Pass 3:** Render fullscreen quad where stencil != 0. This gives the cross-section.

The stencil buffer implements a winding number algorithm — for a watertight mesh, only pixels inside the geometry will have non-zero stencil values after both passes.

**Performance benchmarks (from Formware, Microtome):**
- GPU slicing: ~100ms per slice at 2560×1920 resolution
- 500 slices in ~50 seconds (Chrome/WebGL)
- 20-120x faster than CPU implementations (mslicer benchmarks)
- Formware reports GPU is "much faster to process millions of triangles in parallel"
- The GPU approach has a fixed cost (transferring framebuffer to RAM) regardless of mesh complexity

**Key advantage:** Works with self-intersecting and boolean-intersecting meshes without repair. The stencil buffer naturally handles non-manifold geometry.

### 1.2 Anti-Aliasing Techniques

#### Greyscale Anti-Aliasing (CBDDLP approach)
Each layer is rendered multiple times with slightly different thresholds, producing `level_set_count` binary images per layer. The images are combined by averaging pixel values:
- AA level 1: threshold = 127
- AA level 2: thresholds = 255, 127
- AA level 4: thresholds = 255, 191, 127, 63
- AA level 8: thresholds = 255, 223, 191, 159, 127, 95, 63, 31

Each pixel's final value = (count of threshold passes) × (256 / AA_level)

#### GPU Native AA
When using GPU rendering approach, leverage hardware MSAA (multi-sample anti-aliasing):
- Render to MSAA framebuffer (4x, 8x samples)
- Resolve to single-sample texture
- Read back greyscale pixel values directly
- This is essentially free on modern GPUs

#### CTB Greyscale Approach
CTB format stores 7-bit pixel intensity values (0-127) directly. The slicer computes pixel coverage at boundaries by calculating the fraction of the pixel area covered by the polygon.

### 1.3 Key Rust Libraries for Mesh Processing

| Crate | Purpose | Notes |
|-------|---------|-------|
| `parry3d` | Collision detection, TriMesh | Can compute intersection polylines between mesh and plane. From Dimforge. |
| `tri-mesh` | Half-edge mesh data structure | Efficient for traversal, editing. Has intersection operations. |
| `rs-read-trimesh` | STL/OBJ/PLY/DAE loader | Loads directly into parry3d TriMesh format. |
| `meshopt` | Mesh optimization | Vertex cache optimization, overdraw reduction, simplification. |
| `wgpu` | WebGPU API for Rust | GPU rendering for the stencil-buffer slicing approach. Cross-platform. |
| `glam` | Math library | Fast SIMD-accelerated vectors, matrices, quaternions. Used by Bevy. |
| `image` | Image processing | Encoding layer bitmaps to PNG or raw formats. |

---

## 2. Support Generation Algorithms

### 2.1 Overhang Detection

Overhangs are mesh faces whose normal vector makes an angle greater than a threshold (typically 45°) with the vertical (Z-up) axis:

```
function detect_overhangs(mesh, threshold_angle=45°):
    overhang_faces = []
    for face in mesh.faces:
        normal = face.normal()
        angle = acos(dot(normal, (0, 0, -1)))  # angle from downward
        if angle < threshold_angle:
            overhang_faces.append(face)
    return overhang_faces
```

### 2.2 Island Detection

Islands are disconnected regions in a layer that have no connection to the layer below. This is a connected-component analysis problem:

```
function detect_islands(layer_image, previous_layer_image):
    # Find connected components in current layer
    components = connected_components(layer_image)

    islands = []
    for component in components:
        # Check if this component overlaps with any pixel in the layer below
        overlap = bitwise_and(component.mask, previous_layer_image)
        if count_nonzero(overlap) == 0:
            islands.append(component)

    return islands
```

**Lychee Slicer's approach (4 accuracy levels):**
- Fast: searches for large islands at ~3mm height
- Normal: medium islands at ~1.6mm height
- Detailed: islands on every layer under 0.1mm
- Real: full analysis of every layer

**Implementation:** Uses standard image processing — flood fill or `cv::connectedComponents` to label regions, then compare overlap between consecutive layers.

### 2.3 Support Point Placement

**PrusaSlicer's algorithm (v2.9.1):**
- Small islands → single support at center of mass
- Medium islands → Voronoi diagram to optimally place 2+ supports
- Large islands → divided into thin/thick sections:
  - Thin sections: supports along central axis (medial axis transform)
  - Thick sections: supports around perimeter for stability

### 2.4 Tree Support Generation

#### The Local Barycenter Tree Support (LBTS) Algorithm

From "Local Barycenter Based Efficient Tree-Support Generation for 3D Printing" (Wang et al., 2019):

```
function generate_tree_supports(support_points):
    # Phase 1: Group support points into clusters
    clusters = spatial_clustering(support_points, max_radius)

    # Phase 2: Build tree from top down
    for cluster in clusters:
        tree = Tree()
        tree.root = create_node(cluster.centroid, z=build_plate_z)

        # Iteratively subdivide
        build_branches(tree.root, cluster.points)

    return trees

function build_branches(parent_node, points):
    if len(points) <= 1:
        # Connect directly to support point
        parent_node.add_child(points[0])
        return

    # Divide points into sub-regions using k-means or spatial partitioning
    left, right = partition(points)

    # New branch node at barycenter of children, interpolated Z
    left_center = barycenter(left)
    right_center = barycenter(right)

    left_node = create_node(left_center, z=interpolated_z)
    right_node = create_node(right_center, z=interpolated_z)

    parent_node.add_child(left_node)
    parent_node.add_child(right_node)

    build_branches(left_node, left)
    build_branches(right_node, right)
```

#### Pillar/Column Support Generation

Simpler approach used as fallback:

```
function generate_pillar_supports(support_points, params):
    pillars = []
    for point in support_points:
        pillar = Pillar()
        pillar.top = point.position
        pillar.bottom = (point.x, point.y, 0)  # build plate
        pillar.top_diameter = params.contact_diameter    # 0.4-0.6mm
        pillar.shaft_diameter = params.shaft_diameter    # 0.8-1.2mm
        pillar.contact_depth = params.contact_depth      # 0.2mm penetration
        pillars.append(pillar)
    return pillars
```

#### Support Parameters (typical ranges)
| Parameter | Light | Medium | Heavy |
|-----------|-------|--------|-------|
| Contact diameter | 0.3mm | 0.5mm | 0.8mm |
| Contact depth | 0.1mm | 0.2mm | 0.3mm |
| Shaft diameter | 0.6mm | 1.0mm | 1.5mm |
| Base diameter | 2.0mm | 3.0mm | 5.0mm |

### 2.5 PrusaSlicer SLA Source Structure

Key files in `src/libslic3r/SLA/`:

| File | Purpose |
|------|---------|
| `SupportTree.cpp/hpp` | Core support tree data structures |
| `SupportTreeBuilder.cpp/hpp` | Constructs support structures from points |
| `SupportTreeMesher.cpp/hpp` | Converts tree structures to triangle mesh |
| `BranchingTreeSLA.cpp/hpp` | Tree-branching support strategy |
| `DefaultSupportTree.cpp/hpp` | Default pillar support algorithm |
| `SupportPointGenerator.cpp/hpp` | Auto-generates support points from geometry |
| `Hollowing.cpp/hpp` | Model hollowing using OpenVDB |
| `Pad.cpp/hpp` | Build platform pad/raft generation |
| `Rotfinder.cpp/hpp` | Optimal orientation finder |
| `RasterBase.cpp/hpp` | Layer rasterization |
| `AGGRaster.hpp` | AGG library-based rasterizer |
| `Clustering.cpp/hpp` | Spatial clustering for support grouping |
| `ConcaveHull.cpp/hpp` | Concave hull for pad outlines |
| `SpatIndex.cpp/hpp` | Spatial indexing (R-tree) for performance |

### 2.6 Auto-Orientation (Rotation Finding)

PrusaSlicer's `Rotfinder.cpp` implements multi-objective optimization to find the best print orientation:

**Objectives to minimize:**
1. Total overhang area (less supports needed)
2. Total support volume (less resin wasted)
3. Model height (fewer layers = faster print)

**Approach:** Sample orientations on a unit sphere, evaluate cost function for each, use optimization (genetic algorithm or simulated annealing) to find the best orientation.

---

## 3. Printer File Formats

### 3.1 CTB Format (ChiTuBox) — Most Common

Used by: Anycubic, Elegoo, Creality, and most consumer resin printers.

**Magic numbers:**
- CBDDLP: `0x12FD0019`
- CTB: `0x12FD0086`
- CTBv4: `0x12FD0106`
- CTBv4 GKtwo: `0xFF220810`

**Reference implementation:** [catibo](https://github.com/cbiffle/catibo) (Rust!) — first known decoding of the encryption algorithm.

#### File Layout
```
┌─────────────────────────┐
│  Header (112 bytes)     │  Magic, version, bed size, resolution,
│                         │  layer height, exposure times, offsets
├─────────────────────────┤
│  ExtConfig              │  Lift/retract speeds, resin cost
├─────────────────────────┤
│  ExtConfig2             │  Machine name, encryption mode, AA level
├─────────────────────────┤
│  Large Preview Image    │  400×300 RGB565 RLE-compressed
├─────────────────────────┤
│  Small Preview Image    │  200×125 RGB565 RLE-compressed
├─────────────────────────┤
│  Layer Table            │  Array of LayerHeader (36 bytes each)
├─────────────────────────┤
│  Layer Data Blobs       │  RLE-encoded layer images
├─────────────────────────┤
│  SlicerInfo             │  Software version, machine name, timestamps
├─────────────────────────┤
│  PrintParametersV4      │  (v4+) Additional retract params, disclaimer
├─────────────────────────┤
│  ResinParameters        │  (v5) Resin color, density, type/name strings
└─────────────────────────┘
```

#### Header Structure (Offset 0x00)
```rust
struct CtbHeader {
    magic: u32,                    // 0x00: 0x12FD0086 for CTB
    version: u32,                  // 0x04: 2-5
    bed_size_x_mm: f32,           // 0x08
    bed_size_y_mm: f32,           // 0x0C
    bed_size_z_mm: f32,           // 0x10
    unknown1: u32,                // 0x14
    unknown2: u32,                // 0x18
    total_height_mm: f32,         // 0x1C
    layer_height_mm: f32,         // 0x20
    exposure_s: f32,              // 0x24
    bottom_exposure_s: f32,       // 0x28
    light_off_delay_s: f32,       // 0x2C
    bottom_layer_count: u32,      // 0x30
    resolution_x: u32,            // 0x34
    resolution_y: u32,            // 0x38
    large_preview_offset: u32,    // 0x3C
    layer_table_offset: u32,      // 0x40
    layer_count: u32,             // 0x44
    small_preview_offset: u32,    // 0x48
    print_time_s: u32,            // 0x4C
    projector_type: u32,          // 0x50: 0=normal, 1=mirrored (LCD)
    print_params_offset: u32,     // 0x54
    print_params_size: u32,       // 0x58
    aa_level: u32,                // 0x5C: level_set_count
    light_pwm: u16,               // 0x60: 0x00-0xFF
    bottom_light_pwm: u16,        // 0x62
    encryption_key: u32,          // 0x64: 0 = no encryption
    slicer_offset: u32,           // 0x68
    slicer_size: u32,             // 0x6C
}
```

All multi-byte fields are **little-endian**. Floats are IEEE754 single-precision.

#### Layer Header (36 bytes each)
```rust
struct LayerHeader {
    z_mm: f32,              // 0x00: platform Z position
    exposure_s: f32,        // 0x04
    light_off_s: f32,       // 0x08
    data_offset: u32,       // 0x0C: offset to RLE data
    data_length: u32,       // 0x10: compressed size in bytes
    unknown: u32,           // 0x14
    table_size: u32,        // 0x18
    unknown2: u32,          // 0x1C
    unknown3: u32,          // 0x20
}
```

#### Preview Image Encoding (RLE15 — RGB565)

16-bit pixels with run-length encoding:
- Bits 15-11: Red (5 bits)
- Bits 10-6: Green (5 bits)
- **Bit 5: Run flag** (0 = single pixel, 1 = run follows)
- Bits 4-0: Blue (5 bits)

When run flag is set, the next 16-bit word encodes the run length: `0x3nnn` where nnn is a 12-bit count.

Standard thumbnail sizes: Large = 400×300, Small = 200×125.

#### CBDDLP Layer Encoding (RLE1 — Binary)

One byte per run:
- **Bit 7:** pixel value (0 = off, 1 = on)
- **Bits 6-0:** run length (1-125, capped at `RLE8EncodingLimit = 0x7D`)

```rust
fn encode_cbddlp(image: &[u8], threshold: u8) -> Vec<u8> {
    let mut result = Vec::new();
    let mut current_bit = false;
    let mut run_length = 0u8;

    for &pixel in image {
        let bit = pixel >= threshold;
        if bit == current_bit && run_length < 125 {
            run_length += 1;
        } else {
            if run_length > 0 {
                let byte = run_length | if current_bit { 0x80 } else { 0 };
                result.push(byte);
            }
            current_bit = bit;
            run_length = 1;
        }
    }
    // Flush remaining
    if run_length > 0 {
        let byte = run_length | if current_bit { 0x80 } else { 0 };
        result.push(byte);
    }
    result
}
```

#### CTB Layer Encoding (RLE7 — Greyscale)

7-bit pixel intensity (0-127), with variable-length run encoding:

```rust
fn encode_ctb(image: &[u8]) -> Vec<u8> {
    let mut result = Vec::new();
    let mut color = 0u8;
    let mut stride = 0u32;

    for &pixel in image {
        let grey7 = pixel >> 1;  // 8-bit to 7-bit
        if grey7 == color {
            stride += 1;
        } else {
            flush_run(&mut result, color, stride);
            color = grey7;
            stride = 1;
        }
    }
    flush_run(&mut result, color, stride);
    result
}

fn flush_run(result: &mut Vec<u8>, color: u8, stride: u32) {
    if stride == 0 { return; }

    if stride > 1 {
        result.push(color | 0x80);  // Set run flag
        // Variable-length run encoding (big-endian):
        if stride <= 0x7F {
            result.push(stride as u8);
        } else if stride <= 0x3FFF {
            result.push(((stride >> 8) as u8) | 0x80);
            result.push(stride as u8);
        } else if stride <= 0x1FFFFF {
            result.push(((stride >> 16) as u8) | 0xC0);
            result.push((stride >> 8) as u8);
            result.push(stride as u8);
        } else if stride <= 0xFFFFFFF {
            result.push(((stride >> 24) as u8) | 0xE0);
            result.push((stride >> 16) as u8);
            result.push((stride >> 8) as u8);
            result.push(stride as u8);
        }
    } else {
        result.push(color);  // Single pixel, no run flag
    }
}
```

#### CTB Encryption (86 Cipher)

XOR-based stream cipher:
```
c = (key × 0x2D83CDAC + 0xD8A83424) mod 2³²
X[0] = ((layer_index × 0x1E1530CD + 0xEC3D47CD) × c) mod 2³²
X[n+1] = (X[n] + c) mod 2³²
ciphertext[n] = plaintext[n] XOR X[n]
```

Key comes from header `encryption_key` field. When key = 0, encryption is disabled.

**Reference implementation:** `catibo` (Rust) has the first open-source implementation of this cipher.

### 3.2 GOO Format (Elegoo)

Used by newer Elegoo printers (Saturn 4, Mars 5, etc.). Big-endian byte order (unlike CTB).

```rust
struct GooHeader {
    version: [u8; 4],           // "V3.0"
    magic: [u8; 8],             // 0x07000000 44_4C_50_00
    software_name: [u8; 32],    // null-terminated string
    software_version: [u8; 24],
    file_create_time: [u8; 24], // "yyyy-mm-dd HH:mm:ss"
    machine_name: [u8; 32],
    machine_type: [u8; 32],     // "DLP"
    profile_name: [u8; 32],
    aa_level: u16,              // typically 8
    grey_level: u16,            // 0=4-bit, 1=8-bit greyscale
    blur_level: u16,
    small_preview: [u8; 116*116*2],  // RGB565
    small_preview_delimiter: [u8; 2], // 0x0D 0x0A
    big_preview: [u8; 290*290*2],    // RGB565
    big_preview_delimiter: [u8; 2],
    layer_count: u32,
    resolution_x: u16,
    resolution_y: u16,
    mirror_x: bool,
    mirror_y: bool,
    display_width_mm: f32,
    display_height_mm: f32,
    machine_z_mm: f32,
    layer_height_mm: f32,
    exposure_time_s: f32,
    delay_mode: u8,             // 0=light_off, 1=wait_time
    // ... extensive motion parameters ...
    grayscale_level: u8,        // 0=4-bit range, 1=8-bit range
    transition_layer_count: u16,
}
```

Each layer starts with magic byte `0x55`, followed by per-layer parameters and RLE data, terminated by `0x0D 0x0A`.

### 3.3 Other Formats

| Format | Used By | Notes |
|--------|---------|-------|
| `.photon` | Anycubic Photon (original) | Predecessor to CTB, similar structure |
| `.pwmx` | Anycubic Photon Workshop | Anycubic's own slicer format |
| `.pm3m` | Anycubic Photon Mono series | Variant of pwmx |
| `.fdg` | Voxelab | Very similar to CTB |
| `.phz` | Phrozen | Similar to CTB with same encryption |
| `.cxdlp` | Creality | Creality's proprietary format |
| `.sl1` | Prusa SL1 | ZIP archive containing PNG layers + config |
| `.nanodlp` | NanoDLP | Web-based controller format |

### 3.4 UVtools Format Support

[UVtools](https://github.com/sn4k3/UVtools) (C# .NET, AGPL-3.0) supports 30+ formats. Key source files:

| File | Format |
|------|--------|
| `ChituboxFile.cs` | CTB/CBDDLP (all versions) |
| `GooFile.cs` | Elegoo GOO |
| `AnycubicFile.cs` | Anycubic formats |
| `SL1File.cs` | Prusa SL1 |
| `CrealityCXDLPv4File.cs` | Creality CXDLP |
| `GR1File.cs` | Workshop formats |
| `FileFormat.cs` | Base class with common operations |

UVtools is primarily a file viewer/editor/converter — it does NOT do mesh slicing or support generation. It's invaluable as a reference for encoding/decoding printer file formats.

---

## 4. Auto-Layout / Bin Packing

### 4.1 Build Plate Arrangement

The problem: given N model footprints (2D convex hulls or bounding boxes), arrange them on the build plate to minimize wasted space while maintaining minimum clearance.

#### MaxRects Algorithm (Jukka Jylänki)

The most commonly used 2D bin packing algorithm:

```
function pack_rectangles(items, bin_width, bin_height):
    # Start with one free rectangle covering entire bin
    free_rects = [Rect(0, 0, bin_width, bin_height)]
    placed = []

    # Sort items by area (largest first)
    items.sort(key=lambda i: i.area, reverse=True)

    for item in items:
        # Find best free rectangle (Best Short Side Fit)
        best_rect = None
        best_score = infinity
        for rect in free_rects:
            if item.w <= rect.w and item.h <= rect.h:
                score = min(rect.w - item.w, rect.h - item.h)
                if score < best_score:
                    best_score = score
                    best_rect = rect

        if best_rect:
            # Place item in best_rect
            placed.append(Place(item, best_rect.x, best_rect.y))
            # Split remaining space into new free rects
            split(free_rects, best_rect, item)

    return placed
```

### 4.2 Rust Crates for Bin Packing

| Crate | Description |
|-------|-------------|
| `rectangle-pack` | 2D/3D rectangle packing with configurable strategies |
| `max_rects` | MaxRects algorithm implementation |
| `binpack2d` | Simple 2D bin packing |
| `u-nesting` | 2D polygon nesting + 3D bin packing with C FFI |

`rectangle-pack` is the most mature, supporting both 2D and 3D packing with customizable placement strategies.

### 4.3 Auto-Orientation

Finding the optimal rotation to minimize support volume / print time:

**Approach:** Sample orientations on a fibonacci sphere (uniformly distributed), compute cost metric for each:

```
cost(orientation) =
    w1 * overhang_area(mesh, orientation) +
    w2 * estimated_support_volume(mesh, orientation) +
    w3 * bounding_box_height(mesh, orientation)
```

PrusaSlicer's `Rotfinder.cpp` uses a similar multi-objective approach with gradient-free optimization.

---

## 5. Existing Open Source Code

### 5.1 mslicer (Rust) — THE Reference Implementation

**Repository:** [github.com/connorslade/mslicer](https://github.com/connorslade/mslicer)
**License:** GPL-3.0
**Language:** Rust (98.8%) + WGSL shaders (1.2%)
**Latest:** v0.5.0 (Feb 21, 2026)

**What it does:**
- Full MSLA resin slicer with GUI
- 20-120x faster than competing slicers
- Outputs: .ctb, .goo, .nanodlp, .svg
- Network printing: connect to Chitu-mainboard printers remotely

**Architecture:**
```
mslicer/
├── mslicer/          # GUI application (Tauri? or native?)
├── slicer/           # Core slicing library
│   └── src/
│       ├── lib.rs
│       ├── mesh.rs         # Mesh data structures
│       ├── half_edge.rs    # Half-edge mesh topology
│       ├── builder.rs      # Mesh construction
│       ├── util.rs
│       ├── slicer/         # Slicing algorithm
│       ├── geometry/       # Geometric primitives & ops
│       ├── supports/       # Support generation
│       ├── post_process/   # Layer post-processing
│       └── tools/          # Utility tools
├── format/           # File format encoders/decoders
├── common/           # Shared types
├── remote_send/      # Network/printer communication
└── dist/             # Distribution/packaging
```

**This is the single most important project to study.** It proves the Rust + WGSL GPU approach works and is fast.

### 5.2 catibo (Rust) — CTB Format Reference

**Repository:** [github.com/cbiffle/catibo](https://github.com/cbiffle/catibo)
**License:** MPL-2.0

Pure Rust implementation of CTB/CBDDLP/PHZ file format reading and writing. Contains the first open-source implementation of the CTB encryption algorithm. Includes detailed [format documentation](https://github.com/cbiffle/catibo/blob/master/doc/cbddlp-ctb.adoc).

### 5.3 UVtools (C# .NET) — Format Swiss Army Knife

**Repository:** [github.com/sn4k3/UVtools](https://github.com/sn4k3/UVtools)
**License:** AGPL-3.0

Supports 30+ file formats. Not a slicer — it's a viewer/editor/converter. Key value:
- Authoritative format implementations
- Layer manipulation operations (crop, flip, rotate, repair)
- Island detection and repair
- Cross-platform GUI (Avalonia)

### 5.4 PrusaSlicer SLA Engine (C++)

**Repository:** [github.com/prusa3d/PrusaSlicer](https://github.com/prusa3d/PrusaSlicer)
**License:** AGPL-3.0
**Key directory:** `src/libslic3r/SLA/`

The most sophisticated open-source SLA support system:
- Tree and pillar support generation
- Hollowing via OpenVDB signed distance fields
- Automatic support point placement (Voronoi-based)
- Pad/raft generation
- Auto-orientation optimization
- Full rasterization pipeline

Written in C++ with heavy use of CGAL, OpenVDB, and custom spatial indexing. Not directly portable to Rust but the algorithms are well-documented in the source.

### 5.5 Microtome (TypeScript/WebGL)

**Repository:** [github.com/Microtome/microtome](https://github.com/Microtome/microtome)
**License:** MIT

Browser-based GPU slicer using Three.js. Demonstrates the stencil-buffer slicing approach in JavaScript/WebGL. Handles self-intersecting meshes. Performance: ~100ms/slice at 2560×1920.

### 5.6 PhotonFileValidator (Java)

**Repository:** [github.com/Photonsters/PhotonFileValidator](https://github.com/Photonsters/PhotonFileValidator)

Viewer for .photon and .cbddlp files with island detection and overhang visualization.

---

## 6. Tech Stack Recommendation

### 6.1 Recommended Stack

```
┌─────────────────────────────────────────────┐
│                  Desktop App                │
│  Tauri 2.0 (Rust backend + WebView frontend)│
├─────────────────────────────────────────────┤
│              3D Preview Layer               │
│  Three.js / React Three Fiber               │
│  (model viewing, support preview,           │
│   build plate layout)                       │
├─────────────────────────────────────────────┤
│              Core Engine (Rust)              │
│  ┌───────────┬───────────┬───────────┐      │
│  │  Slicer   │ Supports  │  Formats  │      │
│  │  (wgpu    │ (tree,    │  (ctb,    │      │
│  │   GPU     │  pillar,  │   goo,    │      │
│  │   slice)  │  island)  │   sl1)    │      │
│  └───────────┴───────────┴───────────┘      │
│  ┌───────────┬───────────┬───────────┐      │
│  │   Mesh    │  Layout   │  Hollow   │      │
│  │  (load,   │  (bin     │  (SDF,    │      │
│  │   repair, │   pack,   │   drain   │      │
│  │   orient) │   arrange)│   holes)  │      │
│  └───────────┴───────────┴───────────┘      │
└─────────────────────────────────────────────┘
```

### 6.2 Key Dependencies

**Rust Core:**
| Crate | Version | Purpose |
|-------|---------|---------|
| `wgpu` | latest | GPU compute + rendering for slicing |
| `glam` | 0.x | Vector/matrix math (SIMD) |
| `parry3d` | latest | Mesh collision, plane intersection |
| `tri-mesh` | latest | Half-edge mesh data structure |
| `rs-read-trimesh` | latest | STL/OBJ/PLY file loading |
| `image` | latest | Image encoding/manipulation |
| `rayon` | latest | CPU parallelism |
| `rectangle-pack` | latest | 2D bin packing for layout |
| `bincode` / `bytemuck` | latest | Binary serialization for file formats |
| `flate2` | latest | zlib compression (some formats) |

**Frontend (Tauri WebView):**
| Package | Purpose |
|---------|---------|
| `three` | 3D rendering |
| `@react-three/fiber` | React bindings for Three.js |
| `@react-three/drei` | Three.js helpers (controls, loaders) |
| `@tauri-apps/api` | Tauri IPC bridge |
| `zustand` | State management |

### 6.3 Tauri + Three.js Considerations

**Known issues:**
- WebGL context loss reported on some platforms ([issue #6559](https://github.com/tauri-apps/tauri/issues/6559))
- WebGL2 support varies by platform WebView ([issue #2866](https://github.com/tauri-apps/tauri/issues/2866))
- Use `convertFileSrc()` for loading local mesh files

**Mitigations:**
- Use Tauri 2.0 which has better WebView support
- On Linux, ensure WebKitGTK is recent enough for WebGL2
- Fallback: consider wgpu-native for 3D viewport instead of WebView (like mslicer does)

**Alternative:** Skip Tauri entirely and use `egui` + `wgpu` for a fully native Rust GUI (this is what mslicer appears to do). Faster, no WebView issues, but more code to write for UI.

### 6.4 GPU Acceleration Architecture

```
┌─────────────────────────────────────┐
│         wgpu Rendering Pipeline     │
│                                     │
│  1. Load mesh as vertex buffer      │
│  2. Per layer:                      │
│     a. Set Z-clip plane             │
│     b. Pass 1: front faces → stencil++ │
│     c. Pass 2: back faces → stencil-- │
│     d. Pass 3: render where stencil≠0 │
│     e. Read back framebuffer pixels │
│     f. RLE encode → layer data      │
│                                     │
│  Compute shaders (WGSL):            │
│  - RLE encoding on GPU              │
│  - Anti-aliasing resolve            │
│  - Island detection (parallel flood)│
└─────────────────────────────────────┘
```

---

## 7. Architecture Blueprint

### 7.1 Crate Structure

```
resin-slicer/
├── Cargo.toml              # workspace
├── crates/
│   ├── slicer-core/        # Mesh loading, slicing, supports
│   │   ├── src/
│   │   │   ├── mesh/       # Mesh types, loading (STL/OBJ/GLB)
│   │   │   ├── slicer/     # GPU slicing pipeline
│   │   │   ├── supports/   # Support generation (tree, pillar)
│   │   │   ├── hollow/     # Hollowing (SDF-based)
│   │   │   ├── layout/     # Bin packing, auto-arrange
│   │   │   ├── orient/     # Auto-orientation
│   │   │   └── island/     # Island detection
│   │   └── Cargo.toml
│   ├── slicer-formats/     # File format encoders/decoders
│   │   ├── src/
│   │   │   ├── ctb.rs      # CTB/CBDDLP (v2-v5)
│   │   │   ├── goo.rs      # Elegoo GOO
│   │   │   ├── sl1.rs      # Prusa SL1 (ZIP+PNG)
│   │   │   ├── photon.rs   # Photon/PHZ
│   │   │   ├── cxdlp.rs    # Creality
│   │   │   ├── rle.rs      # Shared RLE encoding
│   │   │   ├── crypto.rs   # CTB encryption
│   │   │   └── preview.rs  # Thumbnail encoding (RGB565)
│   │   └── Cargo.toml
│   ├── slicer-gpu/         # wgpu rendering pipeline
│   │   ├── src/
│   │   │   ├── pipeline.rs # Render pipeline setup
│   │   │   ├── stencil.rs  # Stencil-buffer slicing
│   │   │   ├── shaders/    # WGSL shaders
│   │   │   └── readback.rs # Framebuffer → image
│   │   └── Cargo.toml
│   └── slicer-app/         # Desktop application
│       ├── src/
│       │   ├── main.rs
│       │   ├── ui/         # egui panels
│       │   └── viewport/   # 3D viewport
│       └── Cargo.toml
├── shaders/
│   ├── slice.wgsl          # Slicing vertex/fragment shaders
│   └── preview.wgsl        # 3D preview shaders
└── profiles/               # Printer profiles (JSON)
    ├── anycubic-photon-mono.json
    ├── elegoo-saturn-4.json
    └── ...
```

### 7.2 Slicing Pipeline

```
STL/OBJ/GLB file
    │
    ▼
┌──────────┐     ┌──────────┐     ┌──────────┐
│  Load    │────▶│  Orient  │────▶│  Layout  │
│  Mesh    │     │  (auto)  │     │  (pack)  │
└──────────┘     └──────────┘     └──────────┘
                                       │
                                       ▼
                                 ┌──────────┐
                                 │ Supports │
                                 │ Generate │
                                 └──────────┘
                                       │
                                       ▼
                                 ┌──────────┐
                                 │  Hollow  │
                                 │ (optional)│
                                 └──────────┘
                                       │
                                       ▼
                              ┌────────────────┐
                              │  GPU Slice     │
                              │  (wgpu stencil)│
                              │  → layer images│
                              └────────────────┘
                                       │
                                       ▼
                              ┌────────────────┐
                              │  Post Process  │
                              │  - AA resolve  │
                              │  - Island check│
                              │  - Edge smooth │
                              └────────────────┘
                                       │
                                       ▼
                              ┌────────────────┐
                              │  Encode        │
                              │  - RLE compress│
                              │  - Encrypt     │
                              │  - Write CTB/  │
                              │    GOO/SL1     │
                              └────────────────┘
                                       │
                                       ▼
                              .ctb / .goo file
```

### 7.3 MVP Feature Roadmap

**Phase 1 — Slice & Export (core value)**
- [ ] Load STL/OBJ files
- [ ] GPU slicing via wgpu stencil buffer
- [ ] CTB v3 file export (covers most printers)
- [ ] GOO file export (Elegoo)
- [ ] Basic CLI tool
- [ ] Thumbnail generation

**Phase 2 — Desktop App**
- [ ] egui or Tauri GUI with 3D viewport
- [ ] Model positioning/rotation/scaling
- [ ] Exposure time / layer height controls
- [ ] Printer profile management
- [ ] Slice preview (layer-by-layer viewer)

**Phase 3 — Supports**
- [ ] Overhang detection & visualization
- [ ] Island detection
- [ ] Pillar support generation
- [ ] Support mesh → slice integration
- [ ] Manual support point placement

**Phase 4 — Advanced**
- [ ] Tree support generation
- [ ] Auto-orientation
- [ ] Auto-layout (bin packing)
- [ ] Model hollowing
- [ ] Per-layer parameter editing
- [ ] Network printing (Chitu protocol)
- [ ] Additional formats (SL1, CXDLP, photon)

---

## References

### Source Code
- **mslicer** (Rust MSLA slicer): https://github.com/connorslade/mslicer
- **catibo** (Rust CTB format + docs): https://github.com/cbiffle/catibo
- **CTB format spec**: https://github.com/cbiffle/catibo/blob/master/doc/cbddlp-ctb.adoc
- **UVtools** (C# format reference): https://github.com/sn4k3/UVtools
- **PrusaSlicer SLA** (C++ supports/hollowing): https://github.com/prusa3d/PrusaSlicer/tree/master/src/libslic3r/SLA
- **Microtome** (WebGL GPU slicer): https://github.com/Microtome/microtome
- **PhotonFileValidator** (Java island detection): https://github.com/Photonsters/PhotonFileValidator

### Papers & Articles
- "An Optimal Algorithm for 3D Triangle Mesh Slicing" (2016) — https://www.sciencedirect.com/science/article/abs/pii/S0010448517301215
- "A GPU-based Parallel Slicer for 3D Printing" (2017) — https://ieeexplore.ieee.org/document/8256075
- "Slicing Algorithm and Partition Scanning Strategy Based on GPU Parallel Computing" (2021) — https://www.mdpi.com/1996-1944/14/15/4297
- "Local Barycenter Based Efficient Tree-Support Generation for 3D Printing" (2019) — https://www.sciencedirect.com/science/article/abs/pii/S0010448518303701
- "Escaping Tree-Support: Minimizing Contact Points" — https://users.encs.concordia.ca/~thkwok/publication/RPJ21_ETSup.pdf
- "GPU 3D Printing Slicer for DLP using PySLM" — https://lukeparry.uk/gpu-3d-printing-bitmap-slicer-for-dlp-jetting-using-pyslm/
- "Formware GPU vs CPU Slicing" — https://www.formware.co/article/SlicingSpeed
- "CTB v4 format info (Lychee docs)" — https://docs.mango3d.io/docs/lychee-slicer-resin/resin-and-printing-settings/essential-information-about-the-ctb-v4-file-format-2/

### Rust Crates
- `parry3d` — https://crates.io/crates/parry3d
- `tri-mesh` — https://crates.io/crates/tri-mesh
- `rs-read-trimesh` — https://crates.io/crates/rs-read-trimesh
- `wgpu` — https://crates.io/crates/wgpu
- `rectangle-pack` — https://crates.io/crates/rectangle-pack
- `max_rects` — https://crates.io/crates/max_rects
- `u-nesting` — https://github.com/iyulab/u-nesting
