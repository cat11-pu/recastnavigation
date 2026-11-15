# Navmesh generation input hardening — reproducer, fix and boundary data

This directory contains the evidence and tooling for hardening the Recast
rasterization / heightfield entry points against malformed input.

## The four failure classes and how to reproduce them

All reproducers are built against the ASan+UBSan build of the library:

```sh
cmake -S . -B buildasan -DRECASTNAVIGATION_DEMO=OFF -DRECASTNAVIGATION_EXAMPLES=OFF \
      -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined -fno-sanitize-recover=all -g -O1" \
      -DCMAKE_C_FLAGS="-fsanitize=address,undefined -fno-sanitize-recover=all -g -O1" \
      -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address,undefined"
cmake --build buildasan -j

g++ -std=c++17 -fsanitize=address,undefined -fno-sanitize-recover=all -fno-rtti \
    -IRecast/Include fuzzdemo/repro.cpp buildasan/Recast/libRecast-d.a -o fuzzdemo/repro
ASAN_OPTIONS=detect_leaks=0 ./fuzzdemo/repro
```

`repro.cpp` groups the cases (`REPRO_CASE=empty|huge|degen|index|mark`):

| Class | Sample input | Observed before the fix |
|---|---|---|
| Empty sub-grid | `rcCreateHeightfield(..., 0, 10, ...)` or negative size | `malloc(0)`-style state left `spans == NULL`; freeing the heightfield freed a stack pointer (ASan `attempting free on address which was not malloc()-ed`) |
| Over-allocation / overflow | `rcCreateHeightfield(..., 50000, 50000, ...)`; `rcCalcGridSize` on a `1e15` span with `cs=0.3` | `width*height` int overflow in the allocation size; `float->int` conversion is undefined behaviour (`signed integer overflow` UBSan report in `rasterizeTri`) |
| Degenerate / duplicate / NaN triangle | collinear points, three identical points, a `NaN` coordinate | NaN defeats `rcClamp`, producing out-of-range cell indices and OOB span writes that corrupt the span free list (ASan bad-free / double-free); collinear/duplicate triangles rasterized phantom volume |
| Bad triangle index | index `99`/`-1`/`65000` with `numVerts=3` | unchecked `verts[index*3]` out-of-bounds read in `rasterizeTri` / `calcTriNormal` |

`nan_oob.cpp` is the minimal isolated NaN OOB-write reproducer.

## Fix summary

Public API contract (source compatible; boolean return values preserved):

- New `rcErrorCode` enum and `rcContext::getLastError()`,
  `rcContext::getLastErrorLocation()`, `rcContext::clearError()`,
  `rcContext::raiseError()`. Every rejection records a code and a printf-style
  location that names the API plus the offending triangle/vertex index.
- `rcCreateHeightfield` rejects null bounds, non-positive dimensions,
  non-positive/non-finite cell sizes, malformed bounds and grids whose cell
  count exceeds `RC_MAX_HEIGHTFIELD_CELLS` (2^20), so the span array can never
  overflow an `int` or over-allocate. A null context still works (historical
  behaviour) but returns `false` on bad input.
- `rcCalcGridSize` validates its arguments, zeroes outputs on bad input and
  clamps huge spans to the cell limit instead of relying on undefined
  float-to-int conversion.
- `rcRasterizeTriangle` / the three `rcRasterizeTriangles` overloads validate
  pointers, counts, the heightfield, `flagMergeThreshold`, every triangle index
  (against `numVerts`, including negative and `unsigned short` ranges) and every
  vertex coordinate (finite check). Degenerate zero-area triangles are skipped
  with a warning and create no spans; NaN/Inf vertices are a hard
  `RC_ERROR_NON_FINITE_VERTEX`.
- `rasterizeTri` gains an internal finite/range guard before the float-to-int
  grid footprint conversion so extreme finite coordinates cannot reach the
  overflow path in `dividePoly`.
- `rcMarkWalkableTriangles` / `rcClearUnwalkableTriangles` validate pointers,
  counts and every index, leaving the output area buffer untouched on failure.

## Boundary scan data

`boundary_scan.md` (regenerate with `boundary_scan.cpp`) is a 56-probe sweep
across grid dimensions, cell sizes, bounds, coordinate magnitudes, index ranges
and null/count arguments. Result on the fixed library:

```
SCAN COMPLETE: 56 probes, 35 rejected, 21 accepted
```

Every rejected probe carries an explicit `rcErrorCode` and location string; the
whole scan runs clean under strict ASan+UBSan.

## Legal-output preservation

- `golden.cpp` and `golden2.cpp` run full valid pipelines (terrain, slopes,
  large-but-finite origins, all three rasterize overloads, filters and compact
  heightfield). Their output is byte-identical between the original and fixed
  libraries (2003-line and 963-line diffs are empty).
- `fuzz.cpp` adds 200 randomized valid meshes (identical span signatures vs the
  original) and 300 randomized malformed stress iterations (no crash, no
  sanitizer report).
- The Catch2 suite passes fully (39 test cases, 5265 assertions) both normally
  and under ASan+UBSan. The previously `SKIP`-disabled "degenerate triangles
  rasterize to nothing" test is now enabled and passes.
