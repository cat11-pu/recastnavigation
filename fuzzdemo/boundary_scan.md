| API | Input boundary | Result | rcErrorCode | Location |
|---|---|---|---|---|
| rcCreateHeightfield | w=-1 | REJECTED | 2 | rcCreateHeightfield: empty sub-grid (sizeX=-1, sizeZ=4) |
| rcCreateHeightfield | w=0 | REJECTED | 2 | rcCreateHeightfield: empty sub-grid (sizeX=0, sizeZ=4) |
| rcCreateHeightfield | w=1 (min legal) | accepted | 0 |  |
| rcCreateHeightfield | h=0 | REJECTED | 2 | rcCreateHeightfield: empty sub-grid (sizeX=4, sizeZ=0) |
| rcCreateHeightfield | h=-7 | REJECTED | 2 | rcCreateHeightfield: empty sub-grid (sizeX=4, sizeZ=-7) |
| rcCreateHeightfield | h=1 (min legal) | accepted | 0 |  |
| rcCreateHeightfield | 2^19 x 1 (under cap) | accepted | 0 |  |
| rcCreateHeightfield | 2^20 x 1 (at per-axis cap, total over) | accepted | 0 |  |
| rcCreateHeightfield | 1024x1024=2^20 (exact total cap) | accepted | 0 |  |
| rcCreateHeightfield | 1025x1024 (just over cap) | REJECTED | 8 | rcCreateHeightfield: grid 1025 x 1024 exceeds the 1048576 cell limit |
| rcCreateHeightfield | 50000x50000 (huge) | REJECTED | 8 | rcCreateHeightfield: grid 50000 x 50000 exceeds the 1048576 cell limit |
| rcCreateHeightfield | cs=0 | REJECTED | 3 | rcCreateHeightfield: cellSize=0 cellHeight=0.2 must be finite and positive |
| rcCreateHeightfield | cs<0 | REJECTED | 3 | rcCreateHeightfield: cellSize=-0.3 cellHeight=0.2 must be finite and positive |
| rcCreateHeightfield | cs=NaN | REJECTED | 3 | rcCreateHeightfield: cellSize=nan cellHeight=0.2 must be finite and positive |
| rcCreateHeightfield | ch=0 | REJECTED | 3 | rcCreateHeightfield: cellSize=0.3 cellHeight=0 must be finite and positive |
| rcCreateHeightfield | ch<0 | REJECTED | 3 | rcCreateHeightfield: cellSize=0.3 cellHeight=-1 must be finite and positive |
| rcCreateHeightfield | ch=Inf | REJECTED | 3 | rcCreateHeightfield: cellSize=0.3 cellHeight=inf must be finite and positive |
| rcCreateHeightfield | cs=tiny(1e-30) | accepted | 0 |  |
| rcCreateHeightfield | cs=max-finite | accepted | 0 |  |
| rcCreateHeightfield | normal | accepted | 0 |  |
| rcCreateHeightfield | zero X span | accepted | 0 |  |
| rcCreateHeightfield | inverted X | REJECTED | 4 | rcCreateHeightfield: malformed or non-finite bounding box |
| rcCreateHeightfield | NaN in max | REJECTED | 4 | rcCreateHeightfield: malformed or non-finite bounding box |
| rcCreateHeightfield | Inf in min | REJECTED | 4 | rcCreateHeightfield: malformed or non-finite bounding box |
| rcCalcGridSize | span=1e15 cs=.3 -> sx=1048576 sz=1048576 | accepted | 0 | sizes within range |
| rcCalcGridSize | span=FLT_MAX -> sx=1048576 sz=1048576 | accepted | 0 | sizes within range |
| rcCalcGridSize | cs=0 -> sx=0 sz=0 | REJECTED | 0 | zeroed/clamped, no overflow |
| rcCalcGridSize | cs<0 -> sx=0 sz=0 | REJECTED | 0 | zeroed/clamped, no overflow |
| rcCalcGridSize | cs=NaN -> sx=0 sz=0 | REJECTED | 0 | zeroed/clamped, no overflow |
| rcCalcGridSize | span=NaN -> sx=0 sz=0 | REJECTED | 0 | zeroed/clamped, no overflow |
| rcCalcGridSize | zero span -> sx=0 sz=0 | accepted | 0 | sizes within range |
| rcCalcGridSize | span=9 cs=.3 (legal) -> sx=30 sz=30 | accepted | 0 | sizes within range |
| rcRasterizeTriangle | normal triangle | accepted | 0 |  |
| rcRasterizeTriangle | collinear (zero area) | accepted | 0 |  |
| rcRasterizeTriangle | duplicate vertex | accepted | 0 |  |
| rcRasterizeTriangle | NaN coordinate | REJECTED | 6 | rcRasterizeTriangle: vertex coordinate is NaN or infinity |
| rcRasterizeTriangle | +Inf coordinate | REJECTED | 6 | rcRasterizeTriangle: vertex coordinate is NaN or infinity |
| rcRasterizeTriangle | -Inf coordinate | REJECTED | 6 | rcRasterizeTriangle: vertex coordinate is NaN or infinity |
| rcRasterizeTriangle | FLT_MAX coordinate | accepted | 0 |  |
| rcRasterizeTriangles(int) | all valid [0,3) | accepted | 0 |  |
| rcRasterizeTriangles(int) | index == numVerts | REJECTED | 5 | rcRasterizeTriangles: triangle 0 vertex index 3 (corner 2) is outside [0, 3) |
| rcRasterizeTriangles(int) | index > numVerts | REJECTED | 5 | rcRasterizeTriangles: triangle 0 vertex index 4 (corner 2) is outside [0, 3) |
| rcRasterizeTriangles(int) | negative index | REJECTED | 5 | rcRasterizeTriangles: triangle 0 vertex index -1 (corner 0) is outside [0, 3) |
| rcRasterizeTriangles(int) | INT_MIN index | REJECTED | 5 | rcRasterizeTriangles: triangle 0 vertex index -2147483648 (corner 1) is outside [0, 3) |
| rcRasterizeTriangles(ushort) | all valid | accepted | 0 |  |
| rcRasterizeTriangles(ushort) | index == numVerts | REJECTED | 5 | rcRasterizeTriangles: triangle 0 vertex index 3 (corner 2) is outside [0, 3) |
| rcRasterizeTriangles(ushort) | USHRT_MAX index | REJECTED | 5 | rcRasterizeTriangles: triangle 0 vertex index 65535 (corner 2) is outside [0, 3) |
| rcRasterizeTriangles(int) | numTris=0 (legal empty) | accepted | 0 |  |
| rcRasterizeTriangles(int) | numTris=-1 | REJECTED | 2 | rcRasterizeTriangles: numTris -1 must be >= 0 |
| rcRasterizeTriangles(int) | numVerts=0 | REJECTED | 2 | rcRasterizeTriangles: numVerts 0 must be > 0 |
| rcRasterizeTriangles(int) | flagMergeThreshold=-1 | REJECTED | 2 | rcRasterizeTriangles: flagMergeThreshold -1 must be >= 0 |
| rcRasterizeTriangles(int) | null verts | REJECTED | 1 | rcRasterizeTriangles: null input pointer |
| rcRasterizeTriangles(int) | null tris | REJECTED | 1 | rcRasterizeTriangles: null input pointer |
| rcMarkWalkableTriangles | valid | accepted | 0 |  |
| rcMarkWalkableTriangles | OOB index | REJECTED | 5 | rcMarkWalkableTriangles: triangle 0 vertex index 3 is outside [0, 3) |
| rcMarkWalkableTriangles | negative index | REJECTED | 5 | rcMarkWalkableTriangles: triangle 0 vertex index -1 is outside [0, 3) |
