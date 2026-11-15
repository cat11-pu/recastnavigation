//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#ifndef DETOURLOCALREBUILD_H
#define DETOURLOCALREBUILD_H

#include "DetourNavMesh.h"
#include <stddef.h>

/// A rectangular range of tile grid coordinates.
/// The range is empty when minX > maxX or minY > maxY.
/// @ingroup detour
struct dtTileRange
{
	dtTileRange();

	int minX;
	int maxX;
	int minY;
	int maxY;

	bool isEmpty() const { return minX > maxX || minY > maxY; }
};

/// Statistics from #dtRebuildAffectedTiles.
/// @ingroup detour
struct dtTileRebuildStats
{
	dtTileRebuildStats();

	int tilesRequested;	///< Number of tile coordinates in the affected range.
	int tilesBuilt;		///< Number of non-empty tile data sets produced.
	int tilesRemoved;	///< Number of existing tiles removed during commit.
	int tilesEmpty;		///< Number of affected coordinates left without a tile.
	int tilesFailed;	///< Number of tile data sets that could not be committed.
	size_t bytesBuilt;	///< Total size of the non-empty tile data sets.
};

/// Builds one tile without modifying the navigation mesh.
///
/// The callback must use the same tile bounds, border, and configuration as a
/// full build. Return DT_SUCCESS with *outData == 0 and *outDataSize == 0 to
/// remove the tile. Non-empty data must be allocated with dtAlloc(); ownership
/// is transferred to #dtRebuildAffectedTiles. The data header must be for layer
/// 0 at the requested (tx, ty) coordinate.
///
/// @param[in]	user		Opaque user pointer.
/// @param[in]	tx			Tile x coordinate.
/// @param[in]	ty			Tile y coordinate.
/// @param[out]	outData		New tile data, or null for an empty tile.
/// @param[out]	outDataSize	Size of the new tile data.
/// @return The status of the build.
/// @ingroup detour
typedef dtStatus (*dtTileBuildCallback)(void* user, int tx, int ty,
										unsigned char** outData, int* outDataSize);

/// Calculates the tiles whose build input intersects a changed world-space
/// bounding box.
///
/// @p haloRadius must be the world-space size of the Recast build border
/// (rcConfig::borderSize * rcConfig::cs). A tile is included when its padded
/// build footprint intersects the closed changed box. This includes a change
/// exactly on a tile seam in the tile on the positive side. Tile footprints are
/// half-open on their positive edges.
///
/// @param[in]	params		Navigation mesh tile parameters.
/// @param[in]	changedBmin	Minimum corner of the changed world-space bounds. [(x, y, z)]
/// @param[in]	changedBmax	Maximum corner of the changed world-space bounds. [(x, y, z)]
/// @param[in]	haloRadius	Build halo in world units. [Limit: >= 0]
/// @return The affected range, or an empty range for invalid input.
/// @ingroup detour
dtTileRange dtCalcAffectedTileRange(const dtNavMeshParams& params,
									const float* changedBmin,
									const float* changedBmax,
									float haloRadius);

/// Rebuilds a range of layer-0 navigation mesh tiles.
///
/// All tile data is built before the navigation mesh is modified. The commit
/// removes the affected tiles and then adds the new data through
/// dtNavMesh::addTile(), which reconnects internal, external, and off-mesh
/// links in both directions. Existing tile references are restored when
/// possible. Tiles returned as empty are removed and are not re-added.
///
/// The range should come from #dtCalcAffectedTileRange. Passing a smaller range
/// can leave stale border data in neighboring tiles and is not supported.
///
/// Memory use is O(number of affected tiles) for tile data and bookkeeping.
/// The build callback is invoked sequentially, so its temporary build memory
/// only needs to be bounded by one tile.
///
/// @param[in,out]	navMesh		Navigation mesh to update.
/// @param[in]		range		Affected tile range.
/// @param[in]		buildTile	Tile build callback.
/// @param[in]		user		Opaque pointer passed to the callback.
/// @param[out]		stats		Optional rebuild statistics.
/// @return DT_SUCCESS, or the first failure encountered.
/// @ingroup detour
dtStatus dtRebuildAffectedTiles(dtNavMesh* navMesh,
								const dtTileRange& range,
								dtTileBuildCallback buildTile,
								void* user,
								dtTileRebuildStats* stats = 0);

#endif // DETOURLOCALREBUILD_H
