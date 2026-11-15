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

#include "DetourLocalRebuild.h"
#include "DetourAlloc.h"
#include "DetourMath.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>

dtTileRange::dtTileRange() :
	minX(1),
	maxX(0),
	minY(1),
	maxY(0)
{
}

dtTileRebuildStats::dtTileRebuildStats() :
	tilesRequested(0),
	tilesBuilt(0),
	tilesRemoved(0),
	tilesEmpty(0),
	tilesFailed(0),
	bytesBuilt(0)
{
}

namespace
{
struct TileBuildEntry
{
	int x;
	int y;
	unsigned char* data;
	int dataSize;
	dtTileRef oldRef;
};

bool CalcTileAxisRange(const double changedMin, const double changedMax,
					   const double origin, const double tileSize,
					   const double halo, int& outMin, int& outMax)
{
	if (tileSize <= 0.0 || halo < 0.0)
		return false;

	const double minTile = floor((changedMin - halo - origin) / tileSize);
	const double maxTile = floor((changedMax + halo - origin) / tileSize);
	if (minTile < (double)INT_MIN || minTile > (double)INT_MAX ||
		maxTile < (double)INT_MIN || maxTile > (double)INT_MAX)
	{
		return false;
	}

	outMin = (int)minTile;
	outMax = (int)maxTile;
	return outMin <= outMax;
}
}

dtTileRange dtCalcAffectedTileRange(const dtNavMeshParams& params,
									const float* changedBmin,
									const float* changedBmax,
									float haloRadius)
{
	dtTileRange range;
	if (!changedBmin || !changedBmax)
		return range;

	if (!dtMathIsfinite(params.tileWidth) || !dtMathIsfinite(params.tileHeight) ||
		!dtMathIsfinite(params.orig[0]) || !dtMathIsfinite(params.orig[2]) ||
		!dtMathIsfinite(changedBmin[0]) || !dtMathIsfinite(changedBmin[2]) ||
		!dtMathIsfinite(changedBmax[0]) || !dtMathIsfinite(changedBmax[2]) ||
		!dtMathIsfinite(haloRadius))
	{
		return range;
	}

	if (params.tileWidth <= 0.0f || params.tileHeight <= 0.0f ||
		haloRadius < 0.0f ||
		changedBmin[0] > changedBmax[0] ||
		changedBmin[2] > changedBmax[2])
	{
		return range;
	}

	if (!CalcTileAxisRange(changedBmin[0], changedBmax[0], params.orig[0],
						   params.tileWidth, haloRadius, range.minX, range.maxX))
	{
		return dtTileRange();
	}
	if (!CalcTileAxisRange(changedBmin[2], changedBmax[2], params.orig[2],
						   params.tileHeight, haloRadius, range.minY, range.maxY))
	{
		return dtTileRange();
	}

	return range;
}

dtStatus dtRebuildAffectedTiles(dtNavMesh* navMesh,
								const dtTileRange& range,
								dtTileBuildCallback buildTile,
								void* user,
								dtTileRebuildStats* stats)
{
	dtTileRebuildStats localStats;
	if (stats)
		*stats = localStats;

	if (!navMesh || !buildTile)
		return DT_FAILURE | DT_INVALID_PARAM;
	if (range.isEmpty())
		return DT_SUCCESS;

	const double widthD = (double)range.maxX - (double)range.minX + 1.0;
	const double heightD = (double)range.maxY - (double)range.minY + 1.0;
	if (widthD > (double)INT_MAX || heightD > (double)INT_MAX)
		return DT_FAILURE | DT_INVALID_PARAM;

	const size_t width = (size_t)widthD;
	const size_t height = (size_t)heightD;
	if (height > (size_t)-1 / width)
		return DT_FAILURE | DT_INVALID_PARAM;

	const size_t count = width * height;
	if (count > (size_t)INT_MAX || count > (size_t)-1 / sizeof(TileBuildEntry))
		return DT_FAILURE | DT_INVALID_PARAM;

	TileBuildEntry* entries = (TileBuildEntry*)dtAlloc(
		sizeof(TileBuildEntry) * count, DT_ALLOC_TEMP);
	if (!entries)
		return DT_FAILURE | DT_OUT_OF_MEMORY;

	localStats.tilesRequested = (int)count;
	dtStatus result = DT_SUCCESS;

	for (size_t entryIndex = 0; entryIndex < count; ++entryIndex)
	{
		TileBuildEntry& entry = entries[entryIndex];
		entry.x = range.minX + (int)(entryIndex % width);
		entry.y = range.minY + (int)(entryIndex / width);
		entry.data = 0;
		entry.dataSize = 0;
		entry.oldRef = 0;

		dtStatus status = buildTile(user, entry.x, entry.y,
									&entry.data, &entry.dataSize);
		if (dtStatusFailed(status))
		{
			result = status;
			break;
		}

		if ((!entry.data && entry.dataSize != 0) ||
			(entry.data && entry.dataSize <= 0))
		{
			if (entry.data)
				dtFree(entry.data);
			entry.data = 0;
			entry.dataSize = 0;
			result = DT_FAILURE | DT_INVALID_PARAM;
			break;
		}

		if (entry.data)
		{
			const dtMeshHeader* header = (const dtMeshHeader*)entry.data;
			if (entry.dataSize < (int)sizeof(dtMeshHeader) ||
				header->magic != DT_NAVMESH_MAGIC ||
				header->version != DT_NAVMESH_VERSION ||
				header->x != entry.x ||
				header->y != entry.y ||
				header->layer != 0)
			{
				dtFree(entry.data);
				entry.data = 0;
				entry.dataSize = 0;
				result = DT_FAILURE | DT_INVALID_PARAM;
				break;
			}
			localStats.tilesBuilt++;
			localStats.bytesBuilt += (size_t)entry.dataSize;
		}
	}

	if (dtStatusFailed(result))
	{
		for (size_t i = 0; i < count; ++i)
			dtFree(entries[i].data);
		dtFree(entries);
		if (stats)
			*stats = localStats;
		return result;
	}

	// Build succeeded for the whole range. The build callback has not modified
	// the navmesh, so a build failure above leaves the old tiles untouched.
	for (size_t i = 0; i < count; ++i)
	{
		TileBuildEntry& entry = entries[i];
		entry.oldRef = navMesh->getTileRefAt(entry.x, entry.y, 0);
		if (entry.oldRef)
		{
			dtStatus status = navMesh->removeTile(entry.oldRef, 0, 0);
			if (dtStatusFailed(status))
			{
				result = status;
				break;
			}
			localStats.tilesRemoved++;
		}
	}

	if (dtStatusFailed(result))
	{
		for (size_t i = 0; i < count; ++i)
			dtFree(entries[i].data);
		dtFree(entries);
		if (stats)
			*stats = localStats;
		return result;
	}

	// Restore tiles with an existing reference first. Otherwise a new tile can
	// consume a slot that a later addTile(lastRef) call needs to restore.
	for (size_t i = 0; i < count; ++i)
	{
		if (!entries[i].data)
			localStats.tilesEmpty++;
	}

	for (int pass = 0; pass < 2; ++pass)
	{
		for (size_t i = 0; i < count; ++i)
		{
			TileBuildEntry& entry = entries[i];
			if (!entry.data)
				continue;
			if ((pass == 0) != (entry.oldRef != 0))
				continue;

			dtStatus status = navMesh->addTile(entry.data, entry.dataSize,
											   DT_TILE_FREE_DATA, entry.oldRef, 0);
			if (dtStatusFailed(status))
			{
				dtFree(entry.data);
				localStats.tilesFailed++;
				if (!dtStatusFailed(result))
					result = status;
			}
		}
	}

	dtFree(entries);
	if (stats)
		*stats = localStats;
	return result;
}
