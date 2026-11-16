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

#include <math.h>
#include <string.h>
#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"

/// Check whether two bounding boxes overlap
///
/// @param[in]	aMin	Min axis extents of bounding box A
/// @param[in]	aMax	Max axis extents of bounding box A
/// @param[in]	bMin	Min axis extents of bounding box B
/// @param[in]	bMax	Max axis extents of bounding box B
/// @returns true if the two bounding boxes overlap.  False otherwise.
static bool overlapBounds(const float* aMin, const float* aMax, const float* bMin, const float* bMax)
{
	return
		aMin[0] <= bMax[0] && aMax[0] >= bMin[0] &&
		aMin[1] <= bMax[1] && aMax[1] >= bMin[1] &&
		aMin[2] <= bMax[2] && aMax[2] >= bMin[2];
}

/// Allocates a new span in the heightfield.
/// Use a memory pool and free list to minimize actual allocations.
/// 
/// @param[in]	heightfield		The heightfield
/// @returns A pointer to the allocated or re-used span memory. 
static rcSpan* allocSpan(rcHeightfield& heightfield)
{
	// If necessary, allocate new page and update the freelist.
	if (heightfield.freelist == NULL || heightfield.freelist->next == NULL)
	{
		// Create new page.
		// Allocate memory for the new pool.
		rcSpanPool* spanPool = (rcSpanPool*)rcAlloc(sizeof(rcSpanPool), RC_ALLOC_PERM);
		if (spanPool == NULL)
		{
			return NULL;
		}

		// Add the pool into the list of pools.
		spanPool->next = heightfield.pools;
		heightfield.pools = spanPool;
		
		// Add new spans to the free list.
		rcSpan* freeList = heightfield.freelist;
		rcSpan* head = &spanPool->items[0];
		rcSpan* it = &spanPool->items[RC_SPANS_PER_POOL];
		do
		{
			--it;
			it->next = freeList;
			freeList = it;
		}
		while (it != head);
		heightfield.freelist = it;
	}

	// Pop item from the front of the free list.
	rcSpan* newSpan = heightfield.freelist;
	heightfield.freelist = heightfield.freelist->next;
	return newSpan;
}

/// Releases the memory used by the span back to the heightfield, so it can be re-used for new spans.
/// @param[in]	heightfield		The heightfield.
/// @param[in]	span	A pointer to the span to free
static void freeSpan(rcHeightfield& heightfield, rcSpan* span)
{
	if (span == NULL)
	{
		return;
	}
	// Add the span to the front of the free list.
	span->next = heightfield.freelist;
	heightfield.freelist = span;
}

/// Adds a span to the heightfield.  If the new span overlaps existing spans,
/// it will merge the new span with the existing ones.
///
/// @param[in]	heightfield					Heightfield to add spans to
/// @param[in]	x					The new span's column cell x index
/// @param[in]	z					The new span's column cell z index
/// @param[in]	min					The new span's minimum cell index
/// @param[in]	max					The new span's maximum cell index
/// @param[in]	areaID				The new span's area type ID
/// @param[in]	flagMergeThreshold	How close two spans maximum extents need to be to merge area type IDs
static inline bool addSpan(rcHeightfield& heightfield,
                           const int x, const int z,
                           const unsigned short min, const unsigned short max,
                           const unsigned char areaID, const int flagMergeThreshold)
{
	const int columnIndex = x + z * heightfield.width;
	const bool columnWasEmpty = heightfield.spans[columnIndex] == NULL;
	if (columnWasEmpty)
	{
		rcSpan* span = allocSpan(heightfield);
		if (span == NULL)
			return false;
		span->smin = min;
		span->smax = max;
		span->area = areaID;
		span->next = NULL;
		heightfield.spans[columnIndex] = span;
		if (heightfield.columnMask)
			heightfield.columnMask[columnIndex >> 5] |= 1u << (columnIndex & 31);
		return true;
	}

	rcSpan* firstSpan = heightfield.spans[columnIndex];
	if (firstSpan->next == NULL && firstSpan->smax >= min && firstSpan->smin <= max)
	{
		const unsigned short mergedSmin = rcMin(static_cast<unsigned short>(firstSpan->smin), min);
		const unsigned short mergedSmax = rcMax(static_cast<unsigned short>(firstSpan->smax), max);
		unsigned char mergedArea = areaID;
		if (rcAbs((int)mergedSmax - (int)firstSpan->smax) <= flagMergeThreshold)
			mergedArea = rcMax(mergedArea, static_cast<unsigned char>(firstSpan->area));
		firstSpan->smin = mergedSmin;
		firstSpan->smax = mergedSmax;
		firstSpan->area = mergedArea;
		return true;
	}

	rcSpan* previousSpan = NULL;
	rcSpan* currentSpan = heightfield.spans[columnIndex];
	rcSpan* newSpan = NULL;
	unsigned short newSpanMin = min;
	unsigned short newSpanMax = max;
	unsigned char newSpanArea = areaID;
	
	// Insert the new span, possibly merging it with existing spans.
	while (currentSpan != NULL)
	{
		if (currentSpan->smin > newSpanMax)
		{
			// Current span is completely after the new span, break.
			break;
		}
		
		if (currentSpan->smax < newSpanMin)
		{
			// Current span is completely before the new span.  Keep going.
			previousSpan = currentSpan;
			currentSpan = currentSpan->next;
		}
		else
		{
			// The new span overlaps with an existing span.  Merge them.
			if (currentSpan->smin < newSpanMin)
			{
				newSpanMin = currentSpan->smin;
			}
			if (currentSpan->smax > newSpanMax)
			{
				newSpanMax = currentSpan->smax;
			}
			
			// Merge flags.
			if (rcAbs((int)newSpanMax - (int)currentSpan->smax) <= flagMergeThreshold)
			{
				// Higher area ID numbers indicate higher resolution priority.
				newSpanArea = rcMax(newSpanArea, static_cast<unsigned char>(currentSpan->area));
			}
			
			rcSpan* next = currentSpan->next;
			if (newSpan == NULL)
			{
				// Reuse the first overlapping span instead of allocating a new
				// one. This keeps the hot rasterization path allocation-free
				// when adding spans to an existing surface.
				newSpan = currentSpan;
				previousSpan = currentSpan;
			}
			else
			{
				// Remove the current span since it's now merged with newSpan.
				// Keep going because there might be other overlapping spans
				// that also need to be merged.
				freeSpan(heightfield, currentSpan);
				if (previousSpan)
				{
					previousSpan->next = next;
				}
				else
				{
					heightfield.spans[columnIndex] = next;
				}
			}
			currentSpan = next;
		}
	}

	if (newSpan != NULL)
	{
		newSpan->smin = newSpanMin;
		newSpan->smax = newSpanMax;
		newSpan->area = newSpanArea;
	}
	else
	{
		newSpan = allocSpan(heightfield);
		if (newSpan == NULL)
		{
			return false;
		}
		newSpan->smin = newSpanMin;
		newSpan->smax = newSpanMax;
		newSpan->area = newSpanArea;

		// Insert new span after prev
		if (previousSpan != NULL)
		{
			newSpan->next = previousSpan->next;
			previousSpan->next = newSpan;
		}
		else
		{
			// This span should go before the others in the list
			newSpan->next = heightfield.spans[columnIndex];
			heightfield.spans[columnIndex] = newSpan;
		}
	}

	return true;
}

bool rcAddSpan(rcContext* context, rcHeightfield& heightfield,
               const int x, const int z,
               const unsigned short spanMin, const unsigned short spanMax,
               const unsigned char areaID, const int flagMergeThreshold)
{
	rcAssert(context);

	// Span is zero size or inverted size. Ignore.
	if (spanMin >= spanMax)
	{
		context->log(RC_LOG_WARNING, "rcAddSpan: Adding a span with zero or negative size. Ignored.");
		return true;
	}

	if (!addSpan(heightfield, x, z, spanMin, spanMax, areaID, flagMergeThreshold))
	{
		context->log(RC_LOG_ERROR, "rcAddSpan: Out of memory.");
		return false;
	}

	return true;
}

enum rcAxis
{
	RC_AXIS_X = 0,
	RC_AXIS_Y = 1,
	RC_AXIS_Z = 2
};

/// Divides a convex polygon of max 12 vertices into two convex polygons
/// across a separating axis.
/// 
/// @param[in]	inVerts			The input polygon vertices
/// @param[in]	inVertsCount	The number of input polygon vertices
/// @param[out]	outVerts1		Resulting polygon 1's vertices
/// @param[out]	outVerts1Count	The number of resulting polygon 1 vertices
/// @param[out]	outVerts2		Resulting polygon 2's vertices
/// @param[out]	outVerts2Count	The number of resulting polygon 2 vertices
/// @param[in]	axisOffset		THe offset along the specified axis
/// @param[in]	axis			The separating axis
static inline void dividePoly(const float* inVerts, int inVertsCount,
                              float* outVerts1, int* outVerts1Count,
                              float* outVerts2, int* outVerts2Count,
                              float axisOffset, rcAxis axis)
{
	rcAssert(inVertsCount <= 12);
	if (inVertsCount == 0)
	{
		*outVerts1Count = 0;
		*outVerts2Count = 0;
		return;
	}

	// How far positive or negative away from the separating axis is each vertex.
	int inVertB = inVertsCount - 1;
	float deltaB = axisOffset - inVerts[inVertB * 3 + axis];

	int poly1Vert = 0;
	int poly2Vert = 0;
	for (int inVertA = 0; inVertA < inVertsCount; inVertB = inVertA, ++inVertA)
	{
		const float deltaA = axisOffset - inVerts[inVertA * 3 + axis];

		// If the two vertices are on the same side of the separating axis
		bool sameSide = (deltaA >= 0) == (deltaB >= 0);

		if (!sameSide)
		{
			float s = deltaB / (deltaB - deltaA);
			outVerts1[poly1Vert * 3 + 0] = inVerts[inVertB * 3 + 0] + (inVerts[inVertA * 3 + 0] - inVerts[inVertB * 3 + 0]) * s;
			outVerts1[poly1Vert * 3 + 1] = inVerts[inVertB * 3 + 1] + (inVerts[inVertA * 3 + 1] - inVerts[inVertB * 3 + 1]) * s;
			outVerts1[poly1Vert * 3 + 2] = inVerts[inVertB * 3 + 2] + (inVerts[inVertA * 3 + 2] - inVerts[inVertB * 3 + 2]) * s;
			rcVcopy(&outVerts2[poly2Vert * 3], &outVerts1[poly1Vert * 3]);
			poly1Vert++;
			poly2Vert++;
			
			// add the inVertA point to the right polygon. Do NOT add points that are on the dividing line
			// since these were already added above
			if (deltaA > 0)
			{
				rcVcopy(&outVerts1[poly1Vert * 3], &inVerts[inVertA * 3]);
				poly1Vert++;
			}
			else if (deltaA < 0)
			{
				rcVcopy(&outVerts2[poly2Vert * 3], &inVerts[inVertA * 3]);
				poly2Vert++;
			}
		}
		else
		{
			// add the inVertA point to the right polygon. Addition is done even for points on the dividing line
			if (deltaA >= 0)
			{
				rcVcopy(&outVerts1[poly1Vert * 3], &inVerts[inVertA * 3]);
				poly1Vert++;
				if (deltaA == 0)
				{
					rcVcopy(&outVerts2[poly2Vert * 3], &inVerts[inVertA * 3]);
					poly2Vert++;
				}
			}
			else
			{
				rcVcopy(&outVerts2[poly2Vert * 3], &inVerts[inVertA * 3]);
				poly2Vert++;
			}
		}
		deltaB = deltaA;
	}

	*outVerts1Count = poly1Vert;
	*outVerts2Count = poly2Vert;
}

///	Rasterize a single triangle to the heightfield.
///
///	This code is extremely hot, so much care should be given to maintaining maximum perf here.
/// 
/// @param[in] 	v0					Triangle vertex 0
/// @param[in] 	v1					Triangle vertex 1
/// @param[in] 	v2					Triangle vertex 2
/// @param[in] 	areaID				The area ID to assign to the rasterized spans
/// @param[in] 	heightfield			Heightfield to rasterize into
/// @param[in] 	heightfieldBBMin	The min extents of the heightfield bounding box
/// @param[in] 	heightfieldBBMax	The max extents of the heightfield bounding box
/// @param[in] 	cellSize			The x and z axis size of a voxel in the heightfield
/// @param[in] 	inverseCellSize		1 / cellSize
/// @param[in] 	inverseCellHeight	1 / cellHeight
/// @param[in] 	flagMergeThreshold	The threshold in which area flags will be merged 
/// @returns true if the operation completes successfully.  false if there was an error adding spans to the heightfield.
static bool rasterizeTri(const float* v0, const float* v1, const float* v2,
                         const unsigned char areaID, rcHeightfield& heightfield,
                         const float* heightfieldBBMin, const float* heightfieldBBMax,
                         const float cellSize, const float inverseCellSize, const float inverseCellHeight,
                         const int flagMergeThreshold,
                         const int clipX0, const int clipX1,
                         const int clipZ0, const int clipZ1)
{
	// Calculate the bounding box of the triangle.
	float triBBMin[3];
	rcVcopy(triBBMin, v0);
	rcVmin(triBBMin, v1);
	rcVmin(triBBMin, v2);

	float triBBMax[3];
	rcVcopy(triBBMax, v0);
	rcVmax(triBBMax, v1);
	rcVmax(triBBMax, v2);

	// If the triangle does not touch the bounding box of the heightfield, skip the triangle.
	if (!overlapBounds(triBBMin, triBBMax, heightfieldBBMin, heightfieldBBMax))
	{
		return true;
	}

	const int w = heightfield.width;
	const int h = heightfield.height;
	const float by = heightfieldBBMax[1] - heightfieldBBMin[1];

	// Calculate the footprint of the triangle on the grid's z-axis
	int z0 = (int)((triBBMin[2] - heightfieldBBMin[2]) * inverseCellSize);
	int z1 = (int)((triBBMax[2] - heightfieldBBMin[2]) * inverseCellSize);

	// use -1 rather than 0 to cut the polygon properly at the start of the tile
	z0 = rcClamp(z0, -1, h - 1);
	z1 = rcClamp(z1, 0, h - 1);

	// Clip the triangle into all grid cells it touches.
	float buf[7 * 3 * 4];
	float* in = buf;
	float* inRow = buf + 7 * 3;
	float* p1 = inRow + 7 * 3;
	float* p2 = p1 + 7 * 3;

	rcVcopy(&in[0], v0);
	rcVcopy(&in[1 * 3], v1);
	rcVcopy(&in[2 * 3], v2);
	int nvRow;
	int nvIn = 3;

	const int zEnd = rcMin(z1, clipZ1);
	for (int z = z0; z <= zEnd; ++z)
	{
		// Clip polygon to row. Store the remaining polygon as well
		const float cellZ = heightfieldBBMin[2] + (float)z * cellSize;
		dividePoly(in, nvIn, inRow, &nvRow, p1, &nvIn, cellZ + cellSize, RC_AXIS_Z);
		rcSwap(in, p1);
		
		if (nvRow < 3)
		{
			continue;
		}
		if (z < 0)
		{
			continue;
		}
		if (z < clipZ0)
			continue;
		
		// find X-axis bounds of the row
		float minX = inRow[0];
		float maxX = inRow[0];
		for (int vert = 1; vert < nvRow; ++vert)
		{
			if (minX > inRow[vert * 3])
			{
				minX = inRow[vert * 3];
			}
			if (maxX < inRow[vert * 3])
			{
				maxX = inRow[vert * 3];
			}
		}
		int x0 = (int)((minX - heightfieldBBMin[0]) * inverseCellSize);
		int x1 = (int)((maxX - heightfieldBBMin[0]) * inverseCellSize);
		if (x1 < 0 || x0 >= w)
		{
			continue;
		}
		x0 = rcClamp(x0, -1, w - 1);
		x1 = rcClamp(x1, 0, w - 1);

		int nv;
		int nv2 = nvRow;

		const int xEnd = rcMin(x1, clipX1);
		for (int x = x0; x <= xEnd; ++x)
		{
			// Clip polygon to column. store the remaining polygon as well
			const float cx = heightfieldBBMin[0] + (float)x * cellSize;
			dividePoly(inRow, nv2, p1, &nv, p2, &nv2, cx + cellSize, RC_AXIS_X);
			rcSwap(inRow, p2);
			
			if (nv < 3)
			{
				continue;
			}
			if (x < 0 || x < clipX0)
			{
				continue;
			}
			// Calculate min and max of the span.
			float spanMin = p1[1];
			float spanMax = p1[1];
			for (int vert = 1; vert < nv; ++vert)
			{
				spanMin = rcMin(spanMin, p1[vert * 3 + 1]);
				spanMax = rcMax(spanMax, p1[vert * 3 + 1]);
			}
			spanMin -= heightfieldBBMin[1];
			spanMax -= heightfieldBBMin[1];
			
			// Skip the span if it's completely outside the heightfield bounding box
			if (spanMax < 0.0f)
			{
				continue;
			}
			if (spanMin > by)
			{
				continue;
			}
			
			// Clamp the span to the heightfield bounding box.
			if (spanMin < 0.0f)
			{
				spanMin = 0;
			}
			if (spanMax > by)
			{
				spanMax = by;
			}

			// Snap the span to the heightfield height grid.
			unsigned short spanMinCellIndex = (unsigned short)rcClamp((int)floorf(spanMin * inverseCellHeight), 0, RC_SPAN_MAX_HEIGHT);
			unsigned short spanMaxCellIndex = (unsigned short)rcClamp((int)ceilf(spanMax * inverseCellHeight), (int)spanMinCellIndex + 1, RC_SPAN_MAX_HEIGHT);

			if (!addSpan(heightfield, x, z, spanMinCellIndex, spanMaxCellIndex, areaID, flagMergeThreshold))
			{
				return false;
			}
		}
	}

	return true;
}

namespace
{
struct RasterTriRef
{
	const float* v[3];
	unsigned char area;
};

const int RC_RASTER_CHUNK_SIZE = 64;
const int RC_RASTER_MIN_BINNED_TRIS = 512;

bool rasterizeTriRef(const RasterTriRef& tri, rcHeightfield& heightfield,
					 const float inverseCellSize, const float inverseCellHeight,
					 const int flagMergeThreshold,
					 const int clipX0, const int clipX1,
					 const int clipZ0, const int clipZ1)
{
	return rasterizeTri(tri.v[0], tri.v[1], tri.v[2], tri.area, heightfield,
						heightfield.bmin, heightfield.bmax, heightfield.cs,
						inverseCellSize, inverseCellHeight, flagMergeThreshold,
						clipX0, clipX1, clipZ0, clipZ1);
}

bool rasterizeTriangleRefs(const RasterTriRef* tris, const int numTris,
						   rcHeightfield& heightfield, const int flagMergeThreshold)
{
	const int w = heightfield.width;
	const int h = heightfield.height;
	const float inverseCellSize = 1.0f / heightfield.cs;
	const float inverseCellHeight = 1.0f / heightfield.ch;

	const int chunkCols = (w + RC_RASTER_CHUNK_SIZE - 1) / RC_RASTER_CHUNK_SIZE;
	const int chunkRows = (h + RC_RASTER_CHUNK_SIZE - 1) / RC_RASTER_CHUNK_SIZE;
	const int numChunks = chunkCols * chunkRows;
	if (numTris < RC_RASTER_MIN_BINNED_TRIS || numChunks < 4 ||
		numTris >= numChunks * 16)
	{
		for (int i = 0; i < numTris; ++i)
		{
			if (!rasterizeTriRef(tris[i], heightfield, inverseCellSize, inverseCellHeight,
								 flagMergeThreshold, 0, w - 1, 0, h - 1))
				return false;
		}
		return true;
	}
	int* chunkCounts = (int*)rcAlloc(sizeof(int) * numChunks, RC_ALLOC_TEMP);
	if (!chunkCounts)
		return false;
	memset(chunkCounts, 0, sizeof(int) * numChunks);

	int totalRefs = 0;
	for (int i = 0; i < numTris; ++i)
	{
		const RasterTriRef& tri = tris[i];
		float triBBMin[3];
		rcVcopy(triBBMin, tri.v[0]);
		rcVmin(triBBMin, tri.v[1]);
		rcVmin(triBBMin, tri.v[2]);
		float triBBMax[3];
		rcVcopy(triBBMax, tri.v[0]);
		rcVmax(triBBMax, tri.v[1]);
		rcVmax(triBBMax, tri.v[2]);
		if (!overlapBounds(triBBMin, triBBMax, heightfield.bmin, heightfield.bmax))
			continue;

		int z0 = (int)((triBBMin[2] - heightfield.bmin[2]) * inverseCellSize);
		int z1 = (int)((triBBMax[2] - heightfield.bmin[2]) * inverseCellSize);
		z0 = rcClamp(z0, 0, h - 1);
		z1 = rcClamp(z1, 0, h - 1);
		int x0 = (int)((triBBMin[0] - heightfield.bmin[0]) * inverseCellSize);
		int x1 = (int)((triBBMax[0] - heightfield.bmin[0]) * inverseCellSize);
		x0 = rcClamp(x0, 0, w - 1);
		x1 = rcClamp(x1, 0, w - 1);

		const int cx0 = x0 / RC_RASTER_CHUNK_SIZE;
		const int cx1 = x1 / RC_RASTER_CHUNK_SIZE;
		const int cz0 = z0 / RC_RASTER_CHUNK_SIZE;
		const int cz1 = z1 / RC_RASTER_CHUNK_SIZE;
		for (int cz = cz0; cz <= cz1; ++cz)
		{
			for (int cx = cx0; cx <= cx1; ++cx)
			{
				chunkCounts[cx + cz * chunkCols]++;
				totalRefs++;
			}
		}
	}

	if (totalRefs > numTris * 8)
	{
		rcFree(chunkCounts);
		for (int i = 0; i < numTris; ++i)
		{
			if (!rasterizeTriRef(tris[i], heightfield, inverseCellSize, inverseCellHeight,
								 flagMergeThreshold, 0, w - 1, 0, h - 1))
				return false;
		}
		return true;
	}

	int* offsets = (int*)rcAlloc(sizeof(int) * numChunks, RC_ALLOC_TEMP);
	int* cursor = (int*)rcAlloc(sizeof(int) * numChunks, RC_ALLOC_TEMP);
	int* refs = (int*)rcAlloc(sizeof(int) * totalRefs, RC_ALLOC_TEMP);
	if (!offsets || !cursor || (totalRefs > 0 && !refs))
	{
		rcFree(chunkCounts);
		rcFree(offsets);
		rcFree(cursor);
		rcFree(refs);
		return false;
	}

	int offset = 0;
	for (int i = 0; i < numChunks; ++i)
	{
		offsets[i] = offset;
		cursor[i] = offset;
		offset += chunkCounts[i];
	}
	memset(chunkCounts, 0, sizeof(int) * numChunks);

	for (int i = 0; i < numTris; ++i)
	{
		const RasterTriRef& tri = tris[i];
		float triBBMin[3];
		rcVcopy(triBBMin, tri.v[0]);
		rcVmin(triBBMin, tri.v[1]);
		rcVmin(triBBMin, tri.v[2]);
		float triBBMax[3];
		rcVcopy(triBBMax, tri.v[0]);
		rcVmax(triBBMax, tri.v[1]);
		rcVmax(triBBMax, tri.v[2]);
		if (!overlapBounds(triBBMin, triBBMax, heightfield.bmin, heightfield.bmax))
			continue;

		int z0 = (int)((triBBMin[2] - heightfield.bmin[2]) * inverseCellSize);
		int z1 = (int)((triBBMax[2] - heightfield.bmin[2]) * inverseCellSize);
		z0 = rcClamp(z0, 0, h - 1);
		z1 = rcClamp(z1, 0, h - 1);
		int x0 = (int)((triBBMin[0] - heightfield.bmin[0]) * inverseCellSize);
		int x1 = (int)((triBBMax[0] - heightfield.bmin[0]) * inverseCellSize);
		x0 = rcClamp(x0, 0, w - 1);
		x1 = rcClamp(x1, 0, w - 1);

		const int cx0 = x0 / RC_RASTER_CHUNK_SIZE;
		const int cx1 = x1 / RC_RASTER_CHUNK_SIZE;
		const int cz0 = z0 / RC_RASTER_CHUNK_SIZE;
		const int cz1 = z1 / RC_RASTER_CHUNK_SIZE;
		for (int cz = cz0; cz <= cz1; ++cz)
		{
			for (int cx = cx0; cx <= cx1; ++cx)
			{
				const int chunkIndex = cx + cz * chunkCols;
				refs[cursor[chunkIndex]++] = i;
			}
		}
	}

	for (int cz = 0; cz < chunkRows; ++cz)
	{
		const int clipZ0 = cz * RC_RASTER_CHUNK_SIZE;
		const int clipZ1 = rcMin(clipZ0 + RC_RASTER_CHUNK_SIZE - 1, h - 1);
		for (int cx = 0; cx < chunkCols; ++cx)
		{
			const int chunkIndex = cx + cz * chunkCols;
			const int count = cursor[chunkIndex] - offsets[chunkIndex];
			if (count == 0)
				continue;

			const int clipX0 = cx * RC_RASTER_CHUNK_SIZE;
			const int clipX1 = rcMin(clipX0 + RC_RASTER_CHUNK_SIZE - 1, w - 1);
			for (int i = 0; i < count; ++i)
			{
				const RasterTriRef& tri = tris[refs[offsets[chunkIndex] + i]];
				if (!rasterizeTriRef(tri, heightfield, inverseCellSize, inverseCellHeight,
									 flagMergeThreshold, clipX0, clipX1, clipZ0, clipZ1))
				{
					rcFree(chunkCounts);
					rcFree(offsets);
					rcFree(cursor);
					rcFree(refs);
					return false;
				}
			}
		}
	}

	rcFree(chunkCounts);
	rcFree(offsets);
	rcFree(cursor);
	rcFree(refs);
	return true;
}
}  // namespace

bool rcRasterizeTriangle(rcContext* context,
                         const float* v0, const float* v1, const float* v2,
                         const unsigned char areaID, rcHeightfield& heightfield, const int flagMergeThreshold)
{
	rcAssert(context != NULL);

	rcScopedTimer timer(context, RC_TIMER_RASTERIZE_TRIANGLES);

	// Rasterize the single triangle.
	const float inverseCellSize = 1.0f / heightfield.cs;
	const float inverseCellHeight = 1.0f / heightfield.ch;
	if (!rasterizeTri(v0, v1, v2, areaID, heightfield, heightfield.bmin, heightfield.bmax,
					  heightfield.cs, inverseCellSize, inverseCellHeight, flagMergeThreshold,
					  0, heightfield.width - 1, 0, heightfield.height - 1))
	{
		context->log(RC_LOG_ERROR, "rcRasterizeTriangle: Out of memory.");
		return false;
	}

	return true;
}

bool rcRasterizeTriangles(rcContext* context,
                          const float* verts, const int /*nv*/,
                          const int* tris, const unsigned char* triAreaIDs, const int numTris,
                          rcHeightfield& heightfield, const int flagMergeThreshold)
{
	rcAssert(context != NULL);

	rcScopedTimer timer(context, RC_TIMER_RASTERIZE_TRIANGLES);
	
	// Rasterize the triangles.
	RasterTriRef* refs = NULL;
	if (numTris > 0)
	{
		refs = (RasterTriRef*)rcAlloc(sizeof(RasterTriRef) * numTris, RC_ALLOC_TEMP);
		if (!refs)
		{
			context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory 'refs' (%d).", numTris);
			return false;
		}
	}
	for (int triIndex = 0; triIndex < numTris; ++triIndex)
	{
		refs[triIndex].v[0] = &verts[tris[triIndex * 3 + 0] * 3];
		refs[triIndex].v[1] = &verts[tris[triIndex * 3 + 1] * 3];
		refs[triIndex].v[2] = &verts[tris[triIndex * 3 + 2] * 3];
		refs[triIndex].area = triAreaIDs[triIndex];
	}
	const bool ok = rasterizeTriangleRefs(refs, numTris, heightfield, flagMergeThreshold);
	rcFree(refs);
	if (!ok)
	{
		context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
		return false;
	}

	return true;
}

bool rcRasterizeTriangles(rcContext* context,
                          const float* verts, const int /*nv*/,
                          const unsigned short* tris, const unsigned char* triAreaIDs, const int numTris,
                          rcHeightfield& heightfield, const int flagMergeThreshold)
{
	rcAssert(context != NULL);

	rcScopedTimer timer(context, RC_TIMER_RASTERIZE_TRIANGLES);

	// Rasterize the triangles.
	RasterTriRef* refs = NULL;
	if (numTris > 0)
	{
		refs = (RasterTriRef*)rcAlloc(sizeof(RasterTriRef) * numTris, RC_ALLOC_TEMP);
		if (!refs)
		{
			context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory 'refs' (%d).", numTris);
			return false;
		}
	}
	for (int triIndex = 0; triIndex < numTris; ++triIndex)
	{
		refs[triIndex].v[0] = &verts[tris[triIndex * 3 + 0] * 3];
		refs[triIndex].v[1] = &verts[tris[triIndex * 3 + 1] * 3];
		refs[triIndex].v[2] = &verts[tris[triIndex * 3 + 2] * 3];
		refs[triIndex].area = triAreaIDs[triIndex];
	}
	const bool ok = rasterizeTriangleRefs(refs, numTris, heightfield, flagMergeThreshold);
	rcFree(refs);
	if (!ok)
	{
		context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
		return false;
	}

	return true;
}

bool rcRasterizeTriangles(rcContext* context,
                          const float* verts, const unsigned char* triAreaIDs, const int numTris,
                          rcHeightfield& heightfield, const int flagMergeThreshold)
{
	rcAssert(context != NULL);

	rcScopedTimer timer(context, RC_TIMER_RASTERIZE_TRIANGLES);
	
	// Rasterize the triangles.
	RasterTriRef* refs = NULL;
	if (numTris > 0)
	{
		refs = (RasterTriRef*)rcAlloc(sizeof(RasterTriRef) * numTris, RC_ALLOC_TEMP);
		if (!refs)
		{
			context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory 'refs' (%d).", numTris);
			return false;
		}
	}
	for (int triIndex = 0; triIndex < numTris; ++triIndex)
	{
		refs[triIndex].v[0] = &verts[(triIndex * 3 + 0) * 3];
		refs[triIndex].v[1] = &verts[(triIndex * 3 + 1) * 3];
		refs[triIndex].v[2] = &verts[(triIndex * 3 + 2) * 3];
		refs[triIndex].area = triAreaIDs[triIndex];
	}
	const bool ok = rasterizeTriangleRefs(refs, numTris, heightfield, flagMergeThreshold);
	rcFree(refs);
	if (!ok)
	{
		context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
		return false;
	}

	return true;
}
