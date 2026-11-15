#include <math.h>
#include <stdio.h>
#include <string.h>

#include <unordered_map>
#include <vector>

#include "catch2/catch_amalgamated.hpp"

#include "DetourLocalRebuild.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "Recast.h"
#include "RecastAlloc.h"

namespace
{
const int kTileSize = 8;
const int kBorderSize = 3;
const int kMapTiles = 3;
const float kCellSize = 1.0f;
const float kCellHeight = 0.5f;
const float kTileWorldSize = (float)(kTileSize * kCellSize);

struct Geometry
{
	std::vector<float> verts;
	std::vector<int> tris;

	int AddVertex(float x, float y, float z)
	{
		const int index = (int)(verts.size() / 3);
		verts.push_back(x);
		verts.push_back(y);
		verts.push_back(z);
		return index;
	}

	void AddTriangle(int a, int b, int c)
	{
		tris.push_back(a);
		tris.push_back(b);
		tris.push_back(c);
	}

	void AddQuad(int a, int b, int c, int d)
	{
		AddTriangle(a, b, c);
		AddTriangle(a, c, d);
	}
};

Geometry MakeFloor()
{
	Geometry geometry;
	const int a = geometry.AddVertex(0.0f, 0.0f, 0.0f);
	const int b = geometry.AddVertex(0.0f, 0.0f, 24.0f);
	const int c = geometry.AddVertex(24.0f, 0.0f, 24.0f);
	const int d = geometry.AddVertex(24.0f, 0.0f, 0.0f);
	geometry.AddQuad(a, b, c, d);
	return geometry;
}

void AddBox(Geometry& geometry, float x0, float z0, float x1, float z1, float top)
{
	const int b0 = geometry.AddVertex(x0, 0.0f, z0);
	const int b1 = geometry.AddVertex(x1, 0.0f, z0);
	const int b2 = geometry.AddVertex(x1, 0.0f, z1);
	const int b3 = geometry.AddVertex(x0, 0.0f, z1);
	const int t0 = geometry.AddVertex(x0, top, z0);
	const int t1 = geometry.AddVertex(x1, top, z0);
	const int t2 = geometry.AddVertex(x1, top, z1);
	const int t3 = geometry.AddVertex(x0, top, z1);

	geometry.AddQuad(t0, t3, t2, t1);
	geometry.AddQuad(b0, b1, t1, t0);
	geometry.AddQuad(b1, b2, t2, t1);
	geometry.AddQuad(b2, b3, t3, t2);
	geometry.AddQuad(b3, b0, t0, t3);
}

struct TempRecastData
{
	rcHeightfield* heightfield;
	rcCompactHeightfield* compact;
	rcContourSet* contours;
	rcPolyMesh* polyMesh;

	TempRecastData() :
		heightfield(0),
		compact(0),
		contours(0),
		polyMesh(0)
	{
	}

	~TempRecastData()
	{
		rcFreePolyMesh(polyMesh);
		rcFreeContourSet(contours);
		rcFreeCompactHeightfield(compact);
		rcFreeHeightField(heightfield);
	}
};

bool BuildTileData(const Geometry& geometry, int tx, int ty,
				   unsigned char** outData, int* outDataSize)
{
	*outData = 0;
	*outDataSize = 0;

	TempRecastData data;
	rcContext ctx(false);

	rcConfig config;
	memset(&config, 0, sizeof(config));
	config.cs = kCellSize;
	config.ch = kCellHeight;
	config.tileSize = kTileSize;
	config.borderSize = kBorderSize;
	config.width = kTileSize + kBorderSize * 2;
	config.height = kTileSize + kBorderSize * 2;
	config.walkableSlopeAngle = 45.0f;
	config.walkableHeight = (int)ceilf(2.0f / config.ch);
	config.walkableClimb = (int)floorf(0.5f / config.ch);
	config.walkableRadius = (int)ceilf(0.5f / config.cs);
	config.maxEdgeLen = (int)(12.0f / config.cs);
	config.maxSimplificationError = 1.3f;
	config.minRegionArea = 8;
	config.mergeRegionArea = 20;
	config.maxVertsPerPoly = 6;
	config.detailSampleDist = 0.0f;
	config.detailSampleMaxError = 0.5f;

	config.bmin[0] = (float)tx * kTileWorldSize - (float)kBorderSize * config.cs;
	config.bmin[1] = -1.0f;
	config.bmin[2] = (float)ty * kTileWorldSize - (float)kBorderSize * config.cs;
	config.bmax[0] = (float)(tx + 1) * kTileWorldSize + (float)kBorderSize * config.cs;
	config.bmax[1] = 3.0f;
	config.bmax[2] = (float)(ty + 1) * kTileWorldSize + (float)kBorderSize * config.cs;

	data.heightfield = rcAllocHeightfield();
	if (!data.heightfield ||
		!rcCreateHeightfield(&ctx, *data.heightfield, config.width, config.height,
							 config.bmin, config.bmax, config.cs, config.ch))
	{
		return false;
	}

	const int numTris = (int)(geometry.tris.size() / 3);
	std::vector<unsigned char> triAreas((size_t)numTris);
	rcMarkWalkableTriangles(&ctx, config.walkableSlopeAngle,
							geometry.verts.data(), (int)(geometry.verts.size() / 3),
							geometry.tris.data(), numTris, triAreas.data());
	if (!rcRasterizeTriangles(&ctx, geometry.verts.data(),
							  (int)(geometry.verts.size() / 3),
							  geometry.tris.data(), triAreas.data(), numTris,
							  *data.heightfield, config.walkableClimb))
	{
		return false;
	}

	rcFilterLowHangingWalkableObstacles(&ctx, config.walkableClimb, *data.heightfield);
	rcFilterLedgeSpans(&ctx, config.walkableHeight, config.walkableClimb, *data.heightfield);
	rcFilterWalkableLowHeightSpans(&ctx, config.walkableHeight, *data.heightfield);

	data.compact = rcAllocCompactHeightfield();
	if (!data.compact ||
		!rcBuildCompactHeightfield(&ctx, config.walkableHeight, config.walkableClimb,
								   *data.heightfield, *data.compact))
	{
		return false;
	}

	if (!rcErodeWalkableArea(&ctx, config.walkableRadius, *data.compact))
		return false;
	if (!rcBuildRegionsMonotone(&ctx, *data.compact, config.borderSize,
								config.minRegionArea, config.mergeRegionArea))
	{
		return false;
	}

	data.contours = rcAllocContourSet();
	if (!data.contours ||
		!rcBuildContours(&ctx, *data.compact, config.maxSimplificationError,
						 config.maxEdgeLen, *data.contours,
						 RC_CONTOUR_TESS_WALL_EDGES))
	{
		return false;
	}

	data.polyMesh = rcAllocPolyMesh();
	if (!data.polyMesh ||
		!rcBuildPolyMesh(&ctx, *data.contours, config.maxVertsPerPoly, *data.polyMesh))
	{
		return false;
	}

	for (int i = 0; i < data.polyMesh->npolys; ++i)
	{
		data.polyMesh->areas[i] = RC_WALKABLE_AREA;
		data.polyMesh->flags[i] = 1;
	}

	dtNavMeshCreateParams params;
	memset(&params, 0, sizeof(params));
	params.verts = data.polyMesh->verts;
	params.vertCount = data.polyMesh->nverts;
	params.polys = data.polyMesh->polys;
	params.polyAreas = data.polyMesh->areas;
	params.polyFlags = data.polyMesh->flags;
	params.polyCount = data.polyMesh->npolys;
	params.nvp = data.polyMesh->nvp;
	params.walkableHeight = (float)config.walkableHeight * config.ch;
	params.walkableRadius = (float)config.walkableRadius * config.cs;
	params.walkableClimb = (float)config.walkableClimb * config.ch;
	params.tileX = tx;
	params.tileY = ty;
	params.tileLayer = 0;
	params.bmin[0] = (float)tx * kTileWorldSize;
	params.bmin[1] = config.bmin[1];
	params.bmin[2] = (float)ty * kTileWorldSize;
	params.bmax[0] = (float)(tx + 1) * kTileWorldSize;
	params.bmax[1] = config.bmax[1];
	params.bmax[2] = (float)(ty + 1) * kTileWorldSize;
	params.cs = config.cs;
	params.ch = config.ch;
	params.buildBvTree = true;

	return dtCreateNavMeshData(&params, outData, outDataSize);
}

struct BuildCallbackData
{
	const Geometry* geometry;
};

struct FailingCallbackData
{
	const Geometry* geometry;
	int failX;
	int failY;
};

dtStatus BuildTileCallback(void* user, int tx, int ty,
						   unsigned char** outData, int* outDataSize)
{
	BuildCallbackData* data = (BuildCallbackData*)user;
	if (!BuildTileData(*data->geometry, tx, ty, outData, outDataSize))
		return DT_FAILURE;
	return DT_SUCCESS;
}

dtStatus FailingTileCallback(void* user, int tx, int ty,
							 unsigned char** outData, int* outDataSize)
{
	FailingCallbackData* data = (FailingCallbackData*)user;
	if (tx == data->failX && ty == data->failY)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	BuildCallbackData buildData = { data->geometry };
	return BuildTileCallback(&buildData, tx, ty, outData, outDataSize);
}

dtStatus EmptyTileCallback(void*, int, int,
						   unsigned char** outData, int* outDataSize)
{
	*outData = 0;
	*outDataSize = 0;
	return DT_SUCCESS;
}

struct NavMeshParams
{
	dtNavMeshParams value;

	NavMeshParams()
	{
		memset(&value, 0, sizeof(value));
		value.orig[0] = 0.0f;
		value.orig[1] = 0.0f;
		value.orig[2] = 0.0f;
		value.tileWidth = kTileWorldSize;
		value.tileHeight = kTileWorldSize;
		value.maxTiles = 64;
		value.maxPolys = 512;
	}
};

dtNavMesh* BuildFullNavigationMesh(const Geometry& geometry)
{
	NavMeshParams params;
	dtNavMesh* navMesh = dtAllocNavMesh();
	if (!navMesh)
		return 0;
	if (dtStatusFailed(navMesh->init(&params.value)))
	{
		dtFreeNavMesh(navMesh);
		return 0;
	}

	for (int y = 0; y < kMapTiles; ++y)
	{
		for (int x = 0; x < kMapTiles; ++x)
		{
			unsigned char* tileData = 0;
			int tileDataSize = 0;
			if (!BuildTileData(geometry, x, y, &tileData, &tileDataSize))
			{
				dtFreeNavMesh(navMesh);
				return 0;
			}
			if (dtStatusFailed(navMesh->addTile(tileData, tileDataSize,
												DT_TILE_FREE_DATA, 0, 0)))
			{
				dtFree(tileData);
				dtFreeNavMesh(navMesh);
				return 0;
			}
		}
	}

	return navMesh;
}

int CountLinksToTile(const dtNavMesh& navMesh, const dtMeshTile* from,
					 const dtMeshTile* to)
{
	if (!from || !to)
		return 0;

	const unsigned int targetTile = navMesh.decodePolyIdTile(navMesh.getTileRef(to));
	int count = 0;
	for (int i = 0; i < from->header->polyCount; ++i)
	{
		const dtPoly& poly = from->polys[i];
		for (unsigned int linkIndex = poly.firstLink;
			 linkIndex != DT_NULL_LINK;
			 linkIndex = from->links[linkIndex].next)
		{
			if (navMesh.decodePolyIdTile(from->links[linkIndex].ref) == targetTile)
				count++;
		}
	}
	return count;
}

struct StaticTileComparison
{
	int tilesCompared;
	int verticesCompared;
	int polygonsCompared;
	int mismatches;
};

struct CellComparison
{
	int cellsCompared;
	int mismatches;
};

bool CompareHeader(const dtMeshHeader& a, const dtMeshHeader& b)
{
	return a.magic == b.magic &&
		a.version == b.version &&
		a.x == b.x &&
		a.y == b.y &&
		a.layer == b.layer &&
		a.userId == b.userId &&
		a.polyCount == b.polyCount &&
		a.vertCount == b.vertCount &&
		a.maxLinkCount == b.maxLinkCount &&
		a.detailMeshCount == b.detailMeshCount &&
		a.detailVertCount == b.detailVertCount &&
		a.detailTriCount == b.detailTriCount &&
		a.bvNodeCount == b.bvNodeCount &&
		a.offMeshConCount == b.offMeshConCount &&
		a.offMeshBase == b.offMeshBase &&
		a.walkableHeight == b.walkableHeight &&
		a.walkableRadius == b.walkableRadius &&
		a.walkableClimb == b.walkableClimb &&
		memcmp(a.bmin, b.bmin, sizeof(a.bmin)) == 0 &&
		memcmp(a.bmax, b.bmax, sizeof(a.bmax)) == 0 &&
		a.bvQuantFactor == b.bvQuantFactor;
}

bool CompareStaticTiles(const dtMeshTile* a, const dtMeshTile* b,
						StaticTileComparison& comparison)
{
	if (!a || !b)
		return a == b;

	comparison.tilesCompared++;
	if (!CompareHeader(*a->header, *b->header))
	{
		comparison.mismatches++;
		return false;
	}

	comparison.verticesCompared += a->header->vertCount;
	comparison.polygonsCompared += a->header->polyCount;

	if (memcmp(a->verts, b->verts,
			   sizeof(float) * 3 * a->header->vertCount) != 0)
	{
		comparison.mismatches++;
		return false;
	}

	for (int i = 0; i < a->header->polyCount; ++i)
	{
		const dtPoly& pa = a->polys[i];
		const dtPoly& pb = b->polys[i];
		if (pa.vertCount != pb.vertCount ||
			pa.flags != pb.flags ||
			pa.getArea() != pb.getArea() ||
			pa.getType() != pb.getType() ||
			memcmp(pa.verts, pb.verts, sizeof(pa.verts)) != 0 ||
			memcmp(pa.neis, pb.neis, sizeof(pa.neis)) != 0)
		{
			comparison.mismatches++;
			return false;
		}
	}

	if (a->header->detailVertCount > 0 &&
		memcmp(a->detailVerts, b->detailVerts,
			   sizeof(float) * 3 * a->header->detailVertCount) != 0)
	{
		comparison.mismatches++;
		return false;
	}
	if (a->header->detailTriCount > 0 &&
		memcmp(a->detailTris, b->detailTris,
			   sizeof(unsigned char) * 4 * a->header->detailTriCount) != 0)
	{
		comparison.mismatches++;
		return false;
	}
	for (int i = 0; i < a->header->detailMeshCount; ++i)
	{
		const dtPolyDetail& da = a->detailMeshes[i];
		const dtPolyDetail& db = b->detailMeshes[i];
		if (da.vertBase != db.vertBase ||
			da.vertCount != db.vertCount ||
			da.triBase != db.triBase ||
			da.triCount != db.triCount)
		{
			comparison.mismatches++;
			return false;
		}
	}
	for (int i = 0; i < a->header->bvNodeCount; ++i)
	{
		const dtBVNode& na = a->bvTree[i];
		const dtBVNode& nb = b->bvTree[i];
		if (na.i != nb.i ||
			memcmp(na.bmin, nb.bmin, sizeof(na.bmin)) != 0 ||
			memcmp(na.bmax, nb.bmax, sizeof(na.bmax)) != 0)
		{
			comparison.mismatches++;
			return false;
		}
	}

	return true;
}

void CompareCells(const dtNavMesh& local, const dtNavMesh& full,
				  const dtTileRange& range, CellComparison& comparison)
{
	dtNavMeshQuery* localQuery = dtAllocNavMeshQuery();
	dtNavMeshQuery* fullQuery = dtAllocNavMeshQuery();
	if (!localQuery || !fullQuery ||
		dtStatusFailed(localQuery->init(&local, 2048)) ||
		dtStatusFailed(fullQuery->init(&full, 2048)))
	{
		comparison.mismatches++;
		dtFreeNavMeshQuery(localQuery);
		dtFreeNavMeshQuery(fullQuery);
		return;
	}

	dtQueryFilter filter;
	const float halfExtents[3] = { 0.01f, 1.0f, 0.01f };
	for (int z = range.minY * kTileSize; z < (range.maxY + 1) * kTileSize; ++z)
	{
		for (int x = range.minX * kTileSize; x < (range.maxX + 1) * kTileSize; ++x)
		{
			const float pos[3] = {
				(float)x + 0.5f,
				0.5f,
				(float)z + 0.5f
			};
			dtPolyRef localRef = 0;
			dtPolyRef fullRef = 0;
			localQuery->findNearestPoly(pos, halfExtents, &filter, &localRef, 0);
			fullQuery->findNearestPoly(pos, halfExtents, &filter, &fullRef, 0);

			bool localOver = false;
			bool fullOver = false;
			float localClosest[3] = { 0.0f, 0.0f, 0.0f };
			float fullClosest[3] = { 0.0f, 0.0f, 0.0f };
			if (localRef)
				localQuery->closestPointOnPoly(localRef, pos, localClosest, &localOver);
			if (fullRef)
				fullQuery->closestPointOnPoly(fullRef, pos, fullClosest, &fullOver);

			const bool mismatch = (localRef == 0) != (fullRef == 0) ||
				localOver != fullOver ||
				(localOver && fabsf(localClosest[1] - fullClosest[1]) > 0.001f);
			if (mismatch)
				comparison.mismatches++;
			comparison.cellsCompared++;
		}
	}

	dtFreeNavMeshQuery(localQuery);
	dtFreeNavMeshQuery(fullQuery);
}

struct RcAllocationTracker
{
	std::unordered_map<void*, size_t> sizes;
	size_t current;
	size_t peak;

	RcAllocationTracker() : current(0), peak(0) {}
};

RcAllocationTracker* gRcTracker = 0;

void* TrackedRcAlloc(size_t size, rcAllocHint)
{
	void* ptr = malloc(size);
	if (ptr && gRcTracker)
	{
		gRcTracker->sizes[ptr] = size;
		gRcTracker->current += size;
		if (gRcTracker->current > gRcTracker->peak)
			gRcTracker->peak = gRcTracker->current;
	}
	return ptr;
}

void TrackedRcFree(void* ptr)
{
	if (!ptr)
		return;
	if (gRcTracker)
	{
		std::unordered_map<void*, size_t>::iterator it = gRcTracker->sizes.find(ptr);
		if (it != gRcTracker->sizes.end())
		{
			gRcTracker->current -= it->second;
			gRcTracker->sizes.erase(it);
		}
	}
	free(ptr);
}

struct ScopedRcTracker
{
	RcAllocationTracker tracker;

	ScopedRcTracker()
	{
		gRcTracker = &tracker;
		rcAllocSetCustom(TrackedRcAlloc, TrackedRcFree);
	}

	~ScopedRcTracker()
	{
		rcAllocSetCustom(0, 0);
		gRcTracker = 0;
	}
};
}

TEST_CASE("dtCalcAffectedTileRange", "[detour][local-rebuild]")
{
	NavMeshParams params;

	SECTION("Interior change affects one tile")
	{
		const float bmin[3] = { 10.0f, 0.0f, 10.0f };
		const float bmax[3] = { 12.0f, 0.0f, 12.0f };
		const dtTileRange range = dtCalcAffectedTileRange(params.value, bmin, bmax, 0.0f);
		REQUIRE(range.minX == 1);
		REQUIRE(range.maxX == 1);
		REQUIRE(range.minY == 1);
		REQUIRE(range.maxY == 1);
	}

	SECTION("Build halo expands into neighboring tiles")
	{
		const float bmin[3] = { 10.0f, 0.0f, 10.0f };
		const float bmax[3] = { 12.0f, 0.0f, 12.0f };
		const dtTileRange range = dtCalcAffectedTileRange(params.value, bmin, bmax, 3.0f);
		REQUIRE(range.minX == 0);
		REQUIRE(range.maxX == 1);
		REQUIRE(range.minY == 0);
		REQUIRE(range.maxY == 1);
	}

	SECTION("Halo touching a seam includes the positive tile")
	{
		const float point[3] = { 13.0f, 0.0f, 13.0f };
		const dtTileRange range = dtCalcAffectedTileRange(params.value, point, point, 3.0f);
		REQUIRE(range.minX == 1);
		REQUIRE(range.maxX == 2);
		REQUIRE(range.minY == 1);
		REQUIRE(range.maxY == 2);
	}

	SECTION("Change exactly on a seam includes the positive tile")
	{
		const float point[3] = { 16.0f, 0.0f, 16.0f };
		const dtTileRange range = dtCalcAffectedTileRange(params.value, point, point, 0.0f);
		REQUIRE(range.minX == 2);
		REQUIRE(range.maxX == 2);
		REQUIRE(range.minY == 2);
		REQUIRE(range.maxY == 2);
	}

	SECTION("Negative world coordinates map to negative tiles")
	{
		params.value.orig[0] = -20.0f;
		params.value.orig[2] = -20.0f;
		const float bmin[3] = { -19.0f, 0.0f, -19.0f };
		const float bmax[3] = { -17.0f, 0.0f, -17.0f };
		const dtTileRange range = dtCalcAffectedTileRange(params.value, bmin, bmax, 0.0f);
		REQUIRE(range.minX == 0);
		REQUIRE(range.maxX == 0);
		REQUIRE(range.minY == 0);
		REQUIRE(range.maxY == 0);
	}

	SECTION("Invalid inputs produce an empty range")
	{
		const float bmin[3] = { 10.0f, 0.0f, 10.0f };
		const float bmax[3] = { 12.0f, 0.0f, 12.0f };
		REQUIRE(dtCalcAffectedTileRange(params.value, 0, bmax, 0.0f).isEmpty());
		REQUIRE(dtCalcAffectedTileRange(params.value, bmin, 0, 0.0f).isEmpty());
		REQUIRE(dtCalcAffectedTileRange(params.value, bmin, bmax, -1.0f).isEmpty());

		const float invertedMin[3] = { 12.0f, 0.0f, 10.0f };
		const float invertedMax[3] = { 10.0f, 0.0f, 12.0f };
		REQUIRE(dtCalcAffectedTileRange(params.value, invertedMin, invertedMax, 0.0f).isEmpty());

		params.value.tileWidth = 0.0f;
		REQUIRE(dtCalcAffectedTileRange(params.value, bmin, bmax, 0.0f).isEmpty());
	}

	SECTION("Empty rebuild range is a successful no-op")
	{
		Geometry geometry = MakeFloor();
		BuildCallbackData callback = { &geometry };
		NavMeshParams params;
		dtNavMesh* navMesh = dtAllocNavMesh();
		REQUIRE(navMesh != 0);
		REQUIRE(dtStatusSucceed(navMesh->init(&params.value)));

		dtTileRebuildStats stats;
		const dtTileRange empty;
		REQUIRE(dtStatusSucceed(dtRebuildAffectedTiles(
			navMesh, empty, BuildTileCallback, &callback, &stats)));
		REQUIRE(stats.tilesRequested == 0);
		REQUIRE(stats.tilesBuilt == 0);

		dtFreeNavMesh(navMesh);
	}
}

TEST_CASE("Local tile rebuild matches a full rebuild", "[detour][local-rebuild]")
{
	Geometry before = MakeFloor();
	Geometry after = MakeFloor();
	AddBox(after, 10.0f, 10.0f, 12.0f, 12.0f, 2.0f);

	dtNavMesh* local = BuildFullNavigationMesh(before);
	dtNavMesh* full = BuildFullNavigationMesh(after);
	REQUIRE(local != 0);
	REQUIRE(full != 0);

	BuildCallbackData callback = { &after };
	const float changedMin[3] = { 10.0f, 0.0f, 10.0f };
	const float changedMax[3] = { 12.0f, 2.0f, 12.0f };
	const float haloWorld = (float)kBorderSize * kCellSize;
	NavMeshParams navParams;
	const dtTileRange range = dtCalcAffectedTileRange(
		navParams.value, changedMin, changedMax, haloWorld);
	REQUIRE_FALSE(range.isEmpty());

	const dtTileRef oldCenterRef = local->getTileRefAt(1, 1, 0);
	dtTileRebuildStats stats;
	REQUIRE(dtStatusSucceed(dtRebuildAffectedTiles(
		local, range, BuildTileCallback, &callback, &stats)));
	REQUIRE(stats.tilesRequested == 4);
	REQUIRE(stats.tilesBuilt == 4);
	REQUIRE(stats.tilesRemoved == 4);
	REQUIRE(stats.tilesFailed == 0);
	REQUIRE(stats.bytesBuilt > 0);
	REQUIRE(local->getTileRefAt(1, 1, 0) == oldCenterRef);

	StaticTileComparison comparison = { 0, 0, 0, 0 };
	for (int y = 0; y < kMapTiles; ++y)
	{
		for (int x = 0; x < kMapTiles; ++x)
		{
			REQUIRE(CompareStaticTiles(local->getTileAt(x, y, 0),
									   full->getTileAt(x, y, 0),
									   comparison));
		}
	}
	REQUIRE(comparison.tilesCompared == kMapTiles * kMapTiles);
	REQUIRE(comparison.polygonsCompared > 0);
	REQUIRE(comparison.verticesCompared > 0);
	REQUIRE(comparison.mismatches == 0);

	CellComparison cells = { 0, 0 };
	CompareCells(*local, *full, range, cells);
	REQUIRE(cells.cellsCompared == 256);
	REQUIRE(cells.mismatches == 0);

	int bidirectionalSeams = 0;
	for (int y = 0; y < kMapTiles; ++y)
	{
		for (int x = 0; x < kMapTiles; ++x)
		{
			const dtMeshTile* tile = local->getTileAt(x, y, 0);
			REQUIRE(tile != 0);
			if (x + 1 < kMapTiles)
			{
				const dtMeshTile* east = local->getTileAt(x + 1, y, 0);
				REQUIRE(CountLinksToTile(*local, tile, east) > 0);
				REQUIRE(CountLinksToTile(*local, east, tile) > 0);
				bidirectionalSeams++;
			}
			if (y + 1 < kMapTiles)
			{
				const dtMeshTile* north = local->getTileAt(x, y + 1, 0);
				REQUIRE(CountLinksToTile(*local, tile, north) > 0);
				REQUIRE(CountLinksToTile(*local, north, tile) > 0);
				bidirectionalSeams++;
			}
		}
	}
	REQUIRE(bidirectionalSeams == 12);

	dtNavMeshQuery* query = dtAllocNavMeshQuery();
	REQUIRE(query != 0);
	REQUIRE(dtStatusSucceed(query->init(local, 2048)));
	dtQueryFilter filter;
	const float startPos[3] = { 2.0f, 0.1f, 4.0f };
	const float endPos[3] = { 22.0f, 0.1f, 4.0f };
	const float halfExtents[3] = { 2.0f, 4.0f, 2.0f };
	dtPolyRef startRef = 0;
	dtPolyRef endRef = 0;
	REQUIRE(dtStatusSucceed(query->findNearestPoly(startPos, halfExtents, &filter, &startRef, 0)));
	REQUIRE(dtStatusSucceed(query->findNearestPoly(endPos, halfExtents, &filter, &endRef, 0)));
	REQUIRE(startRef != 0);
	REQUIRE(endRef != 0);

	dtPolyRef path[256];
	int pathCount = 0;
	REQUIRE(dtStatusSucceed(query->findPath(startRef, endRef, startPos, endPos,
											&filter, path, &pathCount, 256)));
	REQUIRE(pathCount > 1);

	printf("[local-rebuild] affected=%d built=%d removed=%d bytes=%zu "
		   "tiles=%d verts=%d polys=%d tileMismatches=%d "
		   "cells=%d cellMismatches=%d seams=%d pathPolys=%d\n",
		   stats.tilesRequested, stats.tilesBuilt, stats.tilesRemoved,
		   stats.bytesBuilt, comparison.tilesCompared,
		   comparison.verticesCompared, comparison.polygonsCompared,
		   comparison.mismatches, cells.cellsCompared, cells.mismatches,
		   bidirectionalSeams, pathCount);

	dtFreeNavMeshQuery(query);
	dtFreeNavMesh(local);
	dtFreeNavMesh(full);
}

TEST_CASE("Local rebuild failure does not modify the navmesh", "[detour][local-rebuild]")
{
	Geometry geometry = MakeFloor();
	dtNavMesh* navMesh = BuildFullNavigationMesh(geometry);
	REQUIRE(navMesh != 0);

	dtTileRef refs[kMapTiles][kMapTiles];
	for (int y = 0; y < kMapTiles; ++y)
	{
		for (int x = 0; x < kMapTiles; ++x)
			refs[y][x] = navMesh->getTileRefAt(x, y, 0);
	}

	FailingCallbackData callback = { &geometry, 1, 1 };
	dtTileRange range;
	range.minX = 0;
	range.maxX = 1;
	range.minY = 0;
	range.maxY = 1;

	dtTileRebuildStats stats;
	const dtStatus status = dtRebuildAffectedTiles(
		navMesh, range, FailingTileCallback, &callback, &stats);
	REQUIRE(dtStatusFailed(status));
	REQUIRE(dtStatusDetail(status, DT_OUT_OF_MEMORY));
	REQUIRE(stats.tilesRequested == 4);
	REQUIRE(stats.tilesBuilt == 3);
	REQUIRE(stats.tilesRemoved == 0);
	REQUIRE(stats.tilesFailed == 0);

	for (int y = 0; y < kMapTiles; ++y)
	{
		for (int x = 0; x < kMapTiles; ++x)
			REQUIRE(navMesh->getTileRefAt(x, y, 0) == refs[y][x]);
	}

	dtFreeNavMesh(navMesh);
}

TEST_CASE("Empty tile output removes the old tile", "[detour][local-rebuild]")
{
	Geometry geometry = MakeFloor();
	dtNavMesh* navMesh = BuildFullNavigationMesh(geometry);
	REQUIRE(navMesh != 0);
	REQUIRE(navMesh->getTileAt(1, 1, 0) != 0);

	dtTileRange range;
	range.minX = 1;
	range.maxX = 1;
	range.minY = 1;
	range.maxY = 1;

	dtTileRebuildStats stats;
	REQUIRE(dtStatusSucceed(dtRebuildAffectedTiles(
		navMesh, range, EmptyTileCallback, 0, &stats)));
	REQUIRE(stats.tilesRequested == 1);
	REQUIRE(stats.tilesBuilt == 0);
	REQUIRE(stats.tilesRemoved == 1);
	REQUIRE(stats.tilesEmpty == 1);
	REQUIRE(navMesh->getTileAt(1, 1, 0) == 0);
	REQUIRE(navMesh->getTileAt(0, 1, 0) != 0);
	REQUIRE(navMesh->getTileAt(2, 1, 0) != 0);

	dtFreeNavMesh(navMesh);
}

TEST_CASE("Local rebuild memory scales with the affected range", "[detour][local-rebuild]")
{
	Geometry geometry = MakeFloor();
	BuildCallbackData callback = { &geometry };

	NavMeshParams navParams;
	dtNavMesh* navMesh = dtAllocNavMesh();
	REQUIRE(navMesh != 0);
	REQUIRE(dtStatusSucceed(navMesh->init(&navParams.value)));

	dtTileRange oneTile;
	oneTile.minX = 1;
	oneTile.maxX = 1;
	oneTile.minY = 1;
	oneTile.maxY = 1;

	dtTileRange fourTiles;
	fourTiles.minX = 0;
	fourTiles.maxX = 1;
	fourTiles.minY = 0;
	fourTiles.maxY = 1;

	dtTileRebuildStats statsOne;
	dtTileRebuildStats statsFour;
	size_t peakOne = 0;
	size_t peakFour = 0;
	{
		ScopedRcTracker tracker;
		REQUIRE(dtStatusSucceed(dtRebuildAffectedTiles(
			navMesh, oneTile, BuildTileCallback, &callback, &statsOne)));
		peakOne = tracker.tracker.peak;
	}
	{
		ScopedRcTracker tracker;
		REQUIRE(dtStatusSucceed(dtRebuildAffectedTiles(
			navMesh, fourTiles, BuildTileCallback, &callback, &statsFour)));
		peakFour = tracker.tracker.peak;
	}

	REQUIRE(statsOne.tilesRequested == 1);
	REQUIRE(statsFour.tilesRequested == 4);
	REQUIRE(statsFour.bytesBuilt >= statsOne.bytesBuilt * 2);
	REQUIRE(statsFour.bytesBuilt <= statsOne.bytesBuilt * 6);
	REQUIRE(peakOne > 0);
	REQUIRE(peakFour == peakOne);

	printf("[local-rebuild] range=1 tilesBytes=%zu peakRc=%zu; "
		   "range=4 tilesBytes=%zu peakRc=%zu\n",
		   statsOne.bytesBuilt, peakOne, statsFour.bytesBuilt, peakFour);

	dtFreeNavMesh(navMesh);
}
