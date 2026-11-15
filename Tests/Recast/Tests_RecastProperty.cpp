#include "catch2/catch_amalgamated.hpp"

#include "DetourAlloc.h"
#include "DetourNavMeshBuilder.h"
#include "Recast.h"
#include "RecastAlloc.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#if defined(__unix__) || defined(__APPLE__)
#include <csignal>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace
{
constexpr int kCaseCount = 200;
constexpr int kMaxReportedCounterexamples = 5;
constexpr int kHeightfieldCells = 16;
constexpr float kCellSize = 1.0f;
constexpr float kCellHeight = 0.25f;
constexpr int kMaxHeightVoxels = RC_SPAN_MAX_HEIGHT;
constexpr int kBuildTimeoutSeconds = 10;

struct SplitMix64
{
	explicit SplitMix64(uint64_t seed) : state(seed) {}

	uint64_t next()
	{
		uint64_t z = (state += 0x9e3779b97f4a7c15ULL);
		z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
		z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
		return z ^ (z >> 31);
	}

	int range(int minValue, int maxValue)
	{
		const uint64_t span = static_cast<uint64_t>(maxValue - minValue + 1);
		return minValue + static_cast<int>(next() % span);
	}

	uint64_t state;
};

struct HeightPoint
{
	int x;
	int z;
	int delta;
};

struct Hole
{
	int x;
	int z;
	int width;
	int height;
};

struct BoxFeature
{
	float minX;
	float minZ;
	float maxX;
	float maxZ;
	int baseY;
	int topY;
};

struct TerrainSpec
{
	int seed = 0;
	int width = kHeightfieldCells;
	int height = kHeightfieldCells;
	int baseHeight = 2;
	std::vector<HeightPoint> heightPoints;
	std::vector<Hole> holes;
	std::vector<BoxFeature> boxes;
};

std::vector<unsigned char> buildSolidMask(const TerrainSpec& spec)
{
	std::vector<unsigned char> solid(static_cast<size_t>(spec.width) * spec.height, 1);
	for (const Hole& hole : spec.holes)
	{
		for (int z = hole.z; z < hole.z + hole.height; ++z)
		{
			for (int x = hole.x; x < hole.x + hole.width; ++x)
			{
				if (x >= 0 && x < spec.width && z >= 0 && z < spec.height)
				{
					solid[static_cast<size_t>(z) * spec.width + x] = 0;
				}
			}
		}
	}
	return solid;
}

std::vector<int> buildVertexHeights(const TerrainSpec& spec)
{
	std::vector<int> heights(static_cast<size_t>(spec.width + 1) * (spec.height + 1), spec.baseHeight);
	for (const HeightPoint& point : spec.heightPoints)
	{
		if (point.x < 0 || point.x > spec.width || point.z < 0 || point.z > spec.height)
		{
			continue;
		}
		int& height = heights[static_cast<size_t>(point.z) * (spec.width + 1) + point.x];
		height = rcClamp(height + point.delta, 0, kMaxHeightVoxels - 1);
	}
	return heights;
}

void addBox(Hole& hole, int width, int height)
{
	hole.width = width;
	hole.height = height;
}

void addBox(BoxFeature& box, float minX, float minZ, float maxX, float maxZ, int baseY, int topY)
{
	box.minX = minX;
	box.minZ = minZ;
	box.maxX = maxX;
	box.maxZ = maxZ;
	box.baseY = baseY;
	box.topY = topY;
}

TerrainSpec generateTerrain(int seed)
{
	SplitMix64 rng(static_cast<uint64_t>(seed) * 0x9e3779b97f4a7c15ULL + 0x517cc1b727220a95ULL);
	TerrainSpec spec;
	spec.seed = seed;
	spec.width = rng.range(13, 20);
	spec.height = rng.range(13, 20);
	spec.baseHeight = rng.range(1, 7);

	// The continuous random component makes each field different, while the
	// four feature blocks below are deliberately ordered and always present.
	for (int z = 0; z <= spec.height; ++z)
	{
		for (int x = 0; x <= spec.width; ++x)
		{
			const int delta = rng.range(0, 5);
			if (delta != 0)
			{
				spec.heightPoints.push_back({x, z, delta});
			}
		}
	}

	const int holeCount = rng.range(1, 3);
	for (int i = 0; i < holeCount; ++i)
	{
		const int holeWidth = rng.range(1, 4);
		const int holeHeight = rng.range(1, 4);
		const int maxX = std::max(1, spec.width - holeWidth - 1);
		const int maxZ = std::max(1, spec.height - holeHeight - 1);
		Hole hole{};
		hole.x = rng.range(1, maxX);
		hole.z = rng.range(1, maxZ);
		addBox(hole, holeWidth, holeHeight);
		spec.holes.push_back(hole);
	}

	const int slopeX = rng.range(1, spec.width - 3);
	const int slopeZ = rng.range(1, spec.height - 3);
	const int slopeWidth = rng.range(1, 2);
	const int slopeHeight = rng.range(2, 4);
	for (int z = slopeZ; z <= slopeZ + slopeHeight; ++z)
	{
		for (int x = slopeX; x <= slopeX + slopeWidth; ++x)
		{
			spec.heightPoints.push_back({x, z, rng.range(14, 22)});
		}
	}

	BoxFeature thinWall{};
	const float wallX = static_cast<float>(rng.range(1, spec.width - 3));
	const float wallZ = static_cast<float>(rng.range(1, spec.height - 6));
	addBox(
		thinWall,
		wallX,
		wallZ,
		wallX + 0.35f,
		wallZ + static_cast<float>(rng.range(3, 5)),
		rng.range(1, 5),
		rng.range(18, 30));
	spec.boxes.push_back(thinWall);

	const int overlapCount = rng.range(2, 4);
	for (int i = 0; i < overlapCount; ++i)
	{
		const float baseX = static_cast<float>(rng.range(2, spec.width - 5));
		const float baseZ = static_cast<float>(rng.range(2, spec.height - 5));
		const float sizeX = static_cast<float>(rng.range(2, 4));
		const float sizeZ = static_cast<float>(rng.range(2, 4));
		BoxFeature box{};
		addBox(
			box,
			baseX + static_cast<float>(i) * 0.5f,
			baseZ + static_cast<float>(i) * 0.35f,
			baseX + sizeX + static_cast<float>(i) * 0.5f,
			baseZ + sizeZ + static_cast<float>(i) * 0.35f,
			rng.range(1, 4),
			rng.range(12, 32));
		spec.boxes.push_back(box);
	}

	return spec;
}

struct MeshData
{
	std::vector<float> verts;
	std::vector<int> tris;
};

int addVertex(MeshData& mesh, float x, float y, float z)
{
	const int index = static_cast<int>(mesh.verts.size() / 3);
	mesh.verts.push_back(x);
	mesh.verts.push_back(y);
	mesh.verts.push_back(z);
	return index;
}

void addQuad(MeshData& mesh, int a, int b, int c, int d)
{
	mesh.tris.push_back(a);
	mesh.tris.push_back(b);
	mesh.tris.push_back(c);
	mesh.tris.push_back(a);
	mesh.tris.push_back(c);
	mesh.tris.push_back(d);
}

MeshData buildMesh(const TerrainSpec& spec)
{
	MeshData mesh;
	const std::vector<int> vertexHeights = buildVertexHeights(spec);

	for (int z = 0; z <= spec.height; ++z)
	{
		for (int x = 0; x <= spec.width; ++x)
		{
			const int height = vertexHeights[static_cast<size_t>(z) * (spec.width + 1) + x];
			addVertex(
				mesh,
				static_cast<float>(x) * kCellSize,
				static_cast<float>(height) * kCellHeight,
				static_cast<float>(z) * kCellSize);
		}
	}

	const std::vector<unsigned char> solid = buildSolidMask(spec);
	for (int z = 0; z < spec.height; ++z)
	{
		for (int x = 0; x < spec.width; ++x)
		{
			if (!solid[static_cast<size_t>(z) * spec.width + x])
			{
				continue;
			}
			const int a = z * (spec.width + 1) + x;
			const int b = a + 1;
			const int c = a + spec.width + 1;
			const int d = c + 1;
			// Recast expects the upward normal to be on the positive-y side.
			addQuad(mesh, a, c, d, b);
		}
	}

	for (const BoxFeature& box : spec.boxes)
	{
		const float y0 = static_cast<float>(box.baseY) * kCellHeight;
		const float y1 = static_cast<float>(box.topY) * kCellHeight;
		const int lb = addVertex(mesh, box.minX * kCellSize, y0, box.minZ * kCellSize);
		const int rb = addVertex(mesh, box.maxX * kCellSize, y0, box.minZ * kCellSize);
		const int rt = addVertex(mesh, box.maxX * kCellSize, y1, box.minZ * kCellSize);
		const int lt = addVertex(mesh, box.minX * kCellSize, y1, box.minZ * kCellSize);
		const int lb2 = addVertex(mesh, box.minX * kCellSize, y0, box.maxZ * kCellSize);
		const int rb2 = addVertex(mesh, box.maxX * kCellSize, y0, box.maxZ * kCellSize);
		const int rt2 = addVertex(mesh, box.maxX * kCellSize, y1, box.maxZ * kCellSize);
		const int lt2 = addVertex(mesh, box.minX * kCellSize, y1, box.maxZ * kCellSize);

		addQuad(mesh, lt, lt2, rt2, rt);
		addQuad(mesh, lb, lt, rt, rb);
		addQuad(mesh, lb2, rb2, rt2, lt2);
		addQuad(mesh, lb, lb2, lt2, lt);
		addQuad(mesh, rb, rt, rt2, rb2);
	}

	return mesh;
}

struct AllocTracker
{
	std::unordered_map<void*, size_t> live;
	uint64_t allocations = 0;
	uint64_t frees = 0;
	uint64_t badFrees = 0;
	uint64_t liveBytes = 0;
	uint64_t peakBytes = 0;
};

AllocTracker* gRecastTracker = nullptr;
AllocTracker* gDetourTracker = nullptr;

void* trackedRecastAlloc(size_t size, rcAllocHint)
{
	void* ptr = std::malloc(size == 0 ? 1 : size);
	if (ptr == nullptr || gRecastTracker == nullptr)
	{
		return ptr;
	}
	gRecastTracker->live[ptr] = size;
	gRecastTracker->allocations++;
	gRecastTracker->liveBytes += size;
	gRecastTracker->peakBytes = std::max(gRecastTracker->peakBytes, gRecastTracker->liveBytes);
	return ptr;
}

void trackedRecastFree(void* ptr)
{
	if (ptr == nullptr)
	{
		return;
	}
	if (gRecastTracker != nullptr)
	{
		const auto it = gRecastTracker->live.find(ptr);
		if (it == gRecastTracker->live.end())
		{
			gRecastTracker->badFrees++;
		}
		else
		{
			gRecastTracker->liveBytes -= it->second;
			gRecastTracker->live.erase(it);
			gRecastTracker->frees++;
		}
	}
	std::free(ptr);
}

void* trackedDetourAlloc(size_t size, dtAllocHint)
{
	void* ptr = std::malloc(size == 0 ? 1 : size);
	if (ptr == nullptr || gDetourTracker == nullptr)
	{
		return ptr;
	}
	gDetourTracker->live[ptr] = size;
	gDetourTracker->allocations++;
	gDetourTracker->liveBytes += size;
	gDetourTracker->peakBytes = std::max(gDetourTracker->peakBytes, gDetourTracker->liveBytes);
	return ptr;
}

void trackedDetourFree(void* ptr)
{
	if (ptr == nullptr)
	{
		return;
	}
	if (gDetourTracker != nullptr)
	{
		const auto it = gDetourTracker->live.find(ptr);
		if (it == gDetourTracker->live.end())
		{
			gDetourTracker->badFrees++;
		}
		else
		{
			gDetourTracker->liveBytes -= it->second;
			gDetourTracker->live.erase(it);
			gDetourTracker->frees++;
		}
	}
	std::free(ptr);
}

struct AllocatorScope
{
	AllocTracker recast;
	AllocTracker detour;

	AllocatorScope()
	{
		gRecastTracker = &recast;
		gDetourTracker = &detour;
		rcAllocSetCustom(trackedRecastAlloc, trackedRecastFree);
		dtAllocSetCustom(trackedDetourAlloc, trackedDetourFree);
	}

	~AllocatorScope()
	{
		rcAllocSetCustom(nullptr, nullptr);
		dtAllocSetCustom(nullptr, nullptr);
		gRecastTracker = nullptr;
		gDetourTracker = nullptr;
	}

	AllocatorScope(const AllocatorScope&) = delete;
	AllocatorScope& operator=(const AllocatorScope&) = delete;
};

uint64_t mix(uint64_t hash, uint64_t value)
{
	hash ^= value + 0x9e3779b97f4a7c15ULL + (hash << 6) + (hash >> 2);
	return hash;
}

uint64_t hashBytes(uint64_t hash, const void* data, size_t size)
{
	const unsigned char* bytes = static_cast<const unsigned char*>(data);
	for (size_t i = 0; i < size; ++i)
	{
		hash ^= bytes[i];
		hash *= 1099511628211ULL;
	}
	return hash;
}

struct BuildResult
{
	bool ok = false;
	int stage = 0;
	std::string error;
	std::vector<uint64_t> cellHashes;
	uint64_t meshHash = 0;
	uint64_t navHash = 0;
	int navBytes = 0;
	int spanCount = 0;
	int polyCount = 0;
	uint64_t recastLiveBlocks = 0;
	uint64_t recastLiveBytes = 0;
	uint64_t detourLiveBlocks = 0;
	uint64_t detourLiveBytes = 0;
	uint64_t badFrees = 0;
};

uint64_t hashCompactHeightfield(const rcCompactHeightfield& chf, std::vector<uint64_t>& cellHashes)
{
	uint64_t hash = 1469598103934665603ULL;
	hash = mix(hash, static_cast<uint64_t>(chf.width));
	hash = mix(hash, static_cast<uint64_t>(chf.height));
	hash = mix(hash, static_cast<uint64_t>(chf.spanCount));
	hash = mix(hash, static_cast<uint64_t>(chf.walkableHeight));
	hash = mix(hash, static_cast<uint64_t>(chf.walkableClimb));
	hash = hashBytes(hash, chf.bmin, sizeof(chf.bmin));
	hash = hashBytes(hash, chf.bmax, sizeof(chf.bmax));
	hash = hashBytes(hash, &chf.cs, sizeof(chf.cs));
	hash = hashBytes(hash, &chf.ch, sizeof(chf.ch));

	cellHashes.clear();
	cellHashes.reserve(static_cast<size_t>(chf.width) * chf.height);
	for (int i = 0; i < chf.width * chf.height; ++i)
	{
		const rcCompactCell& cell = chf.cells[i];
		uint64_t cellHash = 1469598103934665603ULL;
		cellHash = mix(cellHash, static_cast<uint64_t>(cell.index));
		cellHash = mix(cellHash, static_cast<uint64_t>(cell.count));
		for (int j = 0; j < static_cast<int>(cell.count); ++j)
		{
			const int spanIndex = static_cast<int>(cell.index) + j;
			const rcCompactSpan& span = chf.spans[spanIndex];
			cellHash = mix(cellHash, span.y);
			cellHash = mix(cellHash, span.reg);
			cellHash = mix(cellHash, span.con);
			cellHash = mix(cellHash, span.h);
			cellHash = mix(cellHash, chf.areas[spanIndex]);
			cellHash = mix(cellHash, chf.dist != nullptr ? chf.dist[spanIndex] : 0);
		}
		cellHashes.push_back(cellHash);
		hash = mix(hash, cellHash);
	}
	return hash;
}

uint64_t hashPolyMesh(const rcPolyMesh& mesh)
{
	uint64_t hash = 1469598103934665603ULL;
	hash = mix(hash, static_cast<uint64_t>(mesh.nverts));
	hash = mix(hash, static_cast<uint64_t>(mesh.npolys));
	hash = mix(hash, static_cast<uint64_t>(mesh.nvp));
	hash = hashBytes(hash, mesh.bmin, sizeof(mesh.bmin));
	hash = hashBytes(hash, mesh.bmax, sizeof(mesh.bmax));
	hash = hashBytes(hash, &mesh.cs, sizeof(mesh.cs));
	hash = hashBytes(hash, &mesh.ch, sizeof(mesh.ch));
	if (mesh.nverts > 0)
	{
		hash = hashBytes(hash, mesh.verts, static_cast<size_t>(mesh.nverts) * 3 * sizeof(mesh.verts[0]));
	}
	if (mesh.npolys > 0)
	{
		hash = hashBytes(hash, mesh.polys, static_cast<size_t>(mesh.npolys) * mesh.nvp * 2 * sizeof(mesh.polys[0]));
		hash = hashBytes(hash, mesh.regs, static_cast<size_t>(mesh.npolys) * sizeof(mesh.regs[0]));
		hash = hashBytes(hash, mesh.areas, static_cast<size_t>(mesh.npolys) * sizeof(mesh.areas[0]));
		hash = hashBytes(hash, mesh.flags, static_cast<size_t>(mesh.npolys) * sizeof(mesh.flags[0]));
	}
	return hash;
}

uint64_t hashDetailMesh(const rcPolyMeshDetail& mesh)
{
	uint64_t hash = 1469598103934665603ULL;
	hash = mix(hash, static_cast<uint64_t>(mesh.nmeshes));
	hash = mix(hash, static_cast<uint64_t>(mesh.nverts));
	hash = mix(hash, static_cast<uint64_t>(mesh.ntris));
	if (mesh.nmeshes > 0)
	{
		hash = hashBytes(hash, mesh.meshes, static_cast<size_t>(mesh.nmeshes) * 4 * sizeof(mesh.meshes[0]));
	}
	if (mesh.nverts > 0)
	{
		hash = hashBytes(hash, mesh.verts, static_cast<size_t>(mesh.nverts) * 3 * sizeof(mesh.verts[0]));
	}
	if (mesh.ntris > 0)
	{
		hash = hashBytes(hash, mesh.tris, static_cast<size_t>(mesh.ntris) * 4 * sizeof(mesh.tris[0]));
	}
	return hash;
}

uint64_t hashLayerSet(const rcHeightfieldLayerSet& layerSet)
{
	uint64_t hash = 1469598103934665603ULL;
	hash = mix(hash, static_cast<uint64_t>(layerSet.nlayers));
	for (int i = 0; i < layerSet.nlayers; ++i)
	{
		const rcHeightfieldLayer& layer = layerSet.layers[i];
		hash = hashBytes(hash, layer.bmin, sizeof(layer.bmin));
		hash = hashBytes(hash, layer.bmax, sizeof(layer.bmax));
		hash = hashBytes(hash, &layer.cs, sizeof(layer.cs));
		hash = hashBytes(hash, &layer.ch, sizeof(layer.ch));
		hash = mix(hash, static_cast<uint64_t>(layer.width));
		hash = mix(hash, static_cast<uint64_t>(layer.height));
		hash = mix(hash, static_cast<uint64_t>(layer.minx));
		hash = mix(hash, static_cast<uint64_t>(layer.maxx));
		hash = mix(hash, static_cast<uint64_t>(layer.miny));
		hash = mix(hash, static_cast<uint64_t>(layer.maxy));
		hash = mix(hash, static_cast<uint64_t>(layer.hmin));
		hash = mix(hash, static_cast<uint64_t>(layer.hmax));

		const size_t cells = static_cast<size_t>(layer.width) * layer.height;
		hash = hashBytes(hash, layer.heights, cells);
		hash = hashBytes(hash, layer.areas, cells);
		hash = hashBytes(hash, layer.cons, cells);
	}
	return hash;
}

BuildResult runBuildCore(const TerrainSpec& spec)
{
	BuildResult result;
	rcContext context(false);

	const MeshData mesh = buildMesh(spec);
	if (mesh.tris.empty())
	{
		result.ok = true;
		return result;
	}

	rcConfig config{};
	config.width = spec.width;
	config.height = spec.height;
	config.borderSize = 0;
	config.cs = kCellSize;
	config.ch = kCellHeight;
	config.bmin[0] = 0.0f;
	config.bmin[1] = 0.0f;
	config.bmin[2] = 0.0f;
	config.bmax[0] = static_cast<float>(spec.width) * config.cs;
	config.bmax[1] = static_cast<float>(kMaxHeightVoxels) * config.ch;
	config.bmax[2] = static_cast<float>(spec.height) * config.cs;
	config.walkableSlopeAngle = 45.0f;
	config.walkableHeight = 12;
	config.walkableClimb = 4;
	config.walkableRadius = 1;
	config.maxEdgeLen = 12;
	config.maxSimplificationError = 1.3f;
	config.minRegionArea = 2;
	config.mergeRegionArea = 20;
	config.maxVertsPerPoly = 6;
	config.detailSampleDist = 0.9f * config.cs;
	config.detailSampleMaxError = 0.1f * config.ch;

	std::vector<unsigned char> triAreas(mesh.tris.size() / 3, RC_NULL_AREA);
	rcMarkWalkableTriangles(
		&context,
		config.walkableSlopeAngle,
		mesh.verts.data(),
		static_cast<int>(mesh.verts.size() / 3),
		mesh.tris.data(),
		static_cast<int>(mesh.tris.size() / 3),
		triAreas.data());

	using HeightfieldPtr = std::unique_ptr<rcHeightfield, decltype(&rcFreeHeightField)>;
	using CompactPtr = std::unique_ptr<rcCompactHeightfield, decltype(&rcFreeCompactHeightfield)>;
	using LayerSetPtr = std::unique_ptr<rcHeightfieldLayerSet, decltype(&rcFreeHeightfieldLayerSet)>;
	using ContourPtr = std::unique_ptr<rcContourSet, decltype(&rcFreeContourSet)>;
	using PolyMeshPtr = std::unique_ptr<rcPolyMesh, decltype(&rcFreePolyMesh)>;
	using DetailMeshPtr = std::unique_ptr<rcPolyMeshDetail, decltype(&rcFreePolyMeshDetail)>;

	HeightfieldPtr solid(rcAllocHeightfield(), &rcFreeHeightField);
	if (!solid)
	{
		result.error = "rcAllocHeightfield failed";
		return result;
	}
	if (!rcCreateHeightfield(
			&context,
			*solid,
			config.width,
			config.height,
			config.bmin,
			config.bmax,
			config.cs,
			config.ch))
	{
		result.stage = 1;
		result.error = "rcCreateHeightfield failed";
		return result;
	}
	if (!rcRasterizeTriangles(
			&context,
			mesh.verts.data(),
			static_cast<int>(mesh.verts.size() / 3),
			mesh.tris.data(),
			triAreas.data(),
			static_cast<int>(mesh.tris.size() / 3),
			*solid,
			config.walkableClimb))
	{
		result.stage = 2;
		result.error = "rcRasterizeTriangles failed";
		return result;
	}

	rcFilterLowHangingWalkableObstacles(&context, config.walkableClimb, *solid);
	rcFilterLedgeSpans(&context, config.walkableHeight, config.walkableClimb, *solid);
	rcFilterWalkableLowHeightSpans(&context, config.walkableHeight, *solid);

	CompactPtr compact(rcAllocCompactHeightfield(), &rcFreeCompactHeightfield);
	if (!compact)
	{
		result.error = "rcAllocCompactHeightfield failed";
		return result;
	}
	if (!rcBuildCompactHeightfield(
			&context,
			config.walkableHeight,
			config.walkableClimb,
			*solid,
			*compact))
	{
		result.stage = 3;
		result.error = "rcBuildCompactHeightfield failed";
		return result;
	}
	result.spanCount = compact->spanCount;

	if (config.walkableRadius > 0 &&
		!rcErodeWalkableArea(&context, config.walkableRadius, *compact))
	{
		result.stage = 4;
		result.error = "rcErodeWalkableArea failed";
		return result;
	}

	LayerSetPtr layerSet(rcAllocHeightfieldLayerSet(), &rcFreeHeightfieldLayerSet);
	if (!layerSet ||
		!rcBuildHeightfieldLayers(
			&context,
			*compact,
			config.borderSize,
			config.walkableHeight,
			*layerSet))
	{
		result.stage = 5;
		result.error = "rcBuildHeightfieldLayers failed";
		return result;
	}
	const uint64_t layerHash = hashLayerSet(*layerSet);

	if (!rcBuildDistanceField(&context, *compact) ||
		!rcBuildRegions(
			&context,
			*compact,
			config.borderSize,
			config.minRegionArea,
			config.mergeRegionArea))
	{
		result.stage = 6;
		result.error = "region build failed";
		return result;
	}

	const uint64_t compactHash = hashCompactHeightfield(*compact, result.cellHashes);

	ContourPtr contours(rcAllocContourSet(), &rcFreeContourSet);
	if (!contours ||
		!rcBuildContours(
			&context,
			*compact,
			config.maxSimplificationError,
			config.maxEdgeLen,
			*contours))
	{
		result.stage = 7;
		result.error = "rcBuildContours failed";
		return result;
	}

	PolyMeshPtr polyMesh(rcAllocPolyMesh(), &rcFreePolyMesh);
	if (!polyMesh || !rcBuildPolyMesh(&context, *contours, config.maxVertsPerPoly, *polyMesh))
	{
		result.stage = 8;
		result.error = "rcBuildPolyMesh failed";
		return result;
	}
	result.polyCount = polyMesh->npolys;

	DetailMeshPtr detailMesh(rcAllocPolyMeshDetail(), &rcFreePolyMeshDetail);
	if (!detailMesh ||
		!rcBuildPolyMeshDetail(
			&context,
			*polyMesh,
			*compact,
			config.detailSampleDist,
			config.detailSampleMaxError,
			*detailMesh))
	{
		result.stage = 9;
		result.error = "rcBuildPolyMeshDetail failed";
		return result;
	}

	for (int i = 0; i < polyMesh->npolys; ++i)
	{
		polyMesh->flags[i] = 1;
	}

	result.meshHash = mix(compactHash, layerHash);
	result.meshHash = mix(result.meshHash, hashPolyMesh(*polyMesh));
	result.meshHash = mix(result.meshHash, hashDetailMesh(*detailMesh));

	if (polyMesh->npolys > 0)
	{
		dtNavMeshCreateParams params{};
		params.verts = polyMesh->verts;
		params.vertCount = polyMesh->nverts;
		params.polys = polyMesh->polys;
		params.polyFlags = polyMesh->flags;
		params.polyAreas = polyMesh->areas;
		params.polyCount = polyMesh->npolys;
		params.nvp = polyMesh->nvp;
		params.detailMeshes = detailMesh->meshes;
		params.detailVerts = detailMesh->verts;
		params.detailVertsCount = detailMesh->nverts;
		params.detailTris = detailMesh->tris;
		params.detailTriCount = detailMesh->ntris;
		params.walkableHeight = static_cast<float>(config.walkableHeight) * config.ch;
		params.walkableRadius = static_cast<float>(config.walkableRadius) * config.cs;
		params.walkableClimb = static_cast<float>(config.walkableClimb) * config.ch;
		params.cs = config.cs;
		params.ch = config.ch;
		params.buildBvTree = true;
		rcVcopy(params.bmin, polyMesh->bmin);
		rcVcopy(params.bmax, polyMesh->bmax);

		unsigned char* navData = nullptr;
		int navDataSize = 0;
		if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
		{
			result.stage = 10;
			result.error = "dtCreateNavMeshData failed";
			return result;
		}
		using NavDataPtr = std::unique_ptr<unsigned char, decltype(&dtFree)>;
		NavDataPtr nav(navData, &dtFree);
		result.navHash = hashBytes(1469598103934665603ULL, nav.get(), static_cast<size_t>(navDataSize));
		result.navBytes = navDataSize;
	}

	result.ok = true;
	return result;
}

BuildResult runBuild(const TerrainSpec& spec)
{
	AllocatorScope allocator;
	BuildResult result = runBuildCore(spec);
	result.recastLiveBlocks = allocator.recast.live.size();
	result.recastLiveBytes = allocator.recast.liveBytes;
	result.detourLiveBlocks = allocator.detour.live.size();
	result.detourLiveBytes = allocator.detour.liveBytes;
	result.badFrees = allocator.recast.badFrees + allocator.detour.badFrees;
	return result;
}

enum class FailureKind
{
	Pass = 0,
	BuildFailure,
	NondeterministicCells,
	NondeterministicMesh,
	Leak,
	Crash,
	Timeout,
	HarnessFailure
};

struct EvalReport
{
	int kind = static_cast<int>(FailureKind::Pass);
	int stage = 0;
	int signal = 0;
	int firstCellMismatch = -1;
	int cells = 0;
	int spanCountA = 0;
	int spanCountB = 0;
	int polyCountA = 0;
	int polyCountB = 0;
	uint64_t recastLiveA = 0;
	uint64_t recastLiveB = 0;
	uint64_t recastLiveBytesA = 0;
	uint64_t recastLiveBytesB = 0;
	uint64_t detourLiveA = 0;
	uint64_t detourLiveB = 0;
	uint64_t detourLiveBytesA = 0;
	uint64_t detourLiveBytesB = 0;
	uint64_t badFrees = 0;
	uint64_t meshHashA = 0;
	uint64_t meshHashB = 0;
	int navBytesA = 0;
	int navBytesB = 0;
	uint64_t navHashA = 0;
	uint64_t navHashB = 0;
};

EvalReport evaluate(const TerrainSpec& spec)
{
	EvalReport report;
	const BuildResult first = runBuild(spec);

	// Perturb the heap between the two runs so accidental pointer-order
	// dependencies are not masked by identical allocation addresses.
	const size_t perturbSize = 31 + static_cast<size_t>(spec.seed % 251);
	unsigned char* perturb = static_cast<unsigned char*>(std::malloc(perturbSize));
	if (perturb != nullptr)
	{
		std::memset(perturb, 0xa5, perturbSize);
	}
	const BuildResult second = runBuild(spec);
	std::free(perturb);

	report.cells = static_cast<int>(std::max(first.cellHashes.size(), second.cellHashes.size()));
	report.spanCountA = first.spanCount;
	report.spanCountB = second.spanCount;
	report.polyCountA = first.polyCount;
	report.polyCountB = second.polyCount;
	report.recastLiveA = first.recastLiveBlocks;
	report.recastLiveB = second.recastLiveBlocks;
	report.recastLiveBytesA = first.recastLiveBytes;
	report.recastLiveBytesB = second.recastLiveBytes;
	report.detourLiveA = first.detourLiveBlocks;
	report.detourLiveB = second.detourLiveBlocks;
	report.detourLiveBytesA = first.detourLiveBytes;
	report.detourLiveBytesB = second.detourLiveBytes;
	report.badFrees = first.badFrees + second.badFrees;
	report.meshHashA = first.meshHash;
	report.meshHashB = second.meshHash;
	report.navBytesA = first.navBytes;
	report.navBytesB = second.navBytes;
	report.navHashA = first.navHash;
	report.navHashB = second.navHash;

	if (!first.ok || !second.ok)
	{
		report.kind = static_cast<int>(FailureKind::BuildFailure);
		report.stage = first.ok ? second.stage : first.stage;
		return report;
	}

	const size_t commonCells = std::min(first.cellHashes.size(), second.cellHashes.size());
	for (size_t i = 0; i < commonCells; ++i)
	{
		if (first.cellHashes[i] != second.cellHashes[i])
		{
			report.kind = static_cast<int>(FailureKind::NondeterministicCells);
			report.firstCellMismatch = static_cast<int>(i);
			return report;
		}
	}
	if (first.cellHashes.size() != second.cellHashes.size())
	{
		report.kind = static_cast<int>(FailureKind::NondeterministicCells);
		report.firstCellMismatch = static_cast<int>(commonCells);
		return report;
	}
	if (first.meshHash != second.meshHash ||
		first.navHash != second.navHash ||
		first.navBytes != second.navBytes)
	{
		report.kind = static_cast<int>(FailureKind::NondeterministicMesh);
		return report;
	}
	if (first.recastLiveBlocks != 0 || second.recastLiveBlocks != 0 ||
		first.detourLiveBlocks != 0 || second.detourLiveBlocks != 0)
	{
		report.kind = static_cast<int>(FailureKind::Leak);
		return report;
	}
	if (report.badFrees != 0)
	{
		report.kind = static_cast<int>(FailureKind::Leak);
		return report;
	}
	return report;
}

EvalReport runIsolated(const TerrainSpec& spec)
{
#if defined(__unix__) || defined(__APPLE__)
	int pipeFds[2] = {-1, -1};
	if (pipe(pipeFds) != 0)
	{
		EvalReport report;
		report.kind = static_cast<int>(FailureKind::HarnessFailure);
		return report;
	}

	const pid_t pid = fork();
	if (pid == 0)
	{
		close(pipeFds[0]);
		alarm(kBuildTimeoutSeconds);
		const EvalReport report = evaluate(spec);
		const unsigned char* data = reinterpret_cast<const unsigned char*>(&report);
		size_t written = 0;
		while (written < sizeof(report))
		{
			const ssize_t n = write(pipeFds[1], data + written, sizeof(report) - written);
			if (n <= 0)
			{
				break;
			}
			written += static_cast<size_t>(n);
		}
		close(pipeFds[1]);
		_exit(written == sizeof(report) ? 0 : 2);
	}

	close(pipeFds[1]);
	if (pid < 0)
	{
		close(pipeFds[0]);
		EvalReport report;
		report.kind = static_cast<int>(FailureKind::HarnessFailure);
		return report;
	}

	EvalReport report{};
	unsigned char* data = reinterpret_cast<unsigned char*>(&report);
	size_t received = 0;
	while (received < sizeof(report))
	{
		const ssize_t n = read(pipeFds[0], data + received, sizeof(report) - received);
		if (n <= 0)
		{
			break;
		}
		received += static_cast<size_t>(n);
	}
	close(pipeFds[0]);

	int status = 0;
	while (waitpid(pid, &status, 0) < 0)
	{
	}

	if (WIFSIGNALED(status))
	{
		report.kind = static_cast<int>(WTERMSIG(status) == SIGALRM ? FailureKind::Timeout : FailureKind::Crash);
		report.signal = WTERMSIG(status);
	}
	else if (received != sizeof(report))
	{
		report.kind = static_cast<int>(FailureKind::HarnessFailure);
	}
	return report;
#else
	return evaluate(spec);
#endif
}

const char* failureKindName(FailureKind kind)
{
	switch (kind)
	{
	case FailureKind::Pass: return "pass";
	case FailureKind::BuildFailure: return "build-failure";
	case FailureKind::NondeterministicCells: return "nondeterministic-cells";
	case FailureKind::NondeterministicMesh: return "nondeterministic-mesh";
	case FailureKind::Leak: return "leak";
	case FailureKind::Crash: return "crash";
	case FailureKind::Timeout: return "timeout";
	case FailureKind::HarnessFailure: return "harness-failure";
	}
	return "unknown";
}

template <typename T>
bool tryRemoveRange(
	TerrainSpec& spec,
	std::vector<T> TerrainSpec::*member,
	int start,
	int count,
	FailureKind expectedFailure)
{
	TerrainSpec candidate = spec;
	auto& items = candidate.*member;
	items.erase(items.begin() + start, items.begin() + start + count);
	const EvalReport report = runIsolated(candidate);
	if (report.kind != static_cast<int>(expectedFailure))
	{
		return false;
	}
	spec = std::move(candidate);
	return true;
}

template <typename T>
void minimizeItems(
	TerrainSpec& spec,
	std::vector<T> TerrainSpec::*member,
	FailureKind expectedFailure)
{
	int granularity = 2;
	while (!(spec.*member).empty())
	{
		if ((spec.*member).size() == 1)
		{
			if (!tryRemoveRange(spec, member, 0, 1, expectedFailure))
			{
				break;
			}
			continue;
		}

		const int itemCount = static_cast<int>((spec.*member).size());
		const int chunk = std::max(1, (itemCount + granularity - 1) / granularity);
		bool reduced = false;
		for (int start = 0; start < itemCount; start += chunk)
		{
			const int count = std::min(chunk, itemCount - start);
			if (tryRemoveRange(spec, member, start, count, expectedFailure))
			{
				reduced = true;
				granularity = std::max(2, granularity - 1);
				break;
			}
		}
		if (!reduced)
		{
			if (granularity >= itemCount)
			{
				break;
			}
			granularity = std::min(itemCount, granularity * 2);
		}
	}
}

TerrainSpec cropTerrain(const TerrainSpec& spec, int minX, int minZ, int maxX, int maxZ)
{
	TerrainSpec cropped;
	cropped.seed = spec.seed;
	cropped.width = maxX - minX + 1;
	cropped.height = maxZ - minZ + 1;
	cropped.baseHeight = spec.baseHeight;

	for (const HeightPoint& point : spec.heightPoints)
	{
		if (point.x >= minX && point.x <= maxX + 1 && point.z >= minZ && point.z <= maxZ + 1)
		{
			cropped.heightPoints.push_back({point.x - minX, point.z - minZ, point.delta});
		}
	}

	for (const Hole& hole : spec.holes)
	{
		const int x0 = std::max(hole.x, minX);
		const int z0 = std::max(hole.z, minZ);
		const int x1 = std::min(hole.x + hole.width, maxX + 1);
		const int z1 = std::min(hole.z + hole.height, maxZ + 1);
		if (x0 < x1 && z0 < z1)
		{
			cropped.holes.push_back({x0 - minX, z0 - minZ, x1 - x0, z1 - z0});
		}
	}

	for (const BoxFeature& box : spec.boxes)
	{
		const float x0 = std::max(box.minX, static_cast<float>(minX));
		const float z0 = std::max(box.minZ, static_cast<float>(minZ));
		const float x1 = std::min(box.maxX, static_cast<float>(maxX + 1));
		const float z1 = std::min(box.maxZ, static_cast<float>(maxZ + 1));
		if (x1 - x0 < 0.05f || z1 - z0 < 0.05f)
		{
			continue;
		}
		BoxFeature clipped{};
		addBox(clipped, x0 - minX, z0 - minZ, x1 - minX, z1 - minZ, box.baseY, box.topY);
		cropped.boxes.push_back(clipped);
	}
	return cropped;
}

void minimizeBounds(TerrainSpec& spec, FailureKind expectedFailure)
{
	bool changed = true;
	while (changed)
	{
		changed = false;
		if (spec.width > 1)
		{
			TerrainSpec candidate = cropTerrain(spec, 1, 0, spec.width - 1, spec.height - 1);
			if (runIsolated(candidate).kind == static_cast<int>(expectedFailure))
			{
				spec = std::move(candidate);
				changed = true;
				continue;
			}
			candidate = cropTerrain(spec, 0, 0, spec.width - 2, spec.height - 1);
			if (runIsolated(candidate).kind == static_cast<int>(expectedFailure))
			{
				spec = std::move(candidate);
				changed = true;
				continue;
			}
		}
		if (spec.height > 1)
		{
			TerrainSpec candidate = cropTerrain(spec, 0, 1, spec.width - 1, spec.height - 1);
			if (runIsolated(candidate).kind == static_cast<int>(expectedFailure))
			{
				spec = std::move(candidate);
				changed = true;
				continue;
			}
			candidate = cropTerrain(spec, 0, 0, spec.width - 1, spec.height - 2);
			if (runIsolated(candidate).kind == static_cast<int>(expectedFailure))
			{
				spec = std::move(candidate);
				changed = true;
			}
		}
	}
}

TerrainSpec minimizeTerrain(const TerrainSpec& original, FailureKind expectedFailure)
{
	TerrainSpec minimized = original;
	for (int pass = 0; pass < 4; ++pass)
	{
		const size_t before =
			minimized.heightPoints.size() + minimized.holes.size() + minimized.boxes.size() +
			static_cast<size_t>(minimized.width * minimized.height);
		minimizeItems(minimized, &TerrainSpec::heightPoints, expectedFailure);
		minimizeItems(minimized, &TerrainSpec::holes, expectedFailure);
		minimizeItems(minimized, &TerrainSpec::boxes, expectedFailure);
		minimizeBounds(minimized, expectedFailure);
		const size_t after =
			minimized.heightPoints.size() + minimized.holes.size() + minimized.boxes.size() +
			static_cast<size_t>(minimized.width * minimized.height);
		if (after >= before)
		{
			break;
		}
	}
	return minimized;
}

std::string terrainSnippet(const TerrainSpec& spec)
{
	std::ostringstream out;
	out << "seed=" << spec.seed
		<< " size=" << spec.width << "x" << spec.height
		<< " baseHeight=" << spec.baseHeight
		<< " heightPoints=" << spec.heightPoints.size()
		<< " holes=" << spec.holes.size()
		<< " boxes=" << spec.boxes.size() << '\n';

	out << "heightDeltas={";
	for (size_t i = 0; i < spec.heightPoints.size(); ++i)
	{
		const HeightPoint& point = spec.heightPoints[i];
		if (i != 0)
		{
			out << ',';
		}
		out << '{' << point.x << ',' << point.z << ',' << point.delta << '}';
	}
	out << "}\n";

	out << "holes={";
	for (size_t i = 0; i < spec.holes.size(); ++i)
	{
		const Hole& hole = spec.holes[i];
		if (i != 0)
		{
			out << ',';
		}
		out << '{' << hole.x << ',' << hole.z << ',' << hole.width << ',' << hole.height << '}';
	}
	out << "}\n";

	out << "boxes={";
	for (size_t i = 0; i < spec.boxes.size(); ++i)
	{
		const BoxFeature& box = spec.boxes[i];
		if (i != 0)
		{
			out << ',';
		}
		out << '{' << box.minX << ',' << box.minZ << ',' << box.maxX << ',' << box.maxZ
			<< ',' << box.baseY << ',' << box.topY << '}';
	}
	out << "}\n";

	const std::vector<unsigned char> solid = buildSolidMask(spec);
	for (int z = 0; z < spec.height; ++z)
	{
		out << "  ";
		for (int x = 0; x < spec.width; ++x)
		{
			bool inBox = false;
			for (const BoxFeature& box : spec.boxes)
			{
				if (static_cast<float>(x) + 0.5f >= box.minX &&
					static_cast<float>(x) + 0.5f <= box.maxX &&
					static_cast<float>(z) + 0.5f >= box.minZ &&
					static_cast<float>(z) + 0.5f <= box.maxZ)
				{
					inBox = true;
					break;
				}
			}
			if (inBox)
			{
				out << 'B';
			}
			else if (!solid[static_cast<size_t>(z) * spec.width + x])
			{
				out << '#';
			}
			else
			{
				out << '.';
			}
		}
		out << '\n';
	}
	out << "  legend: . solid  # hole  B obstacle";
	return out.str();
}

std::string describeReport(const TerrainSpec& spec, const EvalReport& report)
{
	std::ostringstream out;
	out << "case seed=" << spec.seed
		<< " kind=" << failureKindName(static_cast<FailureKind>(report.kind));
	if (report.stage != 0)
	{
		out << " stage=" << report.stage;
	}
	if (report.signal != 0)
	{
		out << " signal=" << report.signal;
	}
	if (report.firstCellMismatch >= 0)
	{
		out << " firstCell=" << report.firstCellMismatch;
	}
	out << " cells=" << report.cells
		<< " spans=" << report.spanCountA << '/' << report.spanCountB
		<< " polys=" << report.polyCountA << '/' << report.polyCountB
		<< " live=" << report.recastLiveA << '+' << report.detourLiveA
		<< "blocks/" << report.recastLiveBytesA << '+' << report.detourLiveBytesA << "B";
	return out.str();
}
} // namespace

TEST_CASE("Random heightfield builds are deterministic and leak-free", "[recast][property][heightfield]")
{
	int passed = 0;
	int crashes = 0;
	int timeouts = 0;
	int leaks = 0;
	int nondeterministic = 0;
	int buildFailures = 0;
	int missingFeatureCases = 0;
	int totalHoles = 0;
	int totalSteepPoints = 0;
	int totalThinWalls = 0;
	int totalOverlappingBoxes = 0;
	std::vector<std::string> counterexamples;

	for (int seed = 0; seed < kCaseCount; ++seed)
	{
		const TerrainSpec spec = generateTerrain(seed);
		bool hasSteepPoint = false;
		for (const HeightPoint& point : spec.heightPoints)
		{
			hasSteepPoint = hasSteepPoint || point.delta >= 14;
		}
		const bool hasHoles = !spec.holes.empty();
		const bool hasThinWall = !spec.boxes.empty() &&
			spec.boxes.front().maxX - spec.boxes.front().minX <= 0.5f;
		const bool hasOverlappingBoxes = spec.boxes.size() >= 3;
		if (!hasHoles || !hasSteepPoint || !hasThinWall || !hasOverlappingBoxes)
		{
			missingFeatureCases++;
		}
		totalHoles += static_cast<int>(spec.holes.size());
		for (const HeightPoint& point : spec.heightPoints)
		{
			if (point.delta >= 14)
			{
				totalSteepPoints++;
			}
		}
		totalThinWalls += hasThinWall ? 1 : 0;
		totalOverlappingBoxes += hasOverlappingBoxes ? static_cast<int>(spec.boxes.size() - 1) : 0;

		const EvalReport report = runIsolated(spec);
		const FailureKind kind = static_cast<FailureKind>(report.kind);
		if (kind == FailureKind::Pass)
		{
			passed++;
			continue;
		}

		switch (kind)
		{
		case FailureKind::Crash: crashes++; break;
		case FailureKind::Timeout: timeouts++; break;
		case FailureKind::Leak: leaks++; break;
		case FailureKind::NondeterministicCells:
		case FailureKind::NondeterministicMesh: nondeterministic++; break;
		case FailureKind::BuildFailure: buildFailures++; break;
		default: break;
		}

		if (static_cast<int>(counterexamples.size()) < kMaxReportedCounterexamples)
		{
			const TerrainSpec minimal = minimizeTerrain(spec, kind);
			const EvalReport minimalReport = runIsolated(minimal);
			std::ostringstream message;
			message << "original: " << describeReport(spec, report) << '\n'
				<< "minimized: " << describeReport(minimal, minimalReport) << '\n'
				<< terrainSnippet(minimal);
			counterexamples.push_back(message.str());
		}
	}

	std::printf(
		"[heightfield-property] cases=%d passed=%d crashes=%d timeouts=%d leaks=%d "
		"nondeterministic=%d build_failures=%d\n",
		kCaseCount,
		passed,
		crashes,
		timeouts,
		leaks,
		nondeterministic,
		buildFailures);
	std::printf(
		"[heightfield-property] features holes=%d steep_points=%d thin_walls=%d "
		"overlapping_boxes=%d missing_feature_cases=%d\n",
		totalHoles,
		totalSteepPoints,
		totalThinWalls,
		totalOverlappingBoxes,
		missingFeatureCases);

	if (!counterexamples.empty())
	{
		std::string message = "Heightfield property failures:\n";
		for (size_t i = 0; i < counterexamples.size(); ++i)
		{
			message += "--- counterexample " + std::to_string(i + 1) + " ---\n";
			message += counterexamples[i];
			message += '\n';
		}
		FAIL(message);
	}

	REQUIRE(missingFeatureCases == 0);
	REQUIRE(passed == kCaseCount);
}
