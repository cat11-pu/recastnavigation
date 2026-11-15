#include "Recast.h"
#include "catch2/catch_amalgamated.hpp"

#include <limits>

namespace
{
constexpr float CS = 0.3f;
constexpr float CH = 0.2f;

void makeBounds(float bmin[3], float bmax[3])
{
	bmin[0] = 0.0f;
	bmin[1] = 0.0f;
	bmin[2] = 0.0f;
	bmax[0] = 10.0f;
	bmax[1] = 5.0f;
	bmax[2] = 10.0f;
}

const float* goodVerts()
{
	static const float verts[] = {
		1.0f, 0.0f, 1.0f,
		2.0f, 1.0f, 1.0f,
		1.0f, 0.0f, 2.0f,
	};
	return verts;
}
}

TEST_CASE("rcCreateHeightfield rejects empty, oversized and malformed sub-grids", "[recast][validation]")
{
	rcContext ctx(false);
	float bmin[3], bmax[3];
	makeBounds(bmin, bmax);

	SECTION("Zero width")
	{
		rcHeightfield hf;
		REQUIRE_FALSE(rcCreateHeightfield(&ctx, hf, 0, 10, bmin, bmax, CS, CH));
		REQUIRE(ctx.getLastError() == RC_ERROR_INVALID_COUNT);
	}

	SECTION("Negative height")
	{
		rcHeightfield hf;
		REQUIRE_FALSE(rcCreateHeightfield(&ctx, hf, 10, -1, bmin, bmax, CS, CH));
		REQUIRE(ctx.getLastError() == RC_ERROR_INVALID_COUNT);
	}

	SECTION("Oversized grid would over-allocate / overflow")
	{
		rcHeightfield hf;
		REQUIRE_FALSE(rcCreateHeightfield(&ctx, hf, 50000, 50000, bmin, bmax, CS, CH));
		REQUIRE(ctx.getLastError() == RC_ERROR_GRID_TOO_LARGE);
	}

	SECTION("Non-positive cell size")
	{
		rcHeightfield hf;
		REQUIRE_FALSE(rcCreateHeightfield(&ctx, hf, 4, 4, bmin, bmax, 0.0f, CH));
		REQUIRE(ctx.getLastError() == RC_ERROR_INVALID_CELL_SIZE);
	}

	SECTION("Inverted bounds")
	{
		rcHeightfield hf;
		float badMax[3] = {bmin[0] - 1.0f, 5.0f, 10.0f};
		REQUIRE_FALSE(rcCreateHeightfield(&ctx, hf, 4, 4, bmin, badMax, CS, CH));
		REQUIRE(ctx.getLastError() == RC_ERROR_INVALID_BOUNDS);
	}

	SECTION("Non-finite cell height")
	{
		rcHeightfield hf;
		REQUIRE_FALSE(rcCreateHeightfield(&ctx, hf, 4, 4, bmin, bmax, CS,
		                                  std::numeric_limits<float>::quiet_NaN()));
		REQUIRE(ctx.getLastError() == RC_ERROR_INVALID_CELL_SIZE);
	}

	SECTION("Error location mentions the API and parameters")
	{
		rcHeightfield hf;
		REQUIRE_FALSE(rcCreateHeightfield(&ctx, hf, 0, 7, bmin, bmax, CS, CH));
		REQUIRE(std::string(ctx.getLastErrorLocation()).find("rcCreateHeightfield") != std::string::npos);
	}
}

TEST_CASE("rcCalcGridSize clamps huge coordinates instead of overflowing", "[recast][validation]")
{
	SECTION("Huge world span is clamped")
	{
		float bmin[3] = {0.0f, 0.0f, 0.0f};
		float bmax[3] = {1e15f, 100.0f, 1e15f};
		int sx = -1;
		int sz = -1;
		rcCalcGridSize(bmin, bmax, CS, &sx, &sz);
		REQUIRE(sx > 0);
		REQUIRE(sz > 0);
		// Each dimension is clamped instead of producing an int overflow; the
		// combined cell-count cap is enforced later by rcCreateHeightfield.
		REQUIRE(sx <= (1 << 20));
		REQUIRE(sz <= (1 << 20));
	}

	SECTION("Bad inputs report zero sizes")
	{
		float bmin[3] = {0.0f, 0.0f, 0.0f};
		float bmax[3] = {10.0f, 10.0f, 10.0f};
		int sx = -99;
		int sz = -99;
		rcCalcGridSize(bmin, bmax, 0.0f, &sx, &sz);
		REQUIRE(sx == 0);
		REQUIRE(sz == 0);

		rcCalcGridSize(nullptr, bmax, CS, &sx, &sz);
		REQUIRE(sx == 0);
		REQUIRE(sz == 0);
	}
}

TEST_CASE("Degenerate and non-finite triangles are reported, never rasterized", "[recast][validation]")
{
	rcContext ctx(false);
	float bmin[3], bmax[3];
	makeBounds(bmin, bmax);
	rcHeightfield hf;
	REQUIRE(rcCreateHeightfield(&ctx, hf, 32, 32, bmin, bmax, CS, CH));

	SECTION("Collinear triangle adds no spans")
	{
		float v0[] = {1.0f, 0.0f, 1.0f};
		float v1[] = {2.0f, 0.0f, 1.0f};
		float v2[] = {4.0f, 0.0f, 1.0f};
		REQUIRE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));
		REQUIRE(rcGetHeightFieldSpanCount(&ctx, hf) == 0);
	}

	SECTION("Duplicate vertices add no spans")
	{
		float v0[] = {1.0f, 0.0f, 1.0f};
		REQUIRE(rcRasterizeTriangle(&ctx, v0, v0, v0, RC_WALKABLE_AREA, hf, 1));
		REQUIRE(rcGetHeightFieldSpanCount(&ctx, hf) == 0);
	}

	SECTION("NaN coordinates are an explicit error with a location")
	{
		float v0[] = {std::numeric_limits<float>::quiet_NaN(), 0.0f, 1.0f};
		float v1[] = {2.0f, 1.0f, 1.0f};
		float v2[] = {1.0f, 0.0f, 2.0f};
		REQUIRE_FALSE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));
		REQUIRE(ctx.getLastError() == RC_ERROR_NON_FINITE_VERTEX);
		REQUIRE(std::string(ctx.getLastErrorLocation()).find("rcRasterizeTriangle") != std::string::npos);
		REQUIRE(rcGetHeightFieldSpanCount(&ctx, hf) == 0);
	}

	SECTION("Infinity coordinates are an explicit error")
	{
		float v0[] = {1.0f, 0.0f, 1.0f};
		float v1[] = {std::numeric_limits<float>::infinity(), 1.0f, 1.0f};
		float v2[] = {1.0f, 0.0f, 2.0f};
		REQUIRE_FALSE(rcRasterizeTriangle(&ctx, v0, v1, v2, RC_WALKABLE_AREA, hf, 1));
		REQUIRE(ctx.getLastError() == RC_ERROR_NON_FINITE_VERTEX);
	}
}

TEST_CASE("Out-of-range triangle indices are rejected with their location", "[recast][validation]")
{
	rcContext ctx(false);
	float bmin[3], bmax[3];
	makeBounds(bmin, bmax);
	rcHeightfield hf;
	REQUIRE(rcCreateHeightfield(&ctx, hf, 32, 32, bmin, bmax, CS, CH));

	const float* verts = goodVerts();
	unsigned char areas[2] = {RC_WALKABLE_AREA, RC_WALKABLE_AREA};

	SECTION("Integer index above vertex count")
	{
		int tris[] = {0, 1, 99};
		REQUIRE_FALSE(rcRasterizeTriangles(&ctx, verts, 3, tris, areas, 1, hf, 1));
		REQUIRE(ctx.getLastError() == RC_ERROR_INVALID_TRIANGLE_INDEX);
		REQUIRE(std::string(ctx.getLastErrorLocation()).find("triangle 0") != std::string::npos);
	}

	SECTION("Negative integer index")
	{
		int tris[] = {0, -1, 2};
		REQUIRE_FALSE(rcRasterizeTriangles(&ctx, verts, 3, tris, areas, 1, hf, 1));
		REQUIRE(ctx.getLastError() == RC_ERROR_INVALID_TRIANGLE_INDEX);
	}

	SECTION("Unsigned short index above vertex count")
	{
		unsigned short tris[] = {0, 1, 65000};
		REQUIRE_FALSE(rcRasterizeTriangles(&ctx, verts, 3, tris, areas, 1, hf, 1));
		REQUIRE(ctx.getLastError() == RC_ERROR_INVALID_TRIANGLE_INDEX);
	}

	SECTION("Triangle position identifies the bad triangle")
	{
		int tris[6] = {0, 1, 2, 0, 1, 7};
		REQUIRE_FALSE(rcRasterizeTriangles(&ctx, verts, 3, tris, areas, 2, hf, 1));
		REQUIRE(ctx.getLastError() == RC_ERROR_INVALID_TRIANGLE_INDEX);
		REQUIRE(std::string(ctx.getLastErrorLocation()).find("triangle 1") != std::string::npos);
	}

	SECTION("Null pointers and negative counts")
	{
		int tris[] = {0, 1, 2};
		REQUIRE_FALSE(rcRasterizeTriangles(&ctx, (const float*)nullptr, 3, tris, areas, 1, hf, 1));
		REQUIRE(ctx.getLastError() == RC_ERROR_NULL_ARGUMENT);

		REQUIRE_FALSE(rcRasterizeTriangles(&ctx, verts, 3, tris, areas, -1, hf, 1));
		REQUIRE(ctx.getLastError() == RC_ERROR_INVALID_COUNT);
	}
}

TEST_CASE("Slope marking validates indices without touching the output buffer", "[recast][validation]")
{
	rcContext ctx(false);
	const float verts[] = {
		0.0f, 0.0f, 0.0f,
		1.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f,
	};

	SECTION("rcMarkWalkableTriangles rejects out-of-range index")
	{
		int tris[] = {0, 1, 7};
		unsigned char triAreas[1] = {0};
		rcMarkWalkableTriangles(&ctx, 45.0f, verts, 3, tris, 1, triAreas);
		REQUIRE(ctx.getLastError() == RC_ERROR_INVALID_TRIANGLE_INDEX);
		REQUIRE(triAreas[0] == 0);
	}

	SECTION("rcClearUnwalkableTriangles rejects out-of-range index")
	{
		int tris[] = {0, 1, 7};
		unsigned char triAreas[1] = {42};
		rcClearUnwalkableTriangles(&ctx, 45.0f, verts, 3, tris, 1, triAreas);
		REQUIRE(ctx.getLastError() == RC_ERROR_INVALID_TRIANGLE_INDEX);
		REQUIRE(triAreas[0] == 42);
	}
}

TEST_CASE("Error state can be cleared", "[recast][validation]")
{
	rcContext ctx(false);
	float bmin[3], bmax[3];
	makeBounds(bmin, bmax);
	rcHeightfield hf;
	REQUIRE_FALSE(rcCreateHeightfield(&ctx, hf, 0, 4, bmin, bmax, CS, CH));
	REQUIRE(ctx.getLastError() != RC_ERROR_NONE);

	ctx.clearError();
	REQUIRE(ctx.getLastError() == RC_ERROR_NONE);
	REQUIRE(ctx.getLastErrorLocation()[0] == '\0');
}
