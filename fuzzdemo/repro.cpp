#include "Recast.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

static const float CS = 0.3f;
static const float CH = 0.2f;

static void makeHfBounds(float bmin[3], float bmax[3])
{
	bmin[0] = 0.0f; bmin[1] = 0.0f; bmin[2] = 0.0f;
	bmax[0] = 10.0f; bmax[1] = 5.0f; bmax[2] = 10.0f;
}

// 1. Empty sub-grid: zero / negative / oversized dimensions
static void caseEmptyGrid()
{
	printf("== caseEmptyGrid ==\n");
	rcContext ctx(true);
	float bmin[3], bmax[3];
	makeHfBounds(bmin, bmax);

	struct Probe { int w, h; const char* label; } probes[] = {
		{0, 10, "width=0"},
		{10, -5, "height=-5"},
		{50000, 50000, "50000x50000 (over-allocation)"},
	};
	for (int i = 0; i < 3; ++i)
	{
		rcHeightfield* hf = rcAllocHeightfield();
		bool ok = rcCreateHeightfield(&ctx, *hf, probes[i].w, probes[i].h, bmin, bmax, CS, CH);
		printf("  %-38s -> %s  code=%d  loc='%s'\n", probes[i].label,
		       ok ? "ACCEPTED (BUG)" : "rejected", (int)ctx.getLastError(), ctx.getLastErrorLocation());
		rcFreeHeightField(hf);
	}
}

// 2. Huge coordinates fed through rcCalcGridSize + create + rasterize
static void caseHugeCoords()
{
	printf("== caseHugeCoords ==\n");
	rcContext ctx(true);

	float bmin[3] = {0.0f, 0.0f, 0.0f};
	float bmax[3] = {1e15f, 100.0f, 1e15f};
	int sx = 0, sz = 0;
	rcCalcGridSize(bmin, bmax, CS, &sx, &sz);
	printf("  rcCalcGridSize(1e15 span, cs=0.3) -> %d x %d (clamped, no UB)\n", sx, sz);

	rcHeightfield* hf = rcAllocHeightfield();
	bool ok = rcCreateHeightfield(&ctx, *hf, sx, sz, bmin, bmax, CS, CH);
	printf("  create clamped grid -> %s  code=%d  loc='%s'\n",
	       ok ? "ok" : "rejected", (int)ctx.getLastError(), ctx.getLastErrorLocation());
	if (ok)
	{
		float verts[] = {
			1.0f, 0.0f, 1.0f,
			2.0f, 1.0f, 1.0f,
			1.0f, 0.0f, 2.0f,
		};
		unsigned char areas[] = {RC_WALKABLE_AREA};
		ok = rcRasterizeTriangles(&ctx, verts, areas, 1, *hf, 1);
		printf("  rasterize small tri -> %s\n", ok ? "ok" : "failed");
	}
	rcFreeHeightField(hf);

	// Direct huge-coordinate triangle against a normal-sized heightfield
	rcHeightfield* hf2 = rcAllocHeightfield();
	float nmin[3], nmax[3];
	makeHfBounds(nmin, nmax);
	ok = rcCreateHeightfield(&ctx, *hf2, 32, 32, nmin, nmax, CS, CH);
	if (ok)
	{
		float hv[] = {
			1.0f, 0.0f, 1.0f,
			1e15f, 1.0f, 1.0f,
			1.0f, 0.0f, 1e15f,
		};
		ok = rcRasterizeTriangle(&ctx, &hv[0], &hv[3], &hv[6], RC_WALKABLE_AREA, *hf2, 1);
		printf("  rasterize huge-coord tri -> %s (no UB/OOB)\n", ok ? "handled" : "rejected");
	}
	rcFreeHeightField(hf2);
}

// 3. Degenerate triangles (collinear / duplicate vertices), incl. NaN
static void caseDegenerate()
{
	printf("== caseDegenerate ==\n");
	rcContext ctx(true);
	float bmin[3], bmax[3];
	makeHfBounds(bmin, bmax);
	rcHeightfield* hf = rcAllocHeightfield();
	bool ok = rcCreateHeightfield(&ctx, *hf, 32, 32, bmin, bmax, CS, CH);
	if (!ok) { printf("  hf create failed\n"); rcFreeHeightField(hf); return; }

	float c0[] = {1.0f, 0.0f, 1.0f};
	float c1[] = {2.0f, 0.0f, 1.0f};
	float c2[] = {4.0f, 0.0f, 1.0f};
	ok = rcRasterizeTriangle(&ctx, c0, c1, c2, RC_WALKABLE_AREA, *hf, 1);
	printf("  collinear tri -> %s (skipped, no spans)\n", ok ? "accepted" : "rejected");

	float d0[] = {1.0f, 0.0f, 1.0f};
	ok = rcRasterizeTriangle(&ctx, d0, d0, d0, RC_WALKABLE_AREA, *hf, 1);
	printf("  duplicate-vertex tri -> %s (skipped, no spans)\n", ok ? "accepted" : "rejected");

	float nan = NAN;
	float n0[] = {nan, 0.0f, 1.0f};
	float n1[] = {2.0f, 1.0f, 1.0f};
	float n2[] = {1.0f, 0.0f, 2.0f};
	ok = rcRasterizeTriangle(&ctx, n0, n1, n2, RC_WALKABLE_AREA, *hf, 1);
	printf("  NaN-coord tri -> %s  code=%d  loc='%s'\n",
	       ok ? "ACCEPTED (BUG)" : "rejected", (int)ctx.getLastError(), ctx.getLastErrorLocation());

	int spans = rcGetHeightFieldSpanCount(&ctx, *hf);
	printf("  spans after degenerate inputs = %d (expected 0)\n", spans);
	rcFreeHeightField(hf);
}

// 4. Indexed triangles with out-of-range / negative indices
static void caseBadIndices()
{
	printf("== caseBadIndices ==\n");
	rcContext ctx(true);
	float bmin[3], bmax[3];
	makeHfBounds(bmin, bmax);
	rcHeightfield* hf = rcAllocHeightfield();
	bool ok = rcCreateHeightfield(&ctx, *hf, 32, 32, bmin, bmax, CS, CH);
	if (!ok) { printf("  hf create failed\n"); rcFreeHeightField(hf); return; }

	float verts[] = {
		1.0f, 0.0f, 1.0f,
		2.0f, 1.0f, 1.0f,
		1.0f, 0.0f, 2.0f,
	};
	unsigned char areas[] = {RC_WALKABLE_AREA, RC_WALKABLE_AREA, RC_WALKABLE_AREA};

	int badInt[] = {0, 1, 99};
	ok = rcRasterizeTriangles(&ctx, verts, 3, badInt, areas, 1, *hf, 1);
	printf("  int index 99 (nv=3) -> %s code=%d loc='%s'\n",
	       ok ? "ACCEPTED (BUG)" : "rejected", (int)ctx.getLastError(), ctx.getLastErrorLocation());

	int negInt[] = {0, -1, 2};
	ok = rcRasterizeTriangles(&ctx, verts, 3, negInt, areas, 1, *hf, 1);
	printf("  int index -1 -> %s code=%d loc='%s'\n",
	       ok ? "ACCEPTED (BUG)" : "rejected", (int)ctx.getLastError(), ctx.getLastErrorLocation());

	unsigned short badShort[] = {0, 1, 65000};
	ok = rcRasterizeTriangles(&ctx, verts, 3, badShort, areas, 1, *hf, 1);
	printf("  ushort index 65000 (nv=3) -> %s code=%d loc='%s'\n",
	       ok ? "ACCEPTED (BUG)" : "rejected", (int)ctx.getLastError(), ctx.getLastErrorLocation());

	ok = rcRasterizeTriangles(&ctx, (const float*)0, 3, badInt, areas, 1, *hf, 1);
	printf("  null verts -> %s code=%d\n", ok ? "ACCEPTED (BUG)" : "rejected", (int)ctx.getLastError());
	ok = rcRasterizeTriangles(&ctx, verts, 3, badInt, areas, -1, *hf, 1);
	printf("  numTris=-1 -> %s code=%d\n", ok ? "ACCEPTED (BUG)" : "rejected", (int)ctx.getLastError());

	rcFreeHeightField(hf);
}

// 5. Mark/clear walkable with bad indices (void API)
static void caseMarkBadIndices()
{
	printf("== caseMarkBadIndices ==\n");
	rcContext ctx(true);
	float verts[] = {
		0.0f, 0.0f, 0.0f,
		1.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f,
	};
	int tris[] = {0, 1, 7};
	unsigned char triAreas[1] = {0};
	rcMarkWalkableTriangles(&ctx, 45.0f, verts, 3, tris, 1, triAreas);
	printf("  rcMarkWalkableTriangles OOB -> code=%d loc='%s' area=%u (unchanged)\n",
	       (int)ctx.getLastError(), ctx.getLastErrorLocation(), (unsigned)triAreas[0]);
	ctx.clearError();
	rcClearUnwalkableTriangles(&ctx, 45.0f, verts, 3, tris, 1, triAreas);
	printf("  rcClearUnwalkableTriangles OOB -> code=%d loc='%s' area=%u (unchanged)\n",
	       (int)ctx.getLastError(), ctx.getLastErrorLocation(), (unsigned)triAreas[0]);
}

int main()
{
	const char* which = std::getenv("REPRO_CASE");
	if (!which || !strcmp(which, "empty")) caseEmptyGrid();
	if (!which || !strcmp(which, "huge")) caseHugeCoords();
	if (!which || !strcmp(which, "degen")) caseDegenerate();
	if (!which || !strcmp(which, "index")) caseBadIndices();
	if (!which || !strcmp(which, "mark")) caseMarkBadIndices();
	printf("ALL SURVIVED\n");
	return 0;
}
