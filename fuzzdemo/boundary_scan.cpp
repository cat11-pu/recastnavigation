#include "Recast.h"
#include <cstdio>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

struct Row
{
	const char* api;
	std::string input;
	bool accepted;
	int code;
	std::string location;
};

static std::vector<Row> rows;

static void record(const char* api, const char* input, bool accepted, rcContext& ctx)
{
	rows.push_back(Row{api, std::string(input), accepted, (int)ctx.getLastError(), ctx.getLastErrorLocation()});
}

int main()
{
	const float nan = std::numeric_limits<float>::quiet_NaN();
	const float inf = std::numeric_limits<float>::infinity();
	const float CS = 0.3f, CH = 0.2f;

	float goodMin[3] = {0, 0, 0};
	float goodMax[3] = {10, 5, 10};

	// ---- rcCreateHeightfield boundary sweep ----
	{
		struct P { int w, h; const char* label; } grid[] = {
			{-1, 4, "w=-1"}, {0, 4, "w=0"}, {1, 4, "w=1 (min legal)"},
			{4, 0, "h=0"}, {4, -7, "h=-7"}, {4, 1, "h=1 (min legal)"},
			{1 << 19, 1, "2^19 x 1 (under cap)"},
			{1 << 20, 1, "2^20 x 1 (at per-axis cap, total over)"},
			{1024, 1024, "1024x1024=2^20 (exact total cap)"},
			{1025, 1024, "1025x1024 (just over cap)"},
			{50000, 50000, "50000x50000 (huge)"},
		};
		for (size_t i = 0; i < sizeof(grid) / sizeof(grid[0]); ++i)
		{
			rcContext ctx(false);
			rcHeightfield* hf = rcAllocHeightfield();
			bool ok = rcCreateHeightfield(&ctx, *hf, grid[i].w, grid[i].h, goodMin, goodMax, CS, CH);
			record("rcCreateHeightfield", grid[i].label, ok, ctx);
			rcFreeHeightField(hf);
		}

		struct C { float cs, ch; const char* label; } cell[] = {
			{0.0f, CH, "cs=0"}, {-0.3f, CH, "cs<0"}, {nan, CH, "cs=NaN"},
			{CS, 0.0f, "ch=0"}, {CS, -1.0f, "ch<0"}, {CS, inf, "ch=Inf"},
			{1e-30f, CH, "cs=tiny(1e-30)"}, {3.4e38f, CH, "cs=max-finite"},
		};
		for (size_t i = 0; i < sizeof(cell) / sizeof(cell[0]); ++i)
		{
			rcContext ctx(false);
			rcHeightfield* hf = rcAllocHeightfield();
			bool ok = rcCreateHeightfield(&ctx, *hf, 4, 4, goodMin, goodMax, cell[i].cs, cell[i].ch);
			record("rcCreateHeightfield", cell[i].label, ok, ctx);
			rcFreeHeightField(hf);
		}

		struct B { float m[3], x[3]; const char* label; } bounds[] = {
			{{0,0,0},{10,5,10}, "normal"},
			{{5,0,0},{5,5,10}, "zero X span"},
			{{11,0,0},{10,5,10}, "inverted X"},
			{{0,0,0},{10,nan,10}, "NaN in max"},
			{{inf,0,0},{10,5,10}, "Inf in min"},
		};
		for (size_t i = 0; i < sizeof(bounds) / sizeof(bounds[0]); ++i)
		{
			rcContext ctx(false);
			rcHeightfield* hf = rcAllocHeightfield();
			bool ok = rcCreateHeightfield(&ctx, *hf, 4, 4, bounds[i].m, bounds[i].x, CS, CH);
			record("rcCreateHeightfield", bounds[i].label, ok, ctx);
			rcFreeHeightField(hf);
		}
	}

	// ---- rcCalcGridSize boundary sweep ----
	{
		struct G { float span; float cs; const char* label; } cases[] = {
			{1e15f, 0.3f, "span=1e15 cs=.3"},
			{3.4e38f, 0.1f, "span=FLT_MAX"},
			{10.0f, 0.0f, "cs=0"},
			{10.0f, -1.0f, "cs<0"},
			{10.0f, nan, "cs=NaN"},
			{nan, 0.3f, "span=NaN"},
			{0.0f, 0.3f, "zero span"},
			{9.0f, 0.3f, "span=9 cs=.3 (legal)"},
		};
		for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i)
		{
			float bmin[3] = {0, 0, 0};
			float bmax[3] = {
				std::isfinite(cases[i].span) ? cases[i].span : cases[i].span,
				5.0f,
				std::isfinite(cases[i].span) ? cases[i].span : cases[i].span};
			int sx = -99, sz = -99;
			rcCalcGridSize(bmin, bmax, cases[i].cs, &sx, &sz);
			bool finiteCs = (cases[i].cs == cases[i].cs) && cases[i].cs > 0.0f;
			bool finiteSpan = (cases[i].span == cases[i].span);
			bool ok = finiteCs && finiteSpan && sx >= 0 && sz >= 0 &&
			          sx <= (1 << 20) && sz <= (1 << 20);
			char label[96];
			snprintf(label, sizeof(label), "%s -> sx=%d sz=%d", cases[i].label, sx, sz);
			rows.push_back(Row{"rcCalcGridSize", label, ok, RC_ERROR_NONE,
			                   ok ? "sizes within range" : "zeroed/clamped, no overflow"});
		}
	}

	// ---- rasterization boundary sweep ----
	{
		rcContext ctx(false);
		rcHeightfield* hf = rcAllocHeightfield();
		rcCreateHeightfield(&ctx, *hf, 32, 32, goodMin, goodMax, CS, CH);

		float goodVerts[9] = {1, 0, 1, 2, 1, 1, 1, 0, 2};
		struct T { float v[9]; const char* label; } tris[] = {
			{{1,0,1, 2,1,1, 1,0,2}, "normal triangle"},
			{{1,0,1, 2,0,1, 4,0,1}, "collinear (zero area)"},
			{{1,0,1, 1,0,1, 1,0,1}, "duplicate vertex"},
			{{nan,0,1, 2,1,1, 1,0,2}, "NaN coordinate"},
			{{inf,0,1, 2,1,1, 1,0,2}, "+Inf coordinate"},
			{{-inf,0,1, 2,1,1, 1,0,2}, "-Inf coordinate"},
			{{3.4e38f,0,1, 2,1,1, 1,0,2}, "FLT_MAX coordinate"},
		};
		for (size_t i = 0; i < sizeof(tris) / sizeof(tris[0]); ++i)
		{
			rcContext c(false);
			bool ok = rcRasterizeTriangle(&c, &tris[i].v[0], &tris[i].v[3], &tris[i].v[6],
			                              RC_WALKABLE_AREA, *hf, 1);
			record("rcRasterizeTriangle", tris[i].label, ok, c);
		}

		struct Idx { int t[3]; const char* label; } idx[] = {
			{{0, 1, 2}, "all valid [0,3)"},
			{{0, 1, 3}, "index == numVerts"},
			{{0, 1, 4}, "index > numVerts"},
			{{-1, 1, 2}, "negative index"},
			{{0, -2147483647 - 1, 2}, "INT_MIN index"},
		};
		for (size_t i = 0; i < sizeof(idx) / sizeof(idx[0]); ++i)
		{
			rcContext c(false);
			unsigned char area = RC_WALKABLE_AREA;
			bool ok = rcRasterizeTriangles(&c, goodVerts, 3, idx[i].t, &area, 1, *hf, 1);
			record("rcRasterizeTriangles(int)", idx[i].label, ok, c);
		}

		struct UIdx { unsigned short t[3]; const char* label; } uidx[] = {
			{{0, 1, 2}, "all valid"},
			{{0, 1, 3}, "index == numVerts"},
			{{0, 1, 65535}, "USHRT_MAX index"},
		};
		for (size_t i = 0; i < sizeof(uidx) / sizeof(uidx[0]); ++i)
		{
			rcContext c(false);
			unsigned char area = RC_WALKABLE_AREA;
			bool ok = rcRasterizeTriangles(&c, goodVerts, 3, uidx[i].t, &area, 1, *hf, 1);
			record("rcRasterizeTriangles(ushort)", uidx[i].label, ok, c);
		}

		struct Arg { int nt; int nv; int thr; const char* label; } args[] = {
			{0, 3, 1, "numTris=0 (legal empty)"},
			{-1, 3, 1, "numTris=-1"},
			{1, 0, 1, "numVerts=0"},
			{1, 3, -1, "flagMergeThreshold=-1"},
		};
		for (size_t i = 0; i < sizeof(args) / sizeof(args[0]); ++i)
		{
			rcContext c(false);
			int goodTri[3] = {0, 1, 2};
			unsigned char area = RC_WALKABLE_AREA;
			bool ok = rcRasterizeTriangles(&c, goodVerts, args[i].nv, goodTri, &area,
			                               args[i].nt, *hf, args[i].thr);
			record("rcRasterizeTriangles(int)", args[i].label, ok, c);
		}

		// null pointers
		{
			rcContext c(false);
			int goodTri[3] = {0, 1, 2};
			unsigned char area = RC_WALKABLE_AREA;
			bool ok = rcRasterizeTriangles(&c, (const float*)0, 3, goodTri, &area, 1, *hf, 1);
			record("rcRasterizeTriangles(int)", "null verts", ok, c);
		}
		{
			rcContext c(false);
			unsigned char area = RC_WALKABLE_AREA;
			bool ok = rcRasterizeTriangles(&c, goodVerts, 3, (const int*)0, &area, 1, *hf, 1);
			record("rcRasterizeTriangles(int)", "null tris", ok, c);
		}

		rcFreeHeightField(hf);
	}

	// ---- slope-marking boundary sweep ----
	{
		float verts[9] = {0, 0, 0, 1, 0, 0, 0, 0, 1};
		struct M { int t[3]; const char* label; } marks[] = {
			{{0, 1, 2}, "valid"},
			{{0, 1, 3}, "OOB index"},
			{{-1, 1, 2}, "negative index"},
		};
		for (size_t i = 0; i < sizeof(marks) / sizeof(marks[0]); ++i)
		{
			rcContext c(false);
			unsigned char area = 0;
			rcMarkWalkableTriangles(&c, 45.0f, verts, 3, marks[i].t, 1, &area);
			record("rcMarkWalkableTriangles", marks[i].label, c.getLastError() == RC_ERROR_NONE, c);
		}
	}

	// ---- Emit table ----
	printf("| API | Input boundary | Result | rcErrorCode | Location |\n");
	printf("|---|---|---|---|---|\n");
	for (size_t i = 0; i < rows.size(); ++i)
	{
		printf("| %s | %s | %s | %d | %s |\n",
		       rows[i].api, rows[i].input.c_str(), rows[i].accepted ? "accepted" : "REJECTED",
		       rows[i].code, rows[i].location.c_str());
	}

	int rejected = 0;
	for (size_t i = 0; i < rows.size(); ++i)
		if (!rows[i].accepted) ++rejected;
	fprintf(stderr, "SCAN COMPLETE: %zu probes, %d rejected, %zu accepted\n",
	        rows.size(), rejected, rows.size() - rejected);
	return 0;
}
