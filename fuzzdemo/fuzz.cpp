#include "Recast.h"
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <string>
#include <vector>

static unsigned seed = 12345;
static float frand(float lo, float hi) { return lo + (hi - lo) * (rand_r(&seed) / (float)RAND_MAX); }

static std::string signature(const rcHeightfield& hf)
{
	std::string s;
	char buf[64];
	for (int z = 0; z < hf.height; ++z)
		for (int x = 0; x < hf.width; ++x)
			for (rcSpan* sp = hf.spans[x + z * hf.width]; sp; sp = sp->next)
			{
				snprintf(buf, sizeof(buf), "%d,%d,%u,%u,%u;", x, z, sp->smin, sp->smax, (unsigned)sp->area);
				s += buf;
			}
	return s;
}

int main()
{
	rcContext ctx(false);

	// Differential: 200 random valid meshes, both libraries must match.
	for (int iter = 0; iter < 200; ++iter)
	{
		std::vector<float> verts;
		std::vector<int> tris;
		int n = 4 + (rand_r(&seed) % 12);
		for (int i = 0; i < n; ++i)
		{
			verts.push_back(frand(0, 9));
			verts.push_back(frand(0, 4));
			verts.push_back(frand(0, 9));
		}
		for (int t = 0; t < n; t += 3)
		{
			tris.push_back(t % n);
			tris.push_back((t + 1) % n);
			tris.push_back((t + 2) % n);
		}
		float bmin[3], bmax[3];
		rcCalcBounds(&verts[0], (int)verts.size() / 3, bmin, bmax);
		int w, h;
		rcCalcGridSize(bmin, bmax, 0.4f, &w, &h);
		if (w <= 0 || h <= 0) continue;

		rcHeightfield* hf = rcAllocHeightfield();
		bool ok = rcCreateHeightfield(&ctx, *hf, w, h, bmin, bmax, 0.4f, 0.3f);
		if (!ok) { rcFreeHeightField(hf); continue; }

		std::vector<unsigned char> areas(tris.size() / 3, RC_WALKABLE_AREA);
		rcMarkWalkableTriangles(&ctx, 45.0f, &verts[0], (int)verts.size() / 3,
		                        &tris[0], (int)tris.size() / 3, &areas[0]);
		ok = rcRasterizeTriangles(&ctx, &verts[0], (int)verts.size() / 3, &tris[0],
		                          &areas[0], (int)tris.size() / 3, *hf, 1);
		if (!ok)
		{
			printf("UNEXPECTED rasterize failure iter=%d code=%d %s\n", iter,
			       (int)ctx.getLastError(), ctx.getLastErrorLocation());
			rcFreeHeightField(hf);
			return 2;
		}
		printf("ITER %d %s\n", iter, signature(*hf).c_str());
		rcFreeHeightField(hf);
	}

	// Malformed stress: must always return cleanly (no crash under ASan/UBSan).
	const float nan = NAN, inf = INFINITY;
	for (int iter = 0; iter < 300; ++iter)
	{
		rcHeightfield* hf = rcAllocHeightfield();
		float bmin[3] = {0, 0, 0}, bmax[3] = {10, 5, 10};
		rcCreateHeightfield(&ctx, *hf, 16, 16, bmin, bmax, 0.3f, 0.2f);

		float junk[9];
		for (int k = 0; k < 9; ++k)
		{
			int pick = rand_r(&seed) % 5;
			junk[k] = pick == 0 ? nan : pick == 1 ? inf : pick == 2 ? -inf
			        : pick == 3 ? frand(-1e20f, 1e20f) : frand(-5, 15);
		}
		rcRasterizeTriangle(&ctx, &junk[0], &junk[3], &junk[6], RC_WALKABLE_AREA, *hf, 1);

		float good[9] = {1, 0, 1, 2, 1, 1, 1, 0, 2};
		int badTri[3] = {(int)frand(-5, 10), (int)frand(-5, 10), (int)frand(-5, 10)};
		unsigned char area = RC_WALKABLE_AREA;
		rcRasterizeTriangles(&ctx, good, 3, badTri, &area, 1, *hf, 1);

		rcFreeHeightField(hf);
	}
	printf("FUZZ DONE no crashes\n");
	return 0;
}
