#include "Recast.h"
#include <cstdio>
#include <cstring>
#include <vector>

// A deterministic, valid level: a set of well-formed triangles forming a
// stepped terrain plus a flat platform.
static void buildLevel(std::vector<float>& verts, std::vector<int>& tris)
{
	const float step = 1.0f;
	for (int gx = 0; gx < 8; ++gx)
	{
		for (int gz = 0; gz < 8; ++gz)
		{
			float x = gx * step;
			float z = gz * step;
			float y = (float)((gx + gz) % 3);
			int base = (int)(verts.size() / 3);
			float q[12] = {
				x, y, z,
				x + step, y, z,
				x + step, y, z + step,
				x, y, z + step,
			};
			for (int i = 0; i < 12; ++i) verts.push_back(q[i]);
			int t[6] = {base, base + 1, base + 2, base, base + 2, base + 3};
			for (int i = 0; i < 6; ++i) tris.push_back(t[i]);
		}
	}
}

static void dump(const char* tag, const rcHeightfield& hf)
{
	printf("HF %s w=%d h=%d cs=%g ch=%g bmin=(%g,%g,%g) bmax=(%g,%g,%g)\n",
	       tag, hf.width, hf.height, hf.cs, hf.ch, hf.bmin[0], hf.bmin[1], hf.bmin[2],
	       hf.bmax[0], hf.bmax[1], hf.bmax[2]);
	int total = 0;
	for (int z = 0; z < hf.height; ++z)
	{
		for (int x = 0; x < hf.width; ++x)
		{
			for (rcSpan* s = hf.spans[x + z * hf.width]; s; s = s->next)
			{
				printf("S %d %d %u %u %u\n", x, z, s->smin, s->smax, (unsigned)s->area);
				++total;
			}
		}
	}
	printf("HF %s spanTotal=%d spanCount=%d\n", tag, total, rcGetHeightFieldSpanCount(0, hf));
}

int main()
{
	std::vector<float> verts;
	std::vector<int> tris;
	buildLevel(verts, tris);

	float bmin[3], bmax[3];
	rcCalcBounds(&verts[0], (int)verts.size() / 3, bmin, bmax);

	const float cs = 0.3f, ch = 0.2f;
	int w, h;
	rcCalcGridSize(bmin, bmax, cs, &w, &h);

	rcContext ctx(false);
	rcHeightfield* solid = rcAllocHeightfield();
	if (!rcCreateHeightfield(&ctx, *solid, w, h, bmin, bmax, cs, ch))
	{
		printf("create failed %d %s\n", (int)ctx.getLastError(), ctx.getLastErrorLocation());
		return 2;
	}

	std::vector<unsigned char> triAreas(tris.size() / 3);
	rcMarkWalkableTriangles(&ctx, 45.0f, &verts[0], (int)verts.size() / 3,
	                        &tris[0], (int)tris.size() / 3, &triAreas[0]);

	bool ok = rcRasterizeTriangles(&ctx, &verts[0], (int)verts.size() / 3, &tris[0],
	                               &triAreas[0], (int)tris.size() / 3, *solid, 1);
	if (!ok)
	{
		printf("rasterize failed %d %s\n", (int)ctx.getLastError(), ctx.getLastErrorLocation());
		return 3;
	}

	dump("golden", *solid);

	// Run a few downstream filters to make sure their inputs/outputs are stable.
	rcFilterLowHangingWalkableObstacles(&ctx, 1, *solid);
	rcFilterLedgeSpans(&ctx, 10, 1, *solid);
	rcFilterWalkableLowHeightSpans(&ctx, 10, *solid);
	dump("filtered", *solid);

	rcCompactHeightfield* chf = rcAllocCompactHeightfield();
	ok = rcBuildCompactHeightfield(&ctx, 10, 1, *solid, *chf);
	printf("compact=%d cells=%d spans=%d\n", (int)ok, chf->width * chf->height, chf->spanCount);

	rcFreeCompactHeightfield(chf);
	rcFreeHeightField(solid);
	return 0;
}
