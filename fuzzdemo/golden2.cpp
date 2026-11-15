#include "Recast.h"
#include <cstdio>
#include <cmath>
#include <vector>

// Diverse valid geometry exercising slopes, varied heights and large (but
// finite, in-range) coordinates, rasterized through all three API overloads.
int main()
{
	std::vector<float> verts;
	std::vector<int> intTris;

	// A ramp plus flat tiles, offset to a large-ish origin.
	const float ox = 10000.0f, oz = -5000.0f;
	for (int i = 0; i < 20; ++i)
	{
		float x = ox + i * 0.5f;
		float y = i * 0.05f;
		int base = (int)(verts.size() / 3);
		float q[12] = {
			x, y, oz,
			x + 0.5f, y, oz,
			x + 0.5f, y + 0.05f, oz + 2.0f,
			x, y + 0.05f, oz + 2.0f,
		};
		for (int k = 0; k < 12; ++k) verts.push_back(q[k]);
		int t[6] = {base, base + 1, base + 2, base, base + 2, base + 3};
		for (int k = 0; k < 6; ++k) intTris.push_back(t[k]);
	}

	float bmin[3], bmax[3];
	rcCalcBounds(&verts[0], (int)verts.size() / 3, bmin, bmax);
	const float cs = 0.25f, ch = 0.2f;
	int w, hgt;
	rcCalcGridSize(bmin, bmax, cs, &w, &hgt);

	rcContext ctx(false);
	int nt = (int)(intTris.size() / 3);
	std::vector<unsigned char> areas(nt, RC_WALKABLE_AREA);

	// Overload 1: int indices
	rcHeightfield* hfInt = rcAllocHeightfield();
	bool ok = rcCreateHeightfield(&ctx, *hfInt, w, hgt, bmin, bmax, cs, ch) &&
	          rcRasterizeTriangles(&ctx, &verts[0], (int)verts.size() / 3,
	                               &intTris[0], &areas[0], nt, *hfInt, 1);
	printf("int-overload ok=%d spans=%d\n", (int)ok, rcGetHeightFieldSpanCount(&ctx, *hfInt));

	// Overload 2: unsigned short indices
	std::vector<unsigned short> shortTris(intTris.begin(), intTris.end());
	rcHeightfield* hfShort = rcAllocHeightfield();
	ok = rcCreateHeightfield(&ctx, *hfShort, w, hgt, bmin, bmax, cs, ch) &&
	     rcRasterizeTriangles(&ctx, &verts[0], (int)verts.size() / 3,
	                          &shortTris[0], &areas[0], nt, *hfShort, 1);
	printf("short-overload ok=%d spans=%d\n", (int)ok, rcGetHeightFieldSpanCount(&ctx, *hfShort));

	// Overload 3: raw triangle list (re-index verts sequentially)
	std::vector<float> listVerts;
	for (int ti = 0; ti < nt; ++ti)
		for (int c = 0; c < 3; ++c)
			for (int ax = 0; ax < 3; ++ax)
				listVerts.push_back(verts[intTris[ti * 3 + c] * 3 + ax]);
	rcHeightfield* hfList = rcAllocHeightfield();
	ok = rcCreateHeightfield(&ctx, *hfList, w, hgt, bmin, bmax, cs, ch) &&
	     rcRasterizeTriangles(&ctx, &listVerts[0], &areas[0], nt, *hfList, 1);
	printf("list-overload ok=%d spans=%d\n", (int)ok, rcGetHeightFieldSpanCount(&ctx, *hfList));

	// Dump every span for byte-level comparison.
	for (int pass = 0; pass < 3; ++pass)
	{
		rcHeightfield* hf = pass == 0 ? hfInt : pass == 1 ? hfShort : hfList;
		for (int z = 0; z < hf->height; ++z)
			for (int x = 0; x < hf->width; ++x)
				for (rcSpan* s = hf->spans[x + z * hf->width]; s; s = s->next)
					printf("P%d %d %d %u %u %u\n", pass, x, z, s->smin, s->smax, (unsigned)s->area);
	}

	rcFreeHeightField(hfInt);
	rcFreeHeightField(hfShort);
	rcFreeHeightField(hfList);
	return 0;
}
