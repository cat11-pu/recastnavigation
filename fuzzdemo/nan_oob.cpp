#include "Recast.h"
#include <cstdio>
#include <cmath>

int main()
{
	rcContext ctx(true);
	const float cs = 0.3f, ch = 0.2f;
	float bmin[3] = {0, 0, 0};
	float bmax[3] = {10, 5, 10};
	rcHeightfield* hf = rcAllocHeightfield();
	if (!rcCreateHeightfield(&ctx, *hf, 32, 32, bmin, bmax, cs, ch))
	{
		return 1;
	}

	float nan = NAN;
	float n0[] = {nan, 0.0f, 1.0f};
	float n1[] = {2.0f, 1.0f, 1.0f};
	float n2[] = {1.0f, 0.0f, 2.0f};
	bool ok = rcRasterizeTriangle(&ctx, n0, n1, n2, RC_WALKABLE_AREA, *hf, 1);
	printf("rasterize NaN tri returned %d, error=%d loc='%s'\n", (int)ok,
	       (int)ctx.getLastError(), ctx.getLastErrorLocation());

	int spans = rcGetHeightFieldSpanCount(&ctx, *hf);
	printf("span count = %d (32x32 grid, expected 0)\n", spans);
	rcFreeHeightField(hf);
	printf("clean free ok\n");
	return 0;
}
