#include "Recast.h"
#include <cstdio>
#include <cmath>

enum rcAxis { RC_AXIS_X = 0, RC_AXIS_Y = 1, RC_AXIS_Z = 2 };

// Exact copy of dividePoly from RecastRasterization.cpp, instrumented.
static void dividePoly(const float* inVerts, int inVertsCount,
                       float* outVerts1, int* outVerts1Count,
                       float* outVerts2, int* outVerts2Count,
                       float axisOffset, rcAxis axis)
{
	float inVertAxisDelta[12];
	for (int inVert = 0; inVert < inVertsCount; ++inVert)
		inVertAxisDelta[inVert] = axisOffset - inVerts[inVert * 3 + axis];

	int poly1Vert = 0;
	int poly2Vert = 0;
	for (int inVertA = 0, inVertB = inVertsCount - 1; inVertA < inVertsCount; inVertB = inVertA, ++inVertA)
	{
		bool sameSide = (inVertAxisDelta[inVertA] >= 0) == (inVertAxisDelta[inVertB] >= 0);
		if (!sameSide)
		{
			float s = inVertAxisDelta[inVertB] / (inVertAxisDelta[inVertB] - inVertAxisDelta[inVertA]);
			(void)s;
			poly1Vert++;
			poly2Vert++;
			if (inVertAxisDelta[inVertA] > 0) poly1Vert++;
			else if (inVertAxisDelta[inVertA] < 0) poly2Vert++;
		}
		else
		{
			if (inVertAxisDelta[inVertA] >= 0)
			{
				poly1Vert++;
				if (inVertAxisDelta[inVertA] != 0) continue;
			}
			rcVcopy(&outVerts2[poly2Vert * 3], &inVerts[inVertA * 3]);
			poly2Vert++;
		}
	}
	*outVerts1Count = poly1Vert;
	*outVerts2Count = poly2Vert;
}

static void probe(const char* name, float v[9])
{
	float buf[7 * 3 * 4];
	float* in = buf;
	float* inRow = buf + 7 * 3;
	float* p1 = inRow + 7 * 3;
	float* p2 = p1 + 7 * 3;
	rcVcopy(&in[0], &v[0]); rcVcopy(&in[3], &v[3]); rcVcopy(&in[6], &v[6]);
	int nvIn = 3, nvRow = 0;
	int maxSeen = 3;
	// emulate row clipping over a range of cell offsets
	for (int i = 0; i < 40 && nvIn >= 3; ++i)
	{
		dividePoly(in, nvIn, inRow, &nvRow, p1, &nvIn, (float)i * 0.3f + 0.3f, RC_AXIS_Z);
		rcSwap(in, p1);
		if (nvIn > maxSeen) maxSeen = nvIn;
		if (nvRow > maxSeen) maxSeen = nvRow;
		if (nvIn > 7 || nvRow > 7)
		{
			printf("%s: vertex count exceeded buffer (nvIn=%d nvRow=%d) at iter %d\n", name, nvIn, nvRow, i);
			return;
		}
	}
	printf("%s: max vertices seen = %d (buffer holds 7)\n", name, maxSeen);
}

int main()
{
	float collinear[9] = {1,0,1, 2,0,1, 4,0,1};
	float dup[9] = {1,0,1, 1,0,1, 1,0,1};
	float nanv[9] = {NAN,0,1, 2,1,1, 1,0,2};
	float infv[9] = {1,0,1, INFINITY,1,1, 1,0,2};
	probe("collinear", collinear);
	probe("duplicate", dup);
	probe("nan", nanv);
	probe("inf", infv);
	return 0;
}
