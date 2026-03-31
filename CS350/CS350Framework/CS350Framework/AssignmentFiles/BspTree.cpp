///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Roman's Stuff

void UpdateLists(
	Plane const& plane,
	std::vector<Vector3>& frontPoints,
	std::vector<Vector3>& backPoints,
	Triangle const& tri,
	int frontIndex, int backIndex,
	bool intersect, unsigned AI
)
{
	// order to add
	// 1st: intersection points
	if (intersect)
	{
		// calculate the point
		Vector3 A = tri.mPoints[AI];
		Vector3 B = ((frontIndex == -1) ? tri.mPoints[backIndex] : tri.mPoints[frontIndex]);
		float t = 0.0f;
		RayPlane(B, (A - B).Normalized(), plane.mData, t); // from B to A
		Vector3 intersection = B + ((A - B).Normalized() * t);
		frontPoints.push_back(intersection); // add it to front
		backPoints.push_back(intersection); // add it to back
	}

	// 2nd: triangle points
	if (frontIndex != -1) { frontPoints.push_back(tri.mPoints[frontIndex]); } // add it
	if (backIndex != -1) { backPoints.push_back(tri.mPoints[backIndex]); } // add it
}

void BuildAndPushTriangles(
	TriangleList& front,
	TriangleList& back,
	std::vector<Vector3>& frontPoints,
	std::vector<Vector3>& backPoints
)
{
	if (frontPoints.size() > 3) // front has 4, back has 3
	{
		front.push_back(Triangle(frontPoints[0], frontPoints[1], frontPoints[2]));
		front.push_back(Triangle(frontPoints[0], frontPoints[2], frontPoints[3]));
		back.push_back(Triangle(backPoints[0], backPoints[1], backPoints[2]));
	}
	else if (backPoints.size() > 3) { // front has 3, back has 4
		back.push_back(Triangle(backPoints[0], backPoints[1], backPoints[2]));
		back.push_back(Triangle(backPoints[0], backPoints[2], backPoints[3]));
		front.push_back(Triangle(frontPoints[0], frontPoints[1], frontPoints[2]));
	}
	else { // both have 3
		back.push_back(Triangle(backPoints[0], backPoints[1], backPoints[2]));
		front.push_back(Triangle(frontPoints[0], frontPoints[1], frontPoints[2]));
	}
}
// End of Roman's stuff
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

BspTreeQueryData::BspTreeQueryData()
{
	mDepth = 0;
}

// Triangle splitting must be performed with the robust method described in class (the table with 9 states).
// That is, you must properly handle going between all permutations of Inside, Outside, and Coplanar points.
// Note that I do not want you to try and handle the edge clipping order(A to B vs. B to A) for this assignment.
// You should only attempt to use the table if the triangle is overlapping the split plane,
// if it is classified as either being in front or behind the plane then just insert it into the corresponding array.
// If the triangle is coplanar then use the normal of the triangle to determine if it belongs to
// coplanar back or front(front if the normal points the same direction as the split plane).
// Also, when the result from a split is a quad, turn the quad into two triangles of the indices[0, 1, 2] and [0, 2, 3].
void BspTree::SplitTriangle(
	const Plane& plane,
	const Triangle& tri,
	TriangleList& coplanarFront,
	TriangleList& coplanarBack,
	TriangleList& front,
	TriangleList& back,
	float epsilon
)
{
	IntersectionType::Type type = PlaneTriangle(plane.mData, tri.mPoints[0], tri.mPoints[1], tri.mPoints[2], epsilon);

	switch (type)
	{
	case(IntersectionType::Inside):
		front.push_back(tri);
		break;
	case(IntersectionType::Outside):
		back.push_back(tri);
		break;
	case(IntersectionType::Coplanar):
	{
		// use normal
		Vector3 normal = (tri.mPoints[2] - tri.mPoints[0]).Cross(tri.mPoints[1] - tri.mPoints[0]);
		if (normal.Dot(plane.GetNormal()) >= 0.0f)
		{
			coplanarBack.push_back(tri);
		} else {
			coplanarFront.push_back(tri);
		}
		break;
	}
	case(IntersectionType::Overlaps):
	{
		// use table
		IntersectionType::Type types[3];
		types[0] = PointPlane(tri.mPoints[0], plane.mData, epsilon);
		types[1] = PointPlane(tri.mPoints[1], plane.mData, epsilon);
		types[2] = PointPlane(tri.mPoints[2], plane.mData, epsilon);

		std::vector<Vector3> frontPoints;
		std::vector<Vector3> backPoints;

		// at the end of this loop, frontPoints and backPoints will have all the points belonging to their respective sides in order.
		for (int i = 0; i < 3; ++i)
		{
			// get A [0, 1, 2]
			unsigned AI = i;
			IntersectionType::Type A = types[AI];
			// get B [1, 2, 0]
			unsigned BI = (i + 1) % 3;
			IntersectionType::Type B = types[BI];

			if (A == IntersectionType::Inside) { // A is front
				if (B == IntersectionType::Outside) { UpdateLists(plane, frontPoints, backPoints, tri, -1, BI, true, AI); } // B is back
				else { UpdateLists(plane, frontPoints, backPoints, tri, BI, -1, false, AI); } // B is coplanar or in front
			}
			else if (A == IntersectionType::Outside) { // A is back
				if (B == IntersectionType::Inside) { UpdateLists(plane, frontPoints, backPoints, tri, BI, -1, true, AI); } // B is front
				else if (B == IntersectionType::Outside) { UpdateLists(plane, frontPoints, backPoints, tri, -1, BI, false, AI); } // B is back
				else { UpdateLists(plane, frontPoints, backPoints, tri, BI, BI, false, AI); } // B is coplanar
			}
			else { // A is coplanar
				if (B == IntersectionType::Outside) { // B is back
					UpdateLists(plane, frontPoints, backPoints, tri, -1, AI, false, AI);
					UpdateLists(plane, frontPoints, backPoints, tri, -1, BI, false, AI);
				}
				else { UpdateLists(plane, frontPoints, backPoints, tri, BI, -1, false, AI); }// B is coplanar or in front
			}
		}
		// after this loop all I need to do is check the points in front and back of the plane,
		// and build valid triangles to push to the lists.

		BuildAndPushTriangles(front, back, frontPoints, backPoints);
	}
	}
}

float BspTree::CalculateScore(const TriangleList& triangles, size_t testIndex, float k, float epsilon)
{
	/******Student:Assignment4******/
	Warn("Assignment4: Required function un-implemented");
	return Math::PositiveMax();
}

size_t BspTree::PickSplitPlane(const TriangleList& triangles, float k, float epsilon)
{
	/******Student:Assignment4******/
	Warn("Assignment4: Required function un-implemented");
	return 0;
}

void BspTree::Construct(const TriangleList& triangles, float k, float epsilon)
{
	/******Student:Assignment4******/
	Warn("Assignment4: Required function un-implemented");
}

bool BspTree::RayCast(const Ray& ray, float& t, float planeEpsilon, float triExpansionEpsilon, int debuggingIndex)
{
	/******Student:Assignment4******/
	Warn("Assignment4: Required function un-implemented");
	t = Math::PositiveMax();
	return false;
}

void BspTree::AllTriangles(TriangleList& triangles) const
{
	/******Student:Assignment4******/
	Warn("Assignment4: Required function un-implemented");
}

void BspTree::Invert()
{
	/******Student:Assignment4******/
	Warn("Assignment4: Required function un-implemented");
}

void BspTree::ClipTo(BspTree* tree, float epsilon)
{
	/******Student:Assignment4******/
	Warn("Assignment4: Required function un-implemented");
}

void BspTree::Union(BspTree* tree, float k, float epsilon)
{
	/******Student:Assignment4******/
	Warn("Assignment4: Required function un-implemented");
}

void BspTree::Intersection(BspTree* tree, float k, float epsilon)
{
	/******Student:Assignment4******/
	Warn("Assignment4: Required function un-implemented");
}

void BspTree::Subtract(BspTree* tree, float k, float epsilon)
{
	/******Student:Assignment4******/
	Warn("Assignment4: Required function un-implemented");
}

void BspTree::FilloutData(std::vector<BspTreeQueryData>& results) const
{
	/******Student:Assignment4******/
	Warn("Assignment4: Required function un-implemented");
}

void BspTree::DebugDraw(int level, const Vector4& color, int bitMask)
{
	/******Student:Assignment4******/
	Warn("Assignment4: Required function un-implemented");
}

