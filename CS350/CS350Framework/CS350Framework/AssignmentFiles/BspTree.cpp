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

// Use the method for calculating score as described in class :
//		s𝑐𝑜𝑟𝑒 = 𝐾∗𝑁𝑠 + (1−𝐾)∗𝐴𝑏𝑠(𝑁𝑓 −𝑁𝑏),
// where 𝑁𝑠 is the number of triangles straddling the plane, 𝑁𝑓 is the number in front, and 𝑁𝑏 is the number behind.
// Build the split plane from the triangle at the testIndex.
//		Note : Do not include coplanar triangles in the score!
// Also, be careful of degenerate triangles causing degenerate plane normals.
// You should return Math::PositiveMax() if the triangle is degenerate.
// Finally, in the case of a tie in score choose the first triangle of that score(from 0 to size).
float BspTree::CalculateScore(const TriangleList& triangles, size_t testIndex, float k, float epsilon)
{
	Triangle triangle = triangles[testIndex];

	// check if triangle is degenerate
	Vector3 normal = (triangle.mPoints[2] - triangle.mPoints[0]).Cross(triangle.mPoints[1] - triangle.mPoints[0]);
	if (normal.Length() <= epsilon) return Math::PositiveMax();

	Plane plane = Plane(normal, triangle.mPoints[0]);

	int Ns = 0;
	int Nf = 0;
	int Nb = 0;
	for (Triangle const&tri : triangles)
	{
		IntersectionType::Type type = PlaneTriangle(plane.mData, tri.mPoints[0], tri.mPoints[1], tri.mPoints[2], epsilon);
		switch (type)
		{
		case (IntersectionType::Inside):
			++Nf;
			break;
		case (IntersectionType::Outside):
			++Nb;
			break;
		case (IntersectionType::Overlaps):
			++Ns;
		}
	}

	return (k * Ns) + ((1 - k) * Math::Abs(Nf - Nb));
}

// Simply choose the triangle that produces the lowest score.
size_t BspTree::PickSplitPlane(const TriangleList& triangles, float k, float epsilon)
{
	/******Student:Assignment4******/
	float lowestScore = Math::PositiveMax();
	size_t lowestScoreIndex = 0; // TODO: make sure we dont need to ensure that a valid plane was found. if we do, this should be able to convey failure.
	for (size_t i = 0; i < triangles.size(); ++i)
	{
		float newScore = CalculateScore(triangles, i, k, epsilon);
		if (newScore < lowestScore) // if its equal, dont record it, ties should result in us choosing the first triangle with that score
		{
			lowestScore = newScore;
			lowestScoreIndex = i;
		}
	}
	return lowestScoreIndex;
}

void BspTree::Construct(const TriangleList& triangles, float k, float epsilon)
{
	/******Student:Assignment4******/
	if (!m_root)
	{
		m_root = new BSPNode();
	}
	RecursiveConstruct(m_root, triangles, k, epsilon);
}

bool BspTree::RayCast(const Ray& ray, float& t, float planeEpsilon, float triExpansionEpsilon, int debuggingIndex)
{
	/******Student:Assignment4******/
	t = Math::PositiveMax();
	RecursiveRayCast(m_root, ray, 0.0f, Math::PositiveMax(), t, planeEpsilon, triExpansionEpsilon);
	return t >= 0.0f && t != Math::PositiveMax();
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
	RecursiveFillOut(m_root, results, 0);
}

void BspTree::DebugDraw(int level, const Vector4& color, int bitMask)
{
	/******Student:Assignment4******/
	Warn("Assignment4: Required function un-implemented");
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

// Recursively build the tree by splitting the data set with the best scoring triangle’s plane.
// Recursion should stop when a there is only 1 triangle remaining.
// Also note that this is a Node Storing tree, that is all coplanar triangles should be stored in the node.
//		Important!
//			Make sure to not pick a split - plane that is almost the zero vector.
//			If you fail to do this you can end up with every triangle being coplanar.
//			For this assignment, if the normal’s length is below the construction epsilon then you should not choose that plane.
void BspTree::RecursiveConstruct(BSPNode *node, TriangleList const& triangles, float k, float epsilon)
{
	// base case
	if (triangles.size() == 1)
	{
		node->m_splitTri = triangles[0];
		node->m_splitPlane = Plane(
			node->m_splitTri.mPoints[0],
			node->m_splitTri.mPoints[1],
			node->m_splitTri.mPoints[2]
		);
		return;
	}

	// create split plane
	size_t spi = PickSplitPlane(triangles, k, epsilon);
	node->m_splitTri = triangles[spi]; // TODO: i am assuming PickSplitPlane() always returns a valid result
	node->m_splitPlane = Plane(
		node->m_splitTri.mPoints[0],
		node->m_splitTri.mPoints[1],
		node->m_splitTri.mPoints[2]
	);
	
	// make a triangle list of all triangles in front
	TriangleList front;
	// make a triangle list of all triangles in coplanarFront
	TriangleList back;
	
	// split the triangles
	for (size_t i = 0; i < triangles.size(); ++i)
	{
		if (i == spi) continue;
		SplitTriangle(
			node->m_splitPlane,
			triangles[i],
			node->m_coplanerFront,
			node->m_coplanerBack,
			front,
			back,
			epsilon
		);
	}
	
	// recurse
	if (back.size() >= 1)
	{
		BSPNode* left = new BSPNode(); // should hold the "back" triangles
		node->m_left = left;
		RecursiveConstruct(left, back, k, epsilon);
	}
	if (front.size() >= 1)
	{
		BSPNode* right = new BSPNode(); // should hold the "front" triangles
		node->m_right = right;
		RecursiveConstruct(right, front, k, epsilon);
	}
}

void BspTree::RecursiveFillOut(BSPNode* node, std::vector<BspTreeQueryData>& results, unsigned depth) const
{
	if (!node) return;

	BspTreeQueryData result;
	result.mDepth = depth;
	TriangleList list;

	// concatenate coplanar triangle lists
	list.insert(list.end(), node->m_coplanerBack.begin(), node->m_coplanerBack.end());
	list.insert(list.end(), node->m_coplanerFront.begin(), node->m_coplanerFront.end());
	list.push_back(node->m_splitTri);
	result.mTriangles = list;
	results.push_back(result);

	RecursiveFillOut(node->m_right, results, depth+1);
	RecursiveFillOut(node->m_left, results, depth+1);
}

// Optimized ray - casting should be implemented as described in class (using tMin and tMax).
// Do not implement the “basic traversal” but rather the 4 main cases with the 3 edge cases.
// Make sure that you only check the geometry in the plane when the ray hits the thick plane
// otherwise you will end up with more PlaneTriangle tests.
// Use planeEpsilon to classify the ray’s start.
// Use triExpansionEpsilon for RayTriangle, if you don’t use this then some tests will slip between two triangles.
// Be careful as there are a lot of edge cases with raycasting(all should have a unit test).
bool BspTree::RecursiveRayCast(BSPNode* node, const Ray& ray, float tMin, float tMax, float& t, float planeEpsilon, float triExpansionEpsilon)
{
	if (!node) return false;

	// before calling RayPlane() we must check edge case 1
	Vector3 rayStart = ray.mStart + (ray.mDirection * tMin);
	bool coplanar = false;
	if (PointPlane(rayStart, node->m_splitPlane.mData, planeEpsilon) == IntersectionType::Coplanar)
	{
		coplanar = true;
		CheckCoplanarGeometry(node, rayStart, ray, t, triExpansionEpsilon);
		if (!node->m_left && !node->m_right) return false; // early-out if we are on a leaf
	}

	// check the ray against the splitplane
	float tPlane = Math::PositiveMax();
	if (RayPlane(ray.mStart, ray.mDirection, node->m_splitPlane.mData, tPlane)) // use the default epsilon value
	{
		// the ray hits the splitplane, if we get here, tPlane is garanteed to be set and greater than 0

		// TODO: THIS NEEDS TO BE IMPLEMENTED!!!
		// edge case 3:
		//	- clip correctly (set the new tMin and tMax)
		//	- check for intersection against the thick planes

		if (tPlane < tMin) return RayCast_Case_4(node, ray, tMin, tMax, t, planeEpsilon, triExpansionEpsilon, tPlane); // this swaps places depending on whether or not we use an adjusted rayStart
		if (tPlane > tMax) return RayCast_Case_3(node, ray, tMin, tMax, t, planeEpsilon, triExpansionEpsilon, tPlane);
		return RayCast_Case_1(node, ray, tMin, tMax, t, planeEpsilon, triExpansionEpsilon, tPlane, coplanar);
	}
	else
	{
		// the ray is either parallel or pointing away from the splitplane
		// if we get here, tPlane might NOT have been set, but if it was, its less than 0

		// negative:
		if (tPlane < 0.0f) return RayCast_Case_2(node, ray, tMin, tMax, t, planeEpsilon, triExpansionEpsilon, tPlane);

		// parallel:
		return RayCast_EdgeCase_2(node, ray, tMin, tMax, t, planeEpsilon, triExpansionEpsilon, tPlane);
	}
	return false;
}

void BspTree::GetSides(BSPNode* node, BSPNode** nearSide, BSPNode** farSide, Vector3 rayStart)
{
	IntersectionType::Type type = PointPlane(rayStart, node->m_splitPlane.mData, 0.0f); // TODO: magic number epsilon???
	if (type == IntersectionType::Inside) // front
	{
		// right holds front
		*nearSide = node->m_right;
		*farSide = node->m_left;
	}
	else if (type == IntersectionType::Outside) // back
	{
		// left holds back
		*nearSide = node->m_left;
		*farSide = node->m_right;
	}
}

// case 1: the clipped ray hits the splitplane
//	- tMin <= tPlane <= tMax
//	- recurse down the near side
//	- then recurse through the coplanar triangles
//	- then recurse down the far side
//	- update tMin and tMax during recursions
bool BspTree::RayCast_Case_1(BSPNode* node, const Ray& ray, float tMin, float tMax, float& t, float planeEpsilon, float triExpansionEpsilon, float tPlane, bool coplanar)
{
	BSPNode* nearSide = nullptr;
	BSPNode* farSide = nullptr;
	GetSides(node, &nearSide, &farSide, ray.mStart + (ray.mDirection * tMin));

	// recurse near side with tPlane as the new tMax
	RecursiveRayCast(nearSide, ray, tMin, tPlane, t, planeEpsilon, triExpansionEpsilon);

	// check coplanar geometry if not already checked
	if (!coplanar) CheckCoplanarGeometry(node, ray.mStart + (ray.mDirection * tMin), ray, t, triExpansionEpsilon);

	// recurse far side with tPlane as the new tMin
	RecursiveRayCast(farSide, ray, tPlane, tMax, t, planeEpsilon, triExpansionEpsilon);
	return false;
}

// case 2: splitplane is behind the original rayStart
//	- tPlane < 0
//	- traverse the near side
//	- update tMin and tMax during recursions
bool BspTree::RayCast_Case_2(BSPNode* node, const Ray& ray, float tMin, float tMax, float& t, float planeEpsilon, float triExpansionEpsilon, float tPlane)
{
	return false;

	if (Math::Abs(tPlane) <= planeEpsilon) CheckCoplanarGeometry(node, ray.mStart, ray, t, triExpansionEpsilon); // edge case 1
	// then do stuff
	return false;
}

// case 3: splitplane is beyond tMax
//	- tMax < tPlane
//	- traverse the near side
//	- update tMin and tMax during recursions
bool BspTree::RayCast_Case_3(BSPNode* node, const Ray& ray, float tMin, float tMax, float& t, float planeEpsilon, float triExpansionEpsilon, float tPlane)
{
	return false;

	// do stuff
	if (Math::Abs(tPlane) <= planeEpsilon) CheckCoplanarGeometry(node, ray.mStart, ray, t, triExpansionEpsilon); // edge case 1
	return false;
}

// case 4: splitplane is between the actual ray start and the adjusted ray start
//	- 0 < tPlane < tMin
//	- only traverse the far side
//	- update tMin and tMax during recursions
bool BspTree::RayCast_Case_4(BSPNode* node, const Ray& ray, float tMin, float tMax, float& t, float planeEpsilon, float triExpansionEpsilon, float tPlane)
{
	return false;

	if (Math::Abs(tPlane) <= planeEpsilon) CheckCoplanarGeometry(node, ray.mStart, ray, t, triExpansionEpsilon); // edge case 1
	// then do stuff
	return false;
}

// edge case 1: the rayStart is coplanar with the splitplane
//	- visit both sides of the geometry in the plane
//	- we must deal with this before testing edge case 2!!!
void BspTree::CheckCoplanarGeometry(BSPNode* node, Vector3 const&rayStart, const Ray& ray, float& t, float triExpansionEpsilon)
{
	float dist = Math::PositiveMax();
	for (Triangle const& tri : node->m_coplanerBack)
	{
		if (RayTriangle(rayStart, ray.mDirection, tri.mPoints[0], tri.mPoints[1], tri.mPoints[2], dist, triExpansionEpsilon))
		{
			if (dist > 0.0f && dist < t) t = dist;
		}
	}
	for (Triangle const& tri : node->m_coplanerBack)
	{
		if (RayTriangle(rayStart, ray.mDirection, tri.mPoints[0], tri.mPoints[1], tri.mPoints[2], dist, triExpansionEpsilon))
		{
			if (dist > 0.0f && dist < t) t = dist;
		}
	}
	if (RayTriangle(rayStart, ray.mDirection, node->m_splitTri.mPoints[0], node->m_splitTri.mPoints[1], node->m_splitTri.mPoints[2], dist, triExpansionEpsilon))
	{
		if (dist > 0.0f && dist < t) t = dist;
	}
}

// edge case 2: the ray is parallel to the splitplane
//	- only traverse the near side
//	- assumes we have already dealt with edge case 1!!!
bool BspTree::RayCast_EdgeCase_2(BSPNode* node, const Ray& ray, float tMin, float tMax, float& t, float planeEpsilon, float triExpansionEpsilon, float tPlane)
{
	if (!node->m_left && !node->m_right) return false; // we know edge case 1 was already dealt with
	
	BSPNode* nearSide = nullptr;
	BSPNode* farSide = nullptr;
	GetSides(node, &nearSide, &farSide, ray.mStart + (ray.mDirection * tMin));
	return RecursiveRayCast(nearSide, ray, tMin, tMax, t, planeEpsilon, triExpansionEpsilon);
}

