///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"
#include <limits>



//-----------------------------------------------------------------------------SupportShape

// For GetCenter just compute the centroid of the point set.
Vector3 SupportShape::GetCenter(const std::vector<Vector3>& localPoints, const Matrix4& transform) const
{
	/******Student:Assignment5******/

	Vector3 centroid = Vector3::cZero;
	for (Vector3 const& point : localPoints) centroid += point;
	centroid /= static_cast<float>(localPoints.size());
	Vector4 center = Math::Transform(transform, Vector4(centroid.x, centroid.y, centroid.z, 1.0f));
	return Vector3(center.x, center.y, center.z);
}

// The Support function should just find the point furthest in the given direction with the applied transformation.
// When there is a tie you should pick the first point,
// that is only choose a new point if the distance is strictly greater than the old furthest distance.
// You should not transform every point into world space to test as this is less efficient!
// You should instead transform the search direction back into local space,
// do the search there, and then transform the resultant point into world space.
// Doing the correct local - space test will account for some points for this test!
Vector3 SupportShape::Support(const Vector3& worldDirection, const std::vector<Vector3>& localPoints, const Matrix4& localToWorldTransform) const
{
	Vector3 result = Vector3::cZero;
	/******Student:Assignment5******/

	Vector3 localDir = Math::Transform(Math::ToMatrix3(localToWorldTransform).Inverted(), worldDirection.Normalized());
	float dist = std::numeric_limits<float>::lowest();
	for (Vector3 const& point : localPoints)
	{
		float newDist = point.Dot(localDir);
		if (newDist > dist)
		{
			dist = newDist;
			result = point;
		}
	}
	Vector4 worldResult = Math::Transform(localToWorldTransform, Vector4(result.x, result.y, result.z, 1.0f));
	return Vector3(worldResult.x, worldResult.y, worldResult.z);
}

// The DebugDraw function should draw the point set with the given transform applied.
void SupportShape::DebugDraw(const std::vector<Vector3>& localPoints, const Matrix4& localToWorldTransform, const Vector4& color) const
{
	/******Student:Assignment5******/

	for (Vector3 const& point : localPoints)
	{
		Vector4 worldPoint = Math::Transform(localToWorldTransform, Vector4(point.x, point.y, point.z, 1.0f));
		DebugShape& debugShape = gDebugDrawer->DrawPoint(Vector3(worldPoint.x, worldPoint.y, worldPoint.z));
		debugShape.Color(color);
	}
}

//-----------------------------------------------------------------------------ModelSupportShape
Vector3 ModelSupportShape::GetCenter() const
{
	return SupportShape::GetCenter(mModel->mMesh->mVertices, mModel->mOwner->has(Transform)->GetTransform());
}

Vector3 ModelSupportShape::Support(const Vector3& worldDirection) const
{
	return SupportShape::Support(worldDirection, mModel->mMesh->mVertices, mModel->mOwner->has(Transform)->GetTransform());
}

void ModelSupportShape::DebugDraw(const Vector4& color) const
{
	SupportShape::DebugDraw(mModel->mMesh->mVertices, mModel->mOwner->has(Transform)->GetTransform());
}

//-----------------------------------------------------------------------------PointsSupportShape
PointsSupportShape::PointsSupportShape()
{
	mScale = Vector3(1);
	mRotation = Matrix3::cIdentity;
	mTranslation = Vector3::cZero;
}

Vector3 PointsSupportShape::GetCenter() const
{
	Matrix4 transform = Math::BuildTransform(mTranslation, mRotation, mScale);
	return SupportShape::GetCenter(mLocalSpacePoints, transform);
}

Vector3 PointsSupportShape::Support(const Vector3& worldDirection) const
{
	Matrix4 transform = Math::BuildTransform(mTranslation, mRotation, mScale);
	return SupportShape::Support(worldDirection, mLocalSpacePoints, transform);
}

void PointsSupportShape::DebugDraw(const Vector4& color) const
{
	Matrix4 transform = Math::BuildTransform(mTranslation, mRotation, mScale);
	SupportShape::DebugDraw(mLocalSpacePoints, transform, color);
}

//-----------------------------------------------------------------------------SphereSupportShape
Vector3 SphereSupportShape::GetCenter() const
{
	return mSphere.mCenter;
}

// Implement the optimized support function for a sphere.
// That is, find the point on the sphere furthest in the direction passed in.
Vector3 SphereSupportShape::Support(const Vector3& worldDirection) const
{
	/******Student:Assignment5******/
	
	return mSphere.mCenter + (worldDirection.Normalized() * mSphere.mRadius);
}

void SphereSupportShape::DebugDraw(const Vector4& color) const
{
	DebugShape& shape = gDebugDrawer->DrawSphere(mSphere);
	shape.Color(color);
}

//-----------------------------------------------------------------------------ObbSupportShape
Vector3 ObbSupportShape::GetCenter() const
{
	return mTranslation;
}

// Implement the optimized support function for an Obb.
// Do not just build up all 8 points and test them.
// Instead use the optimized function covered in class (in SAT slides).
// This basically amounts to transforming the search direction to local space,
// computing the support of an Aabb, then transforming back to world space.
Vector3 ObbSupportShape::Support(const Vector3& worldDirection) const
{
	/******Student:Assignment5******/

	// bring direction to local rotation
	Vector3 localDir = Math::Transform(mRotation.Transposed(), worldDirection.Normalized());

	// calculate point of local-space-aabb
	Vector3 localPoint = Vector3::cZero;
	localPoint.x = (localDir.x >= 0.0f) ? 0.5f : -0.5f;
	localPoint.y = (localDir.y >= 0.0f) ? 0.5f : -0.5f;
	localPoint.z = (localDir.z >= 0.0f) ? 0.5f : -0.5f;

	// bring that point back to world space
	Matrix4 transform = Math::BuildTransform(mTranslation, mRotation, mScale);
	Vector4 worldPoint = Math::Transform(transform, Vector4(localPoint.x, localPoint.y, localPoint.z, 1.0f));

	return Vector3(worldPoint.x, worldPoint.y, worldPoint.z);
}

void ObbSupportShape::DebugDraw(const Vector4& color) const
{
	Matrix4 transform = Math::BuildTransform(mTranslation, mRotation, mScale);
	DebugShape& shape = gDebugDrawer->DrawAabb(Aabb(Vector3(-0.5f), Vector3(0.5f)));
	shape.Color(color);
	shape.SetTransform(transform);
}


//------------------------------------------------------------ Voronoi Region Tests

// Point Test
VoronoiRegion::Type Gjk::IdentifyVoronoiRegion(
	const Vector3& q,
	const Vector3& p0,
	size_t& newSize,
	int newIndices[4],
	Vector3& closestPoint,
	Vector3& searchDirection
)
{
	/******Student:Assignment5******/

	newSize = 1;
	newIndices[0] = 0;
	closestPoint = p0;
	searchDirection = q - p0;

	return VoronoiRegion::Point0;
}

// Line Test
VoronoiRegion::Type Gjk::IdentifyVoronoiRegion(
	const Vector3& q,
	const Vector3& p0,
	const Vector3& p1,
	size_t& newSize,
	int newIndices[4],
	Vector3& closestPoint,
	Vector3& searchDirection
)
{
	/******Student:Assignment5******/

	closestPoint = FindClosestPoint(q, p0, p1);

	if (closestPoint != p0 && closestPoint != p1)
	{
		newIndices[0] = 0;
		newIndices[1] = 1;
		newSize = 2;
	} else {
		newIndices[0] = (closestPoint == p0) ? 0 : 1;
		newSize = 1;
	}

	searchDirection = q - closestPoint;

	if (closestPoint == p0) return VoronoiRegion::Point0;
	if (closestPoint == p1) return VoronoiRegion::Point1;
	return VoronoiRegion::Edge01;
}

// Triangle Test
VoronoiRegion::Type Gjk::IdentifyVoronoiRegion(
	const Vector3& q,
	const Vector3& p0,
	const Vector3& p1,
	const Vector3& p2,
	size_t& newSize,
	int newIndices[4],
	Vector3& closestPoint,
	Vector3& searchDirection
)
{
	/*
														 . 
												.    0 .
													 o
													.  .
												   .	 .
												  .		   .	    .
										   . 	 .			 .   .
												o  .  .  .  .  o
											  2	.			   . 1
												.			   .
	*/
	/******Student:Assignment5******/

	// collect line data
	Vector3 p01 = FindClosestPoint(q, p0, p1);
	Vector3 p12 = FindClosestPoint(q, p1, p2);
	Vector3 p20 = FindClosestPoint(q, p2, p0);

	newSize = 1;
	// check point regions
	if (p01 == p0 && p20 == p0)
	{
		newIndices[0] = 0;
		closestPoint = p0;
		searchDirection = q - closestPoint;
		return VoronoiRegion::Point0;
	}
	if (p01 == p1 && p12 == p1)
	{
		newIndices[0] = 1;
		closestPoint = p1;
		searchDirection = q - closestPoint;
		return VoronoiRegion::Point1;
	}
	if (p12 == p2 && p20 == p2)
	{
		newIndices[0] = 2;
		closestPoint = p2;
		searchDirection = q - closestPoint;
		return VoronoiRegion::Point2;
	}

	float u, v, w;
	if (BarycentricCoordinates(q, p0, p1, p2, u, v, w))
	{
		newSize = 3;
		newIndices[0] = 0;
		newIndices[1] = 1;
		newIndices[2] = 2;
		closestPoint = (p0*u) + (p1*v) + (p2*w);
		searchDirection = q - closestPoint;
		return VoronoiRegion::Triangle012;
	}

	newSize = 2;
	// check line regions
	if (w < 0.0f && p01 != p0 && p01 != p1)
	{
		newIndices[0] = 0;
		newIndices[1] = 1;
		closestPoint = p01;
		searchDirection = q - closestPoint;
		return VoronoiRegion::Edge01;
	}
	if (u < 0.0f && p12 != p1 && p12 != p2)
	{
		newIndices[0] = 1;
		newIndices[1] = 2;
		closestPoint = p12;
		searchDirection = q - closestPoint;
		return VoronoiRegion::Edge12;
	}
	if (v < 0.0f && p20 != p0 && p20 != p2)
	{
		newIndices[0] = 0;
		newIndices[1] = 2;
		closestPoint = p20;
		searchDirection = q - closestPoint;
		return VoronoiRegion::Edge02;
	}
	return VoronoiRegion::Unknown;
}

// Tetrahedron Test
VoronoiRegion::Type Gjk::IdentifyVoronoiRegion(
	const Vector3& q,
	const Vector3& p0,
	const Vector3& p1,
	const Vector3& p2,
	const Vector3& p3,
	size_t& newSize,
	int newIndices[4],
	Vector3& closestPoint,
	Vector3& searchDirection
)
{
	/******Student:Assignment5******/

	// collect point data
	Vector3 p01 = FindClosestPoint(q, p0, p1);
	Vector3 p02 = FindClosestPoint(q, p0, p2);
	Vector3 p03 = FindClosestPoint(q, p0, p3);
	Vector3 p12 = FindClosestPoint(q, p1, p2);
	Vector3 p23 = FindClosestPoint(q, p2, p3);
	Vector3 p31 = FindClosestPoint(q, p3, p1);

	newSize = 1;
	// test point regions
	if (p01 == p0 && p02 == p0 && p03 == p0)
	{
		newIndices[0] = 0;
		closestPoint = p0;
		searchDirection = q - closestPoint;
		return VoronoiRegion::Point0;
	}
	if (p01 == p1 && p12 == p1 && p31 == p1)
	{
		newIndices[0] = 1;
		closestPoint = p1;
		searchDirection = q - closestPoint;
		return VoronoiRegion::Point1;
	}
	if (p02 == p2 && p12 == p2 && p23 == p2)
	{
		newIndices[0] = 2;
		closestPoint = p2;
		searchDirection = q - closestPoint;
		return VoronoiRegion::Point2;
	}
	if (p03 == p3 && p23 == p3 && p31 == p3)
	{
		newIndices[0] = 3;
		closestPoint = p3;
		searchDirection = q - closestPoint;
		return VoronoiRegion::Point3;
	}

	// collect triangle data
	float u0, v0, w0;
	bool inTri0 = BarycentricCoordinates(q, p0, p1, p2, u0, v0, w0);
	float u1, v1, w1;
	bool inTri1 = BarycentricCoordinates(q, p0, p2, p3, u1, v1, w1);
	float u2, v2, w2;
	bool inTri2 = BarycentricCoordinates(q, p0, p3, p1, u2, v2, w2);
	float u3, v3, w3;
	bool inTri3 = BarycentricCoordinates(q, p1, p2, p3, u3, v3, w3);

	newSize = 2;
	// check line region 01
	if (w0 < 0.0f && v2 < 0.0f && p01 != p0 && p01 != p1)
	{
		newIndices[0] = 0;
		newIndices[1] = 1;
		closestPoint = p01;
		searchDirection = q - closestPoint;
		return VoronoiRegion::Edge01;
	}
	// check line region 02
	if (v0 < 0.0f && w1 < 0.0f && p02 != p0 && p02 != p2)
	{
		newIndices[0] = 0;
		newIndices[1] = 2;
		closestPoint = p02;
		searchDirection = q - closestPoint;
		return VoronoiRegion::Edge02;
	}
	// check line region 03
	if (v1 < 0.0f && w2 < 0.0f && p03 != p0 && p03 != p3)
	{
		newIndices[0] = 0;
		newIndices[1] = 3;
		closestPoint = p03;
		searchDirection = q - closestPoint;
		return VoronoiRegion::Edge03;
	}
	// check line region 12
	if (u0 < 0.0f && w3 < 0.0f && p12 != p1 && p12 != p2)
	{
		newIndices[0] = 1;
		newIndices[1] = 2;
		closestPoint = p12;
		searchDirection = q - closestPoint;
		return VoronoiRegion::Edge12;
	}
	// check line region 23
	if (u1 < 0.0f && u3 < 0.0f && p23 != p2 && p23 != p3)
	{
		newIndices[0] = 2;
		newIndices[1] = 3;
		closestPoint = p23;
		searchDirection = q - closestPoint;
		return VoronoiRegion::Edge23;
	}
	// check line region 31
	if (u2 < 0.0f && v3 < 0.0f && p31 != p1 && p31 != p3)
	{
		newIndices[0] = 1;
		newIndices[1] = 3;
		closestPoint = p31;
		searchDirection = q - closestPoint;
		return VoronoiRegion::Edge13;
	}

	// calculate barycentric coordinates for a tetra
	Vector3 AD = p0 - p3;
	Vector3 BD = p1 - p3;
	Vector3 CD = p2 - p3;
	Matrix3 matrix = Matrix3(
		AD.x, BD.x, CD.x,
		AD.y, BD.y, CD.y,
		AD.z, BD.z, CD.z
	);
	Vector3 uvw = Math::Transform(matrix.Inverted(), q - p3);
	float t = 1 - uvw.x - uvw.y - uvw.z;

	newSize = 3;
	// check triangle region 012
	if (t < 0.0f && inTri0)
	{
		newIndices[0] = 0;
		newIndices[1] = 1;
		newIndices[2] = 2;
		closestPoint = (p0 * u0) + (p1 * v0) + (p2 * w0);
		searchDirection = q - closestPoint;
		return VoronoiRegion::Triangle012;
	}
	// check triangle region 023
	if (uvw.y < 0.0f && inTri1)
	{
		newIndices[0] = 0;
		newIndices[1] = 2;
		newIndices[2] = 3;
		closestPoint = (p0 * u1) + (p2 * v1) + (p3 * w1);
		searchDirection = q - closestPoint;
		return VoronoiRegion::Triangle023;
	}
	// check triangle region 031
	if (uvw.z < 0.0f && inTri2)
	{
		newIndices[0] = 0;
		newIndices[1] = 1;
		newIndices[2] = 3;
		closestPoint = (p0 * u2) + (p3 * v2) + (p1 * w2);
		searchDirection = q - closestPoint;
		return VoronoiRegion::Triangle013;
	}
	// check triangle region 123
	if (uvw.x < 0.0f && inTri3)
	{
		newIndices[0] = 1;
		newIndices[1] = 2;
		newIndices[2] = 3;
		closestPoint = (p1 * u3) + (p2 * v3) + (p3 * w3);
		searchDirection = q - closestPoint;
		return VoronoiRegion::Triangle123;
	}
	
	newSize = 4;
	newIndices[0] = 0;
	newIndices[1] = 1;
	newIndices[2] = 2;
	newIndices[3] = 3;
	closestPoint = q;
	searchDirection = Vector3::cZero;
	return VoronoiRegion::Tetrahedra0123;
}

Gjk::Gjk()
{
}

bool Gjk::Intersect(
	const SupportShape* shapeA,
	const SupportShape* shapeB,
	unsigned int maxIterations,
	CsoPoint& closestPoint,
	float epsilon,
	
	// debugging parameters
	int debuggingIndex, bool debugDraw
)
{
	// When computing the initial search direction I used the difference of shape A’s center and shape B’s center.
	// If this produced the zero vector I used Vector3(-1, 0, 0) as the starting search direction.
	Vector3 searchDir = shapeA->GetCenter() - shapeB->GetCenter();
	if (searchDir == Vector3::cZero) searchDir = Vector3(-1.0f, 0.0f, 0.0f);

	// step 1. Initialize the simplex(to one point for us) by searching in a random direction(difference of centers)
	CsoPoint simplex[4] = {ComputeSupport(shapeA, shapeB, searchDir)};
	size_t simplexSize = 1;

	int newIndices[4] = {};
	Vector3 closestP = {};
	for (unsigned i = 0; i < maxIterations; ++i)
	{
		// a simplex exists, so check the closest point on the simplex:
		// while we calculate the closest point:
		//	1. determine voronoi region
		//	2. determine reduced simplex size and indices if reduction is possible
		//	3. update the search direction based on the closest point
		switch (simplexSize)
		{
		case 1: { IdentifyVoronoiRegion(Vector3::cZero, simplex[0].mCsoPoint, simplexSize, newIndices, closestP, searchDir); break; }
		case 2: { IdentifyVoronoiRegion(Vector3::cZero, simplex[0].mCsoPoint, simplex[1].mCsoPoint, simplexSize, newIndices, closestP, searchDir); break; }
		case 3: { IdentifyVoronoiRegion(Vector3::cZero, simplex[0].mCsoPoint, simplex[1].mCsoPoint, simplex[2].mCsoPoint, simplexSize, newIndices, closestP, searchDir); break; }
		case 4:   IdentifyVoronoiRegion(Vector3::cZero, simplex[0].mCsoPoint, simplex[1].mCsoPoint, simplex[2].mCsoPoint, simplex[3].mCsoPoint, simplexSize, newIndices, closestP, searchDir);
		}
		if (simplexSize >= 4 || closestP == Vector3::cZero) return true;

		// if the we have not enclosed the origin and the closest point on the simplex is not equal to the origin, we search for a new point
		//	1. search in the new search direction for another point to add to the simplex
		//	2. based on the new point, determine if intersection is possible
		//	3. if intersection is possible, reduce the simplex, add the new point to the simplex, and return to the top of the loop
		CsoPoint newCsoP = ComputeSupport(shapeA, shapeB, searchDir);
		if ((newCsoP.mCsoPoint - closestP).Dot(searchDir.Normalized()) <= epsilon) break;
		
 		CsoPoint temp[4] = {};
		for (unsigned j = 0; j < 4; ++j) temp[j] = simplex[j];
		for (unsigned j = 0; j < simplexSize; ++j) simplex[j] = temp[newIndices[j]];

		simplex[simplexSize++] = newCsoP;
	}

	// If the two shapes do not intersect, then you should fill out closestPoint with the closest features of the two shapes.
	FilloutFinalCso(simplex, simplexSize, closestP, closestPoint);
	return false;
}

Gjk::CsoPoint Gjk::ComputeSupport(const SupportShape* shapeA, const SupportShape* shapeB, const Vector3& direction)
{
	/******Student:Assignment5******/
	CsoPoint result = {};

	// cso point = Support(a,d) - Support(b,-d)
	result.mPointA = shapeA->Support(direction);
	result.mPointB = shapeB->Support(-direction);
	result.mCsoPoint = result.mPointA - result.mPointB;

	return result;
}

Vector3 Gjk::FindClosestPoint(Vector3 q, Vector3 s0, Vector3 s1)
{
	float u, v;
	BarycentricCoordinates(q, s0, s1, u, v);
	if (v <= 0) return s0;
	else if (u <= 0) return s1;
	return u * s0 + v * s1;
}

void Gjk::FilloutFinalCso(CsoPoint const simplex[4], size_t size, Vector3 const&closestSimplexPoint, CsoPoint &result)
{
	result.mCsoPoint = closestSimplexPoint;
	if (size == 1)
	{
		result = simplex[0];
	}
	else if (size == 2)
	{
		float u, v;
		BarycentricCoordinates(closestSimplexPoint, simplex[0].mCsoPoint, simplex[1].mCsoPoint, u, v);
		result.mPointA = simplex[0].mPointA * u + simplex[1].mPointA * v;
		result.mPointB = simplex[0].mPointB * u + simplex[1].mPointB * v;
	}
	else if (size == 3)
	{
		float u, v, w;
		BarycentricCoordinates(closestSimplexPoint, simplex[0].mCsoPoint, simplex[1].mCsoPoint, simplex[2].mCsoPoint, u, v, w);
		result.mPointA = simplex[0].mPointA * u + simplex[1].mPointA * v + simplex[2].mPointA * w;
		result.mPointB = simplex[0].mPointB * u + simplex[1].mPointB * v + simplex[2].mPointB * w;
	}
}
