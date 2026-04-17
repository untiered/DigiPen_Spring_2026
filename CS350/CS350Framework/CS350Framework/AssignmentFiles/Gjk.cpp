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
	return centroid / static_cast<float>(localPoints.size());
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

	Vector4 localDir = Math::Transform(localToWorldTransform.Inverted(), Vector4(worldDirection.x, worldDirection.y, worldDirection.z, 0.0f));
	float dist = std::numeric_limits<float>::lowest();
	for (Vector3 const& point : localPoints)
	{
		float newDist = point.Dot(Vector3(localDir.x, localDir.y, localDir.z));
		if (newDist > dist)
		{
			dist = newDist;
			result = point;
		}
	}
	return result;
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
	
	return mSphere.mCenter + (worldDirection * mSphere.mRadius);;
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

	// bring direction to local space
	Vector3 localDir = Math::Transform(mRotation.Transposed(), worldDirection);

	// calculate point of local-space-aabb
	Vector3 localPoint = Vector3::cZero;
	localPoint.x = (localDir.x >= 0.0f) ? mScale.x : -mScale.x;
	localPoint.y = (localDir.y >= 0.0f) ? mScale.y : -mScale.y;
	localPoint.z = (localDir.z >= 0.0f) ? mScale.z : -mScale.z;

	// bring that point back to world space
	Matrix4 transform = Math::BuildTransform(mTranslation, mRotation, Vector3(1.0f, 1.0f, 1.0f)); // ALREADY SCALED
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
	/******Student:Assignment5******/
	Warn("Assignment5: Required function un-implemented");
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
	Warn("Assignment5: Required function un-implemented");
	return VoronoiRegion::Unknown;
}

Gjk::Gjk()
{
}

bool Gjk::Intersect(const SupportShape* shapeA, const SupportShape* shapeB, unsigned int maxIterations, CsoPoint& closestPoint, float epsilon, int debuggingIndex, bool debugDraw)
{
	Warn("Assignment5: Required function un-implemented");
	return false;
}

Gjk::CsoPoint Gjk::ComputeSupport(const SupportShape* shapeA, const SupportShape* shapeB, const Vector3& direction)
{
	/******Student:Assignment5******/
	CsoPoint result = {};
	Warn("Assignment5: Required function un-implemented");

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
