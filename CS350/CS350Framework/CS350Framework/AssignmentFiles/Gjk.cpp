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
  for (Vector3 const&point : localPoints) centroid += point;
  return centroid / localPoints.size();
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
	  if (newDist > dist) dist = newDist;

  }

  return result;
}

void SupportShape::DebugDraw(const std::vector<Vector3>& localPoints, const Matrix4& localToWorldTransform, const Vector4& color) const
{
  /******Student:Assignment5******/
  Warn("Assignment5: Required function un-implemented");
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

Vector3 SphereSupportShape::Support(const Vector3& worldDirection) const
{
  /******Student:Assignment5******/
  Warn("Assignment5: Required function un-implemented");
  return Vector3::cZero;
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

Vector3 ObbSupportShape::Support(const Vector3& worldDirection) const
{
  /******Student:Assignment5******/
  Warn("Assignment5: Required function un-implemented");
  return Vector3::cZero;
}

void ObbSupportShape::DebugDraw(const Vector4& color) const
{
  Matrix4 transform = Math::BuildTransform(mTranslation, mRotation, mScale);
  DebugShape& shape = gDebugDrawer->DrawAabb(Aabb(Vector3(-0.5f), Vector3(0.5f)));
  shape.Color(color);
  shape.SetTransform(transform);
}


//------------------------------------------------------------ Voronoi Region Tests
VoronoiRegion::Type Gjk::IdentifyVoronoiRegion(const Vector3& q, const Vector3& p0,
  size_t& newSize, int newIndices[4],
  Vector3& closestPoint, Vector3& searchDirection)
{
  /******Student:Assignment5******/
  Warn("Assignment5: Required function un-implemented");
  return VoronoiRegion::Unknown;
}

VoronoiRegion::Type Gjk::IdentifyVoronoiRegion(const Vector3& q, const Vector3& p0, const Vector3& p1,
  size_t& newSize, int newIndices[4],
  Vector3& closestPoint, Vector3& searchDirection)
{
  /******Student:Assignment5******/
  Warn("Assignment5: Required function un-implemented");
  return VoronoiRegion::Unknown;
}

VoronoiRegion::Type Gjk::IdentifyVoronoiRegion(const Vector3& q, const Vector3& p0, const Vector3& p1, const Vector3& p2,
  size_t& newSize, int newIndices[4],
  Vector3& closestPoint, Vector3& searchDirection)
{
  /******Student:Assignment5******/
  Warn("Assignment5: Required function un-implemented");
  return VoronoiRegion::Unknown;
}

VoronoiRegion::Type Gjk::IdentifyVoronoiRegion(const Vector3& q, const Vector3& p0, const Vector3& p1, const Vector3& p2, const Vector3& p3,
  size_t& newSize, int newIndices[4],
  Vector3& closestPoint, Vector3& searchDirection)
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
