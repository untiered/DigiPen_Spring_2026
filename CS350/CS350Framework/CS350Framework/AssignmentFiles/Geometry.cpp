///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"


Vector3 ProjectPointOnPlane(const Vector3& point, const Vector3& normal, float planeDistance)
{
    // find distance to plane
    Vector3 pOnPlane = normal * planeDistance;

    // find distance to plane
    float distanceToPlane = (point - pOnPlane).Dot(normal);

    // then subtract that from the point
    return point - normal * distanceToPlane;
}

bool BarycentricCoordinates(const Vector3& point, const Vector3& a, const Vector3& b,
                            float& u, float& v, float epsilon)
{
    // check if degenerate
    if (a == b) return false;
 
    // calculate u and v
    u = (point - b).Dot(a - b) / (a - b).Dot(a - b);
    v = 1 - u;

    // check if it's within expanded bounds
    return (u >= -epsilon && u <= 1.0f + epsilon) && (v >= -epsilon && v <= 1.0f + epsilon);
}

bool BarycentricCoordinates(const Vector3& point, const Vector3& a, const Vector3& b, const Vector3& c,
                            float& u, float& v, float& w, float epsilon)
{
    // check for degenerate
    if (a == b || a == c || b == c || (a-b).Cross(a-c).Length() == 0.0f) return false;

    // calculate u,v, and w
    // calculate signed area of triangle: ABC
    Vector3 nABC = (c - a).Cross(b - a);
    float saABC = nABC.Dot(nABC.Normalized());

    // calculate signed area of triangle: PBC
    Vector3 nPBC = (c - point).Cross(b - point);
    float saPBC = nPBC.Dot(nABC.Normalized());

    // calculate signed area of triangle: PCA
    Vector3 nPCA = (a - point).Cross(c - point);
    float saPCA = nPCA.Dot(nABC.Normalized());

    u = saPBC / saABC;
    v = saPCA / saABC;
    w = 1 - u - v;

    // check if within triangle
    return (u >= -epsilon && u <= (1.0f + epsilon)
         && v >= -epsilon && v <= (1.0f + epsilon)
         && w >= -epsilon && w <= (1.0f + epsilon));
}

IntersectionType::Type PointPlane(const Vector3& point, const Vector4& plane, float epsilon)
{
    float d = point.Dot(Vector3(plane.x, plane.y, plane.z)) - plane.w;
    if (d <= epsilon && d >= -epsilon) return IntersectionType::Coplanar;
    if (d > epsilon) return IntersectionType::Inside;
    if (d < epsilon) return IntersectionType::Outside;
    return IntersectionType::NotImplemented;
}

bool PointSphere(const Vector3& point, const Vector3& sphereCenter, float sphereRadius)
{
    /******Student:Assignment1******/
    Warn("Assignment1: Required function un-implemented");
    return false;
    return Length(point - sphereCenter) <= sphereRadius;
}

bool PointAabb(const Vector3& point, const Vector3& aabbMin, const Vector3& aabbMax)
{
    /******Student:Assignment1******/
    Warn("Assignment1: Required function un-implemented");
    return false;
    return point.x >= aabbMin.x
        && point.y >= aabbMin.y
        && point.z >= aabbMin.z
        && point.x <= aabbMax.x
        && point.y <= aabbMax.y
        && point.z <= aabbMax.z;
}

bool RayPlane(const Vector3& rayStart, const Vector3& rayDir,
              const Vector4& plane, float& t, float epsilon)
{
  ++Application::mStatistics.mRayPlaneTests;
  /******Student:Assignment1******/
  Warn("Assignment1: Required function un-implemented");
  return false;
}

bool RayTriangle(const Vector3& rayStart, const Vector3& rayDir,
                 const Vector3& triP0, const Vector3& triP1, const Vector3& triP2,
                 float& t, float triExpansionEpsilon)
{
  ++Application::mStatistics.mRayTriangleTests;
  /******Student:Assignment1******/
  Warn("Assignment1: Required function un-implemented");
  return false;
}

bool RaySphere(const Vector3& rayStart, const Vector3& rayDir,
               const Vector3& sphereCenter, float sphereRadius,
               float& t)
{
  ++Application::mStatistics.mRaySphereTests;
  /******Student:Assignment1******/
  Warn("Assignment1: Required function un-implemented");
  return false;
}

bool RayAabb(const Vector3& rayStart, const Vector3& rayDir,
             const Vector3& aabbMin, const Vector3& aabbMax, float& t)
{
  ++Application::mStatistics.mRayAabbTests;
  /******Student:Assignment1******/
  Warn("Assignment1: Required function un-implemented");
  return false;
}

IntersectionType::Type PlaneTriangle(const Vector4& plane, 
                                     const Vector3& triP0, const Vector3& triP1, const Vector3& triP2,
                                     float epsilon)
{
  ++Application::mStatistics.mPlaneTriangleTests;
  /******Student:Assignment1******/
  Warn("Assignment1: Required function un-implemented");
  return IntersectionType::NotImplemented;
}

IntersectionType::Type PlaneSphere(const Vector4& plane,
                                   const Vector3& sphereCenter, float sphereRadius)
{
  ++Application::mStatistics.mPlaneSphereTests;
  /******Student:Assignment1******/
  Warn("Assignment1: Required function un-implemented");
  return IntersectionType::NotImplemented;
}

IntersectionType::Type PlaneAabb(const Vector4& plane,
                                 const Vector3& aabbMin, const Vector3& aabbMax)
{
  ++Application::mStatistics.mPlaneAabbTests;
  /******Student:Assignment1******/
  Warn("Assignment1: Required function un-implemented");
  return IntersectionType::NotImplemented;
}

IntersectionType::Type FrustumTriangle(const Vector4 planes[6],
                                       const Vector3& triP0, const Vector3& triP1, const Vector3& triP2,
                                       float epsilon)
{
  ++Application::mStatistics.mFrustumTriangleTests;
  /******Student:Assignment1******/
  Warn("Assignment1: Required function un-implemented");
  return IntersectionType::NotImplemented;
}

IntersectionType::Type FrustumSphere(const Vector4 planes[6],
                                     const Vector3& sphereCenter, float sphereRadius, size_t& lastAxis)
{
  ++Application::mStatistics.mFrustumSphereTests;
  /******Student:Assignment1******/
  Warn("Assignment1: Required function un-implemented");
  return IntersectionType::NotImplemented;
}

IntersectionType::Type FrustumAabb(const Vector4 planes[6],
                                   const Vector3& aabbMin, const Vector3& aabbMax, size_t& lastAxis)
{
  ++Application::mStatistics.mFrustumAabbTests;
  /******Student:Assignment1******/
  Warn("Assignment1: Required function un-implemented");
  return IntersectionType::NotImplemented;
}

bool SphereSphere(const Vector3& sphereCenter0, float sphereRadius0,
                  const Vector3& sphereCenter1, float sphereRadius1)
{
  ++Application::mStatistics.mSphereSphereTests;
  /******Student:Assignment1******/
  Warn("Assignment1: Required function un-implemented");
  return false;
}

bool AabbAabb(const Vector3& aabbMin0, const Vector3& aabbMax0,
              const Vector3& aabbMin1, const Vector3& aabbMax1)
{
  ++Application::mStatistics.mAabbAabbTests;
  /******Student:Assignment1******/
  Warn("Assignment1: Required function un-implemented");
  return false;
}
