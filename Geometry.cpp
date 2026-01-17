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
    return (point - sphereCenter).Length() <= sphereRadius;
}

bool PointAabb(const Vector3& point, const Vector3& aabbMin, const Vector3& aabbMax)
{
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

    Vector3 normal = Vector3(plane.x, plane.y, plane.z);
    float n_rayDir_DotNrm = rayDir.Dot(normal);
    if (n_rayDir_DotNrm < -epsilon || n_rayDir_DotNrm > epsilon) // parallel check
    {
        Vector3 p0 = normal * plane.w;
        t = (p0 - rayStart).Dot(normal) / n_rayDir_DotNrm; // check distance to intersection
        return t >= 0.0f;
    }
    return false;
}

bool RayTriangle(const Vector3& rayStart, const Vector3& rayDir,
                 const Vector3& triP0, const Vector3& triP1, const Vector3& triP2,
                 float& t, float triExpansionEpsilon)
{
    ++Application::mStatistics.mRayTriangleTests;

    // build a plane
    Plane plane;
    plane.Set(triP0, triP1, triP2);
    float tempT = 0.0f;

    // check if its on the plane
    if (!RayPlane(rayStart, rayDir, plane.mData, tempT)) return false;
    
    // check if its in the triangle
    float u, v, w;
    if (BarycentricCoordinates(rayStart + (rayDir.Normalized() * t), triP0, triP1, triP2, u, v, w, triExpansionEpsilon))
    {
        t = tempT;
        return true;
    }
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

    // check if ray starts inside
    if ((rayStart.x < aabbMax.x && rayStart.x > aabbMin.x) &&
        (rayStart.y < aabbMax.y && rayStart.y > aabbMin.y) &&
        (rayStart.z < aabbMax.z && rayStart.z > aabbMin.z)) {
        t = 0.0f;
        return true;
    }

    // record all tmin's and tmax's
    float mins[3] = { 0.0f };
    float maxes[3] = { 0.0f };

    // loop through the 3 axes: x -> y -> z
    for (int i = 0; i < 3; ++i)
    {
        // create the plane's normalized normal
        Vector3 n_normal = Vector3(0.0f);
        n_normal[i] = 1.0f;

        // checks
        float dir = rayDir.Dot(n_normal);
        if (dir == 0.0f)
        {
            if (rayStart[i] > aabbMax[i] || rayStart[i] < aabbMin[i]) return false;
            // do this to avoid problems when determining the FINAL min and max
            mins[i] = std::numeric_limits<float>::min();
            maxes[i] = std::numeric_limits<float>::max();
            continue;
        }

        if (dir > 0.0f)
        {
            // calculate tmin and tmax for the current axis
            mins[i] = (aabbMin[i] - rayStart[i]) / rayDir[i];
            maxes[i] = (aabbMax[i] - rayStart[i]) / rayDir[i];
        } else {
            // calculate tmin and tmax for the current axis
            mins[i] = (aabbMax[i] - rayStart[i]) / rayDir[i];
            maxes[i] = (aabbMin[i] - rayStart[i]) / rayDir[i];
        }
    }

    for (int i = 0; i < 3; ++i)
    {
        if (mins[i] < 0.0f && maxes[i] < 0.0f) return false;
    }

    // find tmin and tmax
    float tmin = Math::Max(Math::Max(mins[0], mins[1]), mins[2]);
    float tmax = Math::Min(Math::Min(maxes[0], maxes[1]), maxes[2]);

    // check for intersection
    if (tmin > tmax) return false;
    t = tmin;
    return true;
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
    return PointSphere(sphereCenter1, sphereCenter0, sphereRadius0 + sphereRadius1);
}

bool AabbAabb(const Vector3& aabbMin0, const Vector3& aabbMax0,
              const Vector3& aabbMin1, const Vector3& aabbMax1)
{
    ++Application::mStatistics.mAabbAabbTests;

    if (aabbMax0.x < aabbMin1.x || aabbMax1.x < aabbMin0.x) return false;
    if (aabbMax0.y < aabbMin1.y || aabbMax1.y < aabbMin0.y) return false;
    if (aabbMax0.z < aabbMin1.z || aabbMax1.z < aabbMin0.z) return false;
    return true;
}
