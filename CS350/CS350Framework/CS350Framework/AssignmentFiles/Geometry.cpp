///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////

/* Start Header ------------------------------------------------------
Copyright (C) 2025 DigiPen Institute of Technology.
File Name: Geometry.cpp
Purpose: This file provides an implementation for testing various types of collisions between primitive shapes.
Language: ISO C++ 14 Standard.
Platform: Legacy MSVC, x64/x86, Windows 10 OS.
Project: r.marqueztwisdale_CS350_1.
Author: Roman Marquez-Twisdale, r.marqueztwisdale@digipen.edu, 0065807.
Creation date: 01/26/2026
End Header -------------------------------------------------------*/

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
    if (a == b)
    {
        u = v = 0.0f;
        return false;
    }
 
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
    if (a == b || a == c || b == c || (a - b).Cross(a - c).Length() == 0.0f)
    {
        u = v = w = 0.0f;
        return false;
    }

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

    // check if start is within sphere
    if (PointSphere(rayStart, sphereCenter, sphereRadius))
    {
        t = 0.0f;
        return true;
    }

    // check for intersection
    // C = sphereCenter
    // S = rayStart
    // D = rayDir
    // r = sphereRadius

    // s_c = (rayStart - sphereCenter)
    // a = D^2
    // b = 2D(s_c)
    // c = C^2 + S^2 - 2CS - r^2

    Vector3 s_c = rayStart - sphereCenter;
    float a = rayDir.Dot(rayDir);
    float b = (2.0f * rayDir).Dot(s_c);
    float c = sphereCenter.Dot(sphereCenter) + rayStart.Dot(rayStart) - (2.0f * sphereCenter.Dot(rayStart)) - (sphereRadius * sphereRadius);

    float descrim = (b * b) - (4.0f * a * c);
    float tTemp = 0.0f;
    if (descrim == 0.0f)
    {
        // 1 solution
        tTemp = (-b + Math::Sqrt(descrim)) / (2.0f * a);
    } else if (descrim > 0.0f) {
        // 2 solutions
        float tTemp1 = (-b + Math::Sqrt(descrim)) / (2.0f * a);
        float tTemp2 = (-b - Math::Sqrt(descrim)) / (2.0f * a);
        tTemp = Math::Min(tTemp1, tTemp2);
    } else {
        // 0 solutions
        return false;
    }
    if (tTemp < 0.0f) return false;
    t = tTemp;
    return true;
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

    // get a point on the plane
    Vector3 normal = Vector3(plane.x, plane.y, plane.z);
    Vector3 pop = normal * plane.w;

    // calculate distance from plane for each point of the triangle
    float dist0 = (triP0 - pop).Dot(normal);
    float dist1 = (triP1 - pop).Dot(normal);
    float dist2 = (triP2 - pop).Dot(normal);

    // check intersection type
    if (Math::Abs(dist0) <= epsilon && Math::Abs(dist1) <= epsilon && Math::Abs(dist2) <= epsilon) return IntersectionType::Coplanar;
    if (dist0 <= epsilon && dist1 <= epsilon && dist2 <= epsilon) return IntersectionType::Outside;
    if (dist0 >= -epsilon && dist1 >= -epsilon && dist2 >= -epsilon) return IntersectionType::Inside;
    return IntersectionType::Overlaps;
}

IntersectionType::Type PlaneSphere(const Vector4& plane,
                                   const Vector3& sphereCenter, float sphereRadius)
{
    ++Application::mStatistics.mPlaneSphereTests;

    // calculate distance from center of sphere to plane
    Vector3 normal = Vector3(plane.x, plane.y, plane.z);
    Vector3 pop = normal * plane.w;
    float d = (sphereCenter - pop).Dot(normal);
    if (d > sphereRadius) return IntersectionType::Inside;
    if (d < -sphereRadius) return IntersectionType::Outside;
    return IntersectionType::Overlaps;
}

IntersectionType::Type PlaneAabb(const Vector4& plane,
                                 const Vector3& aabbMin, const Vector3& aabbMax)
{
    ++Application::mStatistics.mPlaneAabbTests;

    // build variables
    Vector3 normal = Vector3(plane.x, plane.y, plane.z);
    Vector3 pop = normal * plane.w;
    Vector3 pMin = Vector3();
    pMin.x = (normal.x > 0.0f) ? aabbMin.x : aabbMax.x;
    pMin.y = (normal.y > 0.0f) ? aabbMin.y : aabbMax.y;
    pMin.z = (normal.z > 0.0f) ? aabbMin.z : aabbMax.z;
    Vector3 pMax = Vector3();
    pMax.x = (normal.x > 0.0f) ? aabbMax.x : aabbMin.x;
    pMax.y = (normal.y > 0.0f) ? aabbMax.y : aabbMin.y;
    pMax.z = (normal.z > 0.0f) ? aabbMax.z : aabbMin.z;
    
    // check if min is inside
    if ((pMin - pop).Dot(normal) > 0.0f) return IntersectionType::Inside;

    // check if max is outside
    if ((pMax - pop).Dot(normal) < 0.0f) return IntersectionType::Outside;

    // must be overlapping
    return IntersectionType::Overlaps;
}

IntersectionType::Type FrustumTriangle(const Vector4 planes[6],
                                       const Vector3& triP0, const Vector3& triP1, const Vector3& triP2,
                                       float epsilon)
{
    ++Application::mStatistics.mFrustumTriangleTests;

    // check each plane
    IntersectionType::Type type0 = PlaneTriangle(planes[0], triP0, triP1, triP2, epsilon);
    IntersectionType::Type type1 = PlaneTriangle(planes[1], triP0, triP1, triP2, epsilon);
    IntersectionType::Type type2 = PlaneTriangle(planes[2], triP0, triP1, triP2, epsilon);
    IntersectionType::Type type3 = PlaneTriangle(planes[3], triP0, triP1, triP2, epsilon);
    IntersectionType::Type type4 = PlaneTriangle(planes[4], triP0, triP1, triP2, epsilon);
    IntersectionType::Type type5 = PlaneTriangle(planes[5], triP0, triP1, triP2, epsilon);

    // check if outside
    if (type0 == IntersectionType::Outside || type1 == IntersectionType::Outside ||
        type2 == IntersectionType::Outside || type3 == IntersectionType::Outside ||
        type4 == IntersectionType::Outside || type5 == IntersectionType::Outside) return IntersectionType::Outside;

    // check if inside
    if (type0 == IntersectionType::Inside && type1 == IntersectionType::Inside &&
        type2 == IntersectionType::Inside && type3 == IntersectionType::Inside &&
        type4 == IntersectionType::Inside && type5 == IntersectionType::Inside) return IntersectionType::Inside;

    // check for overlap
    return IntersectionType::Overlaps;
}

IntersectionType::Type FrustumSphere(const Vector4 planes[6],
                                     const Vector3& sphereCenter, float sphereRadius, size_t& lastAxis)
{
    ++Application::mStatistics.mFrustumSphereTests;

    // check the sphere against each plane
    IntersectionType::Type type0 = PlaneSphere(planes[0], sphereCenter, sphereRadius);
    IntersectionType::Type type1 = PlaneSphere(planes[1], sphereCenter, sphereRadius);
    IntersectionType::Type type2 = PlaneSphere(planes[2], sphereCenter, sphereRadius);
    IntersectionType::Type type3 = PlaneSphere(planes[3], sphereCenter, sphereRadius);
    IntersectionType::Type type4 = PlaneSphere(planes[4], sphereCenter, sphereRadius);
    IntersectionType::Type type5 = PlaneSphere(planes[5], sphereCenter, sphereRadius);

    // check if outside
    if (type0 == IntersectionType::Outside || type1 == IntersectionType::Outside ||
        type2 == IntersectionType::Outside || type3 == IntersectionType::Outside ||
        type4 == IntersectionType::Outside || type5 == IntersectionType::Outside) return IntersectionType::Outside;

    // check if inside
    if (type0 == IntersectionType::Inside && type1 == IntersectionType::Inside &&
        type2 == IntersectionType::Inside && type3 == IntersectionType::Inside &&
        type4 == IntersectionType::Inside && type5 == IntersectionType::Inside) return IntersectionType::Inside;

    // check for overlap
    return IntersectionType::Overlaps;
}

IntersectionType::Type FrustumAabb(const Vector4 planes[6],
                                   const Vector3& aabbMin, const Vector3& aabbMax, size_t& lastAxis)
{
    ++Application::mStatistics.mFrustumAabbTests;

    // check the sphere against each plane
    IntersectionType::Type type0 = PlaneAabb(planes[0], aabbMin, aabbMax);
    IntersectionType::Type type1 = PlaneAabb(planes[1], aabbMin, aabbMax);
    IntersectionType::Type type2 = PlaneAabb(planes[2], aabbMin, aabbMax);
    IntersectionType::Type type3 = PlaneAabb(planes[3], aabbMin, aabbMax);
    IntersectionType::Type type4 = PlaneAabb(planes[4], aabbMin, aabbMax);
    IntersectionType::Type type5 = PlaneAabb(planes[5], aabbMin, aabbMax);

    // check if outside
    if (type0 == IntersectionType::Outside || type1 == IntersectionType::Outside ||
        type2 == IntersectionType::Outside || type3 == IntersectionType::Outside ||
        type4 == IntersectionType::Outside || type5 == IntersectionType::Outside) return IntersectionType::Outside;

    // check if inside
    if (type0 == IntersectionType::Inside && type1 == IntersectionType::Inside &&
        type2 == IntersectionType::Inside && type3 == IntersectionType::Inside &&
        type4 == IntersectionType::Inside && type5 == IntersectionType::Inside) return IntersectionType::Inside;

    // check for overlap
    return IntersectionType::Overlaps;
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
