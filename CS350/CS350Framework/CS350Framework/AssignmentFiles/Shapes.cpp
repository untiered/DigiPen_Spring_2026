///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////

/* Start Header ------------------------------------------------------
Copyright (C) 2025 DigiPen Institute of Technology.
File Name: Shapes.cpp
Purpose: This file provides an implementation for each primitive shape used in this project.
Language: ISO C++ 14 Standard.
Platform: Legacy MSVC, x64/x86, Windows 10 OS.
Project: r.marqueztwisdale_CS350_2.
Author: Roman Marquez-Twisdale, r.marqueztwisdale@digipen.edu, 0065807.
Creation date: 02/24/2026
End Header -------------------------------------------------------*/

#include "Precompiled.hpp"

//-----------------------------------------------------------------------------LineSegment
LineSegment::LineSegment()
{
	mStart = mEnd = Vector3::cZero;
}

LineSegment::LineSegment(Math::Vec3Param start, Math::Vec3Param end)
{
	mStart = start;
	mEnd = end;
}

DebugShape& LineSegment::DebugDraw() const
{
	return gDebugDrawer->DrawLine(*this);
}

//-----------------------------------------------------------------------------Ray
Ray::Ray()
{
	mStart = mDirection = Vector3::cZero;
}

Ray::Ray(Math::Vec3Param start, Math::Vec3Param dir)
{
	mStart = start;
	mDirection = dir;
}

Ray Ray::Transform(const Math::Matrix4& transform) const
{
	Ray transformedRay;
	transformedRay.mStart = Math::TransformPoint(transform, mStart);
	transformedRay.mDirection = Math::TransformNormal(transform, mDirection);
	return transformedRay;
}

Vector3 Ray::GetPoint(float t) const
{
	return mStart + mDirection * t;
}

DebugShape& Ray::DebugDraw(float t) const
{
	return gDebugDrawer->DrawRay(*this, t);
}

//-----------------------------------------------------------------------------PCA Helpers
Matrix3 ComputeCovarianceMatrix(const std::vector<Vector3>& points)
{
	/******Student:Assignment2******/
	Warn("Assignment2: Required function un-implemented");
	return Matrix3::cIdentity;

	//Matrix3 cvMatrix = {};
	//
	//unsigned dim = static_cast<unsigned>(points.size());
	//
	//// calculate mean
	//Vector3 mean = {};
	//for (unsigned k = 0; k < dim; ++k)
	//	mean += points[k];
	//mean /= static_cast<float>(dim);

	//// set each element in the matrix
	//for (unsigned i = 0; i < dim; ++i)
	//{
	//	for (unsigned j = 0; j < dim; ++j)
	//	{
	//		// sum k=0 to n-1: (P^k_i - u_i)(P^k_j - u_j)
	//		for (unsigned k = 0; k < dim; ++k)
	//		{
	//			// calculate matrix element
	//			cvMatrix[i][j] += (points[k][i] - mean[i]) * (points[k][j] - mean[j]);
	//		}
	//		cvMatrix[i][j] /= static_cast<float>(dim); // divide by dim
	//	}
	//}
	//return cvMatrix;
}

Matrix3 ComputeJacobiRotation(const Matrix3& matrix)
{
	/******Student:Assignment2******/
	// Compute the jacobi rotation matrix that will turn the largest (magnitude) off-diagonal element of the input
	// matrix into zero. Note: the input matrix should always be (near) symmetric.
	Warn("Assignment2: Required function un-implemented");
	return Matrix3::cIdentity;
}

void ComputeEigenValuesAndVectors(const Matrix3& covariance, Vector3& eigenValues, Matrix3& eigenVectors, int maxIterations)
{
	/******Student:Assignment2******/
	// Iteratively rotate off the largest off-diagonal elements until the resultant matrix is diagonal or maxIterations.
	Warn("Assignment2: Required function un-implemented");
}


//-----------------------------------------------------------------------------Sphere
Sphere::Sphere()
{
	mCenter = Vector3::cZero;
	mRadius = 0;
}

Sphere::Sphere(const Vector3& center, float radius)
{
	mCenter = center;
	mRadius = radius;
}

void Sphere::ComputeCentroid(const std::vector<Vector3>& points)
{
	unsigned n = static_cast<unsigned>(points.size());

	Vector3 max = points[0];
	Vector3 min = points[0];

	// build aabb
	for (unsigned i = 1; i < n; ++i)
	{
		if (max.x < points[i].x) max.x = points[i].x;
		if (max.y < points[i].y) max.y = points[i].y;
		if (max.z < points[i].z) max.z = points[i].z;

		if (min.x > points[i].x) min.x = points[i].x;
		if (min.y > points[i].y) min.y = points[i].y;
		if (min.z > points[i].z) min.z = points[i].z;
	}

	// set centroid
	mCenter = (min + max) * 0.5f;

	// find radius
	for (unsigned i = 0; i < n; ++i)
	{
		if (mRadius < Math::Distance(mCenter, points[i])) mRadius = Math::Distance(mCenter, points[i]);
	}
}

void Sphere::ComputeRitter(const std::vector<Vector3>& points)
{
	unsigned n = static_cast<unsigned>(points.size());

	Vector3 max = points[0];
	Vector3 min = points[0];

	enum Axes { X = 0, Y, Z, DIMS };

	std::vector<std::pair<Vector3, Vector3>> axisPairs(DIMS);
	for (int i = 0; i < DIMS; ++i) {
		axisPairs[i].first = points[0];
		axisPairs[i].second = points[0];
	}

	// build aabb to find the min and max axis spreads
	for (unsigned i = 1; i < n; ++i) {
		if (max.x < points[i].x) { max.x = points[i].x; axisPairs[X].second = points[i]; }
		if (max.y < points[i].y) { max.y = points[i].y; axisPairs[Y].second = points[i]; }
		if (max.z < points[i].z) { max.z = points[i].z; axisPairs[Z].second = points[i]; }

		if (min.x > points[i].x) { min.x = points[i].x; axisPairs[X].first = points[i]; }
		if (min.y > points[i].y) { min.y = points[i].y; axisPairs[Y].first = points[i]; }
		if (min.z > points[i].z) { min.z = points[i].z; axisPairs[Z].first = points[i]; }
	}

	// generate the centroid
	float euclidDist = 0.0f;
	for (unsigned i = 0; i < DIMS; ++i)
	{
		// set the centroid
		float d = Math::Distance(axisPairs[i].first, axisPairs[i].second);
		if (d > euclidDist)
		{
			euclidDist = d;
			mCenter = (axisPairs[i].first + axisPairs[i].second) * 0.5f;
		}
	}
	
	// set radius
	mRadius = euclidDist * 0.5f;

	// incrementally expand the sphere
	for (unsigned i = 0; i < n; ++i)
	{
		float d = Math::Distance(mCenter, points[i]);
		if (d > mRadius) // check the distance between the current point and the center against the radius
		{
			// calculate new center and radius
			Vector3 b = (mCenter - ((points[i] - mCenter).Normalized() * mRadius));
			mCenter = (b + points[i]) * 0.5f;
			mRadius = Math::Distance(points[i], mCenter);
		}
	}
}

void Sphere::ComputePCA(const std::vector<Vector3>& points)
{
	// The PCA method:
	// Compute the eigen values and vectors. Take the largest eigen vector as the axis of largest spread.
	// Compute the sphere center as the center of this axis then expand by all points.
	/******Student:Assignment2******/
	Warn("Assignment2: Required function un-implemented");


}


bool Sphere::ContainsPoint(const Vector3& point)
{
	return PointSphere(point, mCenter, mRadius);
}

Vector3 Sphere::GetCenter() const
{
	return mCenter;
}

float Sphere::GetRadius() const
{
	return mRadius;
}

bool Sphere::Compare(const Sphere& rhs, float epsilon) const
{
	float posDiff = Math::Length(mCenter - rhs.mCenter);
	float radiusDiff = Math::Abs(mRadius - rhs.mRadius);

	return posDiff < epsilon && radiusDiff < epsilon;
}

DebugShape& Sphere::DebugDraw() const
{
	return gDebugDrawer->DrawSphere(*this);
}

//-----------------------------------------------------------------------------Aabb
Aabb::Aabb()
{
	//set the aabb to an initial bad value (where the min is smaller than the max)
	mMin.Splat(Math::PositiveMax());
	mMax.Splat(-Math::PositiveMax());
}

Aabb::Aabb(const Vector3& min, const Vector3& max)
{
	mMin = min;
	mMax = max;
}

Aabb Aabb::BuildFromCenterAndHalfExtents(const Vector3& center, const Vector3& halfExtents)
{
	return Aabb(center - halfExtents, center + halfExtents);
}

float Aabb::GetVolume() const
{
	Vector3 dim = mMax - mMin;
	return dim.x * dim.y * dim.z;
}

float Aabb::GetSurfaceArea() const
{
	Vector3 dim = mMax - mMin;

	// compute xy surface area
	float xy_sa = dim.x * dim.y * 2.0f;
	// compute xz surface area
	float xz_sa = dim.x * dim.z * 2.0f;
	// compute yz surface area
	float yz_sa = dim.y * dim.z * 2.0f;

	return xy_sa + xz_sa + yz_sa;
}

bool Aabb::Contains(const Aabb& aabb) const
{
	return aabb.mMin.x >= mMin.x
		&& aabb.mMin.y >= mMin.y
		&& aabb.mMin.z >= mMin.z
		&& aabb.mMax.x <= mMax.x
		&& aabb.mMax.y <= mMax.y
		&& aabb.mMax.z <= mMax.z;
}

void Aabb::Expand(const Vector3& point)
{
	for (uint32_t i = 0; i < 3; ++i)
	{
		mMin[i] = Math::Min(mMin[i], point[i]);
		mMax[i] = Math::Max(mMax[i], point[i]);
	}
}

Aabb Aabb::Combine(const Aabb& lhs, const Aabb& rhs)
{
	Aabb result;
	for (uint32_t i = 0; i < 3; ++i)
	{
		result.mMin[i] = Math::Min(lhs.mMin[i], rhs.mMin[i]);
		result.mMax[i] = Math::Max(lhs.mMax[i], rhs.mMax[i]);
	}
	return result;
}

bool Aabb::Compare(const Aabb& rhs, float epsilon) const
{
	float pos1Diff = Math::Length(mMin - rhs.mMin);
	float pos2Diff = Math::Length(mMax - rhs.mMax);

	return pos1Diff < epsilon && pos2Diff < epsilon;
}

void Aabb::Transform(const Vector3& scale, const Matrix3& rotation, const Vector3& translation)
{
	// get center point
	Vector3 c3 = mMin + ((mMax - mMin) * 0.5f);
	Vector4 sc = Vector4(c3.x * scale.x, c3.y * scale.y, c3.z * scale.z, 1.0f);
	
	// get the scaled extent vector
	Vector4 sr = (Vector4(mMax.x - c3.x, 0.0f, 0.0f, 0.0f) * scale.x)
				+(Vector4(0.0f, mMax.y - c3.y, 0.0f, 0.0f) * scale.y)
				+(Vector4(0.0f, 0.0f, mMax.z - c3.z, 0.0f) * scale.z);
		
	// build an rotation matrix
	Matrix4 rotM = Math::BuildTransform(Vector3(0.0f, 0.0f, 0.0f), rotation, Vector3(1.0f, 1.0f, 1.0f));

	// rotate the scaled center
	Vector4 scPrime = Math::Transform(rotM, sc);

	// ABS the matrix
	rotM[0][0] = Math::Abs(rotM[0][0]);
	rotM[1][0] = Math::Abs(rotM[1][0]);
	rotM[2][0] = Math::Abs(rotM[2][0]);
	rotM[3][0] = Math::Abs(rotM[3][0]);

	rotM[0][1] = Math::Abs(rotM[0][1]);
	rotM[1][1] = Math::Abs(rotM[1][1]);
	rotM[2][1] = Math::Abs(rotM[2][1]);
	rotM[3][1] = Math::Abs(rotM[3][1]);

	rotM[0][2] = Math::Abs(rotM[0][2]);
	rotM[1][2] = Math::Abs(rotM[1][2]);
	rotM[2][2] = Math::Abs(rotM[2][2]);
	rotM[3][2] = Math::Abs(rotM[3][2]);

	rotM[0][3] = Math::Abs(rotM[0][3]);
	rotM[1][3] = Math::Abs(rotM[1][3]);
	rotM[2][3] = Math::Abs(rotM[2][3]);
	rotM[3][3] = Math::Abs(rotM[3][3]);


	// ABS rotate the scaled extent
	Vector4 srPrime = Math::Transform(rotM, sr);

	// update the aabb
	mMin = Vector3(scPrime.x, scPrime.y, scPrime.z) + translation - Vector3(srPrime.x, srPrime.y, srPrime.z);
	mMax = Vector3(scPrime.x, scPrime.y, scPrime.z) + translation + Vector3(srPrime.x, srPrime.y, srPrime.z);
}

Vector3 Aabb::GetMin() const
{
	return mMin;
}

Vector3 Aabb::GetMax() const
{
	return mMax;
}

Vector3 Aabb::GetCenter() const
{
	return (mMin + mMax) * 0.5f;
}

Vector3 Aabb::GetHalfSize() const
{
	return (mMax - mMin) * 0.5f;
}

DebugShape& Aabb::DebugDraw() const
{
	return gDebugDrawer->DrawAabb(*this);
}

//-----------------------------------------------------------------------------Triangle
Triangle::Triangle()
{
	mPoints[0] = mPoints[1] = mPoints[2] = Vector3::cZero;
}

Triangle::Triangle(const Vector3& p0, const Vector3& p1, const Vector3& p2)
{
	mPoints[0] = p0;
	mPoints[1] = p1;
	mPoints[2] = p2;
}

DebugShape& Triangle::DebugDraw() const
{
	return gDebugDrawer->DrawTriangle(*this);
}

//-----------------------------------------------------------------------------Plane
Plane::Plane()
{
	mData = Vector4::cZero;
}

Plane::Plane(const Vector3& p0, const Vector3& p1, const Vector3& p2)
{
	Set(p0, p1, p2);
}

Plane::Plane(const Vector3& normal, const Vector3& point)
{
	Set(normal, point);
}

void Plane::Set(const Vector3& p0, const Vector3& p1, const Vector3& p2)
{
	// build the normal
	Vector3 n_normal = Cross(p2 - p0, p2 - p1).Normalized();

	// find d
	float d = n_normal.Dot(p0);

	mData = Vector4(n_normal.x, n_normal.y, n_normal.z, d);
}

void Plane::Set(const Vector3& normal, const Vector3& point)
{
	// build the normal
	Vector3 n_normal = normal.Normalized();

	// find d
	float d = n_normal.Dot(point);

	mData = Vector4(n_normal.x, n_normal.y, n_normal.z, d);
}

Vector3 Plane::GetNormal() const
{
	return Vector3(mData.x, mData.y, mData.z);
}

float Plane::GetDistance() const
{
	return mData.w;
}

DebugShape& Plane::DebugDraw(float size) const
{
	return DebugDraw(size, size);
}

DebugShape& Plane::DebugDraw(float sizeX, float sizeY) const
{
	return gDebugDrawer->DrawPlane(*this, sizeX, sizeY);
}

//-----------------------------------------------------------------------------Frustum
void Frustum::Set(const Vector3& lbn, const Vector3& rbn, const Vector3& rtn, const Vector3& ltn,
	const Vector3& lbf, const Vector3& rbf, const Vector3& rtf, const Vector3& ltf)
{
	mPoints[0] = lbn;
	mPoints[1] = rbn;
	mPoints[2] = rtn;
	mPoints[3] = ltn;
	mPoints[4] = lbf;
	mPoints[5] = rbf;
	mPoints[6] = rtf;
	mPoints[7] = ltf;

	//left
	mPlanes[0].Set(lbf, ltf, lbn);
	//right
	mPlanes[1].Set(rbn, rtf, rbf);
	//top
	mPlanes[2].Set(ltn, ltf, rtn);
	//bot
	mPlanes[3].Set(rbn, lbf, lbn);
	//near
	mPlanes[4].Set(lbn, ltn, rbn);
	//far
	mPlanes[5].Set(rbf, rtf, lbf);
}

Math::Vector4* Frustum::GetPlanes() const
{
	return (Vector4*)mPlanes;
}

DebugShape& Frustum::DebugDraw() const
{
	return gDebugDrawer->DrawFrustum(*this);
}
