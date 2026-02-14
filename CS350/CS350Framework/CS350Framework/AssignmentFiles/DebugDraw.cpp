///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////

/* Start Header ------------------------------------------------------
Copyright (C) 2025 DigiPen Institute of Technology.
File Name: DebugDraw.cpp
Purpose: This file provides an implementation for draw operations related to debugging primitive shapes.
Language: ISO C++ 14 Standard.
Platform: Legacy MSVC, x64/x86, Windows 10 OS.
Project: r.marqueztwisdale_CS350_1.
Author: Roman Marquez-Twisdale, r.marqueztwisdale@digipen.edu, 0065807.
Creation date: 01/26/2026
End Header -------------------------------------------------------*/

#include "Precompiled.hpp"

#define ShowDebugDrawWarnings true

DebugDrawer* gDebugDrawer = new DebugDrawer();

//-----------------------------------------------------------------------------DebugShape
DebugShape::DebugShape()
{
	mColor = Vector4(.6f);
	mMask = (unsigned int)-1;
	mTimer = 0;
	mOnTop = false;
	mTransform.SetIdentity();
}

DebugShape& DebugShape::Color(const Vector4& color)
{
	mColor = color;
	return *this;
}

DebugShape& DebugShape::OnTop(bool state)
{
	mOnTop = state;
	return *this;
}

DebugShape& DebugShape::Time(float time)
{
	mTimer = time;
	return *this;
}

DebugShape& DebugShape::SetMaskBit(int bitIndex)
{
	mMask = 1 << bitIndex;
	return *this;
}

DebugShape& DebugShape::SetTransform(const Matrix4& transform)
{
	mTransform = transform;
	return *this;
}

//-----------------------------------------------------------------------------DebugDrawer
DebugDrawer::DebugDrawer()
{
	mActiveMask = (unsigned int)-1;
	mApplication = NULL;
}

void DebugDrawer::Update(float dt)
{
	std::vector<DebugShape> newShapes;
	for (size_t i = 0; i < mShapes.size(); ++i)
	{
		DebugShape& shape = mShapes[i];
		shape.mTimer -= dt;

		// If the shape still has time left then add it to the list of shapes to keep drawing,
		// anything that has a timer that ran out will not be in the new list
		if (shape.mTimer >= 0)
			newShapes.push_back(shape);
	}

	mShapes.swap(newShapes);
}

void DebugDrawer::Draw()
{
	for (size_t i = 0; i < mShapes.size(); ++i)
	{
		DebugShape& shape = mShapes[i];

		// If the shape doesn't have one of the active mask bits set then don't draw it
		if ((shape.mMask & mActiveMask) == 0)
			continue;

		// If this shape always draws on top then disable depth testing
		if (shape.mOnTop)
			glDisable(GL_DEPTH_TEST);


		// Decompose the matrix to set the gl transform (too lazy to properly transform the matrix between formats)
		float radians;
		Vector3 scale, translation, axis;
		Matrix3 rotationMat;
		shape.mTransform.Decompose(&scale, &rotationMat, &translation);
		Math::ToAxisAngle(Math::ToQuaternion(rotationMat), &axis, &radians);
		glPushMatrix();
		// Set the transform
		glTranslatef(translation.x, translation.y, translation.z);
		glRotatef(Math::RadToDeg(radians), axis.x, axis.y, axis.z);
		glScalef(scale.x, scale.y, scale.z);

		glBegin(GL_LINES);
		glColor3fv(shape.mColor.array);

		// Draw all of the line segments of this shape
		for (size_t j = 0; j < shape.mSegments.size(); ++j)
		{
			LineSegment& segment = shape.mSegments[j];

			glVertex3fv(segment.mStart.array);
			glVertex3fv(segment.mEnd.array);
		}

		glEnd();
		glPopMatrix();

		// Make sure to re-enable depth testing
		if (shape.mOnTop)
			glEnable(GL_DEPTH_TEST);
	}
}

DebugShape& DebugDrawer::GetNewShape()
{
	mShapes.push_back(DebugShape());
	return mShapes.back();
}

DebugShape& DebugDrawer::DrawPoint(const Vector3& point)
{
	return DrawSphere(Sphere(point, 0.1f));
}

DebugShape& DebugDrawer::DrawLine(const LineSegment& line)
{
	/******Student:Assignment2******/
	// Draw a simple line
	DebugShape& shape = GetNewShape();
	WarnIf(ShowDebugDrawWarnings, "Assignment2: Required function un-implemented");
	return shape;

	///******Student:Assignment2******/
	//// Draw a simple line
	//DebugShape& shape = GetNewShape();
	//shape.mSegments.push_back(line);
	//return shape;
}

DebugShape& DebugDrawer::DrawRay(const Ray& ray, float t)
{
	/******Student:Assignment2******/
	// Draw a ray to a given t-length. The ray must have an arrow head for visualization
	DebugShape& shape = GetNewShape();
	WarnIf(ShowDebugDrawWarnings, "Assignment2: Required function un-implemented");
	return shape;

	///******Student:Assignment2******/
	//// Draw a ray to a given t-length. The ray must have an arrow head for visualization
	//DebugShape& shape = GetNewShape();
	//Vector3 rayEnd = ray.mStart + (ray.mDirection * t);
	//shape.mSegments.push_back(LineSegment(ray.mStart, rayEnd));
	//
	//// construct a basis from the direction vector.
	//Vector3 v = ray.mDirection.Cross(Vector3(ray.mDirection.x + 1.0f, ray.mDirection.y + 1.0f, ray.mDirection.z));
	//Vector3 w = ray.mDirection.Cross(v);

	//// make the arrow head
	//float r = t * 0.05f;
	//Vector3 C = ray.mStart + (ray.mDirection * t * 0.95f);
	//float sectionCount = 15.0f;
	//for (int i = 0; i < static_cast<int>(sectionCount); ++i)
	//{
	//	// construct start point
	//	float radians = Math::cTwoPi / sectionCount * i;
	//	Vector3 p0 = C + ((v * Math::Cos(radians)) + (w * Math::Sin(radians))).Normalized() * r;

	//	// construct end point
	//	radians = Math::cTwoPi / sectionCount * (i+1);
	//	Vector3 p1 = C + ((v * Math::Cos(radians)) + (w * Math::Sin(radians))).Normalized() * r;

	//	// add segment
	//	shape.mSegments.push_back(LineSegment(p0, p1));
	//	shape.mSegments.push_back(LineSegment(rayEnd, p0));
	//}
	//return shape;
}

DebugShape& DebugDrawer::DrawSphere(const Sphere& sphere)
{
	/******Student:Assignment2******/
	// Draw a sphere with 4 rings: x-axis, y-axis, z-axis, and the horizon disc.
	// Note: To access the camera's position for the horizon disc calculation use mApplication->mCamera.mTranslation
	DebugShape& shape = GetNewShape();
	WarnIf(ShowDebugDrawWarnings, "Assignment2: Required function un-implemented");
	return shape;

	///******Student:Assignment2******/
	//// Draw a sphere with 4 rings: x-axis, y-axis, z-axis, and the horizon disc.
	//// Note: To access the camera's position for the horizon disc calculation use mApplication->mCamera.mTranslation
	//DebugShape& shape = GetNewShape();
	//
	//// make the horizon disc
	//if (mApplication)
	//{
	//	std::vector<LineSegment> disc = MakeDisc(
	//		mApplication->mCamera.mTranslation.Normalized(),
	//		sphere.mCenter,
	//		sphere.mRadius,
	//		15.0f
	//	);
	//	shape.mSegments.insert(shape.mSegments.end(), disc.begin(), disc.end());
	//
	//	// Append each disc to the segment vector
	//	std::vector<Vector3> eVecs = { Vector3::cXAxis, Vector3::cYAxis, Vector3::cZAxis };
	//	for (int i = 0; i < 3; ++i)
	//	{
	//		std::vector<LineSegment> disc = MakeDisc(eVecs[i], sphere.mCenter, sphere.mRadius, 15.0f);
	//		shape.mSegments.insert(shape.mSegments.end(), disc.begin(), disc.end());
	//	}
	//}
	//
	//return shape;
}

DebugShape& DebugDrawer::DrawAabb(const Aabb& aabb)
{
	/******Student:Assignment2******/
	// Draw all edges of an aabb. Make sure to not mis-match edges!
	DebugShape& shape = GetNewShape();
	WarnIf(ShowDebugDrawWarnings, "Assignment2: Required function un-implemented");
	return shape;

	///******Student:Assignment2******/
	//// Draw all edges of an aabb. Make sure to not mis-match edges!
	//DebugShape& shape = GetNewShape();

	//// Y
	//shape.mSegments.push_back(LineSegment( // back left
	//	aabb.mMin,
	//	Vector3(aabb.mMin.x, aabb.mMax.y, aabb.mMin.z)));
	//shape.mSegments.push_back(LineSegment( // back right
	//	Vector3(aabb.mMax.x, aabb.mMin.y, aabb.mMin.z),
	//	Vector3(aabb.mMax.x, aabb.mMax.y, aabb.mMin.z)));
	//shape.mSegments.push_back(LineSegment( // front left
	//	Vector3(aabb.mMin.x, aabb.mMin.y, aabb.mMax.z),
	//	Vector3(aabb.mMin.x, aabb.mMax.y, aabb.mMax.z)));
	//shape.mSegments.push_back(LineSegment( // front right
	//	Vector3(aabb.mMax.x, aabb.mMin.y, aabb.mMax.z),
	//	aabb.mMax));

	//// X
	//shape.mSegments.push_back(LineSegment( // back bottom
	//	aabb.mMin,
	//	Vector3(aabb.mMax.x, aabb.mMin.y, aabb.mMin.z)));
	//shape.mSegments.push_back(LineSegment( // back top
	//	Vector3(aabb.mMin.x, aabb.mMax.y, aabb.mMin.z),
	//	Vector3(aabb.mMax.x, aabb.mMax.y, aabb.mMin.z)));
	//shape.mSegments.push_back(LineSegment( // front bottom
	//	Vector3(aabb.mMin.x, aabb.mMin.y, aabb.mMax.z),
	//	Vector3(aabb.mMax.x, aabb.mMin.y, aabb.mMax.z)));
	//shape.mSegments.push_back(LineSegment( // front top
	//	Vector3(aabb.mMin.x, aabb.mMax.y, aabb.mMax.z),
	//	aabb.mMax));

	//// Z
	//shape.mSegments.push_back(LineSegment( // left bottom
	//	aabb.mMin,
	//	Vector3(aabb.mMin.x, aabb.mMin.y, aabb.mMax.z)));
	//shape.mSegments.push_back(LineSegment( // left top
	//	Vector3(aabb.mMin.x, aabb.mMax.y, aabb.mMin.z),
	//	Vector3(aabb.mMin.x, aabb.mMax.y, aabb.mMax.z)));
	//shape.mSegments.push_back(LineSegment( // right bottom
	//	Vector3(aabb.mMax.x, aabb.mMin.y, aabb.mMin.z),
	//	Vector3(aabb.mMax.x, aabb.mMin.y, aabb.mMax.z)));
	//shape.mSegments.push_back(LineSegment( // right top
	//	Vector3(aabb.mMax.x, aabb.mMax.y, aabb.mMin.z),
	//	aabb.mMax));

	//return shape;
}

DebugShape& DebugDrawer::DrawTriangle(const Triangle& triangle)
{
	/******Student:Assignment2******/
	// Draw the 3 edges of a triangles
	DebugShape& shape = GetNewShape();
	WarnIf(ShowDebugDrawWarnings, "Assignment2: Required function un-implemented");
	return shape;

	///******Student:Assignment2******/
	//// Draw the 3 edges of a triangles
	//DebugShape& shape = GetNewShape();
	//shape.mSegments.push_back(LineSegment(triangle.mPoints[0], triangle.mPoints[1]));
	//shape.mSegments.push_back(LineSegment(triangle.mPoints[0], triangle.mPoints[2]));
	//shape.mSegments.push_back(LineSegment(triangle.mPoints[1], triangle.mPoints[2]));
	//return shape;
}

DebugShape& DebugDrawer::DrawPlane(const Plane& plane, float sizeX, float sizeY)
{
	/******Student:Assignment2******/
	// Draw a quad with a normal at the plane's center.
	DebugShape& shape = GetNewShape();
	WarnIf(ShowDebugDrawWarnings, "Assignment2: Required function un-implemented");
	return shape;

	///******Student:Assignment2******/
	//// Draw a quad with a normal at the plane's center.
	//DebugShape& shape = GetNewShape();
	//
	//// make a basis
	//Vector3 normal = Vector3(plane.mData.x, plane.mData.y, plane.mData.z);
	//Vector3 v = normal.Cross(Vector3(normal.z, normal.x, normal.y));
	//Vector3 w = normal.Cross(v);

	//// calculate center
	//Vector3 C = Vector3(plane.mData.x, plane.mData.y, plane.mData.z) * plane.mData.w;
	//
	//// build min and max points
	//Vector3 corner0 = C - (v * sizeX * 0.5f) - (w * sizeY * 0.5f);
	//Vector3 corner2 = C + (v * sizeX * 0.5f) + (w * sizeY * 0.5f);
	//Vector3 corner1 = C - (v * sizeX * 0.5f) + (w * sizeY * 0.5f);
	//Vector3 corner3 = C + (v * sizeX * 0.5f) - (w * sizeY * 0.5f);
	//
	//// add segments
	//shape.mSegments.push_back(LineSegment(corner0, corner1));
	//shape.mSegments.push_back(LineSegment(corner0, corner3));
	//shape.mSegments.push_back(LineSegment(corner1, corner2));
	//shape.mSegments.push_back(LineSegment(corner3, corner2));
	//return shape;
}

DebugShape& DebugDrawer::DrawQuad(const Vector3& p0, const Vector3& p1, const Vector3& p2, const Vector3& p3)
{
	/******Student:Assignment2******/
	// Draw the4 edges of a quad. Make sure to look at this and make sure the quad is not bow-tied.
	DebugShape& shape = GetNewShape();
	WarnIf(ShowDebugDrawWarnings, "Assignment2: Required function un-implemented");
	return shape;

	///******Student:Assignment2******/
	//// Draw the4 edges of a quad. Make sure to look at this and make sure the quad is not bow-tied.
	//DebugShape& shape = GetNewShape();

	//// add segments
	//shape.mSegments.push_back(LineSegment(p0, p1));
	//shape.mSegments.push_back(LineSegment(p0, p3));
	//shape.mSegments.push_back(LineSegment(p1, p2));
	//shape.mSegments.push_back(LineSegment(p3, p2));

	//return shape;
}

DebugShape& DebugDrawer::DrawFrustum(const Frustum& frustum)
{
	/******Student:Assignment2******/
	// Draw the 6 faces of the frustum using the 8 frustum points.
	// See Frustum.Set for the point order. For example, Points[4] is left-bottom-front.
	DebugShape& shape = GetNewShape();
	WarnIf(ShowDebugDrawWarnings, "Assignment2: Required function un-implemented");
	return shape;

	///******Student:Assignment2******/
	//// Draw the 6 faces of the frustum using the 8 frustum points.
	//// See Frustum.Set for the point order. For example, Points[4] is left-bottom-far.
	//DebugShape& shape = GetNewShape();
	//
	//// add segments
	//shape.mSegments.push_back(LineSegment(frustum.mPoints[0], frustum.mPoints[4])); // lb
	//shape.mSegments.push_back(LineSegment(frustum.mPoints[1], frustum.mPoints[5])); // rb
	//shape.mSegments.push_back(LineSegment(frustum.mPoints[3], frustum.mPoints[7])); // lt
	//shape.mSegments.push_back(LineSegment(frustum.mPoints[2], frustum.mPoints[6])); // rt

	//shape.mSegments.push_back(LineSegment(frustum.mPoints[0], frustum.mPoints[1])); // nb
	//shape.mSegments.push_back(LineSegment(frustum.mPoints[2], frustum.mPoints[3])); // nt
	//shape.mSegments.push_back(LineSegment(frustum.mPoints[4], frustum.mPoints[5])); // fb
	//shape.mSegments.push_back(LineSegment(frustum.mPoints[6], frustum.mPoints[7])); // ft

	//shape.mSegments.push_back(LineSegment(frustum.mPoints[0], frustum.mPoints[3])); // ln
	//shape.mSegments.push_back(LineSegment(frustum.mPoints[1], frustum.mPoints[2])); // rn
	//shape.mSegments.push_back(LineSegment(frustum.mPoints[4], frustum.mPoints[7])); // lf
	//shape.mSegments.push_back(LineSegment(frustum.mPoints[5], frustum.mPoints[6])); // rf

	//return shape;
}

/// <summary>
/// builds a circular disc around a given axis, at a given center point, and with a given radius.
/// </summary>
/// <param name="normal">: the axis around which the disc will be built (REMEMBER TO NORMALIZE THIS!!!)</param>
/// <param name="C">: the center of the disc </param>
/// <param name="r">: the radius </param>
/// <param name="sections">: the number of line segments to use for the disc</param>
/// <returns>: a vector of line segments that make up the disc</returns>
//std::vector<LineSegment> DebugDrawer::MakeDisc(Vector3 const&normal, Vector3 const& C, float r, float sections)
//{
//	std::vector<LineSegment> disc;
//
//	// construct a basis from the direction vector.
//	Vector3 v = normal.Cross(Vector3(normal.z, normal.x, normal.y));
//	Vector3 w = normal.Cross(v);
//
//	// make the disc
//	for (int i = 0; i < static_cast<int>(sections); ++i)
//	{
//		// construct start point
//		float radians = Math::cTwoPi / sections * i;
//		Vector3 p0 = C + ((v * Math::Cos(radians)) + (w * Math::Sin(radians))).Normalized() * r;
//
//		// construct end point
//		radians = Math::cTwoPi / sections * (i + 1);
//		Vector3 p1 = C + ((v * Math::Cos(radians)) + (w * Math::Sin(radians))).Normalized() * r;
//
//		// add segment
//		disc.push_back(LineSegment(p0, p1));
//	}
//	return disc;
//}
