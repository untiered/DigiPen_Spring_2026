///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////

/* Start Header ------------------------------------------------------
Copyright (C) 2025 DigiPen Institute of Technology.
File Name: DebugDraw.hpp
Purpose: This file provides an interface for calling draw operations related to debugging primitive shapes.
Language: ISO C++ 14 Standard.
Platform: Legacy MSVC, x64/x86, Windows 10 OS.
Project: r.marqueztwisdale_CS350_1.
Author: Roman Marquez-Twisdale, r.marqueztwisdale@digipen.edu, 0065807.
Creation date: 01/26/2026
End Header -------------------------------------------------------*/

#pragma once

#include "Math/Math.hpp"
#include "Shapes.hpp"

class Application;
class DebugDrawer;

//-----------------------------------------------------------------------------DebugShape
class DebugShape
{
public:
  DebugShape();

  // These functions allow chaining of setting properties (C++ closest thing to named parameters)

  // Set the color of this shape.
  DebugShape& Color(const Vector4& color);
  // Does this shape ignore depth test?
  DebugShape& OnTop(bool state);
  // How long should this shape draw for?
  DebugShape& Time(float time);
  // Set a random bit on the mask which can control what is drawn with the number keys.
  DebugShape& SetMaskBit(int bitIndex);
  // A transform to apply to the shape when drawing.
  // Primarily for spatial partitions in local space to draw their data.
  DebugShape& SetTransform(const Matrix4& transform);

private:
  friend DebugDrawer;

  Math::Vector4 mColor;
  float mTimer;
  unsigned int mMask;
  bool mOnTop;
  Matrix4 mTransform;

  // All of the lines use to draw this shape
  std::vector<LineSegment> mSegments;
};

//-----------------------------------------------------------------------------DebugDrawer
class DebugDrawer
{
public:
  DebugDrawer();

  void Update(float dt);
  void Draw();

  // Makes a new debug shape and inserts it into the shapes to draw.
  DebugShape& GetNewShape();


  // You must implement all of the below functions

  DebugShape& DrawPoint(const Vector3& point);
  DebugShape& DrawLine(const LineSegment& line);
  DebugShape& DrawRay(const Ray& ray, float t);
  DebugShape& DrawSphere(const Sphere& sphere);
  DebugShape& DrawAabb(const Aabb& aabb);
  DebugShape& DrawTriangle(const Triangle& triangle);
  DebugShape& DrawPlane(const Plane& plane, float sizeX, float sizeY);
  DebugShape& DrawQuad(const Vector3& p0, const Vector3& p1, const Vector3& p2, const Vector3& p3);
  DebugShape& DrawFrustum(const Frustum& frustum);

private:
  friend Application;

  typedef std::vector<DebugShape> DebugShapeList;
  DebugShapeList mShapes;
  // Any active masks used to filter out what's currently drawing.
  unsigned int mActiveMask;

  Application* mApplication;

  // ROMAN'S HELPER
  //std::vector<LineSegment> MakeDisc(Vector3 const& normal, Vector3 const& C, float r, float sections);
};

extern DebugDrawer* gDebugDrawer;
