///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////

/* Start Header ------------------------------------------------------
Copyright (C) 2025 DigiPen Institute of Technology.
File Name: SimpleNSquared.cpp
Purpose: A primitive implementation for two different types of spatial partitions.
Language: ISO C++ 14 Standard.
Platform: Legacy MSVC, x64/x86, Windows 10 OS.
Project: r.marqueztwisdale_CS350_2.
Author: Roman Marquez-Twisdale, r.marqueztwisdale@digipen.edu, 0065807.
Creation date: 02/24/2026
End Header -------------------------------------------------------*/

#include "Precompiled.hpp"

//-----------------------------------------------------------------------------NSquaredSpatialPartition
NSquaredSpatialPartition::NSquaredSpatialPartition()
{
	mType = SpatialPartitionTypes::NSquared;
}

void NSquaredSpatialPartition::InsertData(SpatialPartitionKey& key, SpatialPartitionData& data)
{
	// Doing this lazily (and bad, but it's n-squared...).
	// Just store as the key what the client data is so we can look it up later.
	key.mVoidKey = data.mClientData;
	mData.push_back(data.mClientData);
}

void NSquaredSpatialPartition::UpdateData(SpatialPartitionKey& key, SpatialPartitionData& data)
{
	// Nothing to do here, update doesn't do anything
}

void NSquaredSpatialPartition::RemoveData(SpatialPartitionKey& key)
{
	// Find the key data and remove it
	for (size_t i = 0; i < mData.size(); ++i)
	{
		if (mData[i] == key.mVoidKey)
		{
			mData[i] = mData.back();
			mData.pop_back();
			break;
		}
	}
}

void NSquaredSpatialPartition::DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color, int bitMask)
{
	// Nothing to debug draw
}

void NSquaredSpatialPartition::CastRay(const Ray& ray, CastResults& results)
{
	// Add everything
	for (size_t i = 0; i < mData.size(); ++i)
	{
		CastResult result;
		result.mClientData = mData[i];
		results.AddResult(result);
	}
}

void NSquaredSpatialPartition::CastFrustum(const Frustum& frustum, CastResults& results)
{
	// Add everything
	for (size_t i = 0; i < mData.size(); ++i)
	{
		CastResult result;
		result.mClientData = mData[i];
		results.AddResult(result);
	}
}

void NSquaredSpatialPartition::SelfQuery(QueryResults& results)
{
	// Add everything
	for (size_t i = 0; i < mData.size(); ++i)
	{
		for (size_t j = i + 1; j < mData.size(); ++j)
		{
			results.AddResult(QueryResult(mData[i], mData[j]));
		}
	}
}

void NSquaredSpatialPartition::GetDataFromKey(const SpatialPartitionKey& key, SpatialPartitionData& data) const
{
	data.mClientData = key.mVoidKey;
}

void NSquaredSpatialPartition::FilloutData(std::vector<SpatialPartitionQueryData>& results) const
{
	for (size_t i = 0; i < mData.size(); ++i)
	{
		SpatialPartitionQueryData data;
		data.mClientData = mData[i];
		results.push_back(data);
	}
}

//-----------------------------------------------------------------------------BoundingSphereSpatialPartition
BoundingSphereSpatialPartition::BoundingSphereSpatialPartition()
{
	mType = SpatialPartitionTypes::NSquaredSphere;
}

void BoundingSphereSpatialPartition::InsertData(SpatialPartitionKey& key, SpatialPartitionData& data)
{
	// Insert the given data into the spatial partition
	if (mFreeList.empty())										// removals have been resolved
	{
		mData.push_back(&data);

		// set the key
		key.mUIntKey = static_cast<unsigned>(mData.size() - 1);
	} else {													// removals have not been resolved
		unsigned freeIndex = mFreeList.back();
		mData[freeIndex] = &data;
		mFreeList.pop_back();

		// set the key
		key.mUIntKey = freeIndex;
	}
}

void BoundingSphereSpatialPartition::UpdateData(SpatialPartitionKey& key, SpatialPartitionData& data)
{
	// Also you should not do a linear search to find an object during update and removal,
	// this operation must be constant time!	
	mData[key.mUIntKey] = &data;
}

void BoundingSphereSpatialPartition::RemoveData(SpatialPartitionKey& key)
{
	// Also you should not do a linear search to find an object during update and removal,
	// this operation must be constant time!

	// later passes of the mData will only process non-null elements
	mData[key.mUIntKey] = nullptr;
	mFreeList.push_back(key.mUIntKey);
}

void BoundingSphereSpatialPartition::DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color, int bitMask)
{
	Warn("Assignment2: Required function un-implemented");
	// When implementing DebugDraw just use the passed in
	// transform, color, and bit mask to
	// set each debug shape(use “.Color()”, “.SetMaskBit()”, and “.SetTransform()”).
	// Make sure modify the actual debug shape and not a copy!
	// For this assignment ‘level’ is not used.

	/*for (size_t i = 0; i < mData.size(); ++i)
	{
		if (!mData[i]) continue;

		DebugShape &ds = mData[i]->mBoundingSphere.DebugDraw();
		ds.Color(color);
		ds.SetMaskBit(bitMask);
		ds.SetTransform(transform);
	}*/
}

void BoundingSphereSpatialPartition::CastRay(const Ray& ray, CastResults& results)
{
	// All cast operations should be linear time.
	for (size_t i = 0; i < mData.size(); ++i)
	{
		// dont process empty slots
		if (!mData[i]) continue;

		// check if colliding
		Sphere sphere = mData[i]->mBoundingSphere;
		float t = 0.0f;
		if (RaySphere(ray.mStart, ray.mDirection, sphere.mCenter, sphere.mRadius, t))
		{
			CastResult result;
			result.mClientData = reinterpret_cast<void*>(i);
			result.mTime = t;
			results.AddResult(result);
		}
	}
}

void BoundingSphereSpatialPartition::CastFrustum(const Frustum& frustum, CastResults& results)
{
	// All cast operations should be linear time.
	for (size_t i = 0; i < mData.size(); ++i)
	{
		// dont process empty slots
		if (!mData[i]) continue;

		// check if colliding
		Sphere sphere = mData[i]->mBoundingSphere;
		size_t lastAxis = 0; // this is required to use the function, but we dont actually use it for anything
		IntersectionType::Type type = FrustumSphere(frustum.GetPlanes(), sphere.mCenter, sphere.mRadius, lastAxis);
		if (type == IntersectionType::Inside || type == IntersectionType::Overlaps)
		{
			results.AddResult(CastResult(reinterpret_cast<void*>(i)));
		}
	}
}

void BoundingSphereSpatialPartition::SelfQuery(QueryResults& results)
{
	// SelfQuery should be no worse than quadratic time. 
	for (size_t i = 0; i < mData.size(); ++i)
	{
		if (!mData[i]) continue;
		for (size_t j = i + 1; j < mData.size(); ++j)
		{
			if (!mData[j]) continue;

			Sphere sphere0 = mData[i]->mBoundingSphere;
			Sphere sphere1 = mData[j]->mBoundingSphere;

			if (SphereSphere(sphere0.mCenter, sphere0.mRadius, sphere1.mCenter, sphere1.mRadius))
			{
				results.AddResult(QueryResult(reinterpret_cast<void*>(i), reinterpret_cast<void*>(j)));
			}
		}
	}
}

void BoundingSphereSpatialPartition::FilloutData(std::vector<SpatialPartitionQueryData>& results) const
{
	//Warn("Assignment2: Required function un-implemented");
	// When implementing FilloutData on your spatial partition make sure to only fill out the
	// array with valid data(don’t add empty items) and just ignore mDepth.

	for (size_t i = 0; i < mData.size(); ++i)
	{
		if (!mData[i]) continue;
		results.push_back(SpatialPartitionQueryData(*mData[i]));
	}
}
