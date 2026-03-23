///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////

/* Start Header ------------------------------------------------------
Copyright (C) 2025 DigiPen Institute of Technology.
File Name: DynamicAabbTree.hpp
Purpose: This file provides the interface of a dynamic aabb spatial partition tree and its methods.
Language: ISO C++ 14 Standard.
Platform: Legacy MSVC, x64/x86, Windows 10 OS.
Project: r.marqueztwisdale_CS350_3.
Author: Roman Marquez-Twisdale, r.marqueztwisdale@digipen.edu, 0065807.
Creation date: 03/15/2026
End Header -------------------------------------------------------*/

#pragma once

#include "SpatialPartition.hpp"
#include "Shapes.hpp"

/******Student:Assignment3******/
/// You must implement a dynamic aabb tree as we discussed in class.
class DynamicAabbTree : public SpatialPartition
{
public:
  DynamicAabbTree();
  ~DynamicAabbTree();

  // Spatial Partition Interface
  void InsertData(SpatialPartitionKey& key, SpatialPartitionData& data) override;
  void UpdateData(SpatialPartitionKey& key, SpatialPartitionData& data) override;
  void RemoveData(SpatialPartitionKey& key) override;

  void DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color = Vector4(1), int bitMask = 0) override;

  void CastRay(const Ray& ray, CastResults& results) override;
  void CastFrustum(const Frustum& frustum, CastResults& results) override;

  void SelfQuery(QueryResults& results) override;

  void FilloutData(std::vector<SpatialPartitionQueryData>& results) const override;

  static const float mFatteningFactor;

  // Add your implementation here
private:
	class Node
	{
	public:
		Aabb mAabb;
		void* mClientData;
		Node* mLeft;
		Node* mRight;
		Node* mParent;
		size_t mHeight;
		unsigned mNodeInsertCount;
		size_t mlastAxis;
		int mDepth;
	};

	Node* mRoot;
	unsigned mInsertCount;

	void Balance(Node *alteredNode);
	void UpdateAfterBalance(Node* oldParent, Node* pivot, Node* grandParent);
	Node* SelectNode(Aabb const&insertingAabb, Node* node0, Node* node1);
	Node* ConstructNode(unsigned height, void* clientData, Node* leftChild, Node* rightChild, Node* parent);
	void BacktrackUpdate(Node* leafNode);
	void RecursiveFillOut(Node* node, std::vector<SpatialPartitionQueryData>& results, int depth) const;
	void RecursiveCastRay(Node *node, Ray const& ray, CastResults& results);
	void RecursiveCastFrustum(Node* node, Frustum const& frustum, CastResults& results);
	void RecursiveFillFrustum(Node* node, CastResults& results);

	// self query
	void TreeQuery(Node* nodeA, Node* nodeB, QueryResults &results);
	void TreeQuery(Node* node, QueryResults& results);
	void SplitNodes(Node* nodeA, Node* nodeB, QueryResults& results);

	// cleanup
	void ClearTree(Node *node);
};
