///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"

const float DynamicAabbTree::mFatteningFactor = 1.1f;

DynamicAabbTree::DynamicAabbTree()
{
	mType = SpatialPartitionTypes::AabbTree;
}

DynamicAabbTree::~DynamicAabbTree()
{
}

void DynamicAabbTree::InsertData(SpatialPartitionKey& key, SpatialPartitionData& data)
{
	Warn("Assignment3: Required function un-implemented");

	// Surface Area Heuristic:
	//		When choosing whether to traverse down the left or right side of the tree when inserting a node,
	//		choose the side that results in the smallest increase of surface area.
	//		If both nodes result in the same increase of surface area then choose the right side.

	// Insertion:
	//		When inserting a node into the tree use the surface area heuristic described in class.
	//		After inserting you must re - balance the tree using rotations.
	//		When you split a leaf node put the old leaf node as the left child and the new node(the one you are inserting) as the right child.
	//		Also make sure to fatten the new aabb’s half extent by the provided mFatteningFactor coefficient before inserting.

	// case 1: empty tree

	// case 2: tree has only one node

	// case 3: tree has two or more nodes and we have to pick one of two branches to traverse
}

void DynamicAabbTree::UpdateData(SpatialPartitionKey& key, SpatialPartitionData& data)
{
	Warn("Assignment3: Required function un-implemented");

	// Update:
	//		If a node fully contains the new aabb then do not alter the tree structure.
	//		Otherwise remove then re - insert the updated node.
}

void DynamicAabbTree::RemoveData(SpatialPartitionKey& key)
{
	Warn("Assignment3: Required function un-implemented");

	// Removal:
	//		After removing the node from the tree you must apply a re - balance to the tree at the sibling of the node that was removed
	//		as well as all parents going up the tree.
	//		Don’t forget that more than one balance can happen!
}

void DynamicAabbTree::DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color, int bitMask)
{
	Warn("Assignment3: Required function un-implemented");

	// Debug Drawing:
	//		The height passed in corresponds to what level of the tree you should draw,
	//		where 0 is the root, 1 is the first set of children, and so on.
	//		If - 1 is passed in draw the entire tree (including all internal nodes).
}

void DynamicAabbTree::CastRay(const Ray& ray, CastResults& results)
{
	Warn("Assignment3: Required function un-implemented");

	// Ray - casting:
	//		You must recursively cast through the tree.
	//		If a node is not hit by the ray then no children should be tested.
	//		Return all client datas of leaf nodes that are hit in the tree.
}

void DynamicAabbTree::CastFrustum(const Frustum& frustum, CastResults& results)
{
	Warn("Assignment3: Required function un-implemented");

	// Frustum - casting:
	//		If you did not already utilize “size_t & lastAxis” in the FrustumAabb test then you must do so now.
	//		This variable should be used to determine which plane axis to test first and should also be filled out with what axis causes the test to fail(if it does).
	//		You must use this when traversing your tree;
	//		- for each node in the tree you should cache what the last axis was that caused a frustum cast to fail.
	//		- In so doing, you can increase the chance to early out based upon last frame’s results.
	//		- This means that when performing a 2nd frustum cast against a scene where the root is off - screen
	//		  the test should terminate with only 1 plane test to cull the entire tree.
	//		- Similarly, if the root is fully contained(not intersecting)
	//		  then all the leaf nodes should be included at a total of only 6 plane tests.
}

void DynamicAabbTree::SelfQuery(QueryResults& results)
{
	Warn("Assignment3: Required function un-implemented");

	// Pair Query:
	//		You must return all pairs in the tree without reporting any duplicates.
	//		You must do this utilizing the method describe in class (the 2 recursive functions).
	//		When you have two internal nodes and you must choose which one to split(which one’s children to check)
	//		you should choose the one with the larger volume.
}

void DynamicAabbTree::FilloutData(std::vector<SpatialPartitionQueryData>& results) const
{
	Warn("Assignment3: Required function un-implemented");

	// Fillout Data:
	//		This function is used for me to investigate the structure of your tree.
	//		You should fill out this array with all of your node’s data(the aabb and client data) in a pre - order depth - first traversal.
	//		That is, add the node’s data, then the left node, then the right node.
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////// - Roman's additions:
void DynamicAabbTree::Balance()
{
	// Balancing:
	//		When balancing the tree, you must use the rotation method described in class.
	//		There is one small ambiguity that can arise when a large and small child can’t be distinguished (they are the same height).
	//		In this case my drivers choose the right node as the smaller(if left < right choose left).
	//		This case should be unlikely to happen.
	// Note:
	//		the tree must be re - balanced after both insertion and removal.
	//		This is a recursive process where you walk up the tree testing each node from the insertion / removal point for an imbalance.
}

/// <summary>
///		This function takes a node and the two branches in question, and determines which branch to take.
/// </summary>
/// <param name="insertingNode"></param>
/// <param name="node0"></param>
/// <param name="node1"></param>
/// <returns></returns>
DynamicAabbTree::Node* DynamicAabbTree::SelectNode(Node* insertingNode, Node* leftNode, Node* rightNode)
{
	// Surface Area Heuristic:
	//		When choosing whether to traverse down the left or right side of the tree when inserting a node,
	//		choose the side that results in the smallest increase of surface area.
	//		If both nodes result in the same increase of surface area then choose the right side.

	// calculate surface area delta for left node
	float leftSurfaceAreaDelta = Aabb::Combine(leftNode->mAabb, insertingNode->mAabb).GetSurfaceArea() - leftNode->mAabb.GetSurfaceArea();
	// calculate surface area delta for right node
	float rightSurfaceAreaDelta = Aabb::Combine(rightNode->mAabb, insertingNode->mAabb).GetSurfaceArea() - rightNode->mAabb.GetSurfaceArea();

	// if delta is smaller going left, return SelectNode left
	if (leftSurfaceAreaDelta < rightSurfaceAreaDelta)
	{
		if (!leftNode->mLeft && !leftNode->mRight) return leftNode;
		return SelectNode(insertingNode, leftNode->mLeft, leftNode->mRight);
	}

	// else return SelectNode right
	else
	{
		if (!rightNode->mLeft && !rightNode->mRight) return rightNode;
		return SelectNode(insertingNode, rightNode->mLeft, rightNode->mRight);
	}
}

/// <summary>
///		This function traverses back up the tree and updates all height values and aabbs along the way.
/// </summary>
/// <param name="leafNode"></param>
void DynamicAabbTree::BacktrackUpdate(Node* leafNode)
{
}



