///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////

/* Start Header ------------------------------------------------------
Copyright (C) 2025 DigiPen Institute of Technology.
File Name: DynamicAabbTree.cpp
Purpose: This file provides the implementation of a dynamic aabb spatial partition tree and its methods.
Language: ISO C++ 14 Standard.
Platform: Legacy MSVC, x64/x86, Windows 10 OS.
Project: r.marqueztwisdale_CS350_3.
Author: Roman Marquez-Twisdale, r.marqueztwisdale@digipen.edu, 0065807.
Creation date: 03/15/2026
End Header -------------------------------------------------------*/


#include "Precompiled.hpp"

const float DynamicAabbTree::mFatteningFactor = 1.1f;

DynamicAabbTree::DynamicAabbTree() : mRoot(nullptr), mInsertCount(0)
{
	mType = SpatialPartitionTypes::AabbTree;
}

DynamicAabbTree::~DynamicAabbTree()
{
}

// Surface Area Heuristic:
//		When choosing whether to traverse down the left or right side of the tree when inserting a node,
//		choose the side that results in the smallest increase of surface area.
//		If both nodes result in the same increase of surface area then choose the right side.
//
// Insertion:
//		When inserting a node into the tree use the surface area heuristic described in class.
//		After inserting you must re - balance the tree using rotations.
//		When you split a leaf node put the old leaf node as the left child and the new node(the one you are inserting) as the right child.
//		Also make sure to fatten the new aabb’s half extent by the provided mFatteningFactor coefficient before inserting.
//
// case 1: empty tree
//	- this is only true if the root node is nullptr.
//	- so just initialize the root node with the new data.
//	- set the key pointer value to the inserted node.
//
// case 2: tree has only one node
//	- create the left child with the old data.
//	- create the right child with new data.
//	- update the root node's aabb.
//	- update the root node's height.
//	- set the key pointer value to the inserted node.
//
// case 3: tree has two or more nodes and we have to pick one of two branches to traverse
//	- create a leaf node to hold the inserted data.
//	- use SelectNode() to find the leaf we are going to split.
//	- split the leaf in the same way we split the old root for case 2.
//	- balance the tree using rotations.
//	- set the key pointer value to the inserted node.
void DynamicAabbTree::InsertData(SpatialPartitionKey& key, SpatialPartitionData& data)
{
	// DONT FORGET THAT THE KEYS ALWAYS NEED TO POINT TO THEIR NODE (VALID MEMORY)!!!!!
	// CASE 1: empty tree
	if (!mRoot)
	{
		mRoot = ConstructNode(
			0,			// height
			&data,		// data
			nullptr,	// left child
			nullptr,	// right child
			nullptr		// parent
		);
		key.mVoidKey = mRoot;
		mRoot->mNodeInsertCount = mInsertCount++;
		return;
	}

	// CASE 2: only the root node exists
	if (!mRoot->mLeft && !mRoot->mRight)
	{
		// keep track of the Node being referenced by an existing key
		Node* existingKeyNode = mRoot;
		Node* newNode = ConstructNode(0, &data, nullptr, nullptr, nullptr);	// NEED TO SET ITS PARENT LATER!!!!
		mRoot = ConstructNode(
			1,													// height
			nullptr,											// data
			existingKeyNode,									// left child
			newNode,											// right child
			nullptr												// parent
		);
		// set parents for both leaf nodes (old and new)
		existingKeyNode->mParent = mRoot;
		newNode->mParent = mRoot;
		// update the new key
		key.mVoidKey = newNode;
		newNode->mNodeInsertCount = mInsertCount++;
		return;
	}

	// CASE 3: there are at least two leaf nodes in the tree
	// keep track of the existing nodes being referenced by keys
	Node* existingKeyNode = SelectNode(reinterpret_cast<SpatialPartitionData*>(&data)->mAabb, mRoot->mLeft, mRoot->mRight);
	Node* newNode = ConstructNode(0, &data, nullptr, nullptr, nullptr);	// NEED TO SET ITS PARENT LATER!!!!
	Node* newInternalNode = ConstructNode(
		static_cast<unsigned>(Math::Max(existingKeyNode->mHeight, newNode->mHeight) + 1),							// height
		nullptr,																			// data
		existingKeyNode,																	// left child
		newNode,																			// right child
		existingKeyNode->mParent															// parent
	);
	// make sure the tree sees the new internal node
	if (existingKeyNode->mParent->mLeft == existingKeyNode)
	{
		existingKeyNode->mParent->mLeft = newInternalNode;
	} else {
		existingKeyNode->mParent->mRight = newInternalNode;
	}
	// set parents for both leaf nodes (old and new)
	existingKeyNode->mParent = newInternalNode;
	newNode->mParent = newInternalNode;
	// update the new key
	key.mVoidKey = newNode;
	newNode->mNodeInsertCount = mInsertCount++;

	// update heights and aabbs from newNode
	BacktrackUpdate(newNode);

	// balance starting with the node whos children were adjusted
	Balance(newInternalNode);
}

// Update:
//		If a node fully contains the new aabb then do not alter the tree structure.
//		Otherwise remove then re - insert the updated node.
void DynamicAabbTree::UpdateData(SpatialPartitionKey& key, SpatialPartitionData& data)
{
	if (!key.mVoidKey) return;

	Node* nodeToUpdate = reinterpret_cast<Node*>(key.mVoidKey);
	if (!nodeToUpdate->mAabb.Contains(data.mAabb))
	{
		// record the insert count
		unsigned oldInsertCount = nodeToUpdate->mNodeInsertCount;

		// remove
		RemoveData(key);

		// re-insert
		InsertData(key, data);

		// set the insert count
		Node* reinsertedNode = reinterpret_cast<Node*>(key.mVoidKey);
		reinsertedNode->mNodeInsertCount = oldInsertCount;
	}
}

// Removal:
//		After removing the node from the tree you must apply a re - balance to the tree at the sibling of the node that was removed
//		as well as all parents going up the tree.
//		Don’t forget that more than one balance can happen!
void DynamicAabbTree::RemoveData(SpatialPartitionKey& key)
{
	if (!mRoot) return;
	if (!key.mVoidKey) return;

	// DONT FORGET THAT THE KEYS ALWAYS NEED TO POINT TO VALID MEMORY!!!!!

	// key.mVoidKey is a pointer to the node we should delete
	Node* nodeToDelete = reinterpret_cast<Node*>(key.mVoidKey);

	// CASE 1: there is only one node in the tree
	if (nodeToDelete == mRoot)
	{
		delete mRoot;
		mRoot = nullptr;
		key.mVoidKey = nullptr;
		return;
	}

	// CASE 2: the node to delete is at a depth of 1
	if (nodeToDelete->mParent == mRoot)
	{
		Node* sibling = nullptr;
		if (mRoot->mLeft == nodeToDelete)
		{
			sibling = mRoot->mRight;
		} else {
			sibling = mRoot->mLeft;
		}
		delete nodeToDelete;
		key.mVoidKey = nullptr;
		delete mRoot;
		mRoot = sibling;
		mRoot->mParent = nullptr;
		return;
	}

	// CASE 3: the node to delete is at a depth of 2 or more
	Node* parent = nodeToDelete->mParent;
	Node* grandParent = parent->mParent;
	Node* sibling = nullptr;

	// get the sibling
	if (parent->mLeft == nodeToDelete)
	{
		sibling = parent->mRight;
	} else {
		sibling = parent->mLeft;
	}

	// connect the grandparent and the sibling
	if (grandParent->mLeft == parent)
	{
		grandParent->mLeft = sibling;
	} else {
		grandParent->mRight = sibling;
	}
	sibling->mParent = grandParent;

	// delete nodeToDelete and parent nodes
	delete nodeToDelete;
	key.mVoidKey = nullptr;
	delete parent;

	// update heights and aabbs
	BacktrackUpdate(sibling);

	// balance from the lowest node who's children were effected
	Balance(grandParent);
}

// Debug Drawing:
//		The height passed in corresponds to what level of the tree you should draw,
//		where 0 is the root, 1 is the first set of children, and so on.
//		If - 1 is passed in draw the entire tree (including all internal nodes).
void DynamicAabbTree::DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color, int bitMask)
{
	Warn("Assignment2: Required function un-implemented");
}

// Ray - casting:
//		You must recursively cast through the tree.
//		If a node is not hit by the ray then no children should be tested.
//		Return all client datas of leaf nodes that are hit in the tree.
void DynamicAabbTree::CastRay(const Ray& ray, CastResults& results)
{
	RecursiveCastRay(mRoot, ray, results);
}

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
void DynamicAabbTree::CastFrustum(const Frustum& frustum, CastResults& results)
{
	RecursiveCastFrustum(mRoot, frustum, results);
}

// Pair Query:
//		You must return all pairs in the tree without reporting any duplicates.
//		You must do this utilizing the method describe in class (the 2 recursive functions).
//		When you have two internal nodes and you must choose which one to split(which one’s children to check)
//		you should choose the one with the larger volume.
void DynamicAabbTree::SelfQuery(QueryResults& results)
{
	TreeQuery(mRoot, results);
}

// Fillout Data:
//		This function is used for me to investigate the structure of your tree.
//		You should fill out this array with all of your node’s data(the aabb and client data) in a pre - order depth - first traversal.
//		That is, add the node’s data, then the left node, then the right node.
void DynamicAabbTree::FilloutData(std::vector<SpatialPartitionQueryData>& results) const
{
	RecursiveFillOut(mRoot, results, 0);
}














/////////////////////////////////////////////////////////////////////////////////////////////////////////// - Roman's additions:

/// <summary>
///		This function recursively traverses back up the tree from a starting altered node,
///		balancing along the way.
//		Balancing:
//			When balancing the tree, you must use the rotation method described in class.
//			There is one small ambiguity that can arise when a large and small child can’t be distinguished (they are the same height).
//			In this case my drivers choose the right node as the smaller(if left < right choose left).
//			This case should be unlikely to happen.
//		Note:
//			the tree must be re - balanced after both insertion and removal.
//			This is a recursive process where you walk up the tree testing each node from the insertion / removal point for an imbalance.
// 
//		Fix any unbalanced sub - trees(using height)
//			1. Identify pivot, small child, and large child
//			2. Detach small child, pivot, and old parent
//			3. Replace the grand - parent link of the old parent to the pivot
//			4. Insert old parent as new small - child
//			5. Insert small child where the pivot node was
/// </summary>
/// <param name="alteredNode">: "the node we will start at as we move up the tree checking for balance"</param>
void DynamicAabbTree::Balance(Node *alteredNode)
{
	if (!alteredNode) return;
	// if passed node is a leaf node call Balance() on its parent
	if (!alteredNode->mLeft && !alteredNode->mRight)
	{
		Balance(alteredNode->mParent);
		return;
	}
	if (!alteredNode->mLeft || !alteredNode->mRight)
	{
		Warn("fatal error in Balance()");
		return;
	}

	size_t leftSubtreeHeight = alteredNode->mLeft->mHeight;
	size_t rightSubtreeHeight = alteredNode->mRight->mHeight;
	if (Math::Abs(static_cast<int>(leftSubtreeHeight - rightSubtreeHeight)) > 1) // check if the alteredNode is out of balance
	{
		// balance it then call Balance() on the pivot node's parent

		// STEP 1:
		Node* pivot = ((leftSubtreeHeight > rightSubtreeHeight) ? alteredNode->mLeft : alteredNode->mRight);
		Node* small = ((pivot->mLeft->mHeight < pivot->mRight->mHeight) ? pivot->mLeft : pivot->mRight);

		// STEP 2: (is pointless)

		// STEP 3: (alteredNode is the "Old Parent" in the slides)
		if (alteredNode->mParent)
		{
			if (alteredNode->mParent->mLeft == alteredNode)
			{
				alteredNode->mParent->mLeft = pivot;
			} else {
				alteredNode->mParent->mRight = pivot;
			}
		}
		pivot->mParent = alteredNode->mParent; // this could be nullptr

		// STEP 4:
		if (pivot->mLeft == small)
		{
			pivot->mLeft = alteredNode;
		} else {
			pivot->mRight = alteredNode;
		}
		alteredNode->mParent = pivot;

		// STEP 5:
		if (alteredNode->mLeft == pivot)
		{
			alteredNode->mLeft = small;
		} else {
			alteredNode->mRight = small;
		}
		small->mParent = alteredNode;

		// if a node's subtrees change, its height could have changed
		// nodes who's subtrees changed during this rotation:
		//	- grandParent (pivot->mParent)
		//	- pivot
		//	- alteredNode
		UpdateAfterBalance(alteredNode, pivot, pivot->mParent);

		// TODO: Do we need to check if the pivot node is balanced?

		if (!pivot->mParent) mRoot = pivot;
		Balance(pivot->mParent);
	} else {
		Balance(alteredNode->mParent);
	}
}

/// <summary>
///		This function updates the heights and aabbs of the three potentially affected nodes in a rotation operation.
/// </summary>
/// <param name="oldParent">: "the old parent node (the node which was unbalanced)"</param>
/// <param name="pivot">: "the pivot node (the node which tool the old parent's spot)"</param>
/// <param name="grandParent">: "the parent of the old parent (could be nullptr)"</param>
void DynamicAabbTree::UpdateAfterBalance(Node* oldParent, Node* pivot, Node* grandParent)
{
	// start with the lowest
	oldParent->mHeight = Math::Max(oldParent->mLeft->mHeight, oldParent->mRight->mHeight) + 1;
	oldParent->mAabb = Aabb::Combine(oldParent->mLeft->mAabb, oldParent->mRight->mAabb);

	pivot->mHeight = Math::Max(pivot->mLeft->mHeight, pivot->mRight->mHeight) + 1;
	pivot->mAabb = Aabb::Combine(pivot->mLeft->mAabb, pivot->mRight->mAabb);

	if (grandParent) // grandParent could be nullptr
	{
		grandParent->mHeight = Math::Max(grandParent->mLeft->mHeight, grandParent->mRight->mHeight) + 1;
		grandParent->mAabb = Aabb::Combine(grandParent->mLeft->mAabb, grandParent->mRight->mAabb);
	}
}

/// <summary>
///		This function takes a node and the two branches in question,
///		recursively traverses the tree based on a surface area heuristic,
///		determining which branches to traverse, and returns the final resulting leaf node.
/// </summary>
/// <param name="insertingNode">: "the node we want to insert"</param>
/// <param name="node0">: "the left child node"</param>
/// <param name="node1">: "the right child node"</param>
/// <returns>: "a pointer to the selected leaf node"</returns>
DynamicAabbTree::Node* DynamicAabbTree::SelectNode(Aabb const&insertingAabb, Node* leftNode, Node* rightNode)
{
	// Surface Area Heuristic:
	//		When choosing whether to traverse down the left or right side of the tree when inserting a node,
	//		choose the side that results in the smallest increase of surface area.
	//		If both nodes result in the same increase of surface area then choose the right side.

	// calculate surface area delta for left node
	float leftSurfaceAreaDelta = Aabb::Combine(leftNode->mAabb, insertingAabb).GetSurfaceArea() - leftNode->mAabb.GetSurfaceArea();
	// calculate surface area delta for right node
	float rightSurfaceAreaDelta = Aabb::Combine(rightNode->mAabb, insertingAabb).GetSurfaceArea() - rightNode->mAabb.GetSurfaceArea();

	// if delta is smaller going left, return SelectNode left
	if (leftSurfaceAreaDelta < rightSurfaceAreaDelta)
	{
		if (!leftNode->mLeft && !leftNode->mRight) return leftNode;
		return SelectNode(insertingAabb, leftNode->mLeft, leftNode->mRight);
	}

	// else return SelectNode right
	else
	{
		if (!rightNode->mLeft && !rightNode->mRight) return rightNode;
		return SelectNode(insertingAabb, rightNode->mLeft, rightNode->mRight);
	}
}

/// <summary>
///		This function allocates a new Node structure and initializes it with the passed values.
/// </summary>
/// <param name="height">: "the height is 0 for leaf nodes, the root has the greatest height"</param>
/// <param name="clientData">: "a pointer to some kind of client data"</param>
/// <param name="leftChild">: "a pointer to a left subtree if any"</param>
/// <param name="rightChild">: "a pointer to a right subtree if any"</param>
/// <param name="parent">: "a pointer to the parent node if any"</param>
/// <returns>: "a pointer to the allocated Node structure"</returns>
DynamicAabbTree::Node* DynamicAabbTree::ConstructNode(unsigned height, void* clientData, Node* leftChild, Node* rightChild, Node* parent)
{
	Node* node = new Node();
	if (node)
	{
		node->mHeight = height;
		node->mClientData = clientData;
		node->mLeft = leftChild;
		node->mRight = rightChild;
		node->mParent = parent;

		// either a leaf node with valid client data
		if (height == 0 && clientData)
		{
			SpatialPartitionData* spd = reinterpret_cast<SpatialPartitionData*>(clientData);
			node->mAabb = Aabb::BuildFromCenterAndHalfExtents(spd->mAabb.GetCenter(), spd->mAabb.GetHalfSize() * mFatteningFactor);
		}
		
		// or an internal node with no client data, but valid left AND right subtrees
		else if (height > 0 && node->mLeft && node->mRight)
		{
			node->mAabb = Aabb::Combine(node->mLeft->mAabb, node->mRight->mAabb);
		}

		// or something had gone horribly wrong
		else
		{
			Warn("ConstructNode(): something has gone horribly wrong!");
			return nullptr;
		}

		return node;
	}
	Warn("ConstructNode() returned nullptr");
	return nullptr;
}

/// <summary>
///		This function traverses back up the tree and updates all height values and aabbs along the way.
/// </summary>
/// <param name="leafNode">: "the leaf node to start at"</param>
void DynamicAabbTree::BacktrackUpdate(Node* leafNode)
{
	Node* temp = leafNode;
	while (temp->mParent)
	{
		temp->mParent->mHeight = Math::Max(temp->mParent->mLeft->mHeight, temp->mParent->mRight->mHeight) + 1;
		temp = temp->mParent;
		if (temp->mLeft && temp->mRight)
		{
			temp->mAabb = Aabb::Combine(temp->mLeft->mAabb, temp->mRight->mAabb);
		}
	}
}

/// <summary>
///		This function will recursively fillout a structure in a pre-order traversal order.
/// </summary>
/// <param name="node">: "the node to work with"</param>
/// <param name="results">: "the structure to fill"</param>
/// <param name="depth">: "the depth of the node being filled out"</param>
void DynamicAabbTree::RecursiveFillOut(Node* node, std::vector<SpatialPartitionQueryData>& results, int depth) const
{
	if (!node) return;
	SpatialPartitionQueryData data;
	
	// SpatialPartitionQueryData variables:
	data.mDepth = depth;

	// SpatialPartitionData variables:
	if (!node->mLeft && !node->mRight)
	{
		//data.mClientData = reinterpret_cast<void*>(dataCount++);
		data.mClientData = reinterpret_cast<void*>(static_cast<unsigned long long>(node->mNodeInsertCount));
	} else {
		data.mClientData = reinterpret_cast<void*>(0);
	}
	data.mAabb = node->mAabb;

	results.push_back(data);
	RecursiveFillOut(node->mLeft, results, depth+1);
	RecursiveFillOut(node->mRight, results, depth+1);
}

/// <summary>
///		This function recursively traverses the tree in pre-order traversal order,
///		and fillout a CastResults container based on ray cast results for each node.
/// </summary>
/// <param name="ray">: "the ray we are casting"</param>
/// <param name="results">: "the container for storing ray cast results"</param>
void DynamicAabbTree::RecursiveCastRay(Node *node, Ray const& ray, CastResults& results)
{
	// if node is null, return
	if (!node) return;

	// test
	float t = 0.0f;
	if (RayAabb(ray.mStart, ray.mDirection, node->mAabb.mMin, node->mAabb.mMax, t))
	{
		if (!node->mLeft && !node->mRight)
		{
			CastResult result;
			result.mClientData = reinterpret_cast<void*>(static_cast<unsigned long long>(node->mNodeInsertCount));
			result.mTime = t;
			results.AddResult(result);
		}

		RecursiveCastRay(node->mLeft, ray, results);
		RecursiveCastRay(node->mRight, ray, results);
	}
}

/// <summary>
///		This function recursively traverses the tree in pre-order traversal order,
///		and fillout a CasrResults container based on frustum cast results for each node.
/// </summary>
/// <param name="node"></param>
/// <param name="frustum"></param>
/// <param name="results"></param>
void DynamicAabbTree::RecursiveCastFrustum(Node* node, Frustum const& frustum, CastResults& results)
{
	if (!node) return;

	IntersectionType::Type it = FrustumAabb(frustum.GetPlanes(), node->mAabb.mMin, node->mAabb.mMax, node->mlastAxis);
	if (it == IntersectionType::Outside)
	{
		//Warn("outside");
		return;
	}
	if (it == IntersectionType::Overlaps)
	{
		//Warn("Overlaps");
		if (!node->mLeft && !node->mRight)
		{
			CastResult result;
			result.mClientData = reinterpret_cast<void*>(static_cast<unsigned long long>(node->mNodeInsertCount));
			results.AddResult(result);
		}

		RecursiveCastFrustum(node->mLeft, frustum, results);
		RecursiveCastFrustum(node->mRight, frustum, results);
	} else if (it == IntersectionType::Inside)
	{
		//Warn("Inside");
		if (!node->mLeft && !node->mRight)
		{
			CastResult result;
			result.mClientData = reinterpret_cast<void*>(static_cast<unsigned long long>(node->mNodeInsertCount));
			results.AddResult(result);
		}
		RecursiveFillFrustum(node->mLeft, results);
		RecursiveFillFrustum(node->mRight, results);
	}
}

/// <summary>
///		This function simply fills the results container with all leaf nodes starting from the node passed in.
/// </summary>
/// <param name="node"></param>
/// <param name="results"></param>
void DynamicAabbTree::RecursiveFillFrustum(Node* node, CastResults& results)
{
	if (!node) return;
	if (!node->mLeft && !node->mRight)
	{
		CastResult result;
		result.mClientData = reinterpret_cast<void*>(static_cast<unsigned long long>(node->mNodeInsertCount));
		results.AddResult(result);
	}
	RecursiveFillFrustum(node->mLeft, results);
	RecursiveFillFrustum(node->mRight, results);
}

// self query helpers

/// <summary>
///		this function handles the node itself
/// </summary>
/// <param name="nodeA"></param>
/// <param name="nodeB"></param>
void DynamicAabbTree::TreeQuery(Node* nodeA, Node* nodeB, QueryResults &results)
{
	if (AabbAabb(nodeA->mAabb.mMin, nodeA->mAabb.mMax, nodeB->mAabb.mMin, nodeB->mAabb.mMax))
	{
		// Case 1: Both are leaf
		if (!nodeA->mLeft && !nodeA->mRight && !nodeB->mLeft && !nodeB->mRight)
		{
			results.AddResult(QueryResult(reinterpret_cast<void*>(static_cast<unsigned long long>(nodeA->mNodeInsertCount)), reinterpret_cast<void*>(static_cast<unsigned long long>(nodeB->mNodeInsertCount))));
		}
		// Case 2: One internal, one leaf
		else if (!nodeA->mLeft && !nodeA->mRight) { // nodeA is a leaf node
			TreeQuery(nodeA, nodeB->mLeft, results);
			TreeQuery(nodeA, nodeB->mRight, results);
		}
		else if (!nodeB->mLeft && !nodeB->mRight) { // nodeB is a leaf node
			TreeQuery(nodeB, nodeA->mLeft, results);
			TreeQuery(nodeB, nodeA->mRight, results);
		}
		// Case 3: Both are internal, split the nodes
		else {
			SplitNodes(nodeA, nodeB, results);
		}
	}
}

/// <summary>
///		This function decides what to do with the child nodes
/// </summary>
/// <param name="node"></param>
void DynamicAabbTree::TreeQuery(Node* node, QueryResults &results)
{
	if (!node->mLeft && !node->mRight) return;

	// Check the two children against each other
	TreeQuery(node->mLeft, node->mRight, results);
	// Recurse on the left and the right tree
	TreeQuery(node->mLeft, results);
	TreeQuery(node->mRight, results);
}

/// <summary>
///		this function splits the nodes based on volume
/// </summary>
/// <param name="nodeA"></param>
/// <param name="nodeB"></param>
void DynamicAabbTree::SplitNodes(Node* nodeA, Node* nodeB, QueryResults &results)
{
	// split the node with largest volume
	float volumeA = nodeA->mAabb.GetVolume();
	float volumeB = nodeB->mAabb.GetVolume();
	if (volumeB > volumeA)
	{
		TreeQuery(nodeA, nodeB->mLeft, results);
		TreeQuery(nodeA, nodeB->mRight, results);
	}
	else
	{
		TreeQuery(nodeA->mLeft, nodeB, results);
		TreeQuery(nodeA->mRight, nodeB, results);
	}
}



