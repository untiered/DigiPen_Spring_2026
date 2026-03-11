///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"

const float DynamicAabbTree::mFatteningFactor = 1.1f;

DynamicAabbTree::DynamicAabbTree() : mRoot(nullptr)
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
	// CASE 1:
	if (!mRoot)
	{
		mRoot = ConstructLeafNode(&data);
		key.mVoidKey = mRoot;
		return;
	}

	// CASE 2:
	if (!mRoot->mLeft && !mRoot->mRight)
	{
		mRoot->mLeft = ConstructLeafNode(mRoot->mClientData, nullptr, nullptr, mRoot);
		mRoot->mRight = ConstructLeafNode(&data, nullptr, nullptr, mRoot);
		mRoot->mAabb = Aabb::Combine(mRoot->mLeft->mAabb, mRoot->mRight->mAabb);
		mRoot->mClientData = nullptr;
		mRoot->mHeight = 1;
		key.mVoidKey = mRoot->mRight;
		return;
	}

	// CASE 3:
	Node* targetLeaf = SelectNode(reinterpret_cast<SpatialPartitionData*>(&data)->mAabb, mRoot->mLeft, mRoot->mRight);
	targetLeaf->mLeft = ConstructLeafNode(targetLeaf->mClientData, nullptr, nullptr, targetLeaf);
	targetLeaf->mClientData = nullptr;
	targetLeaf->mRight = ConstructLeafNode(&data, nullptr, nullptr, targetLeaf);
	targetLeaf->mHeight = 1;
	key.mVoidKey = targetLeaf->mRight;

	// update heights and aabbs
	BacktrackUpdate(targetLeaf->mRight);

	// balance.. -_-
	Balance(targetLeaf);
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
	// Fillout Data:
	//		This function is used for me to investigate the structure of your tree.
	//		You should fill out this array with all of your node’s data(the aabb and client data) in a pre - order depth - first traversal.
	//		That is, add the node’s data, then the left node, then the right node.
	
	unsigned dataCount = 0;
	RecursiveFillOut(mRoot, results, 0, dataCount);
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
	if (!alteredNode->mLeft || !alteredNode->mRight)
	{
		Balance(alteredNode->mParent);
		return;
	}

	size_t leftSubtreeHeight = alteredNode->mLeft->mHeight;
	size_t rightSubtreeHeight = alteredNode->mRight->mHeight;
	if (Math::Abs(static_cast<int>(leftSubtreeHeight - rightSubtreeHeight)) > 1) // check if the alteredNode is out of balance
	{
		// balance it then call Balance() on the pivot node's parent

		// STEP 1:
		Node* pivot = (leftSubtreeHeight > rightSubtreeHeight) ? alteredNode->mLeft : alteredNode->mRight;
		Node* small = (pivot->mLeft->mHeight < pivot->mRight->mHeight) ? small = pivot->mLeft : small = pivot->mRight;

		// STEP 2: (is pointless)

		// STEP 3: (alteredNode is the "Old Parent" in the slides)
		if (alteredNode->mParent)
			if (alteredNode->mParent->mLeft == alteredNode) { alteredNode->mParent->mLeft = pivot; }
			else { alteredNode->mParent->mRight = pivot; }
		pivot->mParent = alteredNode->mParent; // this could be nullptr

		// STEP 4:
		if (pivot->mLeft == small) { pivot->mLeft = alteredNode; }
		else { pivot->mRight = alteredNode; }
		alteredNode->mParent = pivot;

		// STEP 5:
		if (alteredNode->mLeft == pivot) { alteredNode->mLeft = small; }
		else { alteredNode->mRight = small; }
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
	oldParent->mHeight = (oldParent->mLeft->mHeight > oldParent->mRight->mHeight) ? oldParent->mLeft->mHeight + 1 : oldParent->mRight->mHeight + 1;
	oldParent->mAabb = Aabb::Combine(oldParent->mLeft->mAabb, oldParent->mRight->mAabb);

	pivot->mHeight = (pivot->mLeft->mHeight > pivot->mRight->mHeight) ? pivot->mLeft->mHeight + 1 : pivot->mRight->mHeight + 1;
	pivot->mAabb = Aabb::Combine(pivot->mLeft->mAabb, pivot->mRight->mAabb);

	if (grandParent) // grandParent could be nullptr
	{
		grandParent->mHeight = (grandParent->mLeft->mHeight > grandParent->mRight->mHeight) ? grandParent->mLeft->mHeight + 1 : grandParent->mRight->mHeight + 1;
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
///		This function dynamically allocates a new node based on the provided data,
///		initializes the new node with the mFatteningFactor,
///		and returns a pointer to that new node.
/// </summary>
/// <param name="clientData">: "the client data, this MUST be provided"</param>
/// <param name="leftChild">: "a pointer to the left child node"</param>
/// <param name="rightChild">: "a pointer to the right child node"</param>
/// <param name="parent">: "a pointer to the parent node"</param>
/// <returns>: "a pointer to the new node"</returns>
DynamicAabbTree::Node* DynamicAabbTree::ConstructLeafNode(
	void* clientData,
	Node* leftChild,
	Node* rightChild,
	Node* parent 
)
{
	Node* node = new Node();
	node->mClientData = clientData;
	node->mLeft = leftChild;
	node->mRight = rightChild;
	node->mParent = parent;
	node->mHeight = 0;
	SpatialPartitionData* spd = reinterpret_cast<SpatialPartitionData*>(clientData);
	node->mAabb = Aabb::BuildFromCenterAndHalfExtents(spd->mAabb.GetCenter(), spd->mAabb.GetHalfSize() * mFatteningFactor);
	return node;
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
		temp->mParent->mHeight = temp->mHeight + 1;
		temp = temp->mParent;
		temp->mAabb = Aabb::Combine(temp->mLeft->mAabb, temp->mRight->mAabb);
	}
}

/// <summary>
///		This function will recursively fillout a structure in a pre-order traversal order.
/// </summary>
/// <param name="node">: "the node to work with"</param>
/// <param name="results">: "the structure to fill"</param>
void DynamicAabbTree::RecursiveFillOut(Node* node, std::vector<SpatialPartitionQueryData>& results, int depth, unsigned &dataCount) const
{
	if (!node) return;
	SpatialPartitionQueryData data;
	
	// SpatialPartitionQueryData variables:
	data.mDepth = depth;

	// SpatialPartitionData variables:
	if (!node->mLeft && !node->mRight)
	{
		data.mClientData = reinterpret_cast<void*>(dataCount++);
	} else {
		data.mClientData = reinterpret_cast<void*>(0);
	}
	data.mAabb = node->mAabb;

	results.push_back(data);
	RecursiveFillOut(node->mLeft, results, depth+1, dataCount);
	RecursiveFillOut(node->mRight, results, depth+1, dataCount);
}



