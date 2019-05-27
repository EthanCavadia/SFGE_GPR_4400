#include "..\include\p2quadtree.h"
#include <memory>


p2QuadTree::p2QuadTree(){}

p2QuadTree::p2QuadTree(int nodeLevel, p2AABB bounds)
{
	// Set base values
	m_NodeLevel = nodeLevel;
	m_Bounds = bounds;
}

p2QuadTree::~p2QuadTree()
{
}

void p2QuadTree::Clear()
{
	for (p2QuadTree* quad : nodes)
	{
		quad->Clear();

		for (p2Body* body : quad->m_Objects)
		{
			m_Objects.push_back(body);
		}
		delete(quad);
	}
}

void p2QuadTree::Split()
{
	if (m_NodeLevel > MAX_LEVELS)
		return;
	p2Vec2 currentPosition = m_Bounds.bottomLeft;
	p2Vec2 childSizeExtend = (m_Bounds.GetExtends() / 2.f);
	//float childSideSizeY = (currentPosition.y - m_Bounds.topRight.y) / 2.f;
	//float childSideSizeX = ( m_Bounds.topRight.x - currentPosition.x) / 2.f;
	
	/*// Define the corners of the current node
	const p2Vec2 extends = m_Bounds.GetExtends();

	// Set the current position
	

	// Define the size of the child sides depending on the amount of child tree number
	

	for (int i = 0; i < CHILD_TREE_NMB; i++)
	{
		p2AABB childAABB;

		childAABB.bottomLeft = currentPosition;

		childAABB.topRight = { currentPosition.x + childSideSizeX, currentPosition.y + childSideSizeY };

		// Check if it needs to jump on the y axis
		if (currentPosition.x + childSideSizeX >= extends.x)
			currentPosition = { m_Bounds.bottomLeft.x, currentPosition.y + childSideSizeY };
		else
			currentPosition.x = currentPosition.x + childSideSizeY;

		// Add the node to the child array
		nodes[i] = new p2QuadTree(m_NodeLevel + 1, childAABB);
	}*/

	p2AABB childAABB = p2AABB();

	//First quad Top Right child
	childAABB.bottomLeft = m_Bounds.GetCenter();
	childAABB.topRight = m_Bounds.topRight;
	nodes[0] = new p2QuadTree(m_NodeLevel + 1, childAABB);

	//Top left child
	childAABB.bottomLeft = currentPosition - p2Vec2(0,childSizeExtend.y);
	childAABB.topRight = childAABB.bottomLeft + childAABB.GetExtends();
	nodes[1] = new p2QuadTree(m_NodeLevel + 1, childAABB);

	//Bottom left child
	childAABB.bottomLeft = m_Bounds.bottomLeft;
	childAABB.topRight = m_Bounds.GetCenter();
	nodes[2] = new p2QuadTree(m_NodeLevel + 1, childAABB);

	//Bottom right child
	childAABB.bottomLeft = m_Bounds.GetCenter() + p2Vec2(0, childSizeExtend.y);
	childAABB.topRight = m_Bounds.GetCenter() + p2Vec2(0, childSizeExtend.x);
	nodes[3] = new p2QuadTree(m_NodeLevel + 1, childAABB);
}

/*int p2QuadTree::GetIndex(p2Body* obj)
{
	int index = -1;
	
	float verticalMidPoint = obj->GetAABB().GetExtends().x + (obj->GetAABB().GetExtends().x / 2);
	float horizontalMidPoint = obj->GetAABB().GetExtends().y + (obj->GetAABB().GetExtends().y / 2);

	bool topQuad = obj->GetAABB().GetExtends().y < horizontalMidPoint && obj->GetAABB().GetExtends().y + obj->GetAABB().bottomLeft.y < horizontalMidPoint;
	bool bottomQuad = obj->GetAABB().GetExtends().y > horizontalMidPoint;

	if (obj->GetAABB().GetExtends().x < verticalMidPoint && obj->GetAABB().GetExtends().x + obj->GetAABB().bottomLeft.x < verticalMidPoint)
	{
		if (topQuad)
		{
			index = 1;
		}
		else if (bottomQuad)
		{
			index = 2;
		}
	}
	else if (obj->GetAABB().GetExtends().x > verticalMidPoint)
	{
		if (topQuad)
		{
			index = 0;
		}
		else if (bottomQuad)
		{
			index = 3;
		}
	}
	return index;
}*/

void p2QuadTree::Insert(p2Body* obj)
{
	if (nodes[0] != nullptr)
	{
		if (m_Objects.size() < MAX_OBJECTS)
		{
			//Insert body here
			m_Objects.push_back(obj);
		}
		else
		{
			if (m_NodeLevel <= MAX_LEVELS)
			{
				Split();
			//Insert my bodies in children
				for each (auto obj1 in m_Objects)
				{
					std::vector<p2QuadTree*> eligibleChildren;
					//Insert object in children
					for (int i = 0; i < CHILD_TREE_NMB; i++)
					{
						if (FindEligibleChild(obj1))
						{
							eligibleChildren.push_back(nodes[i]);
						}
					}
					if (eligibleChildren.size() == 1)
					{
						eligibleChildren.at(0)->Insert(obj1);
					}
					else
					{
						m_Objects.push_back(obj1);
					}
				}
				//Add last body
				std::vector<p2QuadTree*> eligibleChildren;
				//Insert object in children
				for (int i = 0; i < CHILD_TREE_NMB; i++)
				{
					if (FindEligibleChild(obj))
					{
						eligibleChildren.push_back(nodes[i]);
					}
				}
				if (eligibleChildren.size() == 1)
				{
					eligibleChildren.at(0)->Insert(obj);
				}
				else
				{
					m_Objects.push_back(obj);
				}
			}
			else
			{
				//Insert body here
				m_Objects.push_back(obj);
			}
		}
	}
	else
	{
		std::vector<p2QuadTree*> eligibleChildren;
		//Insert object in children
		for (int i = 0; i < CHILD_TREE_NMB; i++)
		{
			if (FindEligibleChild(obj))
			{
				eligibleChildren.push_back(nodes[i]);
			}
		}
		if (eligibleChildren.size() == 1)
		{
			eligibleChildren.at(0)->Insert(obj);
		}
		else
		{
			m_Objects.push_back(obj);
		}
	}
}

void p2QuadTree::Retrieve(std::vector<p2Body*>& returnObj)
{
	for each (auto* obj in m_Objects)
	{
		returnObj.push_back(obj);
	}

	for (int i = 0; i < CHILD_TREE_NMB; i++)
	{
		nodes[i]->Retrieve(returnObj);
	}
}

bool p2QuadTree::FindEligibleChild(p2Body* obj)
{
	if (m_Objects.size() < MAX_OBJECTS)
	{
		return true;
	}
	else
	{
		if (m_NodeLevel < MAX_LEVELS)
		{
			return true;
		}
		return false;
	}
}

void p2QuadTree::GetAABBRecursively(std::vector<p2AABB>& quad)
{
	quad.push_back(m_Bounds);
	for (int i = 0; i < CHILD_TREE_NMB; i++)
	{
		nodes[i]->GetAABBRecursively(quad);
	}
}