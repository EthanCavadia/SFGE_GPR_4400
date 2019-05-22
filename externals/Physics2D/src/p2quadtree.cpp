#include "..\include\p2quadtree.h"
#include <memory>

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

	// Define the corners of the current node
	const p2Vec2 extends = m_Bounds.GetExtends();

	// Set the current position
	p2Vec2 currentPosition = m_Bounds.bottomLeft;

	// Define the size of the child sides depending on the amount of child tree number
	const float childSideSize = (m_Bounds.topRight.y - currentPosition.y) / sqrt(CHILD_TREE_NMB);

	for (int i = 0; i < CHILD_TREE_NMB; i++)
	{
		p2AABB childAABB;

		childAABB.bottomLeft = currentPosition;

		childAABB.topRight = { currentPosition.x + childSideSize, currentPosition.y + childSideSize };

		// Check if it needs to jump on the y axis
		if (currentPosition.x + childSideSize >= extends.x)
			currentPosition = { m_Bounds.bottomLeft.x, currentPosition.y + childSideSize };
		else
			currentPosition.x = currentPosition.x + childSideSize;

		// Add the node to the child array
		nodes[i] = new p2QuadTree(m_NodeLevel + 1, childAABB);
	}
}

int p2QuadTree::GetIndex(p2Body* obj)
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
}

void p2QuadTree::Insert(p2Body * obj)
{
	if (nodes[0] != nullptr)
	{
		int index = GetIndex(obj);

		if (index != -1)
		{
			nodes[index]->Insert(obj);
			return;
		}
	}
	m_Objects.push_back(obj);

	if (m_Objects.size() > MAX_OBJECTS && m_NodeLevel < MAX_LEVELS)
	{
		if (nodes[0] == nullptr)
		{
			Split();
		}
		for (p2Body* body : m_Objects)
		{
			int i = 0;
			while (i < m_Objects.size())
			{
				int index = GetIndex(body);
				if (index != -1)
				{
					nodes[index]->Insert(body);
				}
				else
				{
					i++;
				}
			}
		}
	}
}

std::vector<p2Body*> p2QuadTree::Retrieve(std::vector<p2Body*> returnObj, p2Body* obj)
{
	int index = GetIndex(obj);
	if (index != -1 && nodes[0] != nullptr)
	{
		nodes[index]->Retrieve(returnObj, obj);
	}

	returnObj.push_back(obj);

	return returnObj;
}
