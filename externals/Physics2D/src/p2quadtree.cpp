#include "p2quadtree.h"
#include <memory>


p2QuadTree::p2QuadTree()
{
	m_NodeLevel = -1;
	m_Bounds = p2AABB();
	m_HasChildren = false;
	nodes[0] = nullptr;
	nodes[1] = nullptr;
	nodes[2] = nullptr;
	nodes[3] = nullptr;
	
	m_Objects = std::vector<p2Body*>();
}

p2QuadTree::p2QuadTree(int nodeLevel, p2AABB bounds)
{
	// Set base values
	m_NodeLevel = nodeLevel;
	m_Bounds = bounds;
	m_HasChildren = false;
	nodes[0] = nullptr;
	nodes[1] = nullptr;
	nodes[2] = nullptr;
	nodes[3] = nullptr;
	m_Objects = std::vector<p2Body*>();
}

p2QuadTree::~p2QuadTree()
{
}

p2QuadTree& p2QuadTree::operator=(p2QuadTree & other)
{
	m_NodeLevel = other.m_NodeLevel;
	m_Bounds = other.m_Bounds;
	m_HasChildren = other.m_HasChildren;
	nodes[0] = std::move(other.nodes[0]);
	nodes[1] = std::move(other.nodes[1]);
	nodes[2] = std::move(other.nodes[2]);
	nodes[3] = std::move(other.nodes[3]);
	m_Objects = other.m_Objects;
	return *this;
}

void p2QuadTree::Clear()
{
	if (m_HasChildren)
	{
		for (int i = 0; i < CHILD_TREE_NMB; i++)
		{
			for (p2Body* body : nodes[i]->m_Objects)
			{
				nodes[i]->Clear();
			}

			nodes[i] = nullptr;
		}
	}
	if (m_NodeLevel == 0 && m_Objects.size() > 0)
	{
		m_Objects.clear();
	}

	m_HasChildren = false;
}

void p2QuadTree::Split()
{
	if (m_NodeLevel > MAX_LEVELS)
		return;

	p2Vec2 currentPosition = m_Bounds.bottomLeft;
	p2Vec2 childSizeExtend = (m_Bounds.GetExtends() / 2.f);
	
	m_HasChildren = true;

	p2AABB childAABB = p2AABB();

	for (int i = 0; i < CHILD_TREE_NMB; i++)
	{
		nodes[i] = std::make_unique<p2QuadTree>();
		nodes[i]->m_NodeLevel = m_NodeLevel + 1;
	}

	//First quad Top Right child
	childAABB.bottomLeft = m_Bounds.GetCenter();
	childAABB.topRight = m_Bounds.topRight;
	nodes[0]->m_Bounds = childAABB;

	//Top left child
	childAABB.bottomLeft = currentPosition - p2Vec2(0,childSizeExtend.y);
	childAABB.topRight = childAABB.bottomLeft + childAABB.GetExtends();
	nodes[1]->m_Bounds = childAABB;

	//Bottom left child
	childAABB.bottomLeft = m_Bounds.bottomLeft;
	childAABB.topRight = m_Bounds.GetCenter();
	nodes[2]->m_Bounds = childAABB;

	//Bottom right child
	childAABB.bottomLeft = m_Bounds.GetCenter() + p2Vec2(0, childSizeExtend.y);
	childAABB.topRight = m_Bounds.GetCenter() + p2Vec2(0, childSizeExtend.x);
	nodes[3]->m_Bounds = childAABB;
}

void p2QuadTree::Insert(p2Body* obj)
{
	if (m_HasChildren)
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
				std::vector<p2Body*> m_ObjectCopy = m_Objects;
				m_Objects.clear();
				for each (p2Body* obj1 in m_ObjectCopy)
				{
					std::vector<p2QuadTree*> eligibleChildren;
					//Insert object in children
					for (int i = 0; i < CHILD_TREE_NMB; i++)
					{
						if (FindEligibleChild(obj1))
						{
							eligibleChildren.push_back(nodes[i].get());
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
						eligibleChildren.push_back(nodes[i].get());
					}
					if (eligibleChildren.size() < 2 && eligibleChildren.size() > 0)
					{
						eligibleChildren[0]->Insert(obj);
					}
					else
					{
						m_Objects.push_back(obj);
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
				eligibleChildren.push_back(nodes[i].get());
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
	if (m_HasChildren)
	{
		for (p2Body* obj : m_Objects)
		{
			returnObj.push_back(obj);
		}

		for (int i = 0; i < CHILD_TREE_NMB; i++)
		{
			nodes[i]->Retrieve(returnObj);
		}
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

void p2QuadTree::GetAABBRecursively(std::vector<p2AABB>& quad) const
{
	quad.push_back(m_Bounds);
	if (m_HasChildren)
	{
		for (int i = 0; i < CHILD_TREE_NMB; i++)
		{
			nodes[i]->GetAABBRecursively(quad);
		}
	}
}