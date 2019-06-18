#include "p2quadtree.h"
#include <memory>
#include <iostream>
#include "p2contact.h"

p2QuadTree::p2QuadTree(){}

p2QuadTree::p2QuadTree(const int nodeLevel, p2AABB bounds)
{
	// Set base values
	m_NodeLevel = nodeLevel;
	m_Bounds = bounds;
}

void p2QuadTree::Clear()
{
	m_Objects.clear();
	nodes.clear();
	nodes.reserve(CHILD_TREE_NMB);
}

void p2QuadTree::Split()
{
	if (m_NodeLevel > MAX_LEVELS) return;

	// Set the current position
	auto currentPosition = m_Bounds.bottomLeft;

	// Define the size of the child sides depending on the amount of child tree number
	const auto childSize = (m_Bounds.topRight - currentPosition) / sqrt(CHILD_TREE_NMB);

	for (auto x = 0; x < sqrt(CHILD_TREE_NMB); ++x)
	{
		for (auto y = 0; y < sqrt(CHILD_TREE_NMB); ++y)
		{
			p2AABB childAabb;

			currentPosition = m_Bounds.bottomLeft + p2Vec2(childSize.x * x, childSize.y * y);

			childAabb.bottomLeft = currentPosition;
			childAabb.topRight = currentPosition + childSize;

			// Add the node to the child array
			nodes.push_back(new p2QuadTree(m_NodeLevel + 1, childAabb));
		}
	}
	for (int i = 0; i < m_Objects.size(); i++)
	{

		for (auto& child : nodes)
		{
			if (child->m_Bounds.DoOverlapWith(m_Objects[i]->GetAABB()))
			{
				child->Insert(m_Objects[i]);
				m_Objects.erase(m_Objects.begin() + i);
			}
			if (m_Objects.size() <= i)
			{
				break;
			}
		}
	}
}

int p2QuadTree::GetIndex(p2Body* rect)
{
	for (p2Body* body : m_Objects)
	{
		if (body == rect) return m_NodeLevel;
	}

	for (auto* child : nodes)
	{
		child->GetIndex(rect);
	}

	return 0;
}

void p2QuadTree::Insert(p2Body* obj)
{
	if (!nodes.empty())
	{
		bool inserted = false;
		for (auto& node : nodes)
		{
			inserted = true;
			if (node->m_Bounds.DoOverlapWith(obj->GetAABB()))
			{
				node->Insert(obj);
			}
		}
		if (inserted == false)
		{
			m_Objects.push_back(obj);
		}
	}
	else
	{
		m_Objects.push_back(obj);
	}

	if (m_Objects.size() > MAX_OBJECTS)
	{
		Split();
	}
}

std::vector<p2Body*> p2QuadTree::Retrieve(p2Body* rect)
{
	std::vector<p2Body*> m_ReturnValue;
	for (auto& body : m_Objects)
	{
		if (body == rect)
		{
			std::vector<p2Body*> m_ChildObject = GetChildrenObj();
			if (!m_ChildObject.empty()) {
				m_ReturnValue.insert(m_ReturnValue.begin(), m_ChildObject.begin(), m_ChildObject.end());
			}
		}
	}
	for (auto& node : nodes)
	{
		const auto retrieve = node->Retrieve(rect);
		if (m_ReturnValue.empty())
		{
			m_ReturnValue = retrieve;
		}
	}
	return m_ReturnValue;
}

void p2QuadTree::SetBounds(p2AABB bounds)
{
	m_Bounds = bounds;
}

p2AABB p2QuadTree::GetBounds() const
{
	return m_Bounds;
}

std::vector<p2Body*> p2QuadTree::GetChildrenObj()
{
	std::vector<p2Body*> m_ReturnValue = m_Objects;
	for (auto node : nodes)
	{
		std::vector<p2Body*> m_ChildObject = node->GetChildrenObj();
		if (!m_ChildObject.empty())
		{
			m_ReturnValue.insert(m_ReturnValue.begin(), m_ChildObject.begin(), m_ChildObject.end());
		}
	}
	return m_ReturnValue;
}

std::vector<p2QuadTree*> p2QuadTree::GetChildren() const
{
	return nodes;
}
