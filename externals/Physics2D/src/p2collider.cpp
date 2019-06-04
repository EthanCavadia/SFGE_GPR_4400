#include "..\include\p2collider.h"

p2Collider::p2Collider()
{
}


void p2Collider::Init(p2ColliderDef* colliderDef)
{
	userData = colliderDef->userData;
	shape = colliderDef->shape;
	restitution = colliderDef->restitution;
	colliderType = colliderDef->colliderType;
	isSensor = colliderDef->isSensor;
}

bool p2Collider::IsSensor() const
{
	return isSensor;
}

void* p2Collider::GetUserData()
{
	return userData;
}

p2Shape* p2Collider::GetShape() const
{
	return shape;
}


p2ColliderType p2Collider::GetColliderType()
{
	return colliderType;
}


void p2Collider::SetUserData(void* colliderData)
{
	userData = colliderData;
}

p2AABB p2Collider::BuildAABBCollider(p2Vec2 position)
{
	switch (colliderType)
	{
	case p2ColliderType::NONE:
	{
		extend = p2Vec2(0, 0);
	}
	break;
	case p2ColliderType::CIRCLE:
	{
		p2CircleShape* circleShape = static_cast<p2CircleShape*>(shape);
		extend = p2Vec2(circleShape->GetRadius(), circleShape->GetRadius());
	}
	break;
	case p2ColliderType::BOX:
	{
		p2RectShape* rectShape = static_cast<p2RectShape*>(shape);
		extend = rectShape->GetSize();
	}
	break;
	case p2ColliderType::POLYGON:
	{
		extend = p2Vec2(0, 0);
	}
	break;
	}

	aabb.SetAABB(position, extend);

	return aabb;
}
