/*
MIT License

Copyright (c) 2017 SAE Institute Switzerland AG

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <p2contact.h>
#include "p2body.h"
#include <complex.h>

void p2Contact::Init(p2Collider* colliderA, p2Collider* colliderB)
{
	this->colliderA = colliderA;
	this->colliderB = colliderB;

	//this->SetContact(colliderA, colliderB);
}
void p2ContactManager::SetContactListener(p2ContactListener* listener)
{
	m_ContactListener = listener;
}

p2Collider * p2Contact::GetColliderA() const
{
	return colliderA;
}

p2Collider * p2Contact::GetColliderB() const
{
	return colliderB;
}

p2Vec2 p2Contact::GetContact() const
{
	return contactPoint;
}


void p2ContactManager::CheckContact(std::vector<p2Body*> bodies, p2Body* body)
{
	for (int j = 0; j < bodies.size(); j++)
	{
		if (bodies[j]->isInit)
		{
			p2Body* otherBody = body;
			p2AABB otherAABB = otherBody->GetAABB();
			if (otherBody->GetType() == p2BodyType::DYNAMIC || otherBody->GetType() == p2BodyType::STATIC)
			{
				if (otherBody != bodies[j])
				{
					if (bodies[j]->GetAABB().DoOverlapWith(otherAABB))
					{
						p2Contact contact = p2Contact();
						contact.Init(bodies[j]->GetCollider(), otherBody->GetCollider());

						int indexContact = -1;

						for (int l = 0; l < m_CurrentContacts.size(); l++)
						{
							if (contact.CheckIfEqual(m_CurrentContacts[l], contact))
							{
								indexContact = l;
								break;
							}
						}

						p2Vec2 mtv;
						mtv = CircleVsCircle(*bodies[j], *otherBody);
						if (mtv == p2Vec2(1, 1))
						{
							mtv = CircleVsRect(*bodies[j], *otherBody);
							if (mtv == p2Vec2(1, 1))
							{
								mtv = MTV(*bodies[j], *otherBody);
							}
						}
						if (indexContact != -1)
						{
							//m_CurrentContacts[indexContact].updated = true;
							continue;
						}
						if (mtv != p2Vec2(0, 0))
						{
							bodies[j]->SetLinearVelocity(bodies[j]->GetLinearVelocity() - (mtv.Normalized() *  p2Vec2::Dot(bodies[j]->GetLinearVelocity(), mtv.Normalized()) * (bodies[j]->GetCollider()->GetRestitution() + bodies[j]->GetCollider()->GetRestitution())));
							otherBody->SetLinearVelocity(otherBody->GetLinearVelocity() - (mtv.Normalized() *  p2Vec2::Dot(otherBody->GetLinearVelocity(), mtv.Normalized()) * (otherBody->GetCollider()->GetRestitution() + otherBody->GetCollider()->GetRestitution())));
						}

						if (bodies[j]->GetType() != p2BodyType::STATIC)
						{
							bodies[j]->SetPosition(bodies[j]->GetPosition() + mtv);
						}
						if (otherBody->GetType() != p2BodyType::STATIC)
						{
							otherBody->SetPosition(otherBody->GetPosition() - mtv);
						}
						if (indexContact == -1 && mtv != p2Vec2(0, 0))
						{
							contact.updated = true;
							m_CurrentContacts.push_back(contact);
							m_ContactListener->BeginContact(&contact);
						}
					}
					else
					{
						p2Contact contact = p2Contact();
						contact.Init(bodies[j]->GetCollider(), otherBody->GetCollider());

						bool check = false;

						for (int k = 0; k < m_CurrentContacts.size(); k++)
						{
							if (contact.CheckIfEqual(m_CurrentContacts[k], contact))
							{
								check = true;
								m_CurrentContacts.erase(m_CurrentContacts.begin() + k);
								break;
							}
						}
						if (check)
						{
							m_ContactListener->EndContact(&contact);
						}
					}
				}
			}
		}
	}
}


bool p2Contact::CheckIfEqual(p2Contact contactA, p2Contact contactB)
{
	return (contactA.GetColliderA() == contactB.GetColliderA() && contactA.GetColliderB() == contactB.GetColliderB()) ||
			(contactA.GetColliderA() == contactB.GetColliderB() && contactA.GetColliderB() == contactB.GetColliderA());
}

p2Vec2 p2ContactManager::MTV(p2Body bodyA, p2Body bodyB)
{
	p2Vec2 mtvX;
	p2Vec2 mtvY;

	p2Vec2 distance;
	p2Vec2 sumSize;
	distance = bodyB.GetPosition() - bodyA.GetPosition();
	sumSize = bodyA.GetAABB().GetExtends() / 2 + bodyB.GetAABB().GetExtends() / 2;

	if (abs(distance.x - sumSize.x) < abs(distance.x + sumSize.x))
	{
		mtvX = p2Vec2(distance.x - sumSize.x, 0);
	}
	else
	{
		mtvX = p2Vec2(distance.x + sumSize.x, 0);
	}

	if (abs(distance.y - sumSize.y) < abs(distance.y + sumSize.y))
	{
		mtvY = p2Vec2(0, distance.y - sumSize.y);
	}
	else
	{
		mtvY = p2Vec2(0, distance.y + sumSize.y);
	}
	if (mtvX.GetMagnitude() < mtvY.GetMagnitude())
	{
		return mtvX;
	}
	else
	{
		return mtvY;
	}
}

p2Vec2 p2ContactManager::CircleVsCircle(p2Body bodyA, p2Body bodyB)
{
	if (p2CircleShape* circleShapeA = dynamic_cast<p2CircleShape*>(bodyA.GetCollider()->GetShape()))
	{
		if (p2CircleShape* circleShapeB = dynamic_cast<p2CircleShape*>(bodyB.GetCollider()->GetShape()))
		{
			if ((bodyB.GetPosition() - bodyA.GetPosition()).GetMagnitude() < circleShapeA->GetRadius() + circleShapeB->GetRadius())
			{
				return ((bodyB.GetPosition() - bodyA.GetPosition()).Normalized() * ((bodyB.GetPosition() - bodyA.GetPosition()).GetMagnitude() - circleShapeA->GetRadius() - circleShapeB->GetRadius()));
			}
			else
			{
				return  p2Vec2(0, 0);
			}
		}
	}
	return p2Vec2(1, 1);
}

p2Vec2 p2ContactManager::CircleVsRect(p2Body bodyA, p2Body bodyB)
{
	if (p2CircleShape* circleShape = dynamic_cast<p2CircleShape*>(bodyA.GetCollider()->GetShape()))
	{
		if (p2RectShape* rectShape = dynamic_cast<p2RectShape*>(bodyB.GetCollider()->GetShape()))
		{
			if ((bodyA.GetPosition().x > bodyB.GetPosition().x + rectShape->GetSize().x ||
				bodyA.GetPosition().x < bodyB.GetPosition().x - rectShape->GetSize().x )&&
				(bodyA.GetPosition().y > bodyB.GetPosition().y + rectShape->GetSize().y ||
				bodyA.GetPosition().y < bodyB.GetPosition().y - rectShape->GetSize().y ))
			{
				float minDistance = circleShape->GetRadius();
				p2Vec2 closestCorner;
				std::vector<p2Vec2> corners;
				corners.push_back(bodyB.GetPosition() + rectShape->GetSize() );
				corners.push_back(bodyB.GetPosition() - rectShape->GetSize() );
				corners.push_back(bodyB.GetPosition() + p2Vec2(rectShape->GetSize().x , -rectShape->GetSize().y));
				corners.push_back(bodyB.GetPosition() - p2Vec2(rectShape->GetSize().x , -rectShape->GetSize().y));

				for (p2Vec2 corner : corners)
				{
					if ((bodyA.GetPosition() - corner).GetMagnitude() < minDistance)
					{
						closestCorner = corner;
						minDistance = (bodyA.GetPosition() - corner).GetMagnitude();
					}
				}
				if ((minDistance < circleShape->GetRadius()))
				{
					return (closestCorner - bodyA.GetPosition()).Normalized() * (minDistance - circleShape->GetRadius());
				}
				return p2Vec2(0, 0);
			}
		}
	}
	if (p2CircleShape* circleShape = dynamic_cast<p2CircleShape*>(bodyB.GetCollider()->GetShape()))
	{
		if (p2RectShape* rectShape = dynamic_cast<p2RectShape*>(bodyA.GetCollider()->GetShape()))
		{
			if ((bodyB.GetPosition().x > bodyA.GetPosition().x + rectShape->GetSize().x ||
				bodyB.GetPosition().x < bodyA.GetPosition().x - rectShape->GetSize().x )&&
				(bodyB.GetPosition().y > bodyA.GetPosition().y + rectShape->GetSize().y ||
				bodyB.GetPosition().y < bodyA.GetPosition().y - rectShape->GetSize().y ))
			{
				float minDistance = circleShape->GetRadius();
				p2Vec2 closestCorner;
				std::vector<p2Vec2> corners;
				corners.push_back(bodyA.GetPosition() + rectShape->GetSize() );
				corners.push_back(bodyA.GetPosition() - rectShape->GetSize() );
				corners.push_back(bodyA.GetPosition() + p2Vec2(rectShape->GetSize().x , -rectShape->GetSize().y ));
				corners.push_back(bodyA.GetPosition() - p2Vec2(rectShape->GetSize().x , -rectShape->GetSize().y ));

				for (p2Vec2 corner : corners)
				{
					if ((bodyB.GetPosition() - corner).GetMagnitude() < minDistance)
					{
						closestCorner = corner;
						minDistance = (bodyB.GetPosition() - corner).GetMagnitude();
					}
				}
				if ((minDistance < circleShape->GetRadius()))
				{
					return (bodyB.GetPosition() - closestCorner).Normalized() * (minDistance - circleShape->GetRadius());
				}
				return p2Vec2(0, 0);
			}
		}
	}
	return p2Vec2(1, 1);
}
