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


void p2ContactManager::CheckContact(std::vector<p2Body*> bodies)
{

	for (int i = 0; i < bodies.size(); i++)
	{
		if (bodies[i]->isInit)
		{
			p2ColliderType collideType = bodies[i]->GetCollider()->GetColliderType();

			for (int j = i + 1; j < bodies.size(); j++)
			{
				if (bodies[j]->isInit)
				{
					p2Body* otherBody = bodies[i];
					p2AABB otherAABB = otherBody->GetAABB();

					if (otherBody->GetType() == p2BodyType::DYNAMIC || otherBody->GetType() == p2BodyType::STATIC)
					{
						if (&otherBody != &bodies[i])
						{
							if (bodies[j]->GetAABB().DoOverlapWith(otherAABB))
							{
								p2Contact contact = p2Contact();
								contact.Init(bodies[j]->GetCollider(), otherBody->GetCollider());

								bool check = false;

								for (p2Contact c : m_CurrentContacts)
								{
									if (contact.CheckIfEqual(c, contact))
									{
										check = true;
										break;
									}
								}
								p2Vec2 mtv = MTV(*bodies[j], *otherBody);
								if (mtv != p2Vec2(0,0))
								{
									bodies[j]->SetLinearVelocity(bodies[j]->GetLinearVelocity() - (mtv.Normalized() *  p2Vec2::Dot(bodies[j]->GetLinearVelocity(), mtv.Normalized()) * 2));
									otherBody->SetLinearVelocity(otherBody->GetLinearVelocity() - (mtv.Normalized() *  p2Vec2::Dot(otherBody->GetLinearVelocity(), mtv.Normalized()) * 2));
								}
								if (bodies[j]->GetType() != p2BodyType::STATIC)
								{
									bodies[j]->SetPosition(bodies[j]->GetPosition() + mtv);
								}
								if (otherBody->GetType() != p2BodyType::STATIC)
								{
									otherBody->SetPosition(otherBody->GetPosition() - mtv);
								}
								if (!check)
								{
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
										m_CurrentContacts.erase(m_CurrentContacts.begin()+k);
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

	if (mtvX.GetMagnitude() < mtvY.GetMagnitude() && mtvX.GetMagnitude() != 0)
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
			if (bodyA.GetPosition() - bodyB.GetPosition() < p2Vec2(circleShapeA->GetRadius(),circleShapeA->GetRadius()) + p2Vec2(circleShapeB->GetRadius(),circleShapeB->GetRadius()))
			{
				return ((bodyA.GetPosition().Normalized() - bodyB.GetPosition()).Normalized() + p2Vec2(circleShapeA->GetRadius(), circleShapeA->GetRadius()) + p2Vec2(circleShapeB->GetRadius(), circleShapeB->GetRadius()));
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
			if (bodyA.GetPosition() - bodyB.GetPosition() < p2Vec2(circleShape->GetRadius(), 0) + p2Vec2(rectShape->GetSize().x, 0) ||
				bodyA.GetPosition() - bodyB.GetPosition() < p2Vec2(0, circleShape->GetRadius()) + p2Vec2(0, rectShape->GetSize().y) ||
				bodyA.GetPosition() - bodyB.GetPosition() < p2Vec2(circleShape->GetRadius(), circleShape->GetRadius()) + rectShape->GetSize())
			{
				return (bodyA.GetPosition().Normalized() - bodyB.GetPosition().Normalized() + p2Vec2(circleShape->GetRadius(), circleShape->GetRadius()) - rectShape->GetSize());
			}
		}
	}
	return p2Vec2(1, 1);
}

/*p2Vec2 p2ContactManager::RectVsRect(p2Body bodyA, p2Body bodyB)
{
	if (p2RectShape* rectShapeA = dynamic_cast<p2RectShape*>(bodyA.GetCollider()->GetShape()))
	{
		if (p2RectShape* rectShapeB = dynamic_cast<p2RectShape*>(bodyB.GetCollider()->GetShape()))
		{
			if (bodyA.GetPosition() - bodyB.GetPosition() < rectShapeA->GetSize().x)
			{
				
			}
		}
	}

	return p2Vec2(1, 1);
}*/
