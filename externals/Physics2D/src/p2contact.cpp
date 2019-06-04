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

p2Vec2 p2Contact::CircleVsCircle(p2CircleShape circle1, p2CircleShape circle2)
{
	float radius1 = circle1.GetRadius();
	float radius2 = circle2.GetRadius();

	float distance = radius1 + radius2;

	if (distance < radius1 + radius2)
	{
		return contactPoint;
	}
}
