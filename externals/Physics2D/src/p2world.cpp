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
#include <p2world.h>
#include <iostream>

p2World::p2World(p2Vec2 gravity, sf::Vector2i screenResolution): m_Gravity(gravity)
{
	p2Vec2 screenRes = p2Vec2(screenResolution.x, screenResolution.y) /100;
	p2AABB rootAABB;
	rootAABB.topRight = p2Vec2(screenRes.x, 0);
	rootAABB.bottomLeft = p2Vec2(0,screenRes.y);
	m_Bodies.resize(MAX_BODY_LEN);
	rootQuad = new p2QuadTree(0, rootAABB);
	m_ContactManager = p2ContactManager();
}

void p2World::Step(float dt)
{
	rootQuad->Clear();
	for (p2Body& body : m_Bodies)
	{
		if (!body.isInit)
		{
			continue;
		}
		if (body.GetType() == p2BodyType::DYNAMIC)
		{
			//body.SetLinearVelocity(body.GetLinearVelocity() + m_Gravity * dt);
			body.ApplyForceToCenter(m_Gravity * dt);
			//std::cout << body.GetLinearVelocity().y << std::endl;
			// Check for collision

		}
		if (body.GetType() != p2BodyType::STATIC)
		{
			body.SetPosition(body.GetPosition() + body.GetLinearVelocity() * dt);
		}

		body.BuildAABB();
		if (body.GetCollider()->GetUserData() != nullptr)
		{
			rootQuad->Insert(&body);
		}
	}

	for (p2Body& bodies : m_Bodies)
	{
		if (!bodies.isInit)
		{
			continue;
		}
		if (bodies.GetCollider()->GetUserData() != nullptr)
		{
			m_ContactManager.CheckContact(rootQuad->Retrieve(&bodies));
		}
	}
}

p2Body* p2World::CreateBody(p2BodyDef* bodyDef)
{
	p2Body& body = m_Bodies[m_BodyIndex];
	m_BodyIndex++;
	body.Init(bodyDef);
	return &body;
}

void p2World::SetContactListener(p2ContactListener * contactListener)
{
	this->m_ContactListener = contactListener;
	this->m_ContactManager.SetContactListener(contactListener);
}

p2QuadTree* p2World::GetQuad() const
{
	return rootQuad;
}