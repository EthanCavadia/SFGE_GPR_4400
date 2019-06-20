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

#ifndef SFGE_P2CONTACT_H
#define SFGE_P2CONTACT_H

#include <p2collider.h>
#include <vector>

class p2Body;

/**
* \brief Representation of a contact given as argument in a p2ContactListener
*/
class p2Contact
{
public:

	void Init(p2Collider* colliderA, p2Collider* colliderB);
	p2Collider* GetColliderA() const;
	p2Collider* GetColliderB() const;
	//void SetContact(p2Collider * colliderA, p2Collider * ColliderB);
	p2Vec2 GetContact() const;
	bool CheckIfEqual(p2Contact contactA, p2Contact contactB);
	
	
	p2Collider* colliderA = nullptr;
	p2Collider* colliderB = nullptr;
	p2Vec2 contactPoint;
	bool updated = false;
};

/**
* \brief Listener of contacts happening in an attached p2World
*/
class p2ContactListener
{
public:
	virtual void BeginContact(p2Contact* contact) = 0;
	virtual void EndContact(p2Contact* contact) = 0;
	
};

/**
* \brief Managing the creation and destruction of contact between colliders
*/
class p2ContactManager
{
public:
	p2ContactManager() {};
	p2ContactManager(p2ContactListener* listener) : m_ContactListener(listener) {};
	void SetContactListener(p2ContactListener * listener);
	void CheckContact(std::vector<p2Body*> bodies,p2Body* body);
	/**
	 * Minimum translation vector. Get if an object is in another object
	 */
	p2Vec2 MTV(p2Body bodyA, p2Body bodyB);
	p2Vec2 CircleVsCircle(p2Body bodyA, p2Body bodyB);
	p2Vec2 CircleVsRect(p2Body bodyA, p2Body bodyB);
	std::vector<p2Contact> m_CurrentContacts = std::vector<p2Contact>();
private:
	p2ContactListener* m_ContactListener = nullptr;
};
#endif