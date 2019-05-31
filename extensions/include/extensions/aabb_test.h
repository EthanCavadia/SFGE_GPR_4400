#ifndef SFGE_EXT_AABB_TEST_H
#define SFGE_EXT_AABB_TEST_H

#include <engine/system.h>
#include <graphics/graphics2d.h>
#include <p2body.h>
#include "p2quadtree.h"
#include "p2world.h"


namespace sfge
{
	struct Transform2d;
	class Transform2dManager;
	class Body2dManager;
	class TextureManager;
	class SpriteManager;
	class Physics2dManager;
}

namespace sfge::ext
{


	class AabbTest : public System
	{
	public:
		AabbTest(Engine& engine);

		void OnEngineInit() override;

		void OnUpdate(float dt) override;

		void OnFixedUpdate() override;

		void OnDraw() override;

	private:


		Transform2dManager* m_Transform2DManager;
		Body2dManager* m_Body2DManager;
		TextureManager* m_TextureManager;
		SpriteManager* m_SpriteManager;
		Graphics2dManager* m_Graphics2DManager;
		Physics2dManager* m_PhysicsManager;
		p2World* m_World;
		
		void DrawAABB(p2AABB aabb) const;
		void DrawQuadTree(p2QuadTree* quadTree) const;
		float fixedDeltaTime = 0.0f;
		const size_t entitiesNmb = 10'000;
		p2QuadTree* quadTree;
		sf::Vector2f screenSize;
		std::vector<p2Body*> bodies;
		std::vector<Entity> entities;
	};


}

#endif