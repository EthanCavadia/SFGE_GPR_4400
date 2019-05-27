#ifndef SFGE_EXT_AABB_TEST_H
#define SFGE_EXT_AABB_TEST_H

#include <engine/system.h>
#include <graphics/graphics2d.h>
#include <p2body.h>
#include "p2quadtree.h"


namespace sfge
{
	struct Transform2d;
	class Transform2dManager;
	class Body2dManager;
	class TextureManager;
	class SpriteManager;
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


		void DrawAABB(p2AABB aabb);
		void DrawQuadTree(p2AABB aabb);
		float fixedDeltaTime = 0.0f;
		const size_t entitiesNmb = 10'000;

		sf::Vector2f screenSize;
		std::vector<p2Body*> bodies;
		std::vector<Entity> entities;
		std::vector<p2AABB> quadTreeAABB;
	};


}

#endif