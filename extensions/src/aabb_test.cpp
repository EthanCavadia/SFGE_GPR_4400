#include <extensions/aabb_test.h>
#include <engine/engine.h>
#include <engine/config.h>
#include <graphics/graphics2d.h>
#include <physics/physics2d.h>



namespace sfge::ext
{

	AabbTest::AabbTest(Engine& engine) :
		System(engine)
	{

	}

	void AabbTest::OnEngineInit()
	{
		m_Transform2DManager = m_Engine.GetTransform2dManager();
		m_Body2DManager = m_Engine.GetPhysicsManager()->GetBodyManager();
		m_TextureManager = m_Engine.GetGraphics2dManager()->GetTextureManager();
		m_SpriteManager = m_Engine.GetGraphics2dManager()->GetSpriteManager();
		m_Graphics2DManager = m_Engine.GetGraphics2dManager();
		m_PhysicsManager = m_Engine.GetPhysicsManager();

		auto config = m_Engine.GetConfig();
		fixedDeltaTime = config->fixedDeltaTime;
		screenSize = sf::Vector2f(config->screenResolution.x, config->screenResolution.y);
		auto* entityManager = m_Engine.GetEntityManager();

		entities = entityManager->GetEntitiesWithType(ComponentType::BODY2D);
		for (auto i = 0u; i < entities.size(); i++)
		{
			auto body = m_Body2DManager->GetComponentPtr(entities[i]);
			bodies.push_back(body->GetBody());
		}
	}

	void AabbTest::OnUpdate(float dt)
	{
		(void)dt;
		
		for (auto i = 0u; i < entities.size(); i++)
		{
			auto transform = m_Transform2DManager->GetComponentPtr(entities[i]);
			transform->EulerAngle += 5*dt;
		}
		
	}


	void AabbTest::OnFixedUpdate()
	{
		rmt_ScopedCPUSample(AabbTestFixedUpdate, 0);
		quadTreeAABB.clear();
		//m_PhysicsManager->GetWorldRaw()->GetQuad()->GetAABBRecursively(quadTreeAABB);
	}

	void AabbTest::OnDraw()
	{
		rmt_ScopedCPUSample(AabbTestDraw, 0);
		for (auto i = 0u; i < bodies.size(); i++)
		{
			DrawAABB(bodies[i]->GetAABB());
		}

		/*for (auto i = 0u; i < quadTreeAABB.size(); i ++)
		{
			DrawQuadTree(quadTreeAABB[i]);
		}*/
	}

	void AabbTest::DrawAABB(p2AABB aabb)
	{
		m_Graphics2DManager->DrawLine(meter2pixel(p2Vec2(aabb.topRight.x, aabb.topRight.y)), meter2pixel(p2Vec2(aabb.bottomLeft.x, aabb.topRight.y)), sf::Color::Red);
		m_Graphics2DManager->DrawLine(meter2pixel(p2Vec2(aabb.topRight.x, aabb.topRight.y)), meter2pixel(p2Vec2(aabb.topRight.x, aabb.bottomLeft.y)), sf::Color::Red);
		m_Graphics2DManager->DrawLine(meter2pixel(p2Vec2(aabb.bottomLeft.x, aabb.bottomLeft.y)), meter2pixel(p2Vec2(aabb.topRight.x, aabb.bottomLeft.y)), sf::Color::Red);
		m_Graphics2DManager->DrawLine(meter2pixel(p2Vec2(aabb.bottomLeft.x, aabb.bottomLeft.y)), meter2pixel(p2Vec2(aabb.bottomLeft.x, aabb.topRight.y)), sf::Color::Red);
	}
	/*void AabbTest::DrawQuadTree(p2AABB aabb)
	{
		m_Graphics2DManager->DrawLine(meter2pixel(p2Vec2(aabb.topRight.x, aabb.topRight.y)), meter2pixel(p2Vec2(aabb.bottomLeft.x, aabb.topRight.y)), sf::Color::Green);
		m_Graphics2DManager->DrawLine(meter2pixel(p2Vec2(aabb.topRight.x, aabb.topRight.y)), meter2pixel(p2Vec2(aabb.topRight.x, aabb.bottomLeft.y)), sf::Color::Green);
		m_Graphics2DManager->DrawLine(meter2pixel(p2Vec2(aabb.bottomLeft.x, aabb.bottomLeft.y)), meter2pixel(p2Vec2(aabb.topRight.x, aabb.bottomLeft.y)), sf::Color::Green);
		m_Graphics2DManager->DrawLine(meter2pixel(p2Vec2(aabb.bottomLeft.x, aabb.bottomLeft.y)), meter2pixel(p2Vec2(aabb.bottomLeft.x, aabb.topRight.y)), sf::Color::Green);
	}*/
}