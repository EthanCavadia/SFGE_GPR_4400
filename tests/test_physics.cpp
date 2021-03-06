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
#include <engine/engine.h>
#include <engine/scene.h>
#include <gtest/gtest.h>
#include "graphics/shape2d.h"
#include "physics/collider2d.h"
#include "json.hpp"

TEST(Physics, TestPlanetPySystemCpp)
{
	sfge::Engine engine;
	std::unique_ptr<sfge::Configuration> initConfig = std::make_unique<sfge::Configuration>();
	initConfig->gravity = p2Vec2();
	initConfig->devMode = false;
	initConfig->maxFramerate = 0;
	engine.Init(std::move(initConfig));
	json sceneJson = {
		{ "name", "Test Planet Component" } };
	json systemJson = {
		{ "systemClassName", "PlanetSystem" }
	};

	sceneJson["systems"] = json::array({ systemJson });
	auto* sceneManager = engine.GetSceneManager();
	sceneManager->LoadSceneFromJson(sceneJson);

	engine.Start();
}

TEST(Physics, TestBallFallingToGround)
{
	sfge::Engine engine;
	auto config = std::make_unique<sfge::Configuration>();
	config->devMode = false;
	engine.Init(std::move(config));

	auto* sceneManager = engine.GetSceneManager();

	json sceneJson;
	sceneJson["name"] = "Ball Falling To Ground";

	json entityBody1;
	entityBody1["name"] = "Bouncing1";

	json transformJson1;
	transformJson1["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson1["position"] = { 1200, 340 };
	transformJson1["scale"] = { 1.0,1.0 };
	transformJson1["angle"] = 2;

	json circleShapeJson;
	circleShapeJson["name"] = "Circle Shape Component";
	circleShapeJson["type"] = sfge::ComponentType::SHAPE2D;
	circleShapeJson["shape_type"] = sfge::ShapeType::CIRCLE;
	circleShapeJson["radius"] = 30;

	json rigidBodyJson1;
	rigidBodyJson1["name"] = "Rigidbody";
	rigidBodyJson1["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson1["body_type"] = p2BodyType::DYNAMIC;
	rigidBodyJson1["velocity"] = { 0, 0 };

	json circleColliderJson;
	circleColliderJson["name"] = "Circle Collider";
	circleColliderJson["type"] = sfge::ComponentType::COLLIDER2D;
	circleColliderJson["collider_type"] = sfge::ColliderType::CIRCLE;
	circleColliderJson["radius"] = 30;
	circleColliderJson["bouncing"] = 1;
	circleColliderJson["sensor"] = false;

	entityBody1["components"] = { transformJson1, circleShapeJson, rigidBodyJson1, circleColliderJson };

	json entityBody2;
	entityBody2["name"] = "Ground";

	json transformJson2;
	transformJson2["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson2["position"] = {640, 600 };
	transformJson2["scale"] = { 1.0,1.0 };
	transformJson2["angle"] = 0.0;

	json rectShapeJson;
	rectShapeJson["name"] = "Rect Shape Component";
	rectShapeJson["type"] = sfge::ComponentType::SHAPE2D;
	rectShapeJson["shape_type"] = sfge::ShapeType::RECTANGLE;
	rectShapeJson["size"] = {1300,200};

	json rigidBodyJson2;
	rigidBodyJson2["name"] = "Rigidbody";
	rigidBodyJson2["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson2["body_type"] = p2BodyType::STATIC;

	json rectColliderJson;
	rectColliderJson["name"] = "Rect Collider";
	rectColliderJson["type"] = sfge::ComponentType::COLLIDER2D;
	rectColliderJson["collider_type"] = sfge::ColliderType::BOX;
	rectColliderJson["size"] = { 1300,200};
	rectColliderJson["sensor"] = true;

	entityBody2["components"] = { transformJson2, rectShapeJson, rigidBodyJson2, rectColliderJson };

	json entityBody3;
	entityBody3["name"] = "Bouncing2";

	json transformJson3;
	transformJson3["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson3["position"] = { 1000, 300 };
	transformJson3["scale"] = { 1.0,1.0 };
	transformJson3["angle"] = 0.0;

	json circleShapeJson2;
	circleShapeJson2["name"] = "Circle Shape Component";
	circleShapeJson2["type"] = sfge::ComponentType::SHAPE2D;
	circleShapeJson2["shape_type"] = sfge::ShapeType::CIRCLE;
	circleShapeJson2["radius"] = 50;

	json rigidBodyJson3;
	rigidBodyJson3["name"] = "Rigidbody";
	rigidBodyJson3["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson3["body_type"] = p2BodyType::DYNAMIC;
	rigidBodyJson3["velocity"] = { 0, 0 };

	json circleColliderJson2;
	circleColliderJson2["name"] = "Circle Collider";
	circleColliderJson2["type"] = sfge::ComponentType::COLLIDER2D;
	circleColliderJson2["collider_type"] = sfge::ColliderType::CIRCLE;
	circleColliderJson2["radius"] = 50;
	circleColliderJson2["bouncing"] = 0.8;
	circleColliderJson2["sensor"] = false;

	entityBody3["components"] = { transformJson3, circleShapeJson2, rigidBodyJson3, circleColliderJson2 };

	json entityBody4;
	entityBody4["name"] = "Bouncing3";

	json transformJson4;
	transformJson4["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson4["position"] = { 800, 340 };
	transformJson4["scale"] = { 1.0,1.0 };
	transformJson4["angle"] = 2;

	json circleShapeJson3;
	circleShapeJson3["name"] = "Circle Shape Component";
	circleShapeJson3["type"] = sfge::ComponentType::SHAPE2D;
	circleShapeJson3["shape_type"] = sfge::ShapeType::CIRCLE;
	circleShapeJson3["radius"] = 50;

	json rigidBodyJson4;
	rigidBodyJson4["name"] = "Rigidbody";
	rigidBodyJson4["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson4["body_type"] = p2BodyType::DYNAMIC;
	rigidBodyJson4["velocity"] = { 0, 0 };

	json circleColliderJson3;
	circleColliderJson3["name"] = "Circle Collider";
	circleColliderJson3["type"] = sfge::ComponentType::COLLIDER2D;
	circleColliderJson3["collider_type"] = sfge::ColliderType::CIRCLE;
	circleColliderJson3["radius"] = 50;
	circleColliderJson3["bouncing"] = 0.5;
	circleColliderJson3["sensor"] = false;

	entityBody4["components"] = { transformJson4, circleShapeJson3, rigidBodyJson4, circleColliderJson3 };

	json entityBody5;
	entityBody5["name"] = "Bouncing3";

	json transformJson5;
	transformJson5["type"] = sfge::ComponentType::TRANSFORM2D;
	transformJson5["position"] = { 600, 340 };
	transformJson5["scale"] = { 1.0,1.0 };
	transformJson5["angle"] = 2;

	json circleShapeJson4;
	circleShapeJson4["name"] = "Circle Shape Component";
	circleShapeJson4["type"] = sfge::ComponentType::SHAPE2D;
	circleShapeJson4["shape_type"] = sfge::ShapeType::CIRCLE;
	circleShapeJson4["radius"] = 50;

	json rigidBodyJson5;
	rigidBodyJson5["name"] = "Rigidbody";
	rigidBodyJson5["type"] = sfge::ComponentType::BODY2D;
	rigidBodyJson5["body_type"] = p2BodyType::DYNAMIC;
	rigidBodyJson5["velocity"] = { 0, 0 };

	json circleColliderJson5;
	circleColliderJson5["name"] = "Circle Collider";
	circleColliderJson5["type"] = sfge::ComponentType::COLLIDER2D;
	circleColliderJson5["collider_type"] = sfge::ColliderType::CIRCLE;
	circleColliderJson5["radius"] = 50;
	circleColliderJson5["bouncing"] = 0.2;
	circleColliderJson5["sensor"] = false;

	entityBody5["components"] = { transformJson5, circleShapeJson4, rigidBodyJson5, circleColliderJson5 };

	sceneJson["entities"] = { entityBody1, entityBody2, entityBody3, entityBody4, entityBody5 };

	sceneJson["systems"] = nlohmann::json::array({
		{
			{ "script_path", "scripts/contact_debug_system.py" }
		},
		{
			{ "script_path", "scripts/stay_onscreen_system.py" }
		},
		{
			{ "script_path",
			//"scripts/mouse_raycast_system.py" 
			"nothing" }
		},
		{
			{"systemClassName", "AabbTest"}
		}
		}
	);
	sceneManager->LoadSceneFromJson(sceneJson);
	engine.Start();

}

TEST(Physics, TestShapeContact)
{
	sfge::Engine engine;
	auto config = std::make_unique<sfge::Configuration>();
	config->gravity = p2Vec2(0.0f, 0.0f);
	engine.Init(std::move(config));

	auto* sceneManager = engine.GetSceneManager();

	json sceneJson;
	sceneJson["name"] = "Contacts";

	const int entitiesNmb = 200;
	json entities[entitiesNmb];

	json shapes[] =
	{
		{
			{"name","Rect Shape Component"}
			,
			{"type",sfge::ComponentType::SHAPE2D},
			{"shape_type", sfge::ShapeType::RECTANGLE},
			{"size",{10,10}}
		},
		{
			{"name","Rect Shape Component"},
			{"type",sfge::ComponentType::SHAPE2D},
			{"shape_type", sfge::ShapeType::CIRCLE},
			{"radius",10}

		}
	};
		json colliders[] =
	{
		{
			{"name","Rect Collider"},
			{"type", sfge::ComponentType::COLLIDER2D},
			{"collider_type",sfge::ColliderType::BOX},
			{"size",{10,10}},
			{"bouncing", 1},
			{"sensor",false}
		},
		{
			{"name","Circle Collider"},
			{"type", sfge::ComponentType::COLLIDER2D},
			{"collider_type",sfge::ColliderType::CIRCLE},
			{"radius",10},
			{"bouncing", 1},
			{"sensor",false}
		}
	};

	for (int i = 0; i < entitiesNmb; i++)
	{
		json& entityJson = entities[i];

		json transformJson =
		{
			{"position",{ rand() % 1280,rand() % 720 }},
			{"type", sfge::ComponentType::TRANSFORM2D}
		};

		json rigidbody =
		{
			{"name", "Rigidbody"},
			{"type", sfge::ComponentType::BODY2D},
			{"body_type",  p2BodyType::DYNAMIC},
			{"velocity", {rand() % 400, rand() % 400}}
		};

		int randShapeIndex = rand() % 2;
		entityJson["components"] = { transformJson, shapes[randShapeIndex] , rigidbody, colliders[randShapeIndex]};

	}

	sceneJson["entities"] = entities;
	sceneJson["systems"] = nlohmann::json::array({
		{
			{ "script_path", "scripts/contact_debug_system.py" }
		},
		{
			{ "script_path", "scripts/stay_onscreen_system.py" }
		},
		/*{
			{ "script_path", 
			"scripts/mouse_raycast_system.py" 
			"nothing" }
		},*/
		{
			{"systemClassName", "AabbTest"}
		}
	}
	);
	sceneManager->LoadSceneFromJson(sceneJson);
	engine.Start();
}