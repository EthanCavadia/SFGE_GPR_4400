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

#include <p2aabb.h>
#include <iostream>

p2Vec2 p2AABB::GetCenter()
{
	return (topRight + bottomLeft) / 2;
}

p2Vec2 p2AABB::GetExtends() const
{
	return { topRight - bottomLeft };
}

void p2AABB::SetAABB(p2Vec2 center, p2Vec2 extend)
{
	topRight = center + extend;
	bottomLeft = center - extend;
}

float p2AABB::XMin() const
{
	return bottomLeft.x;
}

float p2AABB::XMax() const
{
	return topRight.x;
}

float p2AABB::YMin() const
{
	return topRight.y;
}

float p2AABB::YMax() const
{
	return bottomLeft.y;
}

bool p2AABB::DoContain(const p2Vec2 position) const
{
	return position > bottomLeft && position < topRight;
}

bool p2AABB::DoOverlapWith(const p2AABB aabb) const
{
	const auto bottomRight = p2Vec2(aabb.topRight.x, aabb.bottomLeft.y);
	const auto topLeft = p2Vec2(aabb.bottomLeft.x, aabb.topRight.y);

	if (DoContain(aabb.topRight) || DoContain(aabb.bottomLeft) || DoContain(bottomRight) || DoContain(topLeft))
	{
		return true;
	}
	if (aabb.DoContain(topRight) || aabb.DoContain(bottomLeft) || aabb.DoContain(p2Vec2(topRight.x, bottomLeft.y)) || aabb.DoContain(p2Vec2(bottomLeft.x, topRight.y)))
	{
		return true;
	}
	else
	{
		return false;
	}
	
}

	
