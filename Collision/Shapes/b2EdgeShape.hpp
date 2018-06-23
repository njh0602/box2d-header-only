/*
* Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef B2_EDGE_SHAPE_H
#define B2_EDGE_SHAPE_H

#include <new>

#include "b2Shape.hpp"

/// A line segment (edge) shape. These can be connected in chains or loops
/// to other edge shapes. The connectivity information is used to ensure
/// correct contact normals.
class b2EdgeShape : public b2Shape
{
public:
	b2EdgeShape();

	/// Set this as an isolated edge.
    void Set(const b2Vec2& v1, const b2Vec2& v2)
    {
        m_vertex1 = v1;
        m_vertex2 = v2;
        m_hasVertex0 = false;
        m_hasVertex3 = false;
    }

	/// Implement b2Shape.
    b2Shape* Clone(b2BlockAllocator* allocator) const override
    {
        void* mem = allocator->Allocate(sizeof(b2EdgeShape));
        b2EdgeShape* clone = new (mem) b2EdgeShape;
        *clone = *this;
        return clone;
    }

	/// @see b2Shape::GetChildCount
	int32 GetChildCount() const override
    {
        return 1;
    }

	/// @see b2Shape::TestPoint
	bool TestPoint(const b2Transform& transform, const b2Vec2& p) const override
    {
        B2_NOT_USED(transform);
        B2_NOT_USED(p);
        return false;
    }

    // p = p1 + t * d
    // v = v1 + s * e
    // p1 + t * d = v1 + s * e
    // s * e - t * d = p1 - v1
    bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
                              const b2Transform& xf, int32 childIndex) const override
    {
        B2_NOT_USED(childIndex);
        
        // Put the ray into the edge's frame of reference.
        b2Vec2 p1 = b2MulT(xf.q, input.p1 - xf.p);
        b2Vec2 p2 = b2MulT(xf.q, input.p2 - xf.p);
        b2Vec2 d = p2 - p1;
        
        b2Vec2 v1 = m_vertex1;
        b2Vec2 v2 = m_vertex2;
        b2Vec2 e = v2 - v1;
        b2Vec2 normal(e.y, -e.x);
        normal.Normalize();
        
        // q = p1 + t * d
        // dot(normal, q - v1) = 0
        // dot(normal, p1 - v1) + t * dot(normal, d) = 0
        float32 numerator = b2Dot(normal, v1 - p1);
        float32 denominator = b2Dot(normal, d);
        
        if (denominator == 0.0f)
        {
            return false;
        }
        
        float32 t = numerator / denominator;
        if (t < 0.0f || input.maxFraction < t)
        {
            return false;
        }
        
        b2Vec2 q = p1 + t * d;
        
        // q = v1 + s * r
        // s = dot(q - v1, r) / dot(r, r)
        b2Vec2 r = v2 - v1;
        float32 rr = b2Dot(r, r);
        if (rr == 0.0f)
        {
            return false;
        }
        
        float32 s = b2Dot(q - v1, r) / rr;
        if (s < 0.0f || 1.0f < s)
        {
            return false;
        }
        
        output->fraction = t;
        if (numerator > 0.0f)
        {
            output->normal = -b2Mul(xf.q, normal);
        }
        else
        {
            output->normal = b2Mul(xf.q, normal);
        }
        return true;
    }

	/// @see b2Shape::ComputeAABB
	void ComputeAABB(b2AABB* aabb, const b2Transform& transform, int32 childIndex) const override
    {
        B2_NOT_USED(childIndex);
        
        b2Vec2 v1 = b2Mul(transform, m_vertex1);
        b2Vec2 v2 = b2Mul(transform, m_vertex2);
        
        b2Vec2 lower = b2Min(v1, v2);
        b2Vec2 upper = b2Max(v1, v2);
        
        b2Vec2 r(m_radius, m_radius);
        aabb->lowerBound = lower - r;
        aabb->upperBound = upper + r;
    }

	/// @see b2Shape::ComputeMass
	void ComputeMass(b2MassData* massData, float32 density) const override
    {
        B2_NOT_USED(density);
        
        massData->mass = 0.0f;
        massData->center = 0.5f * (m_vertex1 + m_vertex2);
        massData->I = 0.0f;
    }
	
	/// These are the edge vertices
	b2Vec2 m_vertex1, m_vertex2;

	/// Optional adjacent vertices. These are used for smooth collision.
	b2Vec2 m_vertex0, m_vertex3;
	bool m_hasVertex0, m_hasVertex3;
};

inline b2EdgeShape::b2EdgeShape()
{
	m_type = e_edge;
	m_radius = b2_polygonRadius;
	m_vertex0.x = 0.0f;
	m_vertex0.y = 0.0f;
	m_vertex3.x = 0.0f;
	m_vertex3.y = 0.0f;
	m_hasVertex0 = false;
	m_hasVertex3 = false;
}

#endif
