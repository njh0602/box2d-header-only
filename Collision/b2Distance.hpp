/*
 * Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#ifndef B2_DISTANCE_H
#define B2_DISTANCE_H

#include "Shapes/b2CircleShape.hpp"
#include "Shapes/b2EdgeShape.hpp"
#include "Shapes/b2ChainShape.hpp"
#include "Shapes/b2PolygonShape.hpp"
#include "../Common/b2Math.hpp"

/// A distance proxy is used by the GJK algorithm.
/// It encapsulates any shape.
struct b2DistanceProxy
{
    b2DistanceProxy() : m_vertices(nullptr), m_count(0), m_radius(0.0f) {}
    
    /// Initialize the proxy using the given shape. The shape
    /// must remain in scope while the proxy is in use.
    void Set(const b2Shape* shape, int32 index)
    {
        switch (shape->GetType())
        {
            case b2Shape::e_circle:
            {
                const b2CircleShape* circle = static_cast<const b2CircleShape*>(shape);
                m_vertices = &circle->m_p;
                m_count = 1;
                m_radius = circle->m_radius;
                break;
            }
                
            case b2Shape::e_polygon:
            {
                const b2PolygonShape* polygon = static_cast<const b2PolygonShape*>(shape);
                m_vertices = polygon->m_vertices;
                m_count = polygon->m_count;
                m_radius = polygon->m_radius;
                break;
            }
                
            case b2Shape::e_chain:
            {
                const b2ChainShape* chain = static_cast<const b2ChainShape*>(shape);
                b2Assert(0 <= index && index < chain->m_count);
                
                m_buffer[0] = chain->m_vertices[index];
                if (index + 1 < chain->m_count)
                {
                    m_buffer[1] = chain->m_vertices[index + 1];
                }
                else
                {
                    m_buffer[1] = chain->m_vertices[0];
                }
                
                m_vertices = m_buffer;
                m_count = 2;
                m_radius = chain->m_radius;
                break;
            }
                
            case b2Shape::e_edge:
            {
                const b2EdgeShape* edge = static_cast<const b2EdgeShape*>(shape);
                m_vertices = &edge->m_vertex1;
                m_count = 2;
                m_radius = edge->m_radius;
                break;
            }
                
            default: b2Assert(false);
        }
    }
    
    /// Initialize the proxy using a vertex cloud and radius. The vertices
    /// must remain in scope while the proxy is in use.
    void Set(const b2Vec2* vertices, int32 count, float32 radius)
    {
        m_vertices = vertices;
        m_count = count;
        m_radius = radius;
    }
    
    /// Get the supporting vertex index in the given direction.
    int32 GetSupport(const b2Vec2& d) const
    {
        int32 bestIndex = 0;
        float32 bestValue = b2Dot(m_vertices[0], d);
        for (int32 i = 1; i < m_count; ++i)
        {
            float32 value = b2Dot(m_vertices[i], d);
            if (value > bestValue)
            {
                bestIndex = i;
                bestValue = value;
            }
        }
        
        return bestIndex;
    }
    
    /// Get the supporting vertex in the given direction.
    const b2Vec2& GetSupportVertex(const b2Vec2& d) const
    {
        int32 bestIndex = 0;
        float32 bestValue = b2Dot(m_vertices[0], d);
        for (int32 i = 1; i < m_count; ++i)
        {
            float32 value = b2Dot(m_vertices[i], d);
            if (value > bestValue)
            {
                bestIndex = i;
                bestValue = value;
            }
        }
        
        return m_vertices[bestIndex];
    }
    
    /// Get the vertex count.
    int32 GetVertexCount() const
    {
        return m_count;
    }
    
    /// Get a vertex by index. Used by b2Distance.
    const b2Vec2& GetVertex(int32 index) const
    {
        b2Assert(0 <= index && index < m_count);
        return m_vertices[index];
    }
    
    b2Vec2 m_buffer[2];
    const b2Vec2* m_vertices;
    int32 m_count;
    float32 m_radius;
};

/// Used to warm start b2Distance.
/// Set count to zero on first call.
struct b2SimplexCache
{
    float32 metric;		///< length or area
    uint16 count;
    uint8 indexA[3];	///< vertices on shape A
    uint8 indexB[3];	///< vertices on shape B
};

/// Input for b2Distance.
/// You have to option to use the shape radii
/// in the computation. Even 
struct b2DistanceInput
{
    b2DistanceProxy proxyA;
    b2DistanceProxy proxyB;
    b2Transform transformA;
    b2Transform transformB;
    bool useRadii;
};

/// Output for b2Distance.
struct b2DistanceOutput
{
    b2Vec2 pointA;		///< closest point on shapeA
    b2Vec2 pointB;		///< closest point on shapeB
    float32 distance;
    int32 iterations;	///< number of GJK iterations used
};

/// Input parameters for b2ShapeCast
struct b2ShapeCastInput
{
    b2DistanceProxy proxyA;
    b2DistanceProxy proxyB;
    b2Transform transformA;
    b2Transform transformB;
    b2Vec2 translationB;
};

/// Output results for b2ShapeCast
struct b2ShapeCastOutput
{
    b2Vec2 point;
    b2Vec2 normal;
    float32 lambda;
    int32 iterations;
};

struct b2SimplexVertex
{
    b2Vec2 wA;        // support point in proxyA
    b2Vec2 wB;        // support point in proxyB
    b2Vec2 w;        // wB - wA
    float32 a;        // barycentric coordinate for closest point
    int32 indexA;    // wA index
    int32 indexB;    // wB index
};

struct b2Simplex
{
    void ReadCache(const b2SimplexCache* cache,
                   const b2DistanceProxy* proxyA, const b2Transform& transformA,
                   const b2DistanceProxy* proxyB, const b2Transform& transformB)
    {
        b2Assert(cache->count <= 3);
        
        // Copy data from cache.
        m_count = cache->count;
        b2SimplexVertex* vertices = &m_v1;
        for (int32 i = 0; i < m_count; ++i)
        {
            b2SimplexVertex* v = vertices + i;
            v->indexA = cache->indexA[i];
            v->indexB = cache->indexB[i];
            b2Vec2 wALocal = proxyA->GetVertex(v->indexA);
            b2Vec2 wBLocal = proxyB->GetVertex(v->indexB);
            v->wA = b2Mul(transformA, wALocal);
            v->wB = b2Mul(transformB, wBLocal);
            v->w = v->wB - v->wA;
            v->a = 0.0f;
        }
        
        // Compute the new simplex metric, if it is substantially different than
        // old metric then flush the simplex.
        if (m_count > 1)
        {
            float32 metric1 = cache->metric;
            float32 metric2 = GetMetric();
            if (metric2 < 0.5f * metric1 || 2.0f * metric1 < metric2 || metric2 < b2_epsilon)
            {
                // Reset the simplex.
                m_count = 0;
            }
        }
        
        // If the cache is empty or invalid ...
        if (m_count == 0)
        {
            b2SimplexVertex* v = vertices + 0;
            v->indexA = 0;
            v->indexB = 0;
            b2Vec2 wALocal = proxyA->GetVertex(0);
            b2Vec2 wBLocal = proxyB->GetVertex(0);
            v->wA = b2Mul(transformA, wALocal);
            v->wB = b2Mul(transformB, wBLocal);
            v->w = v->wB - v->wA;
            v->a = 1.0f;
            m_count = 1;
        }
    }
    
    void WriteCache(b2SimplexCache* cache) const
    {
        cache->metric = GetMetric();
        cache->count = uint16(m_count);
        const b2SimplexVertex* vertices = &m_v1;
        for (int32 i = 0; i < m_count; ++i)
        {
            cache->indexA[i] = uint8(vertices[i].indexA);
            cache->indexB[i] = uint8(vertices[i].indexB);
        }
    }
    
    b2Vec2 GetSearchDirection() const
    {
        switch (m_count)
        {
            case 1:
                return -m_v1.w;
                
            case 2:
            {
                b2Vec2 e12 = m_v2.w - m_v1.w;
                float32 sgn = b2Cross(e12, -m_v1.w);
                if (sgn > 0.0f)
                {
                    // Origin is left of e12.
                    return b2Cross(1.0f, e12);
                }
                else
                {
                    // Origin is right of e12.
                    return b2Cross(e12, 1.0f);
                }
            }
                
            default:
                b2Assert(false);
                return b2Vec2::ZERO();
        }
    }
    
    b2Vec2 GetClosestPoint() const
    {
        switch (m_count)
        {
            case 0:
                b2Assert(false);
                return b2Vec2::ZERO();
                
            case 1:
                return m_v1.w;
                
            case 2:
                return m_v1.a * m_v1.w + m_v2.a * m_v2.w;
                
            case 3:
                return b2Vec2::ZERO();
                
            default:
                b2Assert(false);
                return b2Vec2::ZERO();
        }
    }
    
    void GetWitnessPoints(b2Vec2* pA, b2Vec2* pB) const
    {
        switch (m_count)
        {
            case 0:
                b2Assert(false);
                break;
                
            case 1:
                *pA = m_v1.wA;
                *pB = m_v1.wB;
                break;
                
            case 2:
                *pA = m_v1.a * m_v1.wA + m_v2.a * m_v2.wA;
                *pB = m_v1.a * m_v1.wB + m_v2.a * m_v2.wB;
                break;
                
            case 3:
                *pA = m_v1.a * m_v1.wA + m_v2.a * m_v2.wA + m_v3.a * m_v3.wA;
                *pB = *pA;
                break;
                
            default:
                b2Assert(false);
                break;
        }
    }
    
    float32 GetMetric() const
    {
        switch (m_count)
        {
            case 0:
                b2Assert(false);
                return 0.0f;
                
            case 1:
                return 0.0f;
                
            case 2:
                return b2Distance(m_v1.w, m_v2.w);
                
            case 3:
                return b2Cross(m_v2.w - m_v1.w, m_v3.w - m_v1.w);
                
            default:
                b2Assert(false);
                return 0.0f;
        }
    }
    
    // Solve a line segment using barycentric coordinates.
    //
    // p = a1 * w1 + a2 * w2
    // a1 + a2 = 1
    //
    // The vector from the origin to the closest point on the line is
    // perpendicular to the line.
    // e12 = w2 - w1
    // dot(p, e) = 0
    // a1 * dot(w1, e) + a2 * dot(w2, e) = 0
    //
    // 2-by-2 linear system
    // [1      1     ][a1] = [1]
    // [w1.e12 w2.e12][a2] = [0]
    //
    // Define
    // d12_1 =  dot(w2, e12)
    // d12_2 = -dot(w1, e12)
    // d12 = d12_1 + d12_2
    //
    // Solution
    // a1 = d12_1 / d12
    // a2 = d12_2 / d12
    void Solve2()
    {
        b2Vec2 w1 = m_v1.w;
        b2Vec2 w2 = m_v2.w;
        b2Vec2 e12 = w2 - w1;
        
        // w1 region
        float32 d12_2 = -b2Dot(w1, e12);
        if (d12_2 <= 0.0f)
        {
            // a2 <= 0, so we clamp it to 0
            m_v1.a = 1.0f;
            m_count = 1;
            return;
        }
        
        // w2 region
        float32 d12_1 = b2Dot(w2, e12);
        if (d12_1 <= 0.0f)
        {
            // a1 <= 0, so we clamp it to 0
            m_v2.a = 1.0f;
            m_count = 1;
            m_v1 = m_v2;
            return;
        }
        
        // Must be in e12 region.
        float32 inv_d12 = 1.0f / (d12_1 + d12_2);
        m_v1.a = d12_1 * inv_d12;
        m_v2.a = d12_2 * inv_d12;
        m_count = 2;
    }
    
    // Possible regions:
    // - points[2]
    // - edge points[0]-points[2]
    // - edge points[1]-points[2]
    // - inside the triangle
    void Solve3()
    {
        b2Vec2 w1 = m_v1.w;
        b2Vec2 w2 = m_v2.w;
        b2Vec2 w3 = m_v3.w;
        
        // Edge12
        // [1      1     ][a1] = [1]
        // [w1.e12 w2.e12][a2] = [0]
        // a3 = 0
        b2Vec2 e12 = w2 - w1;
        float32 w1e12 = b2Dot(w1, e12);
        float32 w2e12 = b2Dot(w2, e12);
        float32 d12_1 = w2e12;
        float32 d12_2 = -w1e12;
        
        // Edge13
        // [1      1     ][a1] = [1]
        // [w1.e13 w3.e13][a3] = [0]
        // a2 = 0
        b2Vec2 e13 = w3 - w1;
        float32 w1e13 = b2Dot(w1, e13);
        float32 w3e13 = b2Dot(w3, e13);
        float32 d13_1 = w3e13;
        float32 d13_2 = -w1e13;
        
        // Edge23
        // [1      1     ][a2] = [1]
        // [w2.e23 w3.e23][a3] = [0]
        // a1 = 0
        b2Vec2 e23 = w3 - w2;
        float32 w2e23 = b2Dot(w2, e23);
        float32 w3e23 = b2Dot(w3, e23);
        float32 d23_1 = w3e23;
        float32 d23_2 = -w2e23;
        
        // Triangle123
        float32 n123 = b2Cross(e12, e13);
        
        float32 d123_1 = n123 * b2Cross(w2, w3);
        float32 d123_2 = n123 * b2Cross(w3, w1);
        float32 d123_3 = n123 * b2Cross(w1, w2);
        
        // w1 region
        if (d12_2 <= 0.0f && d13_2 <= 0.0f)
        {
            m_v1.a = 1.0f;
            m_count = 1;
            return;
        }
        
        // e12
        if (d12_1 > 0.0f && d12_2 > 0.0f && d123_3 <= 0.0f)
        {
            float32 inv_d12 = 1.0f / (d12_1 + d12_2);
            m_v1.a = d12_1 * inv_d12;
            m_v2.a = d12_2 * inv_d12;
            m_count = 2;
            return;
        }
        
        // e13
        if (d13_1 > 0.0f && d13_2 > 0.0f && d123_2 <= 0.0f)
        {
            float32 inv_d13 = 1.0f / (d13_1 + d13_2);
            m_v1.a = d13_1 * inv_d13;
            m_v3.a = d13_2 * inv_d13;
            m_count = 2;
            m_v2 = m_v3;
            return;
        }
        
        // w2 region
        if (d12_1 <= 0.0f && d23_2 <= 0.0f)
        {
            m_v2.a = 1.0f;
            m_count = 1;
            m_v1 = m_v2;
            return;
        }
        
        // w3 region
        if (d13_1 <= 0.0f && d23_1 <= 0.0f)
        {
            m_v3.a = 1.0f;
            m_count = 1;
            m_v1 = m_v3;
            return;
        }
        
        // e23
        if (d23_1 > 0.0f && d23_2 > 0.0f && d123_1 <= 0.0f)
        {
            float32 inv_d23 = 1.0f / (d23_1 + d23_2);
            m_v2.a = d23_1 * inv_d23;
            m_v3.a = d23_2 * inv_d23;
            m_count = 2;
            m_v1 = m_v3;
            return;
        }
        
        // Must be in triangle123
        float32 inv_d123 = 1.0f / (d123_1 + d123_2 + d123_3);
        m_v1.a = d123_1 * inv_d123;
        m_v2.a = d123_2 * inv_d123;
        m_v3.a = d123_3 * inv_d123;
        m_count = 3;
    }
    
    b2SimplexVertex m_v1, m_v2, m_v3;
    int32 m_count;
};

#endif
