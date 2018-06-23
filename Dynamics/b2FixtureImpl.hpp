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

#ifndef B2_FIXTURE_IMPL_H
#define B2_FIXTURE_IMPL_H

#include "b2Body.hpp"
#include "b2Fixture.hpp"
#include "../Collision/b2Collision.hpp"
#include "../Collision/b2BroadPhase.hpp"
#include "../Collision/Shapes/b2Shape.hpp"
#include "../Common/b2BlockAllocator.hpp"

/// A fixture is used to attach a shape to a body for collision detection. A fixture
/// inherits its transform from its parent. Fixtures hold additional non-geometric data
/// such as friction, collision filters, etc.
/// Fixtures are created via b2Body::CreateFixture.
/// @warning you cannot reuse fixtures.
class b2FixtureImpl : public b2Fixture
{
public:
    /// Get the type of the child shape. You can use this to down cast to the concrete shape.
    /// @return the shape type.
    b2Shape::Type GetType() const override;
    
    /// Get the child shape. You can modify the child shape, however you should not change the
    /// number of vertices because this will crash some collision caching mechanisms.
    /// Manipulating the shape may lead to non-physical behavior.
    b2Shape* GetShape() override;
    const b2Shape* GetShape() const override;
    
    /// Set if this fixture is a sensor.
    void SetSensor(bool sensor) override;
    
    /// Is this fixture a sensor (non-solid)?
    /// @return the true if the shape is a sensor.
    bool IsSensor() const override;
    
    /// Set the contact filtering data. This will not update contacts until the next time
    /// step when either parent body is active and awake.
    /// This automatically calls Refilter.
    void SetFilterData(const b2Filter& filter) override;
    
    /// Get the contact filtering data.
    const b2Filter& GetFilterData() const override;
    
    /// Call this if you want to establish collision that was previously disabled by b2ContactFilter::ShouldCollide.
    void Refilter() override;
    
    /// Get the parent body of this fixture. This is nullptr if the fixture is not attached.
    /// @return the parent body.
    b2Body* GetBody() override;
    const b2Body* GetBody() const override;
    
    /// Get the next fixture in the parent body's fixture list.
    /// @return the next shape.
    b2Fixture* GetNext() override;
    const b2Fixture* GetNext() const override;
    
    /// Get the user data that was assigned in the fixture definition. Use this to
    /// store your application specific data.
    void* GetUserData() const override;
    
    /// Set the user data. Use this to store your application specific data.
    void SetUserData(void* data) override;
    
    /// Test a point for containment in this fixture.
    /// @param p a point in world coordinates.
    bool TestPoint(const b2Vec2& p) const override;
    
    /// Cast a ray against this shape.
    /// @param output the ray-cast results.
    /// @param input the ray-cast input parameters.
    bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input, int32 childIndex) const override;
    
    /// Get the mass data for this fixture. The mass data is based on the density and
    /// the shape. The rotational inertia is about the shape's origin. This operation
    /// may be expensive.
    void GetMassData(b2MassData* massData) const override;
    
    /// Set the density of this fixture. This will _not_ automatically adjust the mass
    /// of the body. You must call b2Body::ResetMassData to update the body's mass.
    void SetDensity(float32 density) override;
    
    /// Get the density of this fixture.
    float32 GetDensity() const override;
    
    /// Get the coefficient of friction.
    float32 GetFriction() const override;
    
    /// Set the coefficient of friction. This will _not_ change the friction of
    /// existing contacts.
    void SetFriction(float32 friction) override;
    
    /// Get the coefficient of restitution.
    float32 GetRestitution() const override;
    
    /// Set the coefficient of restitution. This will _not_ change the restitution of
    /// existing contacts.
    void SetRestitution(float32 restitution) override;
    
    /// Get the fixture's AABB. This AABB may be enlarge and/or stale.
    /// If you need a more accurate AABB, compute it using the shape and
    /// the body transform.
    const b2AABB& GetAABB(int32 childIndex) const override;
    
    /// Dump this fixture to the log file.
    void Dump(int32 bodyIndex) override;
    
protected:
    
    friend class b2BodyImpl;
    friend class b2WorldImpl;
    friend class b2Contact;
    friend class b2ContactManager;
    
    b2FixtureImpl()
    {
        m_userData = nullptr;
        m_body = nullptr;
        m_next = nullptr;
        m_proxies = nullptr;
        m_proxyCount = 0;
        m_shape = nullptr;
        m_density = 0.0f;
    }
    
    // We need separation create/destroy functions from the constructor/destructor because
    // the destructor cannot access the allocator (no destructor arguments allowed by C++).
    void Create(b2BlockAllocator* allocator, b2Body* body, const b2FixtureDef* def) override;
    void Destroy(b2BlockAllocator* allocator) override;
    
    // These support body activation/deactivation.
    void CreateProxies(b2BroadPhase* broadPhase, const b2Transform& xf) override;
    void DestroyProxies(b2BroadPhase* broadPhase) override;
    void Synchronize(b2BroadPhase* broadPhase, const b2Transform& xf1, const b2Transform& xf2) override;
    
};

inline b2Shape::Type b2FixtureImpl::GetType() const
{
    return m_shape->GetType();
}

inline b2Shape* b2FixtureImpl::GetShape()
{
    return m_shape;
}

inline const b2Shape* b2FixtureImpl::GetShape() const
{
    return m_shape;
}

inline bool b2FixtureImpl::IsSensor() const
{
    return m_isSensor;
}

inline const b2Filter& b2FixtureImpl::GetFilterData() const
{
    return m_filter;
}

inline void* b2FixtureImpl::GetUserData() const
{
    return m_userData;
}

inline void b2FixtureImpl::SetUserData(void* data)
{
    m_userData = data;
}

inline b2Body* b2FixtureImpl::GetBody()
{
    return m_body;
}

inline const b2Body* b2FixtureImpl::GetBody() const
{
    return m_body;
}

inline b2Fixture* b2FixtureImpl::GetNext()
{
    return m_next;
}

inline const b2Fixture* b2FixtureImpl::GetNext() const
{
    return m_next;
}

inline void b2FixtureImpl::SetDensity(float32 density)
{
    b2Assert(b2IsValid(density) && density >= 0.0f);
    m_density = density;
}

inline float32 b2FixtureImpl::GetDensity() const
{
    return m_density;
}

inline float32 b2FixtureImpl::GetFriction() const
{
    return m_friction;
}

inline void b2FixtureImpl::SetFriction(float32 friction)
{
    m_friction = friction;
}

inline float32 b2FixtureImpl::GetRestitution() const
{
    return m_restitution;
}

inline void b2FixtureImpl::SetRestitution(float32 restitution)
{
    m_restitution = restitution;
}

inline bool b2FixtureImpl::TestPoint(const b2Vec2& p) const
{
    return m_shape->TestPoint(m_body->GetTransform(), p);
}

inline bool b2FixtureImpl::RayCast(b2RayCastOutput* output, const b2RayCastInput& input, int32 childIndex) const
{
    return m_shape->RayCast(output, input, m_body->GetTransform(), childIndex);
}

inline void b2FixtureImpl::GetMassData(b2MassData* massData) const
{
    m_shape->ComputeMass(massData, m_density);
}

inline const b2AABB& b2FixtureImpl::GetAABB(int32 childIndex) const
{
    b2Assert(0 <= childIndex && childIndex < m_proxyCount);
    return m_proxies[childIndex].aabb;
}

inline void b2FixtureImpl::Create(b2BlockAllocator* allocator, b2Body* body, const b2FixtureDef* def)
{
    m_userData = def->userData;
    m_friction = def->friction;
    m_restitution = def->restitution;
    
    m_body = body;
    m_next = nullptr;
    
    m_filter = def->filter;
    
    m_isSensor = def->isSensor;
    
    m_shape = def->shape->Clone(allocator);
    
    // Reserve proxy space
    int32 childCount = m_shape->GetChildCount();
    m_proxies = (b2FixtureProxy*)allocator->Allocate(childCount * sizeof(b2FixtureProxy));
    for (int32 i = 0; i < childCount; ++i)
    {
        m_proxies[i].fixture = nullptr;
        m_proxies[i].proxyId = b2BroadPhase::e_nullProxy;
    }
    m_proxyCount = 0;
    
    m_density = def->density;
}

inline void b2FixtureImpl::Destroy(b2BlockAllocator* allocator)
{
    // The proxies must be destroyed before calling this.
    b2Assert(m_proxyCount == 0);
    
    // Free the proxy array.
    int32 childCount = m_shape->GetChildCount();
    allocator->Free(m_proxies, childCount * sizeof(b2FixtureProxy));
    m_proxies = nullptr;
    
    // Free the child shape.
    switch (m_shape->m_type)
    {
        case b2Shape::e_circle:
        {
            b2CircleShape* s = (b2CircleShape*)m_shape;
            s->~b2CircleShape();
            allocator->Free(s, sizeof(b2CircleShape));
        }
            break;
            
        case b2Shape::e_edge:
        {
            b2EdgeShape* s = (b2EdgeShape*)m_shape;
            s->~b2EdgeShape();
            allocator->Free(s, sizeof(b2EdgeShape));
        }
            break;
            
        case b2Shape::e_polygon:
        {
            b2PolygonShape* s = (b2PolygonShape*)m_shape;
            s->~b2PolygonShape();
            allocator->Free(s, sizeof(b2PolygonShape));
        }
            break;
            
        case b2Shape::e_chain:
        {
            b2ChainShape* s = (b2ChainShape*)m_shape;
            s->~b2ChainShape();
            allocator->Free(s, sizeof(b2ChainShape));
        }
            break;
            
        default:
            b2Assert(false);
            break;
    }
    
    m_shape = nullptr;
}

inline void b2FixtureImpl::CreateProxies(b2BroadPhase* broadPhase, const b2Transform& xf)
{
    b2Assert(m_proxyCount == 0);
    
    // Create proxies in the broad-phase.
    m_proxyCount = m_shape->GetChildCount();
    
    for (int32 i = 0; i < m_proxyCount; ++i)
    {
        b2FixtureProxy* proxy = m_proxies + i;
        m_shape->ComputeAABB(&proxy->aabb, xf, i);
        proxy->proxyId = broadPhase->CreateProxy(proxy->aabb, proxy);
        proxy->fixture = this;
        proxy->childIndex = i;
    }
}

inline void b2FixtureImpl::DestroyProxies(b2BroadPhase* broadPhase)
{
    // Destroy proxies in the broad-phase.
    for (int32 i = 0; i < m_proxyCount; ++i)
    {
        b2FixtureProxy* proxy = m_proxies + i;
        broadPhase->DestroyProxy(proxy->proxyId);
        proxy->proxyId = b2BroadPhase::e_nullProxy;
    }
    
    m_proxyCount = 0;
}

inline void b2FixtureImpl::Synchronize(b2BroadPhase* broadPhase, const b2Transform& transform1, const b2Transform& transform2)
{
    if (m_proxyCount == 0)
    {
        return;
    }
    
    for (int32 i = 0; i < m_proxyCount; ++i)
    {
        b2FixtureProxy* proxy = m_proxies + i;
        
        // Compute an AABB that covers the swept shape (may miss some rotation effect).
        b2AABB aabb1, aabb2;
        m_shape->ComputeAABB(&aabb1, transform1, proxy->childIndex);
        m_shape->ComputeAABB(&aabb2, transform2, proxy->childIndex);
        
        proxy->aabb.Combine(aabb1, aabb2);
        
        b2Vec2 displacement = transform2.p - transform1.p;
        
        broadPhase->MoveProxy(proxy->proxyId, proxy->aabb, displacement);
    }
}

inline void b2FixtureImpl::SetFilterData(const b2Filter& filter)
{
    m_filter = filter;
    
    Refilter();
}

inline void b2FixtureImpl::Refilter()
{
    if (m_body == nullptr)
    {
        return;
    }
    
    // Flag associated contacts for filtering.
    b2ContactEdge* edge = m_body->GetContactList();
    while (edge)
    {
        b2Contact* contact = edge->contact;
        b2Fixture* fixtureA = contact->GetFixtureA();
        b2Fixture* fixtureB = contact->GetFixtureB();
        if (fixtureA == this || fixtureB == this)
        {
            contact->FlagForFiltering();
        }
        
        edge = edge->next;
    }
    
    b2World* world = m_body->GetWorld();
    
    if (world == nullptr)
    {
        return;
    }
    
    // Touch each proxy so that new pairs may be created
    b2BroadPhase* broadPhase = &world->m_contactManager.m_broadPhase;
    for (int32 i = 0; i < m_proxyCount; ++i)
    {
        broadPhase->TouchProxy(m_proxies[i].proxyId);
    }
}

inline void b2FixtureImpl::SetSensor(bool sensor)
{
    if (sensor != m_isSensor)
    {
        m_body->SetAwake(true);
        m_isSensor = sensor;
    }
}

inline void b2FixtureImpl::Dump(int32 bodyIndex)
{
    b2Log("    b2FixtureDef fd;\n");
    b2Log("    fd.friction = %.15lef;\n", m_friction);
    b2Log("    fd.restitution = %.15lef;\n", m_restitution);
    b2Log("    fd.density = %.15lef;\n", m_density);
    b2Log("    fd.isSensor = bool(%d);\n", m_isSensor);
    b2Log("    fd.filter.categoryBits = uint16(%d);\n", m_filter.categoryBits);
    b2Log("    fd.filter.maskBits = uint16(%d);\n", m_filter.maskBits);
    b2Log("    fd.filter.groupIndex = int16(%d);\n", m_filter.groupIndex);
    
    switch (m_shape->m_type)
    {
        case b2Shape::e_circle:
        {
            b2CircleShape* s = (b2CircleShape*)m_shape;
            b2Log("    b2CircleShape shape;\n");
            b2Log("    shape.m_radius = %.15lef;\n", s->m_radius);
            b2Log("    shape.m_p.Set(%.15lef, %.15lef);\n", s->m_p.x, s->m_p.y);
        }
            break;
            
        case b2Shape::e_edge:
        {
            b2EdgeShape* s = (b2EdgeShape*)m_shape;
            b2Log("    b2EdgeShape shape;\n");
            b2Log("    shape.m_radius = %.15lef;\n", s->m_radius);
            b2Log("    shape.m_vertex0.Set(%.15lef, %.15lef);\n", s->m_vertex0.x, s->m_vertex0.y);
            b2Log("    shape.m_vertex1.Set(%.15lef, %.15lef);\n", s->m_vertex1.x, s->m_vertex1.y);
            b2Log("    shape.m_vertex2.Set(%.15lef, %.15lef);\n", s->m_vertex2.x, s->m_vertex2.y);
            b2Log("    shape.m_vertex3.Set(%.15lef, %.15lef);\n", s->m_vertex3.x, s->m_vertex3.y);
            b2Log("    shape.m_hasVertex0 = bool(%d);\n", s->m_hasVertex0);
            b2Log("    shape.m_hasVertex3 = bool(%d);\n", s->m_hasVertex3);
        }
            break;
            
        case b2Shape::e_polygon:
        {
            b2PolygonShape* s = (b2PolygonShape*)m_shape;
            b2Log("    b2PolygonShape shape;\n");
            b2Log("    b2Vec2 vs[%d];\n", b2_maxPolygonVertices);
            for (int32 i = 0; i < s->m_count; ++i)
            {
                b2Log("    vs[%d].Set(%.15lef, %.15lef);\n", i, s->m_vertices[i].x, s->m_vertices[i].y);
            }
            b2Log("    shape.Set(vs, %d);\n", s->m_count);
        }
            break;
            
        case b2Shape::e_chain:
        {
            b2ChainShape* s = (b2ChainShape*)m_shape;
            b2Log("    b2ChainShape shape;\n");
            b2Log("    b2Vec2 vs[%d];\n", s->m_count);
            for (int32 i = 0; i < s->m_count; ++i)
            {
                b2Log("    vs[%d].Set(%.15lef, %.15lef);\n", i, s->m_vertices[i].x, s->m_vertices[i].y);
            }
            b2Log("    shape.CreateChain(vs, %d);\n", s->m_count);
            b2Log("    shape.m_prevVertex.Set(%.15lef, %.15lef);\n", s->m_prevVertex.x, s->m_prevVertex.y);
            b2Log("    shape.m_nextVertex.Set(%.15lef, %.15lef);\n", s->m_nextVertex.x, s->m_nextVertex.y);
            b2Log("    shape.m_hasPrevVertex = bool(%d);\n", s->m_hasPrevVertex);
            b2Log("    shape.m_hasNextVertex = bool(%d);\n", s->m_hasNextVertex);
        }
            break;
            
        default:
            return;
    }
    
    b2Log("\n");
    b2Log("    fd.shape = &shape;\n");
    b2Log("\n");
    b2Log("    bodies[%d]->CreateFixture(&fd);\n", bodyIndex);
}

#endif
