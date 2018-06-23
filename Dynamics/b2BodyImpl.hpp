/*
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#ifndef B2_BODY_IMPL_H
#define B2_BODY_IMPL_H

#include "b2Joint.hpp"
#include "b2World.hpp"
#include "b2FixtureImpl.hpp"

/// A rigid body. These are created via b2World::CreateBody.
class b2BodyImpl : public b2Body
{
    
public:
    
	/// Creates a fixture and attach it to this body. Use this function if you need
	/// to set some fixture parameters, like friction. Otherwise you can create the
	/// fixture directly from a shape.
	/// If the density is non-zero, this function automatically updates the mass of the body.
	/// Contacts are not created until the next time step.
	/// @param def the fixture definition.
	/// @warning This function is locked during callbacks.
    b2Fixture* CreateFixture(const b2FixtureDef* def) override;

	/// Creates a fixture from a shape and attach it to this body.
	/// This is a convenience function. Use b2FixtureDef if you need to set parameters
	/// like friction, restitution, user data, or filtering.
	/// If the density is non-zero, this function automatically updates the mass of the body.
	/// @param shape the shape to be cloned.
	/// @param density the shape density (set to zero for static bodies).
	/// @warning This function is locked during callbacks.
	b2Fixture* CreateFixture(const b2Shape* shape, float32 density) override;

	/// Destroy a fixture. This removes the fixture from the broad-phase and
	/// destroys all contacts associated with this fixture. This will
	/// automatically adjust the mass of the body if the body is dynamic and the
	/// fixture has positive density.
	/// All fixtures attached to a body are implicitly destroyed when the body is destroyed.
	/// @param fixture the fixture to be removed.
	/// @warning This function is locked during callbacks.
	void DestroyFixture(b2Fixture* fixture) override;

	/// Set the position of the body's origin and rotation.
	/// Manipulating a body's transform may cause non-physical behavior.
	/// Note: contacts are updated on the next call to b2World::Step.
	/// @param position the world position of the body's local origin.
	/// @param angle the world rotation in radians.
	void SetTransform(const b2Vec2& position, float32 angle) override;

	/// Get the body transform for the body's origin.
	/// @return the world transform of the body's origin.
	const b2Transform& GetTransform() const override;

	/// Get the world body origin position.
	/// @return the world position of the body's origin.
	const b2Vec2& GetPosition() const override;

	/// Get the angle in radians.
	/// @return the current world rotation angle in radians.
	float32 GetAngle() const override;

	/// Get the world position of the center of mass.
	const b2Vec2& GetWorldCenter() const override;

	/// Get the local position of the center of mass.
	const b2Vec2& GetLocalCenter() const override;

	/// Set the linear velocity of the center of mass.
	/// @param v the new linear velocity of the center of mass.
	void SetLinearVelocity(const b2Vec2& v) override;

	/// Get the linear velocity of the center of mass.
	/// @return the linear velocity of the center of mass.
	const b2Vec2& GetLinearVelocity() const override;

	/// Set the angular velocity.
	/// @param omega the new angular velocity in radians/second.
	void SetAngularVelocity(float32 omega) override;

	/// Get the angular velocity.
	/// @return the angular velocity in radians/second.
	float32 GetAngularVelocity() const override;

	/// Apply a force at a world point. If the force is not
	/// applied at the center of mass, it will generate a torque and
	/// affect the angular velocity. This wakes up the body.
	/// @param force the world force vector, usually in Newtons (N).
	/// @param point the world position of the point of application.
	/// @param wake also wake up the body
	void ApplyForce(const b2Vec2& force, const b2Vec2& point, bool wake) override;

	/// Apply a force to the center of mass. This wakes up the body.
	/// @param force the world force vector, usually in Newtons (N).
	/// @param wake also wake up the body
	void ApplyForceToCenter(const b2Vec2& force, bool wake) override;

	/// Apply a torque. This affects the angular velocity
	/// without affecting the linear velocity of the center of mass.
	/// @param torque about the z-axis (out of the screen), usually in N-m.
	/// @param wake also wake up the body
	void ApplyTorque(float32 torque, bool wake) override;

	/// Apply an impulse at a point. This immediately modifies the velocity.
	/// It also modifies the angular velocity if the point of application
	/// is not at the center of mass. This wakes up the body.
	/// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
	/// @param point the world position of the point of application.
	/// @param wake also wake up the body
	void ApplyLinearImpulse(const b2Vec2& impulse, const b2Vec2& point, bool wake) override;

	/// Apply an impulse to the center of mass. This immediately modifies the velocity.
	/// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
	/// @param wake also wake up the body
	void ApplyLinearImpulseToCenter(const b2Vec2& impulse, bool wake) override;

	/// Apply an angular impulse.
	/// @param impulse the angular impulse in units of kg*m*m/s
	/// @param wake also wake up the body
	void ApplyAngularImpulse(float32 impulse, bool wake) override;

	/// Get the total mass of the body.
	/// @return the mass, usually in kilograms (kg).
	float32 GetMass() const override;

	/// Get the rotational inertia of the body about the local origin.
	/// @return the rotational inertia, usually in kg-m^2.
	float32 GetInertia() const override;

	/// Get the mass data of the body.
	/// @return a struct containing the mass, inertia and center of the body.
	void GetMassData(b2MassData* data) const override;

	/// Set the mass properties to override the mass properties of the fixtures.
	/// Note that this changes the center of mass position.
	/// Note that creating or destroying fixtures can also alter the mass.
	/// This function has no effect if the body isn't dynamic.
	/// @param massData the mass properties.
	void SetMassData(const b2MassData* data) override;

	/// This resets the mass properties to the sum of the mass properties of the fixtures.
	/// This normally does not need to be called unless you called SetMassData to override
	/// the mass and you later want to reset the mass.
	void ResetMassData() override;

	/// Get the world coordinates of a point given the local coordinates.
	/// @param localPoint a point on the body measured relative the the body's origin.
	/// @return the same point expressed in world coordinates.
	b2Vec2 GetWorldPoint(const b2Vec2& localPoint) const override;

	/// Get the world coordinates of a vector given the local coordinates.
	/// @param localVector a vector fixed in the body.
	/// @return the same vector expressed in world coordinates.
	b2Vec2 GetWorldVector(const b2Vec2& localVector) const override;

	/// Gets a local point relative to the body's origin given a world point.
	/// @param a point in world coordinates.
	/// @return the corresponding local point relative to the body's origin.
	b2Vec2 GetLocalPoint(const b2Vec2& worldPoint) const override;

	/// Gets a local vector given a world vector.
	/// @param a vector in world coordinates.
	/// @return the corresponding local vector.
	b2Vec2 GetLocalVector(const b2Vec2& worldVector) const override;

	/// Get the world linear velocity of a world point attached to this body.
	/// @param a point in world coordinates.
	/// @return the world velocity of a point.
	b2Vec2 GetLinearVelocityFromWorldPoint(const b2Vec2& worldPoint) const override;

	/// Get the world velocity of a local point.
	/// @param a point in local coordinates.
	/// @return the world velocity of a point.
	b2Vec2 GetLinearVelocityFromLocalPoint(const b2Vec2& localPoint) const override;

	/// Get the linear damping of the body.
	float32 GetLinearDamping() const override;

	/// Set the linear damping of the body.
	void SetLinearDamping(float32 linearDamping) override;

	/// Get the angular damping of the body.
	float32 GetAngularDamping() const override;

	/// Set the angular damping of the body.
	void SetAngularDamping(float32 angularDamping) override;

	/// Get the gravity scale of the body.
	float32 GetGravityScale() const override;

	/// Set the gravity scale of the body.
	void SetGravityScale(float32 scale) override;

	/// Set the type of this body. This may alter the mass and velocity.
	void SetType(b2BodyType type) override;

	/// Get the type of this body.
	b2BodyType GetType() const override;

	/// Should this body be treated like a bullet for continuous collision detection?
	void SetBullet(bool flag) override;

	/// Is this body treated like a bullet for continuous collision detection?
	bool IsBullet() const override;

	/// You can disable sleeping on this body. If you disable sleeping, the
	/// body will be woken.
	void SetSleepingAllowed(bool flag) override;

	/// Is this body allowed to sleep
	bool IsSleepingAllowed() const override;

	/// Set the sleep state of the body. A sleeping body has very
	/// low CPU cost.
	/// @param flag set to true to wake the body, false to put it to sleep.
	void SetAwake(bool flag) override;

	/// Get the sleeping state of this body.
	/// @return true if the body is awake.
	bool IsAwake() const override;

	/// Set the active state of the body. An inactive body is not
	/// simulated and cannot be collided with or woken up.
	/// If you pass a flag of true, all fixtures will be added to the
	/// broad-phase.
	/// If you pass a flag of false, all fixtures will be removed from
	/// the broad-phase and all contacts will be destroyed.
	/// Fixtures and joints are otherwise unaffected. You may continue
	/// to create/destroy fixtures and joints on inactive bodies.
	/// Fixtures on an inactive body are implicitly inactive and will
	/// not participate in collisions, ray-casts, or queries.
	/// Joints connected to an inactive body are implicitly inactive.
	/// An inactive body is still owned by a b2World object and remains
	/// in the body list.
	void SetActive(bool flag) override;

	/// Get the active state of the body.
	bool IsActive() const override;

	/// Set this body to have fixed rotation. This causes the mass
	/// to be reset.
	void SetFixedRotation(bool flag) override;

	/// Does this body have fixed rotation?
	bool IsFixedRotation() const override;

	/// Get the list of all fixtures attached to this body.
	b2Fixture* GetFixtureList() override;
	const b2Fixture* GetFixtureList() const override;

	/// Get the list of all joints attached to this body.
	b2JointEdge* GetJointList() override;
	const b2JointEdge* GetJointList() const override;

	/// Get the list of all contacts attached to this body.
	/// @warning this list changes during the time step and you may
	/// miss some collisions if you don't use b2ContactListener.
	b2ContactEdge* GetContactList() override;
	const b2ContactEdge* GetContactList() const override;

	/// Get the next body in the world's body list.
	b2Body* GetNext() override;
	const b2Body* GetNext() const override;

	/// Get the user data pointer that was provided in the body definition.
	void* GetUserData() const override;

	/// Set the user data. Use this to store your application specific data.
	void SetUserData(void* data) override;

	/// Get the parent world of this body.
	b2World* GetWorld() override;
	const b2World* GetWorld() const override;

	/// Dump this body to a log file
	void Dump() override;

private:

	b2BodyImpl(const b2BodyDef* bd, b2World* world);
	~b2BodyImpl();

	void SynchronizeFixtures() override;
	void SynchronizeTransform() override;

	// This is used to prevent connected bodies from colliding.
	// It may lie, depending on the collideConnected flag.
    bool ShouldCollide(const b2Body* other) const override;
	void Advance(float32 t) override;

    friend class b2WorldImpl;
    
};

inline b2BodyType b2BodyImpl::GetType() const
{
	return m_type;
}

inline const b2Transform& b2BodyImpl::GetTransform() const
{
	return m_xf;
}

inline const b2Vec2& b2BodyImpl::GetPosition() const
{
	return m_xf.p;
}

inline float32 b2BodyImpl::GetAngle() const
{
	return m_sweep.a;
}

inline const b2Vec2& b2BodyImpl::GetWorldCenter() const
{
	return m_sweep.c;
}

inline const b2Vec2& b2BodyImpl::GetLocalCenter() const
{
	return m_sweep.localCenter;
}

inline void b2BodyImpl::SetLinearVelocity(const b2Vec2& v)
{
	if (m_type == b2_staticBody)
	{
		return;
	}

	if (b2Dot(v,v) > 0.0f)
	{
		SetAwake(true);
	}

	m_linearVelocity = v;
}

inline const b2Vec2& b2BodyImpl::GetLinearVelocity() const
{
	return m_linearVelocity;
}

inline void b2BodyImpl::SetAngularVelocity(float32 w)
{
	if (m_type == b2_staticBody)
	{
		return;
	}

	if (w * w > 0.0f)
	{
		SetAwake(true);
	}

	m_angularVelocity = w;
}

inline float32 b2BodyImpl::GetAngularVelocity() const
{
	return m_angularVelocity;
}

inline float32 b2BodyImpl::GetMass() const
{
	return m_mass;
}

inline float32 b2BodyImpl::GetInertia() const
{
	return m_I + m_mass * b2Dot(m_sweep.localCenter, m_sweep.localCenter);
}

inline void b2BodyImpl::GetMassData(b2MassData* data) const
{
	data->mass = m_mass;
	data->I = m_I + m_mass * b2Dot(m_sweep.localCenter, m_sweep.localCenter);
	data->center = m_sweep.localCenter;
}

inline b2Vec2 b2BodyImpl::GetWorldPoint(const b2Vec2& localPoint) const
{
	return b2Mul(m_xf, localPoint);
}

inline b2Vec2 b2BodyImpl::GetWorldVector(const b2Vec2& localVector) const
{
	return b2Mul(m_xf.q, localVector);
}

inline b2Vec2 b2BodyImpl::GetLocalPoint(const b2Vec2& worldPoint) const
{
	return b2MulT(m_xf, worldPoint);
}

inline b2Vec2 b2BodyImpl::GetLocalVector(const b2Vec2& worldVector) const
{
	return b2MulT(m_xf.q, worldVector);
}

inline b2Vec2 b2BodyImpl::GetLinearVelocityFromWorldPoint(const b2Vec2& worldPoint) const
{
	return m_linearVelocity + b2Cross(m_angularVelocity, worldPoint - m_sweep.c);
}

inline b2Vec2 b2BodyImpl::GetLinearVelocityFromLocalPoint(const b2Vec2& localPoint) const
{
	return GetLinearVelocityFromWorldPoint(GetWorldPoint(localPoint));
}

inline float32 b2BodyImpl::GetLinearDamping() const
{
	return m_linearDamping;
}

inline void b2BodyImpl::SetLinearDamping(float32 linearDamping)
{
	m_linearDamping = linearDamping;
}

inline float32 b2BodyImpl::GetAngularDamping() const
{
	return m_angularDamping;
}

inline void b2BodyImpl::SetAngularDamping(float32 angularDamping)
{
	m_angularDamping = angularDamping;
}

inline float32 b2BodyImpl::GetGravityScale() const
{
	return m_gravityScale;
}

inline void b2BodyImpl::SetGravityScale(float32 scale)
{
	m_gravityScale = scale;
}

inline void b2BodyImpl::SetBullet(bool flag)
{
	if (flag)
	{
		m_flags |= e_bulletFlag;
	}
	else
	{
		m_flags &= ~e_bulletFlag;
	}
}

inline bool b2BodyImpl::IsBullet() const
{
	return (m_flags & e_bulletFlag) == e_bulletFlag;
}

inline void b2BodyImpl::SetAwake(bool flag)
{
	if (flag)
	{
		m_flags |= e_awakeFlag;
		m_sleepTime = 0.0f;
	}
	else
	{
		m_flags &= ~e_awakeFlag;
		m_sleepTime = 0.0f;
		m_linearVelocity.SetZero();
		m_angularVelocity = 0.0f;
		m_force.SetZero();
		m_torque = 0.0f;
	}
}

inline bool b2BodyImpl::IsAwake() const
{
	return (m_flags & e_awakeFlag) == e_awakeFlag;
}

inline bool b2BodyImpl::IsActive() const
{
	return (m_flags & e_activeFlag) == e_activeFlag;
}

inline bool b2BodyImpl::IsFixedRotation() const
{
	return (m_flags & e_fixedRotationFlag) == e_fixedRotationFlag;
}

inline void b2BodyImpl::SetSleepingAllowed(bool flag)
{
	if (flag)
	{
		m_flags |= e_autoSleepFlag;
	}
	else
	{
		m_flags &= ~e_autoSleepFlag;
		SetAwake(true);
	}
}

inline bool b2BodyImpl::IsSleepingAllowed() const
{
	return (m_flags & e_autoSleepFlag) == e_autoSleepFlag;
}

inline b2Fixture* b2BodyImpl::GetFixtureList()
{
	return m_fixtureList;
}

inline const b2Fixture* b2BodyImpl::GetFixtureList() const
{
	return m_fixtureList;
}

inline b2JointEdge* b2BodyImpl::GetJointList()
{
	return m_jointList;
}

inline const b2JointEdge* b2BodyImpl::GetJointList() const
{
	return m_jointList;
}

inline b2ContactEdge* b2BodyImpl::GetContactList()
{
	return m_contactList;
}

inline const b2ContactEdge* b2BodyImpl::GetContactList() const
{
	return m_contactList;
}

inline b2Body* b2BodyImpl::GetNext()
{
	return m_next;
}

inline const b2Body* b2BodyImpl::GetNext() const
{
	return m_next;
}

inline void b2BodyImpl::SetUserData(void* data)
{
	m_userData = data;
}

inline void* b2BodyImpl::GetUserData() const
{
	return m_userData;
}

inline void b2BodyImpl::ApplyForce(const b2Vec2& force, const b2Vec2& point, bool wake)
{
	if (m_type != b2_dynamicBody)
	{
		return;
	}

	if (wake && (m_flags & e_awakeFlag) == 0)
	{
		SetAwake(true);
	}

	// Don't accumulate a force if the body is sleeping.
	if (m_flags & e_awakeFlag)
	{
		m_force += force;
		m_torque += b2Cross(point - m_sweep.c, force);
	}
}

inline void b2BodyImpl::ApplyForceToCenter(const b2Vec2& force, bool wake)
{
	if (m_type != b2_dynamicBody)
	{
		return;
	}

	if (wake && (m_flags & e_awakeFlag) == 0)
	{
		SetAwake(true);
	}

	// Don't accumulate a force if the body is sleeping
	if (m_flags & e_awakeFlag)
	{
		m_force += force;
	}
}

inline void b2BodyImpl::ApplyTorque(float32 torque, bool wake)
{
	if (m_type != b2_dynamicBody)
	{
		return;
	}

	if (wake && (m_flags & e_awakeFlag) == 0)
	{
		SetAwake(true);
	}

	// Don't accumulate a force if the body is sleeping
	if (m_flags & e_awakeFlag)
	{
		m_torque += torque;
	}
}

inline void b2BodyImpl::ApplyLinearImpulse(const b2Vec2& impulse, const b2Vec2& point, bool wake)
{
	if (m_type != b2_dynamicBody)
	{
		return;
	}

	if (wake && (m_flags & e_awakeFlag) == 0)
	{
		SetAwake(true);
	}

	// Don't accumulate velocity if the body is sleeping
	if (m_flags & e_awakeFlag)
	{
		m_linearVelocity += m_invMass * impulse;
		m_angularVelocity += m_invI * b2Cross(point - m_sweep.c, impulse);
	}
}

inline void b2BodyImpl::ApplyLinearImpulseToCenter(const b2Vec2& impulse, bool wake)
{
	if (m_type != b2_dynamicBody)
	{
		return;
	}

	if (wake && (m_flags & e_awakeFlag) == 0)
	{
		SetAwake(true);
	}

	// Don't accumulate velocity if the body is sleeping
	if (m_flags & e_awakeFlag)
	{
		m_linearVelocity += m_invMass * impulse;
	}
}

inline void b2BodyImpl::ApplyAngularImpulse(float32 impulse, bool wake)
{
	if (m_type != b2_dynamicBody)
	{
		return;
	}

	if (wake && (m_flags & e_awakeFlag) == 0)
	{
		SetAwake(true);
	}

	// Don't accumulate velocity if the body is sleeping
	if (m_flags & e_awakeFlag)
	{
		m_angularVelocity += m_invI * impulse;
	}
}

inline void b2BodyImpl::SynchronizeTransform()
{
	m_xf.q.Set(m_sweep.a);
	m_xf.p = m_sweep.c - b2Mul(m_xf.q, m_sweep.localCenter);
}

inline void b2BodyImpl::Advance(float32 alpha)
{
	// Advance to the new safe time. This doesn't sync the broad-phase.
	m_sweep.Advance(alpha);
	m_sweep.c = m_sweep.c0;
	m_sweep.a = m_sweep.a0;
	m_xf.q.Set(m_sweep.a);
	m_xf.p = m_sweep.c - b2Mul(m_xf.q, m_sweep.localCenter);
}

inline b2World* b2BodyImpl::GetWorld()
{
	return m_world;
}

inline const b2World* b2BodyImpl::GetWorld() const
{
	return m_world;
}

inline b2BodyImpl::b2BodyImpl(const b2BodyDef* bd, b2World* world)
{
    b2Assert(bd->position.IsValid());
    b2Assert(bd->linearVelocity.IsValid());
    b2Assert(b2IsValid(bd->angle));
    b2Assert(b2IsValid(bd->angularVelocity));
    b2Assert(b2IsValid(bd->angularDamping) && bd->angularDamping >= 0.0f);
    b2Assert(b2IsValid(bd->linearDamping) && bd->linearDamping >= 0.0f);
    
    m_flags = 0;
    
    if (bd->bullet)
    {
        m_flags |= e_bulletFlag;
    }
    if (bd->fixedRotation)
    {
        m_flags |= e_fixedRotationFlag;
    }
    if (bd->allowSleep)
    {
        m_flags |= e_autoSleepFlag;
    }
    if (bd->awake)
    {
        m_flags |= e_awakeFlag;
    }
    if (bd->active)
    {
        m_flags |= e_activeFlag;
    }
    
    m_world = world;
    
    m_xf.p = bd->position;
    m_xf.q.Set(bd->angle);
    
    m_sweep.localCenter.SetZero();
    m_sweep.c0 = m_xf.p;
    m_sweep.c = m_xf.p;
    m_sweep.a0 = bd->angle;
    m_sweep.a = bd->angle;
    m_sweep.alpha0 = 0.0f;
    
    m_jointList = nullptr;
    m_contactList = nullptr;
    m_prev = nullptr;
    m_next = nullptr;
    
    m_linearVelocity = bd->linearVelocity;
    m_angularVelocity = bd->angularVelocity;
    
    m_linearDamping = bd->linearDamping;
    m_angularDamping = bd->angularDamping;
    m_gravityScale = bd->gravityScale;
    
    m_force.SetZero();
    m_torque = 0.0f;
    
    m_sleepTime = 0.0f;
    
    m_type = bd->type;
    
    if (m_type == b2_dynamicBody)
    {
        m_mass = 1.0f;
        m_invMass = 1.0f;
    }
    else
    {
        m_mass = 0.0f;
        m_invMass = 0.0f;
    }
    
    m_I = 0.0f;
    m_invI = 0.0f;
    
    m_userData = bd->userData;
    
    m_fixtureList = nullptr;
    m_fixtureCount = 0;
}

inline b2BodyImpl::~b2BodyImpl()
{
    // shapes and joints are destroyed in b2World::Destroy
}

inline void b2BodyImpl::SetType(b2BodyType type)
{
    b2Assert(m_world->IsLocked() == false);
    if (m_world->IsLocked() == true)
    {
        return;
    }
    
    if (m_type == type)
    {
        return;
    }
    
    m_type = type;
    
    ResetMassData();
    
    if (m_type == b2_staticBody)
    {
        m_linearVelocity.SetZero();
        m_angularVelocity = 0.0f;
        m_sweep.a0 = m_sweep.a;
        m_sweep.c0 = m_sweep.c;
        SynchronizeFixtures();
    }
    
    SetAwake(true);
    
    m_force.SetZero();
    m_torque = 0.0f;
    
    // Delete the attached contacts.
    b2ContactEdge* ce = m_contactList;
    while (ce)
    {
        b2ContactEdge* ce0 = ce;
        ce = ce->next;
        m_world->m_contactManager.Destroy(ce0->contact);
    }
    m_contactList = nullptr;
    
    // Touch the proxies so that new contacts will be created (when appropriate)
    b2BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
    for (b2Fixture* f = m_fixtureList; f; f = f->m_next)
    {
        int32 proxyCount = f->m_proxyCount;
        for (int32 i = 0; i < proxyCount; ++i)
        {
            broadPhase->TouchProxy(f->m_proxies[i].proxyId);
        }
    }
}

inline b2Fixture* b2BodyImpl::CreateFixture(const b2FixtureDef* def)
{
    b2Assert(m_world->IsLocked() == false);
    if (m_world->IsLocked() == true)
    {
        return nullptr;
    }
    
    b2BlockAllocator* allocator = &m_world->m_blockAllocator;
    
    void* memory = allocator->Allocate(sizeof(b2Fixture));
    b2Fixture* fixture = new (memory) b2FixtureImpl;
    fixture->Create(allocator, this, def);
    
    if (m_flags & e_activeFlag)
    {
        b2BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
        fixture->CreateProxies(broadPhase, m_xf);
    }
    
    fixture->m_next = m_fixtureList;
    m_fixtureList = fixture;
    ++m_fixtureCount;
    
    fixture->m_body = this;
    
    // Adjust mass properties if needed.
    if (fixture->m_density > 0.0f)
    {
        ResetMassData();
    }
    
    // Let the world know we have a new fixture. This will cause new contacts
    // to be created at the beginning of the next time step.
    m_world->m_flags |= b2World::e_newFixture;
    
    return fixture;
}

inline b2Fixture* b2BodyImpl::CreateFixture(const b2Shape* shape, float32 density)
{
    b2FixtureDef def;
    def.shape = shape;
    def.density = density;
    
    return CreateFixture(&def);
}

inline void b2BodyImpl::DestroyFixture(b2Fixture* fixture)
{
    if (fixture == NULL)
    {
        return;
    }
    
    b2Assert(m_world->IsLocked() == false);
    if (m_world->IsLocked() == true)
    {
        return;
    }
    
    b2Assert(fixture->m_body == this);
    
    // Remove the fixture from this body's singly linked list.
    b2Assert(m_fixtureCount > 0);
    b2Fixture** node = &m_fixtureList;
    bool found = false;
    while (*node != nullptr)
    {
        if (*node == fixture)
        {
            *node = fixture->m_next;
            found = true;
            break;
        }
        
        node = &(*node)->m_next;
    }
    
    // You tried to remove a shape that is not attached to this body.
    b2Assert(found);
    
    // Destroy any contacts associated with the fixture.
    b2ContactEdge* edge = m_contactList;
    while (edge)
    {
        b2Contact* c = edge->contact;
        edge = edge->next;
        
        b2Fixture* fixtureA = c->GetFixtureA();
        b2Fixture* fixtureB = c->GetFixtureB();
        
        if (fixture == fixtureA || fixture == fixtureB)
        {
            // This destroys the contact and removes it from
            // this body's contact list.
            m_world->m_contactManager.Destroy(c);
        }
    }
    
    b2BlockAllocator* allocator = &m_world->m_blockAllocator;
    
    if (m_flags & e_activeFlag)
    {
        b2BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
        fixture->DestroyProxies(broadPhase);
    }
    
    fixture->m_body = nullptr;
    fixture->m_next = nullptr;
    fixture->Destroy(allocator);
    fixture->~b2Fixture();
    allocator->Free(fixture, sizeof(b2Fixture));
    
    --m_fixtureCount;
    
    // Reset the mass data.
    ResetMassData();
}

inline void b2BodyImpl::ResetMassData()
{
    // Compute mass data from shapes. Each shape has its own density.
    m_mass = 0.0f;
    m_invMass = 0.0f;
    m_I = 0.0f;
    m_invI = 0.0f;
    m_sweep.localCenter.SetZero();
    
    // Static and kinematic bodies have zero mass.
    if (m_type == b2_staticBody || m_type == b2_kinematicBody)
    {
        m_sweep.c0 = m_xf.p;
        m_sweep.c = m_xf.p;
        m_sweep.a0 = m_sweep.a;
        return;
    }
    
    b2Assert(m_type == b2_dynamicBody);
    
    // Accumulate mass over all fixtures.
    b2Vec2 localCenter = b2Vec2::ZERO();
    for (b2Fixture* f = m_fixtureList; f; f = f->m_next)
    {
        if (f->m_density == 0.0f)
        {
            continue;
        }
        
        b2MassData massData;
        f->GetMassData(&massData);
        m_mass += massData.mass;
        localCenter += massData.mass * massData.center;
        m_I += massData.I;
    }
    
    // Compute center of mass.
    if (m_mass > 0.0f)
    {
        m_invMass = 1.0f / m_mass;
        localCenter *= m_invMass;
    }
    else
    {
        // Force all dynamic bodies to have a positive mass.
        m_mass = 1.0f;
        m_invMass = 1.0f;
    }
    
    if (m_I > 0.0f && (m_flags & e_fixedRotationFlag) == 0)
    {
        // Center the inertia about the center of mass.
        m_I -= m_mass * b2Dot(localCenter, localCenter);
        b2Assert(m_I > 0.0f);
        m_invI = 1.0f / m_I;
        
    }
    else
    {
        m_I = 0.0f;
        m_invI = 0.0f;
    }
    
    // Move center of mass.
    b2Vec2 oldCenter = m_sweep.c;
    m_sweep.localCenter = localCenter;
    m_sweep.c0 = m_sweep.c = b2Mul(m_xf, m_sweep.localCenter);
    
    // Update center of mass velocity.
    m_linearVelocity += b2Cross(m_angularVelocity, m_sweep.c - oldCenter);
}

inline void b2BodyImpl::SetMassData(const b2MassData* massData)
{
    b2Assert(m_world->IsLocked() == false);
    if (m_world->IsLocked() == true)
    {
        return;
    }
    
    if (m_type != b2_dynamicBody)
    {
        return;
    }
    
    m_invMass = 0.0f;
    m_I = 0.0f;
    m_invI = 0.0f;
    
    m_mass = massData->mass;
    if (m_mass <= 0.0f)
    {
        m_mass = 1.0f;
    }
    
    m_invMass = 1.0f / m_mass;
    
    if (massData->I > 0.0f && (m_flags & b2BodyImpl::e_fixedRotationFlag) == 0)
    {
        m_I = massData->I - m_mass * b2Dot(massData->center, massData->center);
        b2Assert(m_I > 0.0f);
        m_invI = 1.0f / m_I;
    }
    
    // Move center of mass.
    b2Vec2 oldCenter = m_sweep.c;
    m_sweep.localCenter =  massData->center;
    m_sweep.c0 = m_sweep.c = b2Mul(m_xf, m_sweep.localCenter);
    
    // Update center of mass velocity.
    m_linearVelocity += b2Cross(m_angularVelocity, m_sweep.c - oldCenter);
}

inline bool b2BodyImpl::ShouldCollide(const b2Body* other) const
{
    // At least one body should be dynamic.
    if (m_type != b2_dynamicBody && other->GetType() != b2_dynamicBody)
    {
        return false;
    }
    
    // Does a joint prevent collision?
    for (b2JointEdge* jn = m_jointList; jn; jn = jn->next)
    {
        if (jn->other == other)
        {
            if (jn->joint->m_collideConnected == false)
            {
                return false;
            }
        }
    }
    
    return true;
}

inline void b2BodyImpl::SetTransform(const b2Vec2& position, float32 angle)
{
    b2Assert(m_world->IsLocked() == false);
    if (m_world->IsLocked() == true)
    {
        return;
    }
    
    m_xf.q.Set(angle);
    m_xf.p = position;
    
    m_sweep.c = b2Mul(m_xf, m_sweep.localCenter);
    m_sweep.a = angle;
    
    m_sweep.c0 = m_sweep.c;
    m_sweep.a0 = angle;
    
    b2BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
    for (b2Fixture* f = m_fixtureList; f; f = f->m_next)
    {
        f->Synchronize(broadPhase, m_xf, m_xf);
    }
}

inline void b2BodyImpl::SynchronizeFixtures()
{
    b2Transform xf1;
    xf1.q.Set(m_sweep.a0);
    xf1.p = m_sweep.c0 - b2Mul(xf1.q, m_sweep.localCenter);
    
    b2BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
    for (b2Fixture* f = m_fixtureList; f; f = f->m_next)
    {
        f->Synchronize(broadPhase, xf1, m_xf);
    }
}

inline void b2BodyImpl::SetActive(bool flag)
{
    b2Assert(m_world->IsLocked() == false);
    
    if (flag == IsActive())
    {
        return;
    }
    
    if (flag)
    {
        m_flags |= e_activeFlag;
        
        // Create all proxies.
        b2BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
        for (b2Fixture* f = m_fixtureList; f; f = f->m_next)
        {
            f->CreateProxies(broadPhase, m_xf);
        }
        
        // Contacts are created the next time step.
    }
    else
    {
        m_flags &= ~e_activeFlag;
        
        // Destroy all proxies.
        b2BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
        for (b2Fixture* f = m_fixtureList; f; f = f->m_next)
        {
            f->DestroyProxies(broadPhase);
        }
        
        // Destroy the attached contacts.
        b2ContactEdge* ce = m_contactList;
        while (ce)
        {
            b2ContactEdge* ce0 = ce;
            ce = ce->next;
            m_world->m_contactManager.Destroy(ce0->contact);
        }
        m_contactList = nullptr;
    }
}

inline void b2BodyImpl::SetFixedRotation(bool flag)
{
    bool status = (m_flags & e_fixedRotationFlag) == e_fixedRotationFlag;
    if (status == flag)
    {
        return;
    }
    
    if (flag)
    {
        m_flags |= e_fixedRotationFlag;
    }
    else
    {
        m_flags &= ~e_fixedRotationFlag;
    }
    
    m_angularVelocity = 0.0f;
    
    ResetMassData();
}

inline void b2BodyImpl::Dump()
{
    int32 bodyIndex = m_islandIndex;
    
    b2Log("{\n");
    b2Log("  b2BodyDef bd;\n");
    b2Log("  bd.type = b2BodyType(%d);\n", m_type);
    b2Log("  bd.position.Set(%.15lef, %.15lef);\n", m_xf.p.x, m_xf.p.y);
    b2Log("  bd.angle = %.15lef;\n", m_sweep.a);
    b2Log("  bd.linearVelocity.Set(%.15lef, %.15lef);\n", m_linearVelocity.x, m_linearVelocity.y);
    b2Log("  bd.angularVelocity = %.15lef;\n", m_angularVelocity);
    b2Log("  bd.linearDamping = %.15lef;\n", m_linearDamping);
    b2Log("  bd.angularDamping = %.15lef;\n", m_angularDamping);
    b2Log("  bd.allowSleep = bool(%d);\n", m_flags & e_autoSleepFlag);
    b2Log("  bd.awake = bool(%d);\n", m_flags & e_awakeFlag);
    b2Log("  bd.fixedRotation = bool(%d);\n", m_flags & e_fixedRotationFlag);
    b2Log("  bd.bullet = bool(%d);\n", m_flags & e_bulletFlag);
    b2Log("  bd.active = bool(%d);\n", m_flags & e_activeFlag);
    b2Log("  bd.gravityScale = %.15lef;\n", m_gravityScale);
    b2Log("  bodies[%d] = m_world->CreateBody(&bd);\n", m_islandIndex);
    b2Log("\n");
    for (b2Fixture* f = m_fixtureList; f; f = f->m_next)
    {
        b2Log("  {\n");
        f->Dump(bodyIndex);
        b2Log("  }\n");
    }
    b2Log("}\n");
}

#endif
