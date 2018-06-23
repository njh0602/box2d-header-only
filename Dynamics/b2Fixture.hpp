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

#ifndef B2_FIXTURE_H
#define B2_FIXTURE_H

#include "b2Body.hpp"
#include "../Collision/b2Collision.hpp"
#include "../Collision/b2BroadPhase.hpp"
#include "../Collision/Shapes/b2Shape.hpp"
#include "../Common/b2BlockAllocator.hpp"

/// This holds contact filtering data.
struct b2Filter
{
	b2Filter()
	{
		categoryBits = 0x0001;
		maskBits = 0xFFFF;
		groupIndex = 0;
	}

	/// The collision category bits. Normally you would just set one bit.
	uint16 categoryBits;

	/// The collision mask bits. This states the categories that this
	/// shape would accept for collision.
	uint16 maskBits;

	/// Collision groups allow a certain group of objects to never collide (negative)
	/// or always collide (positive). Zero means no collision group. Non-zero group
	/// filtering always wins against the mask bits.
	int16 groupIndex;
};

/// A fixture definition is used to create a fixture. This class defines an
/// abstract fixture definition. You can reuse fixture definitions safely.
struct b2FixtureDef
{
	/// The constructor sets the default fixture definition values.
	b2FixtureDef()
	{
		shape = nullptr;
		userData = nullptr;
		friction = 0.2f;
		restitution = 0.0f;
		density = 0.0f;
		isSensor = false;
	}

	/// The shape, this must be set. The shape will be cloned, so you
	/// can create the shape on the stack.
	const b2Shape* shape;

	/// Use this to store application specific fixture data.
	void* userData;

	/// The friction coefficient, usually in the range [0,1].
	float32 friction;

	/// The restitution (elasticity) usually in the range [0,1].
	float32 restitution;

	/// The density, usually in kg/m^2.
	float32 density;

	/// A sensor shape collects contact information but never generates a collision
	/// response.
	bool isSensor;

	/// Contact filtering data.
	b2Filter filter;
};

/// This proxy is used internally to connect fixtures to the broad-phase.
struct b2FixtureProxy
{
	b2AABB aabb;
	b2Fixture* fixture;
	int32 childIndex;
	int32 proxyId;
};

class b2Fixture
{
    
public:
    
	virtual b2Shape::Type GetType() const = 0;
	virtual b2Shape* GetShape() = 0;
	virtual const b2Shape* GetShape() const = 0;
	virtual void SetSensor(bool sensor) = 0;
	virtual bool IsSensor() const = 0;
	virtual void SetFilterData(const b2Filter& filter) = 0;
	virtual const b2Filter& GetFilterData() const = 0;
	virtual void Refilter() = 0;
	virtual b2Body* GetBody() = 0;
	virtual const b2Body* GetBody() const = 0;
	virtual b2Fixture* GetNext() = 0;
	virtual const b2Fixture* GetNext() const = 0;
	virtual void* GetUserData() const = 0;
	virtual void SetUserData(void* data) = 0;
	virtual bool TestPoint(const b2Vec2& p) const = 0;
	virtual bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input, int32 childIndex) const = 0;
	virtual void GetMassData(b2MassData* massData) const = 0;
	virtual void SetDensity(float32 density) = 0;
	virtual float32 GetDensity() const = 0;
	virtual float32 GetFriction() const = 0;
	virtual void SetFriction(float32 friction) = 0;
	virtual float32 GetRestitution() const = 0;
	virtual void SetRestitution(float32 restitution) = 0;
	virtual const b2AABB& GetAABB(int32 childIndex) const = 0;
	virtual void Dump(int32 bodyIndex) = 0;

protected:

	friend class b2BodyImpl;
	friend class b2WorldImpl;
	friend class b2Contact;
	friend class b2ContactManager;

    virtual ~b2Fixture() {}
	virtual void Create(b2BlockAllocator* allocator, b2Body* body, const b2FixtureDef* def) = 0;
	virtual void Destroy(b2BlockAllocator* allocator) = 0;
	virtual void CreateProxies(b2BroadPhase* broadPhase, const b2Transform& xf) = 0;
	virtual void DestroyProxies(b2BroadPhase* broadPhase) = 0;
    virtual void Synchronize(b2BroadPhase* broadPhase, const b2Transform& xf1, const b2Transform& xf2) = 0;

	float32 m_density;
	b2Fixture* m_next;
	b2Body* m_body;
	b2Shape* m_shape;
	float32 m_friction;
	float32 m_restitution;
	b2FixtureProxy* m_proxies;
	int32 m_proxyCount;
	b2Filter m_filter;
	bool m_isSensor;
	void* m_userData;
    
};

#endif
