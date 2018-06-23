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

#ifndef B2_WORLD_H
#define B2_WORLD_H

#include "b2TimeStep.hpp"
#include "b2ContactManager.hpp"
#include "../Common/b2Draw.hpp"
#include "../Common/b2BlockAllocator.hpp"
#include "../Common/b2StackAllocator.hpp"

class b2DestructionListener;
class b2ContactFilter;
class b2ContactListener;
class b2Draw;
class b2Body;
class b2BodyDef;
class b2Joint;
class b2JointDef;
class b2QueryCallback;
class b2AABB;
class b2Contact;
class b2Fixture;


class b2World
{
    
public:
    
    virtual ~b2World() {}
    
	virtual void SetDestructionListener(b2DestructionListener* listener) = 0;
	virtual void SetContactFilter(b2ContactFilter* filter) = 0;
	virtual void SetContactListener(b2ContactListener* listener) = 0;
	virtual void SetDebugDraw(b2Draw* debugDraw) = 0;
	virtual b2Body* CreateBody(const b2BodyDef* def) = 0;
	virtual void DestroyBody(b2Body* body) = 0;
	virtual b2Joint* CreateJoint(const b2JointDef* def) = 0;
	virtual void DestroyJoint(b2Joint* joint) = 0;
	virtual void Step(float32 timeStep, int32 velocityIterations, int32 positionIterations) = 0;
	virtual void ClearForces() = 0;
	virtual void DrawDebugData() = 0;
	virtual void QueryAABB(b2QueryCallback* callback, const b2AABB& aabb) const = 0;
	virtual void RayCast(b2RayCastCallback* callback, const b2Vec2& point1, const b2Vec2& point2) const = 0;
	virtual b2Body* GetBodyList() = 0;
	virtual const b2Body* GetBodyList() const = 0;
	virtual b2Joint* GetJointList() = 0;
	virtual const b2Joint* GetJointList() const = 0;
	virtual b2Contact* GetContactList() = 0;
	virtual const b2Contact* GetContactList() const = 0;
	virtual void SetAllowSleeping(bool flag) = 0;
    virtual bool GetAllowSleeping() const = 0;
    virtual void SetWarmStarting(bool flag) = 0;
    virtual bool GetWarmStarting() const = 0;
    virtual void SetContinuousPhysics(bool flag) = 0;
    virtual bool GetContinuousPhysics() const = 0;
    virtual void SetSubStepping(bool flag) = 0;
    virtual bool GetSubStepping() const = 0;
	virtual int32 GetProxyCount() const = 0;
	virtual int32 GetBodyCount() const = 0;
	virtual int32 GetJointCount() const = 0;
	virtual int32 GetContactCount() const = 0;
	virtual int32 GetTreeHeight() const = 0;
	virtual int32 GetTreeBalance() const = 0;
	virtual float32 GetTreeQuality() const = 0;
	virtual void SetGravity(const b2Vec2& gravity) = 0;
	virtual b2Vec2 GetGravity() const = 0;
	virtual bool IsLocked() const = 0;
	virtual void SetAutoClearForces(bool flag) = 0;
	virtual bool GetAutoClearForces() const = 0;
	virtual void ShiftOrigin(const b2Vec2& newOrigin) = 0;
	virtual const b2ContactManager& GetContactManager() const = 0;
	virtual const b2Profile& GetProfile() const = 0;
	virtual void Dump() = 0;

protected:

	// m_flags
	enum
	{
		e_newFixture	= 0x0001,
		e_locked		= 0x0002,
		e_clearForces	= 0x0004
	};

	friend class b2BodyImpl;
	friend class b2FixtureImpl;
	friend class b2ContactManager;
	friend class b2Controller;

	virtual void Solve(const b2TimeStep& step) = 0;
	virtual void SolveTOI(const b2TimeStep& step) = 0;

	virtual void DrawJoint(b2Joint* joint) = 0;
    virtual void DrawShape(b2Fixture* shape, const b2Transform& xf, const b2Color& color) = 0;

	b2BlockAllocator m_blockAllocator;
	b2StackAllocator m_stackAllocator;

	int32 m_flags;

	b2ContactManager m_contactManager;

	b2Body* m_bodyList;
	b2Joint* m_jointList;

	int32 m_bodyCount;
	int32 m_jointCount;

	b2Vec2 m_gravity;
	bool m_allowSleep;

	b2DestructionListener* m_destructionListener;
	b2Draw* m_debugDraw;

	// This is used to compute the time step ratio to
	// support a variable time step.
	float32 m_inv_dt0;

	// These are for debugging the solver.
	bool m_warmStarting;
	bool m_continuousPhysics;
	bool m_subStepping;

	bool m_stepComplete;

	b2Profile m_profile;
    
};


#endif
