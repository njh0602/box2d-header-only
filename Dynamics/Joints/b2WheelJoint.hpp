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

#ifndef B2_WHEEL_JOINT_H
#define B2_WHEEL_JOINT_H

#include "b2Joint.hpp"

/// Wheel joint definition. This requires defining a line of
/// motion using an axis and an anchor point. The definition uses local
/// anchor points and a local axis so that the initial configuration
/// can violate the constraint slightly. The joint translation is zero
/// when the local anchor points coincide in world space. Using local
/// anchors and a local axis helps when saving and loading a game.
struct b2WheelJointDef : public b2JointDef
{
	b2WheelJointDef()
	{
		type = e_wheelJoint;
		localAnchorA.SetZero();
		localAnchorB.SetZero();
		localAxisA.Set(1.0f, 0.0f);
		enableMotor = false;
		maxMotorTorque = 0.0f;
		motorSpeed = 0.0f;
		frequencyHz = 2.0f;
		dampingRatio = 0.7f;
	}

	/// Initialize the bodies, anchors, axis, and reference angle using the world
	/// anchor and world axis.
	void Initialize(b2Body* bodyA, b2Body* bodyB, const b2Vec2& anchor, const b2Vec2& axis);

	/// The local anchor point relative to bodyA's origin.
	b2Vec2 localAnchorA;

	/// The local anchor point relative to bodyB's origin.
	b2Vec2 localAnchorB;

	/// The local translation axis in bodyA.
	b2Vec2 localAxisA;

	/// Enable/disable the joint motor.
	bool enableMotor;

	/// The maximum motor torque, usually in N-m.
	float32 maxMotorTorque;

	/// The desired motor speed in radians per second.
	float32 motorSpeed;

	/// Suspension frequency, zero indicates no suspension
	float32 frequencyHz;

	/// Suspension damping ratio, one indicates critical damping
	float32 dampingRatio;
};

/// A wheel joint. This joint provides two degrees of freedom: translation
/// along an axis fixed in bodyA and rotation in the plane. In other words, it is a point to
/// line constraint with a rotational motor and a linear spring/damper.
/// This joint is designed for vehicle suspensions.
class b2WheelJoint : public b2Joint
{
public:
	b2Vec2 GetAnchorA() const override;
	b2Vec2 GetAnchorB() const override;

	b2Vec2 GetReactionForce(float32 inv_dt) const override;
	float32 GetReactionTorque(float32 inv_dt) const override;

	/// The local anchor point relative to bodyA's origin.
	const b2Vec2& GetLocalAnchorA() const { return m_localAnchorA; }

	/// The local anchor point relative to bodyB's origin.
	const b2Vec2& GetLocalAnchorB() const  { return m_localAnchorB; }

	/// The local joint axis relative to bodyA.
	const b2Vec2& GetLocalAxisA() const { return m_localXAxisA; }

	/// Get the current joint translation, usually in meters.
	float32 GetJointTranslation() const;

	/// Get the current joint linear speed, usually in meters per second.
	float32 GetJointLinearSpeed() const;

	/// Get the current joint angle in radians.
	float32 GetJointAngle() const;

	/// Get the current joint angular speed in radians per second.
	float32 GetJointAngularSpeed() const;

	/// Is the joint motor enabled?
	bool IsMotorEnabled() const;

	/// Enable/disable the joint motor.
	void EnableMotor(bool flag);

	/// Set the motor speed, usually in radians per second.
	void SetMotorSpeed(float32 speed);

	/// Get the motor speed, usually in radians per second.
	float32 GetMotorSpeed() const;

	/// Set/Get the maximum motor force, usually in N-m.
	void SetMaxMotorTorque(float32 torque);
	float32 GetMaxMotorTorque() const;

	/// Get the current motor torque given the inverse time step, usually in N-m.
	float32 GetMotorTorque(float32 inv_dt) const;

	/// Set/Get the spring frequency in hertz. Setting the frequency to zero disables the spring.
	void SetSpringFrequencyHz(float32 hz);
	float32 GetSpringFrequencyHz() const;

	/// Set/Get the spring damping ratio
	void SetSpringDampingRatio(float32 ratio);
	float32 GetSpringDampingRatio() const;

	/// Dump to b2Log
	void Dump() override;

protected:

	friend class b2JointFactory;
	b2WheelJoint(const b2WheelJointDef* def);

	void InitVelocityConstraints(const b2SolverData& data) override;
	void SolveVelocityConstraints(const b2SolverData& data) override;
	bool SolvePositionConstraints(const b2SolverData& data) override;

	float32 m_frequencyHz;
	float32 m_dampingRatio;

	// Solver shared
	b2Vec2 m_localAnchorA;
	b2Vec2 m_localAnchorB;
	b2Vec2 m_localXAxisA;
	b2Vec2 m_localYAxisA;

	float32 m_impulse;
	float32 m_motorImpulse;
	float32 m_springImpulse;

	float32 m_maxMotorTorque;
	float32 m_motorSpeed;
	bool m_enableMotor;

	// Solver temp
	int32 m_indexA;
	int32 m_indexB;
	b2Vec2 m_localCenterA;
	b2Vec2 m_localCenterB;
	float32 m_invMassA;
	float32 m_invMassB;
	float32 m_invIA;
	float32 m_invIB;

	b2Vec2 m_ax, m_ay;
	float32 m_sAx, m_sBx;
	float32 m_sAy, m_sBy;

	float32 m_mass;
	float32 m_motorMass;
	float32 m_springMass;

	float32 m_bias;
	float32 m_gamma;
};

// Linear constraint (point-to-line)
// d = pB - pA = xB + rB - xA - rA
// C = dot(ay, d)
// Cdot = dot(d, cross(wA, ay)) + dot(ay, vB + cross(wB, rB) - vA - cross(wA, rA))
//      = -dot(ay, vA) - dot(cross(d + rA, ay), wA) + dot(ay, vB) + dot(cross(rB, ay), vB)
// J = [-ay, -cross(d + rA, ay), ay, cross(rB, ay)]

// Spring linear constraint
// C = dot(ax, d)
// Cdot = = -dot(ax, vA) - dot(cross(d + rA, ax), wA) + dot(ax, vB) + dot(cross(rB, ax), vB)
// J = [-ax -cross(d+rA, ax) ax cross(rB, ax)]

// Motor rotational constraint
// Cdot = wB - wA
// J = [0 0 -1 0 0 1]

inline void b2WheelJointDef::Initialize(b2Body* bA, b2Body* bB, const b2Vec2& anchor, const b2Vec2& axis)
{
    bodyA = bA;
    bodyB = bB;
    localAnchorA = bodyA->GetLocalPoint(anchor);
    localAnchorB = bodyB->GetLocalPoint(anchor);
    localAxisA = bodyA->GetLocalVector(axis);
}

inline b2WheelJoint::b2WheelJoint(const b2WheelJointDef* def)
: b2Joint(def)
{
    m_localAnchorA = def->localAnchorA;
    m_localAnchorB = def->localAnchorB;
    m_localXAxisA = def->localAxisA;
    m_localYAxisA = b2Cross(1.0f, m_localXAxisA);
    
    m_mass = 0.0f;
    m_impulse = 0.0f;
    m_motorMass = 0.0f;
    m_motorImpulse = 0.0f;
    m_springMass = 0.0f;
    m_springImpulse = 0.0f;
    
    m_maxMotorTorque = def->maxMotorTorque;
    m_motorSpeed = def->motorSpeed;
    m_enableMotor = def->enableMotor;
    
    m_frequencyHz = def->frequencyHz;
    m_dampingRatio = def->dampingRatio;
    
    m_bias = 0.0f;
    m_gamma = 0.0f;
    
    m_ax.SetZero();
    m_ay.SetZero();
}

inline void b2WheelJoint::InitVelocityConstraints(const b2SolverData& data)
{
    m_indexA = m_bodyA->m_islandIndex;
    m_indexB = m_bodyB->m_islandIndex;
    m_localCenterA = m_bodyA->m_sweep.localCenter;
    m_localCenterB = m_bodyB->m_sweep.localCenter;
    m_invMassA = m_bodyA->m_invMass;
    m_invMassB = m_bodyB->m_invMass;
    m_invIA = m_bodyA->m_invI;
    m_invIB = m_bodyB->m_invI;
    
    float32 mA = m_invMassA, mB = m_invMassB;
    float32 iA = m_invIA, iB = m_invIB;
    
    b2Vec2 cA = data.positions[m_indexA].c;
    float32 aA = data.positions[m_indexA].a;
    b2Vec2 vA = data.velocities[m_indexA].v;
    float32 wA = data.velocities[m_indexA].w;
    
    b2Vec2 cB = data.positions[m_indexB].c;
    float32 aB = data.positions[m_indexB].a;
    b2Vec2 vB = data.velocities[m_indexB].v;
    float32 wB = data.velocities[m_indexB].w;
    
    b2Rot qA(aA), qB(aB);
    
    // Compute the effective masses.
    b2Vec2 rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
    b2Vec2 rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
    b2Vec2 d = cB + rB - cA - rA;
    
    // Point to line constraint
    {
        m_ay = b2Mul(qA, m_localYAxisA);
        m_sAy = b2Cross(d + rA, m_ay);
        m_sBy = b2Cross(rB, m_ay);
        
        m_mass = mA + mB + iA * m_sAy * m_sAy + iB * m_sBy * m_sBy;
        
        if (m_mass > 0.0f)
        {
            m_mass = 1.0f / m_mass;
        }
    }
    
    // Spring constraint
    m_springMass = 0.0f;
    m_bias = 0.0f;
    m_gamma = 0.0f;
    if (m_frequencyHz > 0.0f)
    {
        m_ax = b2Mul(qA, m_localXAxisA);
        m_sAx = b2Cross(d + rA, m_ax);
        m_sBx = b2Cross(rB, m_ax);
        
        float32 invMass = mA + mB + iA * m_sAx * m_sAx + iB * m_sBx * m_sBx;
        
        if (invMass > 0.0f)
        {
            m_springMass = 1.0f / invMass;
            
            float32 C = b2Dot(d, m_ax);
            
            // Frequency
            float32 omega = 2.0f * b2_pi * m_frequencyHz;
            
            // Damping coefficient
            float32 damp = 2.0f * m_springMass * m_dampingRatio * omega;
            
            // Spring stiffness
            float32 k = m_springMass * omega * omega;
            
            // magic formulas
            float32 h = data.step.dt;
            m_gamma = h * (damp + h * k);
            if (m_gamma > 0.0f)
            {
                m_gamma = 1.0f / m_gamma;
            }
            
            m_bias = C * h * k * m_gamma;
            
            m_springMass = invMass + m_gamma;
            if (m_springMass > 0.0f)
            {
                m_springMass = 1.0f / m_springMass;
            }
        }
    }
    else
    {
        m_springImpulse = 0.0f;
    }
    
    // Rotational motor
    if (m_enableMotor)
    {
        m_motorMass = iA + iB;
        if (m_motorMass > 0.0f)
        {
            m_motorMass = 1.0f / m_motorMass;
        }
    }
    else
    {
        m_motorMass = 0.0f;
        m_motorImpulse = 0.0f;
    }
    
    if (data.step.warmStarting)
    {
        // Account for variable time step.
        m_impulse *= data.step.dtRatio;
        m_springImpulse *= data.step.dtRatio;
        m_motorImpulse *= data.step.dtRatio;
        
        b2Vec2 P = m_impulse * m_ay + m_springImpulse * m_ax;
        float32 LA = m_impulse * m_sAy + m_springImpulse * m_sAx + m_motorImpulse;
        float32 LB = m_impulse * m_sBy + m_springImpulse * m_sBx + m_motorImpulse;
        
        vA -= m_invMassA * P;
        wA -= m_invIA * LA;
        
        vB += m_invMassB * P;
        wB += m_invIB * LB;
    }
    else
    {
        m_impulse = 0.0f;
        m_springImpulse = 0.0f;
        m_motorImpulse = 0.0f;
    }
    
    data.velocities[m_indexA].v = vA;
    data.velocities[m_indexA].w = wA;
    data.velocities[m_indexB].v = vB;
    data.velocities[m_indexB].w = wB;
}

inline void b2WheelJoint::SolveVelocityConstraints(const b2SolverData& data)
{
    float32 mA = m_invMassA, mB = m_invMassB;
    float32 iA = m_invIA, iB = m_invIB;
    
    b2Vec2 vA = data.velocities[m_indexA].v;
    float32 wA = data.velocities[m_indexA].w;
    b2Vec2 vB = data.velocities[m_indexB].v;
    float32 wB = data.velocities[m_indexB].w;
    
    // Solve spring constraint
    {
        float32 Cdot = b2Dot(m_ax, vB - vA) + m_sBx * wB - m_sAx * wA;
        float32 impulse = -m_springMass * (Cdot + m_bias + m_gamma * m_springImpulse);
        m_springImpulse += impulse;
        
        b2Vec2 P = impulse * m_ax;
        float32 LA = impulse * m_sAx;
        float32 LB = impulse * m_sBx;
        
        vA -= mA * P;
        wA -= iA * LA;
        
        vB += mB * P;
        wB += iB * LB;
    }
    
    // Solve rotational motor constraint
    {
        float32 Cdot = wB - wA - m_motorSpeed;
        float32 impulse = -m_motorMass * Cdot;
        
        float32 oldImpulse = m_motorImpulse;
        float32 maxImpulse = data.step.dt * m_maxMotorTorque;
        m_motorImpulse = b2Clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
        impulse = m_motorImpulse - oldImpulse;
        
        wA -= iA * impulse;
        wB += iB * impulse;
    }
    
    // Solve point to line constraint
    {
        float32 Cdot = b2Dot(m_ay, vB - vA) + m_sBy * wB - m_sAy * wA;
        float32 impulse = -m_mass * Cdot;
        m_impulse += impulse;
        
        b2Vec2 P = impulse * m_ay;
        float32 LA = impulse * m_sAy;
        float32 LB = impulse * m_sBy;
        
        vA -= mA * P;
        wA -= iA * LA;
        
        vB += mB * P;
        wB += iB * LB;
    }
    
    data.velocities[m_indexA].v = vA;
    data.velocities[m_indexA].w = wA;
    data.velocities[m_indexB].v = vB;
    data.velocities[m_indexB].w = wB;
}

inline bool b2WheelJoint::SolvePositionConstraints(const b2SolverData& data)
{
    b2Vec2 cA = data.positions[m_indexA].c;
    float32 aA = data.positions[m_indexA].a;
    b2Vec2 cB = data.positions[m_indexB].c;
    float32 aB = data.positions[m_indexB].a;
    
    b2Rot qA(aA), qB(aB);
    
    b2Vec2 rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
    b2Vec2 rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
    b2Vec2 d = (cB - cA) + rB - rA;
    
    b2Vec2 ay = b2Mul(qA, m_localYAxisA);
    
    float32 sAy = b2Cross(d + rA, ay);
    float32 sBy = b2Cross(rB, ay);
    
    float32 C = b2Dot(d, ay);
    
    float32 k = m_invMassA + m_invMassB + m_invIA * m_sAy * m_sAy + m_invIB * m_sBy * m_sBy;
    
    float32 impulse;
    if (k != 0.0f)
    {
        impulse = - C / k;
    }
    else
    {
        impulse = 0.0f;
    }
    
    b2Vec2 P = impulse * ay;
    float32 LA = impulse * sAy;
    float32 LB = impulse * sBy;
    
    cA -= m_invMassA * P;
    aA -= m_invIA * LA;
    cB += m_invMassB * P;
    aB += m_invIB * LB;
    
    data.positions[m_indexA].c = cA;
    data.positions[m_indexA].a = aA;
    data.positions[m_indexB].c = cB;
    data.positions[m_indexB].a = aB;
    
    return b2Abs(C) <= b2_linearSlop;
}

inline b2Vec2 b2WheelJoint::GetAnchorA() const
{
    return m_bodyA->GetWorldPoint(m_localAnchorA);
}

inline b2Vec2 b2WheelJoint::GetAnchorB() const
{
    return m_bodyB->GetWorldPoint(m_localAnchorB);
}

inline b2Vec2 b2WheelJoint::GetReactionForce(float32 inv_dt) const
{
    return inv_dt * (m_impulse * m_ay + m_springImpulse * m_ax);
}

inline float32 b2WheelJoint::GetReactionTorque(float32 inv_dt) const
{
    return inv_dt * m_motorImpulse;
}

inline float32 b2WheelJoint::GetJointTranslation() const
{
    b2Body* bA = m_bodyA;
    b2Body* bB = m_bodyB;
    
    b2Vec2 pA = bA->GetWorldPoint(m_localAnchorA);
    b2Vec2 pB = bB->GetWorldPoint(m_localAnchorB);
    b2Vec2 d = pB - pA;
    b2Vec2 axis = bA->GetWorldVector(m_localXAxisA);
    
    float32 translation = b2Dot(d, axis);
    return translation;
}

inline float32 b2WheelJoint::GetJointLinearSpeed() const
{
    b2Body* bA = m_bodyA;
    b2Body* bB = m_bodyB;
    
    b2Vec2 rA = b2Mul(bA->m_xf.q, m_localAnchorA - bA->m_sweep.localCenter);
    b2Vec2 rB = b2Mul(bB->m_xf.q, m_localAnchorB - bB->m_sweep.localCenter);
    b2Vec2 p1 = bA->m_sweep.c + rA;
    b2Vec2 p2 = bB->m_sweep.c + rB;
    b2Vec2 d = p2 - p1;
    b2Vec2 axis = b2Mul(bA->m_xf.q, m_localXAxisA);
    
    b2Vec2 vA = bA->m_linearVelocity;
    b2Vec2 vB = bB->m_linearVelocity;
    float32 wA = bA->m_angularVelocity;
    float32 wB = bB->m_angularVelocity;
    
    float32 speed = b2Dot(d, b2Cross(wA, axis)) + b2Dot(axis, vB + b2Cross(wB, rB) - vA - b2Cross(wA, rA));
    return speed;
}

inline float32 b2WheelJoint::GetJointAngle() const
{
    b2Body* bA = m_bodyA;
    b2Body* bB = m_bodyB;
    return bB->m_sweep.a - bA->m_sweep.a;
}

inline float32 b2WheelJoint::GetJointAngularSpeed() const
{
    float32 wA = m_bodyA->m_angularVelocity;
    float32 wB = m_bodyB->m_angularVelocity;
    return wB - wA;
}

inline bool b2WheelJoint::IsMotorEnabled() const
{
    return m_enableMotor;
}

inline void b2WheelJoint::EnableMotor(bool flag)
{
    if (flag != m_enableMotor)
    {
        m_bodyA->SetAwake(true);
        m_bodyB->SetAwake(true);
        m_enableMotor = flag;
    }
}

inline void b2WheelJoint::SetMotorSpeed(float32 speed)
{
    if (speed != m_motorSpeed)
    {
        m_bodyA->SetAwake(true);
        m_bodyB->SetAwake(true);
        m_motorSpeed = speed;
    }
}

inline void b2WheelJoint::SetMaxMotorTorque(float32 torque)
{
    if (torque != m_maxMotorTorque)
    {
        m_bodyA->SetAwake(true);
        m_bodyB->SetAwake(true);
        m_maxMotorTorque = torque;
    }
}

inline float32 b2WheelJoint::GetMotorTorque(float32 inv_dt) const
{
    return inv_dt * m_motorImpulse;
}

inline void b2WheelJoint::Dump()
{
    int32 indexA = m_bodyA->m_islandIndex;
    int32 indexB = m_bodyB->m_islandIndex;
    
    b2Log("  b2WheelJointDef jd;\n");
    b2Log("  jd.bodyA = bodies[%d];\n", indexA);
    b2Log("  jd.bodyB = bodies[%d];\n", indexB);
    b2Log("  jd.collideConnected = bool(%d);\n", m_collideConnected);
    b2Log("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", m_localAnchorA.x, m_localAnchorA.y);
    b2Log("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", m_localAnchorB.x, m_localAnchorB.y);
    b2Log("  jd.localAxisA.Set(%.15lef, %.15lef);\n", m_localXAxisA.x, m_localXAxisA.y);
    b2Log("  jd.enableMotor = bool(%d);\n", m_enableMotor);
    b2Log("  jd.motorSpeed = %.15lef;\n", m_motorSpeed);
    b2Log("  jd.maxMotorTorque = %.15lef;\n", m_maxMotorTorque);
    b2Log("  jd.frequencyHz = %.15lef;\n", m_frequencyHz);
    b2Log("  jd.dampingRatio = %.15lef;\n", m_dampingRatio);
    b2Log("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
}

inline float32 b2WheelJoint::GetMotorSpeed() const
{
	return m_motorSpeed;
}

inline float32 b2WheelJoint::GetMaxMotorTorque() const
{
	return m_maxMotorTorque;
}

inline void b2WheelJoint::SetSpringFrequencyHz(float32 hz)
{
	m_frequencyHz = hz;
}

inline float32 b2WheelJoint::GetSpringFrequencyHz() const
{
	return m_frequencyHz;
}

inline void b2WheelJoint::SetSpringDampingRatio(float32 ratio)
{
	m_dampingRatio = ratio;
}

inline float32 b2WheelJoint::GetSpringDampingRatio() const
{
	return m_dampingRatio;
}

#endif
