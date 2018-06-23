#ifndef B2_BODY_H
#define B2_BODY_H

#include <memory>

#include "../Common/b2Math.hpp"
#include "../Collision/Shapes/b2Shape.hpp"

/// The body type.
/// static: zero mass, zero velocity, may be manually moved
/// kinematic: zero mass, non-zero velocity set by user, moved by solver
/// dynamic: positive mass, non-zero velocity determined by forces, moved by solver
enum b2BodyType
{
    b2_staticBody = 0,
    b2_kinematicBody,
    b2_dynamicBody
    
    // TODO_ERIN
    //b2_bulletBody,
};

/// A body definition holds all the data needed to construct a rigid body.
/// You can safely re-use body definitions. Shapes are added to a body after construction.
struct b2BodyDef
{
    /// This constructor sets the body definition default values.
    b2BodyDef()
    {
        userData = nullptr;
        position.Set(0.0f, 0.0f);
        angle = 0.0f;
        linearVelocity.Set(0.0f, 0.0f);
        angularVelocity = 0.0f;
        linearDamping = 0.0f;
        angularDamping = 0.0f;
        allowSleep = true;
        awake = true;
        fixedRotation = false;
        bullet = false;
        type = b2_staticBody;
        active = true;
        gravityScale = 1.0f;
    }
    
    /// The body type: static, kinematic, or dynamic.
    /// Note: if a dynamic body would have zero mass, the mass is set to one.
    b2BodyType type;
    
    /// The world position of the body. Avoid creating bodies at the origin
    /// since this can lead to many overlapping shapes.
    b2Vec2 position;
    
    /// The world angle of the body in radians.
    float32 angle;
    
    /// The linear velocity of the body's origin in world co-ordinates.
    b2Vec2 linearVelocity;
    
    /// The angular velocity of the body.
    float32 angularVelocity;
    
    /// Linear damping is use to reduce the linear velocity. The damping parameter
    /// can be larger than 1.0f but the damping effect becomes sensitive to the
    /// time step when the damping parameter is large.
    /// Units are 1/time
    float32 linearDamping;
    
    /// Angular damping is use to reduce the angular velocity. The damping parameter
    /// can be larger than 1.0f but the damping effect becomes sensitive to the
    /// time step when the damping parameter is large.
    /// Units are 1/time
    float32 angularDamping;
    
    /// Set this flag to false if this body should never fall asleep. Note that
    /// this increases CPU usage.
    bool allowSleep;
    
    /// Is this body initially awake or sleeping?
    bool awake;
    
    /// Should this body be prevented from rotating? Useful for characters.
    bool fixedRotation;
    
    /// Is this a fast moving body that should be prevented from tunneling through
    /// other moving bodies? Note that all bodies are prevented from tunneling through
    /// kinematic and static bodies. This setting is only considered on dynamic bodies.
    /// @warning You should use this flag sparingly since it increases processing time.
    bool bullet;
    
    /// Does this body start out active?
    bool active;
    
    /// Use this to store application specific body data.
    void* userData;
    
    /// Scale the gravity applied to this body.
    float32 gravityScale;
};

class b2Fixture;
class b2FixtureDef;
class b2Shape;
class b2Joint;
class b2JointEdge;
class b2ContactEdge;
class b2Contact;
class b2Controller;
class b2World;
class b2MassData;

class b2Body
{
    
public:
    
    virtual b2Fixture* CreateFixture(const b2FixtureDef* def) = 0;
    virtual b2Fixture* CreateFixture(const b2Shape* shape, float32 density) = 0;
    virtual void DestroyFixture(b2Fixture* fixture) = 0;
    virtual void SetTransform(const b2Vec2& position, float32 angle) = 0;
    virtual const b2Transform& GetTransform() const = 0;
    virtual const b2Vec2& GetPosition() const = 0;
    virtual float32 GetAngle() const = 0;
    virtual const b2Vec2& GetWorldCenter() const = 0;
    virtual const b2Vec2& GetLocalCenter() const = 0;
    virtual void SetLinearVelocity(const b2Vec2& v) = 0;
    virtual const b2Vec2& GetLinearVelocity() const = 0;
    virtual void SetAngularVelocity(float32 omega) = 0;
    virtual float32 GetAngularVelocity() const = 0;
    virtual void ApplyForce(const b2Vec2& force, const b2Vec2& point, bool wake)= 0;
    virtual void ApplyForceToCenter(const b2Vec2& force, bool wake) = 0;
    virtual void ApplyTorque(float32 torque, bool wake) = 0;
    virtual void ApplyLinearImpulse(const b2Vec2& impulse, const b2Vec2& point, bool wake) = 0;
    virtual void ApplyLinearImpulseToCenter(const b2Vec2& impulse, bool wake) = 0;
    virtual void ApplyAngularImpulse(float32 impulse, bool wake) = 0;
    virtual float32 GetMass() const = 0;
    virtual float32 GetInertia() const = 0;
    virtual void GetMassData(b2MassData* data) const = 0;
    virtual void SetMassData(const b2MassData* data) = 0;
    virtual void ResetMassData() = 0;
    virtual b2Vec2 GetWorldPoint(const b2Vec2& localPoint) const = 0;
    virtual b2Vec2 GetWorldVector(const b2Vec2& localVector) const = 0;
    virtual b2Vec2 GetLocalPoint(const b2Vec2& worldPoint) const = 0;
    virtual b2Vec2 GetLocalVector(const b2Vec2& worldVector) const = 0;
    virtual b2Vec2 GetLinearVelocityFromWorldPoint(const b2Vec2& worldPoint) const = 0;
    virtual b2Vec2 GetLinearVelocityFromLocalPoint(const b2Vec2& localPoint) const = 0;
    virtual float32 GetLinearDamping() const = 0;
    virtual void SetLinearDamping(float32 linearDamping) = 0;
    virtual float32 GetAngularDamping() const = 0;
    virtual void SetAngularDamping(float32 angularDamping) = 0;
    virtual float32 GetGravityScale() const = 0;
    virtual void SetGravityScale(float32 scale) = 0;
    virtual void SetType(b2BodyType type) = 0;
    virtual b2BodyType GetType() const = 0;
    virtual void SetBullet(bool flag) = 0;
    virtual bool IsBullet() const = 0;
    virtual void SetSleepingAllowed(bool flag) = 0;
    virtual bool IsSleepingAllowed() const = 0;
    virtual void SetAwake(bool flag) = 0;
    virtual bool IsAwake() const = 0;
    virtual void SetActive(bool flag) = 0;
    virtual bool IsActive() const = 0;
    virtual void SetFixedRotation(bool flag) = 0;
    virtual bool IsFixedRotation() const = 0;
    virtual b2Fixture* GetFixtureList() = 0;
    virtual const b2Fixture* GetFixtureList() const = 0;
    virtual b2JointEdge* GetJointList() = 0;
    virtual const b2JointEdge* GetJointList() const = 0;
    virtual b2ContactEdge* GetContactList() = 0;
    virtual const b2ContactEdge* GetContactList() const = 0;
    virtual b2Body* GetNext() = 0;
    virtual const b2Body* GetNext() const = 0;
    virtual void* GetUserData() const = 0;
    virtual void SetUserData(void* data) = 0;
    virtual b2World* GetWorld() = 0;
    virtual const b2World* GetWorld() const = 0;
    virtual void Dump() = 0;
    
protected:
    
    virtual void SynchronizeFixtures() = 0;
    virtual void SynchronizeTransform() = 0;
    virtual bool ShouldCollide(const b2Body* other) const = 0;
    virtual void Advance(float32 t) = 0;
    
    // m_flags
    enum
    {
        e_islandFlag        = 0x0001,
        e_awakeFlag            = 0x0002,
        e_autoSleepFlag        = 0x0004,
        e_bulletFlag        = 0x0008,
        e_fixedRotationFlag    = 0x0010,
        e_activeFlag        = 0x0020,
        e_toiFlag            = 0x0040
    };
    
    virtual ~b2Body() {}
    
    b2BodyType m_type;
    uint16 m_flags;
    int32 m_islandIndex;
    b2Transform m_xf;        // the body origin transform
    b2Sweep m_sweep;        // the swept motion for CCD
    b2Vec2 m_linearVelocity;
    float32 m_angularVelocity;
    b2Vec2 m_force;
    float32 m_torque;
    b2World* m_world;
    b2Body* m_prev;
    b2Body* m_next;
    b2Fixture* m_fixtureList;
    int32 m_fixtureCount;
    b2JointEdge* m_jointList;
    b2ContactEdge* m_contactList;
    float32 m_mass, m_invMass;
    // Rotational inertia about the center of mass.
    float32 m_I, m_invI;
    float32 m_linearDamping;
    float32 m_angularDamping;
    float32 m_gravityScale;
    float32 m_sleepTime;
    void* m_userData;
    
    friend class b2WorldImpl;
    friend class b2Island;
    friend class b2ContactManager;
    friend class b2ContactSolver;
    friend class b2Contact;
    friend class b2DistanceJoint;
    friend class b2FrictionJoint;
    friend class b2GearJoint;
    friend class b2MotorJoint;
    friend class b2MouseJoint;
    friend class b2PrismaticJoint;
    friend class b2PulleyJoint;
    friend class b2RevoluteJoint;
    friend class b2RopeJoint;
    friend class b2WeldJoint;
    friend class b2WheelJoint;
    
};

#endif /* b2Body_h */
