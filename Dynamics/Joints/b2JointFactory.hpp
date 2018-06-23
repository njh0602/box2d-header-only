#ifndef b2JointFactory_h
#define b2JointFactory_h

#include "b2DistanceJoint.hpp"
#include "b2WheelJoint.hpp"
#include "b2MouseJoint.hpp"
#include "b2RevoluteJoint.hpp"
#include "b2PrismaticJoint.hpp"
#include "b2PulleyJoint.hpp"
#include "b2GearJoint.hpp"
#include "b2WeldJoint.hpp"
#include "b2FrictionJoint.hpp"
#include "b2RopeJoint.hpp"
#include "b2MotorJoint.hpp"

struct b2JointFactory
{
    static b2Joint* Create(const b2JointDef* def, b2BlockAllocator* allocator)
    {
        b2Joint* joint = nullptr;
        
        switch (def->type)
        {
            case e_distanceJoint:
            {
                void* mem = allocator->Allocate(sizeof(b2DistanceJoint));
                joint = new (mem) b2DistanceJoint(static_cast<const b2DistanceJointDef*>(def));
            }
                break;
                
            case e_mouseJoint:
            {
                void* mem = allocator->Allocate(sizeof(b2MouseJoint));
                joint = new (mem) b2MouseJoint(static_cast<const b2MouseJointDef*>(def));
            }
                break;
                
            case e_prismaticJoint:
            {
                void* mem = allocator->Allocate(sizeof(b2PrismaticJoint));
                joint = new (mem) b2PrismaticJoint(static_cast<const b2PrismaticJointDef*>(def));
            }
                break;
                
            case e_revoluteJoint:
            {
                void* mem = allocator->Allocate(sizeof(b2RevoluteJoint));
                joint = new (mem) b2RevoluteJoint(static_cast<const b2RevoluteJointDef*>(def));
            }
                break;
                
            case e_pulleyJoint:
            {
                void* mem = allocator->Allocate(sizeof(b2PulleyJoint));
                joint = new (mem) b2PulleyJoint(static_cast<const b2PulleyJointDef*>(def));
            }
                break;
                
            case e_gearJoint:
            {
                void* mem = allocator->Allocate(sizeof(b2GearJoint));
                joint = new (mem) b2GearJoint(static_cast<const b2GearJointDef*>(def));
            }
                break;
                
            case e_wheelJoint:
            {
                void* mem = allocator->Allocate(sizeof(b2WheelJoint));
                joint = new (mem) b2WheelJoint(static_cast<const b2WheelJointDef*>(def));
            }
                break;
                
            case e_weldJoint:
            {
                void* mem = allocator->Allocate(sizeof(b2WeldJoint));
                joint = new (mem) b2WeldJoint(static_cast<const b2WeldJointDef*>(def));
            }
                break;
                
            case e_frictionJoint:
            {
                void* mem = allocator->Allocate(sizeof(b2FrictionJoint));
                joint = new (mem) b2FrictionJoint(static_cast<const b2FrictionJointDef*>(def));
            }
                break;
                
            case e_ropeJoint:
            {
                void* mem = allocator->Allocate(sizeof(b2RopeJoint));
                joint = new (mem) b2RopeJoint(static_cast<const b2RopeJointDef*>(def));
            }
                break;
                
            case e_motorJoint:
            {
                void* mem = allocator->Allocate(sizeof(b2MotorJoint));
                joint = new (mem) b2MotorJoint(static_cast<const b2MotorJointDef*>(def));
            }
                break;
                
            default:
                b2Assert(false);
                break;
        }
        
        return joint;
    }
    
    static void Destroy(b2Joint* joint, b2BlockAllocator* allocator)
    {
        joint->~b2Joint();
        switch (joint->m_type)
        {
            case e_distanceJoint:
                allocator->Free(joint, sizeof(b2DistanceJoint));
                break;
                
            case e_mouseJoint:
                allocator->Free(joint, sizeof(b2MouseJoint));
                break;
                
            case e_prismaticJoint:
                allocator->Free(joint, sizeof(b2PrismaticJoint));
                break;
                
            case e_revoluteJoint:
                allocator->Free(joint, sizeof(b2RevoluteJoint));
                break;
                
            case e_pulleyJoint:
                allocator->Free(joint, sizeof(b2PulleyJoint));
                break;
                
            case e_gearJoint:
                allocator->Free(joint, sizeof(b2GearJoint));
                break;
                
            case e_wheelJoint:
                allocator->Free(joint, sizeof(b2WheelJoint));
                break;
                
            case e_weldJoint:
                allocator->Free(joint, sizeof(b2WeldJoint));
                break;
                
            case e_frictionJoint:
                allocator->Free(joint, sizeof(b2FrictionJoint));
                break;
                
            case e_ropeJoint:
                allocator->Free(joint, sizeof(b2RopeJoint));
                break;
                
            case e_motorJoint:
                allocator->Free(joint, sizeof(b2MotorJoint));
                break;
                
            default:
                b2Assert(false);
                break;
        }
    }
};

#endif /* b2JointFactory_h */
