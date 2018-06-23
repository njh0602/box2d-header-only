#ifndef b2ContactFactory_h
#define b2ContactFactory_h

#include "b2CircleContact.hpp"
#include "b2PolygonAndCircleContact.hpp"
#include "b2PolygonContact.hpp"
#include "b2EdgeAndCircleContact.hpp"
#include "b2EdgeAndPolygonContact.hpp"
#include "b2ChainAndCircleContact.hpp"
#include "b2ChainAndPolygonContact.hpp"

typedef b2Contact* b2ContactCreateFcn(b2Fixture* fixtureA, int32 indexA,
                                      b2Fixture* fixtureB, int32 indexB,
                                      b2BlockAllocator* allocator);

typedef void b2ContactDestroyFcn(b2Contact* contact, b2BlockAllocator* allocator);

struct b2ContactRegister
{
    b2ContactCreateFcn* createFcn;
    b2ContactDestroyFcn* destroyFcn;
    bool primary;
};

template <typename Dummy>
class b2ContactFactoryStaticBase
{
protected:
    static b2ContactRegister s_registers[b2Shape::e_typeCount][b2Shape::e_typeCount];
    static bool s_initialized;
};

template <typename Dummy>
b2ContactRegister b2ContactFactoryStaticBase<Dummy>::s_registers[b2Shape::e_typeCount][b2Shape::e_typeCount];

template <typename Dummy>
bool b2ContactFactoryStaticBase<Dummy>::s_initialized = false;

class b2ContactFactory : public b2ContactFactoryStaticBase<void>
{
    
public:
    
    static b2Contact* Create(b2Fixture* fixtureA, int32 indexA,
                             b2Fixture* fixtureB, int32 indexB,
                             b2BlockAllocator* allocator)
    {
        if (s_initialized == false)
        {
            InitializeRegisters();
            s_initialized = true;
        }
        
        b2Shape::Type type1 = fixtureA->GetType();
        b2Shape::Type type2 = fixtureB->GetType();
        
        b2Assert(0 <= type1 && type1 < b2Shape::e_typeCount);
        b2Assert(0 <= type2 && type2 < b2Shape::e_typeCount);
        
        b2ContactCreateFcn* createFcn = s_registers[type1][type2].createFcn;
        if (createFcn)
        {
            if (s_registers[type1][type2].primary)
            {
                return createFcn(fixtureA, indexA, fixtureB, indexB, allocator);
            }
            else
            {
                return createFcn(fixtureB, indexB, fixtureA, indexA, allocator);
            }
        }
        else
        {
            return nullptr;
        }
    }
    
    static void Destroy(b2Contact* contact, b2BlockAllocator* allocator)
    {
        b2Assert(s_initialized == true);
        
        b2Fixture* fixtureA = contact->m_fixtureA;
        b2Fixture* fixtureB = contact->m_fixtureB;
        
        if (contact->m_manifold.pointCount > 0 &&
            fixtureA->IsSensor() == false &&
            fixtureB->IsSensor() == false)
        {
            fixtureA->GetBody()->SetAwake(true);
            fixtureB->GetBody()->SetAwake(true);
        }
        
        b2Shape::Type typeA = fixtureA->GetType();
        b2Shape::Type typeB = fixtureB->GetType();
        
        b2Assert(0 <= typeA && typeB < b2Shape::e_typeCount);
        b2Assert(0 <= typeA && typeB < b2Shape::e_typeCount);
        
        b2ContactDestroyFcn* destroyFcn = s_registers[typeA][typeB].destroyFcn;
        destroyFcn(contact, allocator);
    }
    
private:
    
    static void AddType(b2ContactCreateFcn* createFcn,
                        b2ContactDestroyFcn* destroyFcn,
                        b2Shape::Type type1,
                        b2Shape::Type type2)
    {
        b2Assert(0 <= type1 && type1 < b2Shape::e_typeCount);
        b2Assert(0 <= type2 && type2 < b2Shape::e_typeCount);
        
        s_registers[type1][type2].createFcn = createFcn;
        s_registers[type1][type2].destroyFcn = destroyFcn;
        s_registers[type1][type2].primary = true;
        
        if (type1 != type2)
        {
            s_registers[type2][type1].createFcn = createFcn;
            s_registers[type2][type1].destroyFcn = destroyFcn;
            s_registers[type2][type1].primary = false;
        }
    }
    
    static void InitializeRegisters()
    {
        AddType(b2CircleContact::Create, b2CircleContact::Destroy, b2Shape::e_circle, b2Shape::e_circle);
        AddType(b2PolygonAndCircleContact::Create, b2PolygonAndCircleContact::Destroy, b2Shape::e_polygon, b2Shape::e_circle);
        AddType(b2PolygonContact::Create, b2PolygonContact::Destroy, b2Shape::e_polygon, b2Shape::e_polygon);
        AddType(b2EdgeAndCircleContact::Create, b2EdgeAndCircleContact::Destroy, b2Shape::e_edge, b2Shape::e_circle);
        AddType(b2EdgeAndPolygonContact::Create, b2EdgeAndPolygonContact::Destroy, b2Shape::e_edge, b2Shape::e_polygon);
        AddType(b2ChainAndCircleContact::Create, b2ChainAndCircleContact::Destroy, b2Shape::e_chain, b2Shape::e_circle);
        AddType(b2ChainAndPolygonContact::Create, b2ChainAndPolygonContact::Destroy, b2Shape::e_chain, b2Shape::e_polygon);
    }
    
};

#endif /* b2ContactFactory_h */
