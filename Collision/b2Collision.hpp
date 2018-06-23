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

#ifndef B2_COLLISION_H
#define B2_COLLISION_H

#include "b2CollisionBase.hpp"
#include "b2CollisionHandyFunctions.hpp"

/// This is used for determining the state of contact points.
enum b2PointState
{
    b2_nullState,		///< point does not exist
    b2_addState,		///< point was added in the update
    b2_persistState,	///< point persisted across the update
    b2_removeState		///< point was removed in the update
};

/// Compute the point states given two manifolds. The states pertain to the transition from manifold1
/// to manifold2. So state1 is either persist or remove while state2 is either add or persist.
static void b2GetPointStates(b2PointState state1[b2_maxManifoldPoints],
                             b2PointState state2[b2_maxManifoldPoints],
                             const b2Manifold* manifold1,
                             const b2Manifold* manifold2)
{
    for (int32 i = 0; i < b2_maxManifoldPoints; ++i)
    {
        state1[i] = b2_nullState;
        state2[i] = b2_nullState;
    }
    
    // Detect persists and removes.
    for (int32 i = 0; i < manifold1->pointCount; ++i)
    {
        b2ContactID id = manifold1->points[i].id;
        
        state1[i] = b2_removeState;
        
        for (int32 j = 0; j < manifold2->pointCount; ++j)
        {
            if (manifold2->points[j].id.key == id.key)
            {
                state1[i] = b2_persistState;
                break;
            }
        }
    }
    
    // Detect persists and adds.
    for (int32 i = 0; i < manifold2->pointCount; ++i)
    {
        b2ContactID id = manifold2->points[i].id;
        
        state2[i] = b2_addState;
        
        for (int32 j = 0; j < manifold1->pointCount; ++j)
        {
            if (manifold1->points[j].id.key == id.key)
            {
                state2[i] = b2_persistState;
                break;
            }
        }
    }
}

/// Determine if two generic shapes overlap.
static bool b2TestOverlap(const b2Shape* shapeA, int32 indexA,
                          const b2Shape* shapeB, int32 indexB,
                          const b2Transform& xfA, const b2Transform& xfB)
{
    b2DistanceInput input;
    input.proxyA.Set(shapeA, indexA);
    input.proxyB.Set(shapeB, indexB);
    input.transformA = xfA;
    input.transformB = xfB;
    input.useRadii = true;
    
    b2SimplexCache cache;
    cache.count = 0;
    
    b2DistanceOutput output;
    b2Distance(&output, &cache, &input);
    
    return output.distance < 10.0f * b2_epsilon;
}

// ---------------- Inline Functions ------------------------------------------
static bool b2TestOverlap(const b2AABB& a, const b2AABB& b)
{
    b2Vec2 d1, d2;
    d1 = b.lowerBound - a.upperBound;
    d2 = a.lowerBound - b.upperBound;
    
    if (d1.x > 0.0f || d1.y > 0.0f)
        return false;
    
    if (d2.x > 0.0f || d2.y > 0.0f)
        return false;
    
    return true;
}

#endif
