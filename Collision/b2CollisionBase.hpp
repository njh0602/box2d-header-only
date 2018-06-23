#ifndef b2CollisionStructures_h
#define b2CollisionStructures_h

#include <limits.h>

#include "../Common/b2Math.hpp"

/// The features that intersect to form the contact point
/// This must be 4 bytes or less.
struct b2ContactFeature
{
    enum Type
    {
        e_vertex = 0,
        e_face = 1
    };
    
    uint8 indexA;        ///< Feature index on shapeA
    uint8 indexB;        ///< Feature index on shapeB
    uint8 typeA;        ///< The feature type on shapeA
    uint8 typeB;        ///< The feature type on shapeB
};

/// Contact ids to facilitate warm starting.
union b2ContactID
{
    b2ContactFeature cf;
    uint32 key;                    ///< Used to quickly compare contact ids.
};

/// A manifold point is a contact point belonging to a contact
/// manifold. It holds details related to the geometry and dynamics
/// of the contact points.
/// The local point usage depends on the manifold type:
/// -e_circles: the local center of circleB
/// -e_faceA: the local center of cirlceB or the clip point of polygonB
/// -e_faceB: the clip point of polygonA
/// This structure is stored across time steps, so we keep it small.
/// Note: the impulses are used for internal caching and may not
/// provide reliable contact forces, especially for high speed collisions.
struct b2ManifoldPoint
{
    b2Vec2 localPoint;        ///< usage depends on manifold type
    float32 normalImpulse;    ///< the non-penetration impulse
    float32 tangentImpulse;    ///< the friction impulse
    b2ContactID id;            ///< uniquely identifies a contact point between two shapes
};

/// A manifold for two touching convex shapes.
/// Box2D supports multiple types of contact:
/// - clip point versus plane with radius
/// - point versus point with radius (circles)
/// The local point usage depends on the manifold type:
/// -e_circles: the local center of circleA
/// -e_faceA: the center of faceA
/// -e_faceB: the center of faceB
/// Similarly the local normal usage:
/// -e_circles: not used
/// -e_faceA: the normal on polygonA
/// -e_faceB: the normal on polygonB
/// We store contacts in this way so that position correction can
/// account for movement, which is critical for continuous physics.
/// All contact scenarios must be expressed in one of these types.
/// This structure is stored across time steps, so we keep it small.
struct b2Manifold
{
    enum Type
    {
        e_circles,
        e_faceA,
        e_faceB
    };
    
    b2ManifoldPoint points[b2_maxManifoldPoints];    ///< the points of contact
    b2Vec2 localNormal;                                ///< not use for Type::e_points
    b2Vec2 localPoint;                                ///< usage depends on manifold type
    Type type;
    int32 pointCount;                                ///< the number of manifold points
};

/// This is used to compute the current state of a contact manifold.
struct b2WorldManifold
{
    /// Evaluate the manifold with supplied transforms. This assumes
    /// modest motion from the original state. This does not change the
    /// point count, impulses, etc. The radii must come from the shapes
    /// that generated the manifold.
    void Initialize(const b2Manifold* manifold,
                    const b2Transform& xfA, float32 radiusA,
                    const b2Transform& xfB, float32 radiusB)
    {
        if (manifold->pointCount == 0)
        {
            return;
        }
        
        switch (manifold->type)
        {
            case b2Manifold::e_circles:
            {
                normal.Set(1.0f, 0.0f);
                b2Vec2 pointA = b2Mul(xfA, manifold->localPoint);
                b2Vec2 pointB = b2Mul(xfB, manifold->points[0].localPoint);
                if (b2DistanceSquared(pointA, pointB) > b2_epsilon * b2_epsilon)
                {
                    normal = pointB - pointA;
                    normal.Normalize();
                }
                
                b2Vec2 cA = pointA + radiusA * normal;
                b2Vec2 cB = pointB - radiusB * normal;
                points[0] = 0.5f * (cA + cB);
                separations[0] = b2Dot(cB - cA, normal);
            }
                break;
                
            case b2Manifold::e_faceA:
            {
                normal = b2Mul(xfA.q, manifold->localNormal);
                b2Vec2 planePoint = b2Mul(xfA, manifold->localPoint);
                
                for (int32 i = 0; i < manifold->pointCount; ++i)
                {
                    b2Vec2 clipPoint = b2Mul(xfB, manifold->points[i].localPoint);
                    b2Vec2 cA = clipPoint + (radiusA - b2Dot(clipPoint - planePoint, normal)) * normal;
                    b2Vec2 cB = clipPoint - radiusB * normal;
                    points[i] = 0.5f * (cA + cB);
                    separations[i] = b2Dot(cB - cA, normal);
                }
            }
                break;
                
            case b2Manifold::e_faceB:
            {
                normal = b2Mul(xfB.q, manifold->localNormal);
                b2Vec2 planePoint = b2Mul(xfB, manifold->localPoint);
                
                for (int32 i = 0; i < manifold->pointCount; ++i)
                {
                    b2Vec2 clipPoint = b2Mul(xfA, manifold->points[i].localPoint);
                    b2Vec2 cB = clipPoint + (radiusB - b2Dot(clipPoint - planePoint, normal)) * normal;
                    b2Vec2 cA = clipPoint - radiusA * normal;
                    points[i] = 0.5f * (cA + cB);
                    separations[i] = b2Dot(cA - cB, normal);
                }
                
                // Ensure normal points from A to B.
                normal = -normal;
            }
                break;
        }
    }
    
    b2Vec2 normal;                                ///< world vector pointing from A to B
    b2Vec2 points[b2_maxManifoldPoints];        ///< world contact point (point of intersection)
    float32 separations[b2_maxManifoldPoints];    ///< a negative value indicates overlap, in meters
};

/// Used for computing contact manifolds.
struct b2ClipVertex
{
    b2Vec2 v;
    b2ContactID id;
};

// Sutherland-Hodgman clipping.
static int32 b2ClipSegmentToLine(b2ClipVertex vOut[2], const b2ClipVertex vIn[2],
                                 const b2Vec2& normal, float32 offset, int32 vertexIndexA)
{
    // Start with no output points
    int32 numOut = 0;
    
    // Calculate the distance of end points to the line
    float32 distance0 = b2Dot(normal, vIn[0].v) - offset;
    float32 distance1 = b2Dot(normal, vIn[1].v) - offset;
    
    // If the points are behind the plane
    if (distance0 <= 0.0f) vOut[numOut++] = vIn[0];
    if (distance1 <= 0.0f) vOut[numOut++] = vIn[1];
    
    // If the points are on different sides of the plane
    if (distance0 * distance1 < 0.0f)
    {
        // Find intersection point of edge and plane
        float32 interp = distance0 / (distance0 - distance1);
        vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);
        
        // VertexA is hitting edgeB.
        vOut[numOut].id.cf.indexA = static_cast<uint8>(vertexIndexA);
        vOut[numOut].id.cf.indexB = vIn[0].id.cf.indexB;
        vOut[numOut].id.cf.typeA = b2ContactFeature::e_vertex;
        vOut[numOut].id.cf.typeB = b2ContactFeature::e_face;
        ++numOut;
    }
    
    return numOut;
}

/// Ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
struct b2RayCastInput
{
    b2Vec2 p1, p2;
    float32 maxFraction;
};

/// Ray-cast output data. The ray hits at p1 + fraction * (p2 - p1), where p1 and p2
/// come from b2RayCastInput.
struct b2RayCastOutput
{
    b2Vec2 normal;
    float32 fraction;
};

/// An axis aligned bounding box.
struct b2AABB
{
    /// Verify that the bounds are sorted.
    bool IsValid() const
    {
        b2Vec2 d = upperBound - lowerBound;
        bool valid = d.x >= 0.0f && d.y >= 0.0f;
        valid = valid && lowerBound.IsValid() && upperBound.IsValid();
        return valid;
    }
    
    /// Get the center of the AABB.
    b2Vec2 GetCenter() const
    {
        return 0.5f * (lowerBound + upperBound);
    }
    
    /// Get the extents of the AABB (half-widths).
    b2Vec2 GetExtents() const
    {
        return 0.5f * (upperBound - lowerBound);
    }
    
    /// Get the perimeter length
    float32 GetPerimeter() const
    {
        float32 wx = upperBound.x - lowerBound.x;
        float32 wy = upperBound.y - lowerBound.y;
        return 2.0f * (wx + wy);
    }
    
    /// Combine an AABB into this one.
    void Combine(const b2AABB& aabb)
    {
        lowerBound = b2Min(lowerBound, aabb.lowerBound);
        upperBound = b2Max(upperBound, aabb.upperBound);
    }
    
    /// Combine two AABBs into this one.
    void Combine(const b2AABB& aabb1, const b2AABB& aabb2)
    {
        lowerBound = b2Min(aabb1.lowerBound, aabb2.lowerBound);
        upperBound = b2Max(aabb1.upperBound, aabb2.upperBound);
    }
    
    /// Does this aabb contain the provided AABB.
    bool Contains(const b2AABB& aabb) const
    {
        bool result = true;
        result = result && lowerBound.x <= aabb.lowerBound.x;
        result = result && lowerBound.y <= aabb.lowerBound.y;
        result = result && aabb.upperBound.x <= upperBound.x;
        result = result && aabb.upperBound.y <= upperBound.y;
        return result;
    }
    
    // From Real-time Collision Detection, p179.
    bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input) const
    {
        float32 tmin = -b2_maxFloat;
        float32 tmax = b2_maxFloat;
        
        b2Vec2 p = input.p1;
        b2Vec2 d = input.p2 - input.p1;
        b2Vec2 absD = b2Abs(d);
        
        b2Vec2 normal;
        
        for (int32 i = 0; i < 2; ++i)
        {
            if (absD(i) < b2_epsilon)
            {
                // Parallel.
                if (p(i) < lowerBound(i) || upperBound(i) < p(i))
                {
                    return false;
                }
            }
            else
            {
                float32 inv_d = 1.0f / d(i);
                float32 t1 = (lowerBound(i) - p(i)) * inv_d;
                float32 t2 = (upperBound(i) - p(i)) * inv_d;
                
                // Sign of the normal vector.
                float32 s = -1.0f;
                
                if (t1 > t2)
                {
                    b2Swap(t1, t2);
                    s = 1.0f;
                }
                
                // Push the min up
                if (t1 > tmin)
                {
                    normal.SetZero();
                    normal(i) = s;
                    tmin = t1;
                }
                
                // Pull the max down
                tmax = b2Min(tmax, t2);
                
                if (tmin > tmax)
                {
                    return false;
                }
            }
        }
        
        // Does the ray start inside the box?
        // Does the ray intersect beyond the max fraction?
        if (tmin < 0.0f || input.maxFraction < tmin)
        {
            return false;
        }
        
        // Intersection.
        output->fraction = tmin;
        output->normal = normal;
        return true;
    }
    
    b2Vec2 lowerBound;    ///< the lower vertex
    b2Vec2 upperBound;    ///< the upper vertex
};

#endif /* b2CollisionStructures_h */
