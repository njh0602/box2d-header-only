#ifndef b2CollisionHandyFunctions_h
#define b2CollisionHandyFunctions_h

#include "b2Distance.hpp"
#include "Shapes/b2CircleShape.hpp"
#include "Shapes/b2PolygonShape.hpp"
#include "Shapes/b2EdgeShape.hpp"
#include "Shapes/b2ChainShape.hpp"
#include "../Common/b2GlobalValues.hpp"

// b2CollideCircle.cpp
static void b2CollideCircles(b2Manifold* manifold,
                             const b2CircleShape* circleA, const b2Transform& xfA,
                             const b2CircleShape* circleB, const b2Transform& xfB)
{
    manifold->pointCount = 0;
    
    b2Vec2 pA = b2Mul(xfA, circleA->m_p);
    b2Vec2 pB = b2Mul(xfB, circleB->m_p);
    
    b2Vec2 d = pB - pA;
    float32 distSqr = b2Dot(d, d);
    float32 rA = circleA->m_radius, rB = circleB->m_radius;
    float32 radius = rA + rB;
    if (distSqr > radius * radius)
    {
        return;
    }
    
    manifold->type = b2Manifold::e_circles;
    manifold->localPoint = circleA->m_p;
    manifold->localNormal.SetZero();
    manifold->pointCount = 1;
    
    manifold->points[0].localPoint = circleB->m_p;
    manifold->points[0].id.key = 0;
}

static void b2CollidePolygonAndCircle(b2Manifold* manifold,
                                      const b2PolygonShape* polygonA, const b2Transform& xfA,
                                      const b2CircleShape* circleB, const b2Transform& xfB)
{
    manifold->pointCount = 0;
    
    // Compute circle position in the frame of the polygon.
    b2Vec2 c = b2Mul(xfB, circleB->m_p);
    b2Vec2 cLocal = b2MulT(xfA, c);
    
    // Find the min separating edge.
    int32 normalIndex = 0;
    float32 separation = -b2_maxFloat;
    float32 radius = polygonA->m_radius + circleB->m_radius;
    int32 vertexCount = polygonA->m_count;
    const b2Vec2* vertices = polygonA->m_vertices;
    const b2Vec2* normals = polygonA->m_normals;
    
    for (int32 i = 0; i < vertexCount; ++i)
    {
        float32 s = b2Dot(normals[i], cLocal - vertices[i]);
        
        if (s > radius)
        {
            // Early out.
            return;
        }
        
        if (s > separation)
        {
            separation = s;
            normalIndex = i;
        }
    }
    
    // Vertices that subtend the incident face.
    int32 vertIndex1 = normalIndex;
    int32 vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
    b2Vec2 v1 = vertices[vertIndex1];
    b2Vec2 v2 = vertices[vertIndex2];
    
    // If the center is inside the polygon ...
    if (separation < b2_epsilon)
    {
        manifold->pointCount = 1;
        manifold->type = b2Manifold::e_faceA;
        manifold->localNormal = normals[normalIndex];
        manifold->localPoint = 0.5f * (v1 + v2);
        manifold->points[0].localPoint = circleB->m_p;
        manifold->points[0].id.key = 0;
        return;
    }
    
    // Compute barycentric coordinates
    float32 u1 = b2Dot(cLocal - v1, v2 - v1);
    float32 u2 = b2Dot(cLocal - v2, v1 - v2);
    if (u1 <= 0.0f)
    {
        if (b2DistanceSquared(cLocal, v1) > radius * radius)
        {
            return;
        }
        
        manifold->pointCount = 1;
        manifold->type = b2Manifold::e_faceA;
        manifold->localNormal = cLocal - v1;
        manifold->localNormal.Normalize();
        manifold->localPoint = v1;
        manifold->points[0].localPoint = circleB->m_p;
        manifold->points[0].id.key = 0;
    }
    else if (u2 <= 0.0f)
    {
        if (b2DistanceSquared(cLocal, v2) > radius * radius)
        {
            return;
        }
        
        manifold->pointCount = 1;
        manifold->type = b2Manifold::e_faceA;
        manifold->localNormal = cLocal - v2;
        manifold->localNormal.Normalize();
        manifold->localPoint = v2;
        manifold->points[0].localPoint = circleB->m_p;
        manifold->points[0].id.key = 0;
    }
    else
    {
        b2Vec2 faceCenter = 0.5f * (v1 + v2);
        float32 s = b2Dot(cLocal - faceCenter, normals[vertIndex1]);
        if (s > radius)
        {
            return;
        }
        
        manifold->pointCount = 1;
        manifold->type = b2Manifold::e_faceA;
        manifold->localNormal = normals[vertIndex1];
        manifold->localPoint = faceCenter;
        manifold->points[0].localPoint = circleB->m_p;
        manifold->points[0].id.key = 0;
    }
}

// b2CollideEdge.cpp

// This structure is used to keep track of the best separating axis.
struct b2EPAxis
{
    enum Type
    {
        e_unknown,
        e_edgeA,
        e_edgeB
    };
    
    Type type;
    int32 index;
    float32 separation;
};

// This holds polygon B expressed in frame A.
struct b2TempPolygon
{
    b2Vec2 vertices[b2_maxPolygonVertices];
    b2Vec2 normals[b2_maxPolygonVertices];
    int32 count;
};

// Reference face used for clipping
struct b2ReferenceFace
{
    int32 i1, i2;
    
    b2Vec2 v1, v2;
    
    b2Vec2 normal;
    
    b2Vec2 sideNormal1;
    float32 sideOffset1;
    
    b2Vec2 sideNormal2;
    float32 sideOffset2;
};

// This class collides and edge and a polygon, taking into account edge adjacency.
struct b2EPCollider
{
    // Algorithm:
    // 1. Classify v1 and v2
    // 2. Classify polygon centroid as front or back
    // 3. Flip normal if necessary
    // 4. Initialize normal range to [-pi, pi] about face normal
    // 5. Adjust normal range according to adjacent edges
    // 6. Visit each separating axes, only accept axes within the range
    // 7. Return if _any_ axis indicates separation
    // 8. Clip
    void Collide(b2Manifold* manifold,
                 const b2EdgeShape* edgeA,
                 const b2Transform& xfA,
                 const b2PolygonShape* polygonB,
                 const b2Transform& xfB)
    {
        m_xf = b2MulT(xfA, xfB);
        
        m_centroidB = b2Mul(m_xf, polygonB->m_centroid);
        
        m_v0 = edgeA->m_vertex0;
        m_v1 = edgeA->m_vertex1;
        m_v2 = edgeA->m_vertex2;
        m_v3 = edgeA->m_vertex3;
        
        bool hasVertex0 = edgeA->m_hasVertex0;
        bool hasVertex3 = edgeA->m_hasVertex3;
        
        b2Vec2 edge1 = m_v2 - m_v1;
        edge1.Normalize();
        m_normal1.Set(edge1.y, -edge1.x);
        float32 offset1 = b2Dot(m_normal1, m_centroidB - m_v1);
        float32 offset0 = 0.0f, offset2 = 0.0f;
        bool convex1 = false, convex2 = false;
        
        // Is there a preceding edge?
        if (hasVertex0)
        {
            b2Vec2 edge0 = m_v1 - m_v0;
            edge0.Normalize();
            m_normal0.Set(edge0.y, -edge0.x);
            convex1 = b2Cross(edge0, edge1) >= 0.0f;
            offset0 = b2Dot(m_normal0, m_centroidB - m_v0);
        }
        
        // Is there a following edge?
        if (hasVertex3)
        {
            b2Vec2 edge2 = m_v3 - m_v2;
            edge2.Normalize();
            m_normal2.Set(edge2.y, -edge2.x);
            convex2 = b2Cross(edge1, edge2) > 0.0f;
            offset2 = b2Dot(m_normal2, m_centroidB - m_v2);
        }
        
        // Determine front or back collision. Determine collision normal limits.
        if (hasVertex0 && hasVertex3)
        {
            if (convex1 && convex2)
            {
                m_front = offset0 >= 0.0f || offset1 >= 0.0f || offset2 >= 0.0f;
                if (m_front)
                {
                    m_normal = m_normal1;
                    m_lowerLimit = m_normal0;
                    m_upperLimit = m_normal2;
                }
                else
                {
                    m_normal = -m_normal1;
                    m_lowerLimit = -m_normal1;
                    m_upperLimit = -m_normal1;
                }
            }
            else if (convex1)
            {
                m_front = offset0 >= 0.0f || (offset1 >= 0.0f && offset2 >= 0.0f);
                if (m_front)
                {
                    m_normal = m_normal1;
                    m_lowerLimit = m_normal0;
                    m_upperLimit = m_normal1;
                }
                else
                {
                    m_normal = -m_normal1;
                    m_lowerLimit = -m_normal2;
                    m_upperLimit = -m_normal1;
                }
            }
            else if (convex2)
            {
                m_front = offset2 >= 0.0f || (offset0 >= 0.0f && offset1 >= 0.0f);
                if (m_front)
                {
                    m_normal = m_normal1;
                    m_lowerLimit = m_normal1;
                    m_upperLimit = m_normal2;
                }
                else
                {
                    m_normal = -m_normal1;
                    m_lowerLimit = -m_normal1;
                    m_upperLimit = -m_normal0;
                }
            }
            else
            {
                m_front = offset0 >= 0.0f && offset1 >= 0.0f && offset2 >= 0.0f;
                if (m_front)
                {
                    m_normal = m_normal1;
                    m_lowerLimit = m_normal1;
                    m_upperLimit = m_normal1;
                }
                else
                {
                    m_normal = -m_normal1;
                    m_lowerLimit = -m_normal2;
                    m_upperLimit = -m_normal0;
                }
            }
        }
        else if (hasVertex0)
        {
            if (convex1)
            {
                m_front = offset0 >= 0.0f || offset1 >= 0.0f;
                if (m_front)
                {
                    m_normal = m_normal1;
                    m_lowerLimit = m_normal0;
                    m_upperLimit = -m_normal1;
                }
                else
                {
                    m_normal = -m_normal1;
                    m_lowerLimit = m_normal1;
                    m_upperLimit = -m_normal1;
                }
            }
            else
            {
                m_front = offset0 >= 0.0f && offset1 >= 0.0f;
                if (m_front)
                {
                    m_normal = m_normal1;
                    m_lowerLimit = m_normal1;
                    m_upperLimit = -m_normal1;
                }
                else
                {
                    m_normal = -m_normal1;
                    m_lowerLimit = m_normal1;
                    m_upperLimit = -m_normal0;
                }
            }
        }
        else if (hasVertex3)
        {
            if (convex2)
            {
                m_front = offset1 >= 0.0f || offset2 >= 0.0f;
                if (m_front)
                {
                    m_normal = m_normal1;
                    m_lowerLimit = -m_normal1;
                    m_upperLimit = m_normal2;
                }
                else
                {
                    m_normal = -m_normal1;
                    m_lowerLimit = -m_normal1;
                    m_upperLimit = m_normal1;
                }
            }
            else
            {
                m_front = offset1 >= 0.0f && offset2 >= 0.0f;
                if (m_front)
                {
                    m_normal = m_normal1;
                    m_lowerLimit = -m_normal1;
                    m_upperLimit = m_normal1;
                }
                else
                {
                    m_normal = -m_normal1;
                    m_lowerLimit = -m_normal2;
                    m_upperLimit = m_normal1;
                }
            }
        }
        else
        {
            m_front = offset1 >= 0.0f;
            if (m_front)
            {
                m_normal = m_normal1;
                m_lowerLimit = -m_normal1;
                m_upperLimit = -m_normal1;
            }
            else
            {
                m_normal = -m_normal1;
                m_lowerLimit = m_normal1;
                m_upperLimit = m_normal1;
            }
        }
        
        // Get polygonB in frameA
        m_polygonB.count = polygonB->m_count;
        for (int32 i = 0; i < polygonB->m_count; ++i)
        {
            m_polygonB.vertices[i] = b2Mul(m_xf, polygonB->m_vertices[i]);
            m_polygonB.normals[i] = b2Mul(m_xf.q, polygonB->m_normals[i]);
        }
        
        m_radius = polygonB->m_radius + edgeA->m_radius;
        
        manifold->pointCount = 0;
        
        b2EPAxis edgeAxis = ComputeEdgeSeparation();
        
        // If no valid normal can be found than this edge should not collide.
        if (edgeAxis.type == b2EPAxis::e_unknown)
        {
            return;
        }
        
        if (edgeAxis.separation > m_radius)
        {
            return;
        }
        
        b2EPAxis polygonAxis = ComputePolygonSeparation();
        if (polygonAxis.type != b2EPAxis::e_unknown && polygonAxis.separation > m_radius)
        {
            return;
        }
        
        // Use hysteresis for jitter reduction.
        const float32 k_relativeTol = 0.98f;
        const float32 k_absoluteTol = 0.001f;
        
        b2EPAxis primaryAxis;
        if (polygonAxis.type == b2EPAxis::e_unknown)
        {
            primaryAxis = edgeAxis;
        }
        else if (polygonAxis.separation > k_relativeTol * edgeAxis.separation + k_absoluteTol)
        {
            primaryAxis = polygonAxis;
        }
        else
        {
            primaryAxis = edgeAxis;
        }
        
        b2ClipVertex ie[2];
        b2ReferenceFace rf;
        if (primaryAxis.type == b2EPAxis::e_edgeA)
        {
            manifold->type = b2Manifold::e_faceA;
            
            // Search for the polygon normal that is most anti-parallel to the edge normal.
            int32 bestIndex = 0;
            float32 bestValue = b2Dot(m_normal, m_polygonB.normals[0]);
            for (int32 i = 1; i < m_polygonB.count; ++i)
            {
                float32 value = b2Dot(m_normal, m_polygonB.normals[i]);
                if (value < bestValue)
                {
                    bestValue = value;
                    bestIndex = i;
                }
            }
            
            int32 i1 = bestIndex;
            int32 i2 = i1 + 1 < m_polygonB.count ? i1 + 1 : 0;
            
            ie[0].v = m_polygonB.vertices[i1];
            ie[0].id.cf.indexA = 0;
            ie[0].id.cf.indexB = static_cast<uint8>(i1);
            ie[0].id.cf.typeA = b2ContactFeature::e_face;
            ie[0].id.cf.typeB = b2ContactFeature::e_vertex;
            
            ie[1].v = m_polygonB.vertices[i2];
            ie[1].id.cf.indexA = 0;
            ie[1].id.cf.indexB = static_cast<uint8>(i2);
            ie[1].id.cf.typeA = b2ContactFeature::e_face;
            ie[1].id.cf.typeB = b2ContactFeature::e_vertex;
            
            if (m_front)
            {
                rf.i1 = 0;
                rf.i2 = 1;
                rf.v1 = m_v1;
                rf.v2 = m_v2;
                rf.normal = m_normal1;
            }
            else
            {
                rf.i1 = 1;
                rf.i2 = 0;
                rf.v1 = m_v2;
                rf.v2 = m_v1;
                rf.normal = -m_normal1;
            }
        }
        else
        {
            manifold->type = b2Manifold::e_faceB;
            
            ie[0].v = m_v1;
            ie[0].id.cf.indexA = 0;
            ie[0].id.cf.indexB = static_cast<uint8>(primaryAxis.index);
            ie[0].id.cf.typeA = b2ContactFeature::e_vertex;
            ie[0].id.cf.typeB = b2ContactFeature::e_face;
            
            ie[1].v = m_v2;
            ie[1].id.cf.indexA = 0;
            ie[1].id.cf.indexB = static_cast<uint8>(primaryAxis.index);
            ie[1].id.cf.typeA = b2ContactFeature::e_vertex;
            ie[1].id.cf.typeB = b2ContactFeature::e_face;
            
            rf.i1 = primaryAxis.index;
            rf.i2 = rf.i1 + 1 < m_polygonB.count ? rf.i1 + 1 : 0;
            rf.v1 = m_polygonB.vertices[rf.i1];
            rf.v2 = m_polygonB.vertices[rf.i2];
            rf.normal = m_polygonB.normals[rf.i1];
        }
        
        rf.sideNormal1.Set(rf.normal.y, -rf.normal.x);
        rf.sideNormal2 = -rf.sideNormal1;
        rf.sideOffset1 = b2Dot(rf.sideNormal1, rf.v1);
        rf.sideOffset2 = b2Dot(rf.sideNormal2, rf.v2);
        
        // Clip incident edge against extruded edge1 side edges.
        b2ClipVertex clipPoints1[2];
        b2ClipVertex clipPoints2[2];
        int32 np;
        
        // Clip to box side 1
        np = b2ClipSegmentToLine(clipPoints1, ie, rf.sideNormal1, rf.sideOffset1, rf.i1);
        
        if (np < b2_maxManifoldPoints)
        {
            return;
        }
        
        // Clip to negative box side 1
        np = b2ClipSegmentToLine(clipPoints2, clipPoints1, rf.sideNormal2, rf.sideOffset2, rf.i2);
        
        if (np < b2_maxManifoldPoints)
        {
            return;
        }
        
        // Now clipPoints2 contains the clipped points.
        if (primaryAxis.type == b2EPAxis::e_edgeA)
        {
            manifold->localNormal = rf.normal;
            manifold->localPoint = rf.v1;
        }
        else
        {
            manifold->localNormal = polygonB->m_normals[rf.i1];
            manifold->localPoint = polygonB->m_vertices[rf.i1];
        }
        
        int32 pointCount = 0;
        for (int32 i = 0; i < b2_maxManifoldPoints; ++i)
        {
            float32 separation;
            
            separation = b2Dot(rf.normal, clipPoints2[i].v - rf.v1);
            
            if (separation <= m_radius)
            {
                b2ManifoldPoint* cp = manifold->points + pointCount;
                
                if (primaryAxis.type == b2EPAxis::e_edgeA)
                {
                    cp->localPoint = b2MulT(m_xf, clipPoints2[i].v);
                    cp->id = clipPoints2[i].id;
                }
                else
                {
                    cp->localPoint = clipPoints2[i].v;
                    cp->id.cf.typeA = clipPoints2[i].id.cf.typeB;
                    cp->id.cf.typeB = clipPoints2[i].id.cf.typeA;
                    cp->id.cf.indexA = clipPoints2[i].id.cf.indexB;
                    cp->id.cf.indexB = clipPoints2[i].id.cf.indexA;
                }
                
                ++pointCount;
            }
        }
        
        manifold->pointCount = pointCount;
    }
    
    b2EPAxis ComputeEdgeSeparation()
    {
        b2EPAxis axis;
        axis.type = b2EPAxis::e_edgeA;
        axis.index = m_front ? 0 : 1;
        axis.separation = FLT_MAX;
        
        for (int32 i = 0; i < m_polygonB.count; ++i)
        {
            float32 s = b2Dot(m_normal, m_polygonB.vertices[i] - m_v1);
            if (s < axis.separation)
            {
                axis.separation = s;
            }
        }
        
        return axis;
    }
    
    b2EPAxis ComputePolygonSeparation()
    {
        b2EPAxis axis;
        axis.type = b2EPAxis::e_unknown;
        axis.index = -1;
        axis.separation = -FLT_MAX;
        
        b2Vec2 perp(-m_normal.y, m_normal.x);
        
        for (int32 i = 0; i < m_polygonB.count; ++i)
        {
            b2Vec2 n = -m_polygonB.normals[i];
            
            float32 s1 = b2Dot(n, m_polygonB.vertices[i] - m_v1);
            float32 s2 = b2Dot(n, m_polygonB.vertices[i] - m_v2);
            float32 s = b2Min(s1, s2);
            
            if (s > m_radius)
            {
                // No collision
                axis.type = b2EPAxis::e_edgeB;
                axis.index = i;
                axis.separation = s;
                return axis;
            }
            
            // Adjacency
            if (b2Dot(n, perp) >= 0.0f)
            {
                if (b2Dot(n - m_upperLimit, m_normal) < -b2_angularSlop)
                {
                    continue;
                }
            }
            else
            {
                if (b2Dot(n - m_lowerLimit, m_normal) < -b2_angularSlop)
                {
                    continue;
                }
            }
            
            if (s > axis.separation)
            {
                axis.type = b2EPAxis::e_edgeB;
                axis.index = i;
                axis.separation = s;
            }
        }
        
        return axis;
    }
    
    enum VertexType
    {
        e_isolated,
        e_concave,
        e_convex
    };
    
    b2TempPolygon m_polygonB;
    
    b2Transform m_xf;
    b2Vec2 m_centroidB;
    b2Vec2 m_v0, m_v1, m_v2, m_v3;
    b2Vec2 m_normal0, m_normal1, m_normal2;
    b2Vec2 m_normal;
    VertexType m_type1, m_type2;
    b2Vec2 m_lowerLimit, m_upperLimit;
    float32 m_radius;
    bool m_front;
};

// Compute contact points for edge versus circle.
// This accounts for edge connectivity.
static void b2CollideEdgeAndCircle(b2Manifold* manifold,
                                   const b2EdgeShape* edgeA,
                                   const b2Transform& xfA,
                                   const b2CircleShape* circleB,
                                   const b2Transform& xfB)
{
    manifold->pointCount = 0;
    
    // Compute circle in frame of edge
    b2Vec2 Q = b2MulT(xfA, b2Mul(xfB, circleB->m_p));
    
    b2Vec2 A = edgeA->m_vertex1, B = edgeA->m_vertex2;
    b2Vec2 e = B - A;
    
    // Barycentric coordinates
    float32 u = b2Dot(e, B - Q);
    float32 v = b2Dot(e, Q - A);
    
    float32 radius = edgeA->m_radius + circleB->m_radius;
    
    b2ContactFeature cf;
    cf.indexB = 0;
    cf.typeB = b2ContactFeature::e_vertex;
    
    // Region A
    if (v <= 0.0f)
    {
        b2Vec2 P = A;
        b2Vec2 d = Q - P;
        float32 dd = b2Dot(d, d);
        if (dd > radius * radius)
        {
            return;
        }
        
        // Is there an edge connected to A?
        if (edgeA->m_hasVertex0)
        {
            b2Vec2 A1 = edgeA->m_vertex0;
            b2Vec2 B1 = A;
            b2Vec2 e1 = B1 - A1;
            float32 u1 = b2Dot(e1, B1 - Q);
            
            // Is the circle in Region AB of the previous edge?
            if (u1 > 0.0f)
            {
                return;
            }
        }
        
        cf.indexA = 0;
        cf.typeA = b2ContactFeature::e_vertex;
        manifold->pointCount = 1;
        manifold->type = b2Manifold::e_circles;
        manifold->localNormal.SetZero();
        manifold->localPoint = P;
        manifold->points[0].id.key = 0;
        manifold->points[0].id.cf = cf;
        manifold->points[0].localPoint = circleB->m_p;
        return;
    }
    
    // Region B
    if (u <= 0.0f)
    {
        b2Vec2 P = B;
        b2Vec2 d = Q - P;
        float32 dd = b2Dot(d, d);
        if (dd > radius * radius)
        {
            return;
        }
        
        // Is there an edge connected to B?
        if (edgeA->m_hasVertex3)
        {
            b2Vec2 B2 = edgeA->m_vertex3;
            b2Vec2 A2 = B;
            b2Vec2 e2 = B2 - A2;
            float32 v2 = b2Dot(e2, Q - A2);
            
            // Is the circle in Region AB of the next edge?
            if (v2 > 0.0f)
            {
                return;
            }
        }
        
        cf.indexA = 1;
        cf.typeA = b2ContactFeature::e_vertex;
        manifold->pointCount = 1;
        manifold->type = b2Manifold::e_circles;
        manifold->localNormal.SetZero();
        manifold->localPoint = P;
        manifold->points[0].id.key = 0;
        manifold->points[0].id.cf = cf;
        manifold->points[0].localPoint = circleB->m_p;
        return;
    }
    
    // Region AB
    float32 den = b2Dot(e, e);
    b2Assert(den > 0.0f);
    b2Vec2 P = (1.0f / den) * (u * A + v * B);
    b2Vec2 d = Q - P;
    float32 dd = b2Dot(d, d);
    if (dd > radius * radius)
    {
        return;
    }
    
    b2Vec2 n(-e.y, e.x);
    if (b2Dot(n, Q - A) < 0.0f)
    {
        n.Set(-n.x, -n.y);
    }
    n.Normalize();
    
    cf.indexA = 0;
    cf.typeA = b2ContactFeature::e_face;
    manifold->pointCount = 1;
    manifold->type = b2Manifold::e_faceA;
    manifold->localNormal = n;
    manifold->localPoint = A;
    manifold->points[0].id.key = 0;
    manifold->points[0].id.cf = cf;
    manifold->points[0].localPoint = circleB->m_p;
}

static void b2CollideEdgeAndPolygon(b2Manifold* manifold,
                                    const b2EdgeShape* edgeA, const b2Transform& xfA,
                                    const b2PolygonShape* polygonB, const b2Transform& xfB)
{
    b2EPCollider collider;
    collider.Collide(manifold, edgeA, xfA, polygonB, xfB);
}

// b2CollidePolygon.cpp
// Find the max separation between poly1 and poly2 using edge normals from poly1.
static float32 b2FindMaxSeparation(int32* edgeIndex,
                                   const b2PolygonShape* poly1, const b2Transform& xf1,
                                   const b2PolygonShape* poly2, const b2Transform& xf2)
{
    int32 count1 = poly1->m_count;
    int32 count2 = poly2->m_count;
    const b2Vec2* n1s = poly1->m_normals;
    const b2Vec2* v1s = poly1->m_vertices;
    const b2Vec2* v2s = poly2->m_vertices;
    b2Transform xf = b2MulT(xf2, xf1);
    
    int32 bestIndex = 0;
    float32 maxSeparation = -b2_maxFloat;
    for (int32 i = 0; i < count1; ++i)
    {
        // Get poly1 normal in frame2.
        b2Vec2 n = b2Mul(xf.q, n1s[i]);
        b2Vec2 v1 = b2Mul(xf, v1s[i]);
        
        // Find deepest point for normal i.
        float32 si = b2_maxFloat;
        for (int32 j = 0; j < count2; ++j)
        {
            float32 sij = b2Dot(n, v2s[j] - v1);
            if (sij < si)
            {
                si = sij;
            }
        }
        
        if (si > maxSeparation)
        {
            maxSeparation = si;
            bestIndex = i;
        }
    }
    
    *edgeIndex = bestIndex;
    return maxSeparation;
}

static void b2FindIncidentEdge(b2ClipVertex c[2],
                               const b2PolygonShape* poly1, const b2Transform& xf1, int32 edge1,
                               const b2PolygonShape* poly2, const b2Transform& xf2)
{
    const b2Vec2* normals1 = poly1->m_normals;
    
    int32 count2 = poly2->m_count;
    const b2Vec2* vertices2 = poly2->m_vertices;
    const b2Vec2* normals2 = poly2->m_normals;
    
    b2Assert(0 <= edge1 && edge1 < poly1->m_count);
    
    // Get the normal of the reference edge in poly2's frame.
    b2Vec2 normal1 = b2MulT(xf2.q, b2Mul(xf1.q, normals1[edge1]));
    
    // Find the incident edge on poly2.
    int32 index = 0;
    float32 minDot = b2_maxFloat;
    for (int32 i = 0; i < count2; ++i)
    {
        float32 dot = b2Dot(normal1, normals2[i]);
        if (dot < minDot)
        {
            minDot = dot;
            index = i;
        }
    }
    
    // Build the clip vertices for the incident edge.
    int32 i1 = index;
    int32 i2 = i1 + 1 < count2 ? i1 + 1 : 0;
    
    c[0].v = b2Mul(xf2, vertices2[i1]);
    c[0].id.cf.indexA = (uint8)edge1;
    c[0].id.cf.indexB = (uint8)i1;
    c[0].id.cf.typeA = b2ContactFeature::e_face;
    c[0].id.cf.typeB = b2ContactFeature::e_vertex;
    
    c[1].v = b2Mul(xf2, vertices2[i2]);
    c[1].id.cf.indexA = (uint8)edge1;
    c[1].id.cf.indexB = (uint8)i2;
    c[1].id.cf.typeA = b2ContactFeature::e_face;
    c[1].id.cf.typeB = b2ContactFeature::e_vertex;
}

// Find edge normal of max separation on A - return if separating axis is found
// Find edge normal of max separation on B - return if separation axis is found
// Choose reference edge as min(minA, minB)
// Find incident edge
// Clip

// The normal points from 1 to 2
static void b2CollidePolygons(b2Manifold* manifold,
                              const b2PolygonShape* polyA, const b2Transform& xfA,
                              const b2PolygonShape* polyB, const b2Transform& xfB)
{
    manifold->pointCount = 0;
    float32 totalRadius = polyA->m_radius + polyB->m_radius;
    
    int32 edgeA = 0;
    float32 separationA = b2FindMaxSeparation(&edgeA, polyA, xfA, polyB, xfB);
    if (separationA > totalRadius)
        return;
    
    int32 edgeB = 0;
    float32 separationB = b2FindMaxSeparation(&edgeB, polyB, xfB, polyA, xfA);
    if (separationB > totalRadius)
        return;
    
    const b2PolygonShape* poly1;    // reference polygon
    const b2PolygonShape* poly2;    // incident polygon
    b2Transform xf1, xf2;
    int32 edge1;                    // reference edge
    uint8 flip;
    const float32 k_tol = 0.1f * b2_linearSlop;
    
    if (separationB > separationA + k_tol)
    {
        poly1 = polyB;
        poly2 = polyA;
        xf1 = xfB;
        xf2 = xfA;
        edge1 = edgeB;
        manifold->type = b2Manifold::e_faceB;
        flip = 1;
    }
    else
    {
        poly1 = polyA;
        poly2 = polyB;
        xf1 = xfA;
        xf2 = xfB;
        edge1 = edgeA;
        manifold->type = b2Manifold::e_faceA;
        flip = 0;
    }
    
    b2ClipVertex incidentEdge[2];
    b2FindIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);
    
    int32 count1 = poly1->m_count;
    const b2Vec2* vertices1 = poly1->m_vertices;
    
    int32 iv1 = edge1;
    int32 iv2 = edge1 + 1 < count1 ? edge1 + 1 : 0;
    
    b2Vec2 v11 = vertices1[iv1];
    b2Vec2 v12 = vertices1[iv2];
    
    b2Vec2 localTangent = v12 - v11;
    localTangent.Normalize();
    
    b2Vec2 localNormal = b2Cross(localTangent, 1.0f);
    b2Vec2 planePoint = 0.5f * (v11 + v12);
    
    b2Vec2 tangent = b2Mul(xf1.q, localTangent);
    b2Vec2 normal = b2Cross(tangent, 1.0f);
    
    v11 = b2Mul(xf1, v11);
    v12 = b2Mul(xf1, v12);
    
    // Face offset.
    float32 frontOffset = b2Dot(normal, v11);
    
    // Side offsets, extended by polytope skin thickness.
    float32 sideOffset1 = -b2Dot(tangent, v11) + totalRadius;
    float32 sideOffset2 = b2Dot(tangent, v12) + totalRadius;
    
    // Clip incident edge against extruded edge1 side edges.
    b2ClipVertex clipPoints1[2];
    b2ClipVertex clipPoints2[2];
    int np;
    
    // Clip to box side 1
    np = b2ClipSegmentToLine(clipPoints1, incidentEdge, -tangent, sideOffset1, iv1);
    
    if (np < 2)
        return;
    
    // Clip to negative box side 1
    np = b2ClipSegmentToLine(clipPoints2, clipPoints1,  tangent, sideOffset2, iv2);
    
    if (np < 2)
    {
        return;
    }
    
    // Now clipPoints2 contains the clipped points.
    manifold->localNormal = localNormal;
    manifold->localPoint = planePoint;
    
    int32 pointCount = 0;
    for (int32 i = 0; i < b2_maxManifoldPoints; ++i)
    {
        float32 separation = b2Dot(normal, clipPoints2[i].v) - frontOffset;
        
        if (separation <= totalRadius)
        {
            b2ManifoldPoint* cp = manifold->points + pointCount;
            cp->localPoint = b2MulT(xf2, clipPoints2[i].v);
            cp->id = clipPoints2[i].id;
            if (flip)
            {
                // Swap features
                b2ContactFeature cf = cp->id.cf;
                cp->id.cf.indexA = cf.indexB;
                cp->id.cf.indexB = cf.indexA;
                cp->id.cf.typeA = cf.typeB;
                cp->id.cf.typeB = cf.typeA;
            }
            ++pointCount;
        }
    }
    
    manifold->pointCount = pointCount;
}

// b2Distance.cpp
static void b2Distance(b2DistanceOutput* output,
                       b2SimplexCache* cache,
                       const b2DistanceInput* input)
{
    ++b2GlobalValues::gjkCalls;
    
    const b2DistanceProxy* proxyA = &input->proxyA;
    const b2DistanceProxy* proxyB = &input->proxyB;
    
    b2Transform transformA = input->transformA;
    b2Transform transformB = input->transformB;
    
    // Initialize the simplex.
    b2Simplex simplex;
    simplex.ReadCache(cache, proxyA, transformA, proxyB, transformB);
    
    // Get simplex vertices as an array.
    b2SimplexVertex* vertices = &simplex.m_v1;
    const int32 k_maxIters = 20;
    
    // These store the vertices of the last simplex so that we
    // can check for duplicates and prevent cycling.
    int32 saveA[3], saveB[3];
    int32 saveCount = 0;
    
    // Main iteration loop.
    int32 iter = 0;
    while (iter < k_maxIters)
    {
        // Copy simplex so we can identify duplicates.
        saveCount = simplex.m_count;
        for (int32 i = 0; i < saveCount; ++i)
        {
            saveA[i] = vertices[i].indexA;
            saveB[i] = vertices[i].indexB;
        }
        
        switch (simplex.m_count)
        {
            case 1:
                break;
                
            case 2:
                simplex.Solve2();
                break;
                
            case 3:
                simplex.Solve3();
                break;
                
            default:
                b2Assert(false);
        }
        
        // If we have 3 points, then the origin is in the corresponding triangle.
        if (simplex.m_count == 3)
        {
            break;
        }
        
        // Get search direction.
        b2Vec2 d = simplex.GetSearchDirection();
        
        // Ensure the search direction is numerically fit.
        if (d.LengthSquared() < b2_epsilon * b2_epsilon)
        {
            // The origin is probably contained by a line segment
            // or triangle. Thus the shapes are overlapped.
            
            // We can't return zero here even though there may be overlap.
            // In case the simplex is a point, segment, or triangle it is difficult
            // to determine if the origin is contained in the CSO or very close to it.
            break;
        }
        
        // Compute a tentative new simplex vertex using support points.
        b2SimplexVertex* vertex = vertices + simplex.m_count;
        vertex->indexA = proxyA->GetSupport(b2MulT(transformA.q, -d));
        vertex->wA = b2Mul(transformA, proxyA->GetVertex(vertex->indexA));
        b2Vec2 wBLocal;
        vertex->indexB = proxyB->GetSupport(b2MulT(transformB.q, d));
        vertex->wB = b2Mul(transformB, proxyB->GetVertex(vertex->indexB));
        vertex->w = vertex->wB - vertex->wA;
        
        // Iteration count is equated to the number of support point calls.
        ++iter;
        ++b2GlobalValues::gjkIters;
        
        // Check for duplicate support points. This is the main termination criteria.
        bool duplicate = false;
        for (int32 i = 0; i < saveCount; ++i)
        {
            if (vertex->indexA == saveA[i] && vertex->indexB == saveB[i])
            {
                duplicate = true;
                break;
            }
        }
        
        // If we found a duplicate support point we must exit to avoid cycling.
        if (duplicate)
        {
            break;
        }
        
        // New vertex is ok and needed.
        ++simplex.m_count;
    }
    
    b2GlobalValues::gjkMaxIters = b2Max(b2GlobalValues::gjkMaxIters, iter);
    
    // Prepare output.
    simplex.GetWitnessPoints(&output->pointA, &output->pointB);
    output->distance = b2Distance(output->pointA, output->pointB);
    output->iterations = iter;
    
    // Cache the simplex.
    simplex.WriteCache(cache);
    
    // Apply radii if requested.
    if (input->useRadii)
    {
        float32 rA = proxyA->m_radius;
        float32 rB = proxyB->m_radius;
        
        if (output->distance > rA + rB && output->distance > b2_epsilon)
        {
            // Shapes are still no overlapped.
            // Move the witness points to the outer surface.
            output->distance -= rA + rB;
            b2Vec2 normal = output->pointB - output->pointA;
            normal.Normalize();
            output->pointA += rA * normal;
            output->pointB -= rB * normal;
        }
        else
        {
            // Shapes are overlapped when radii are considered.
            // Move the witness points to the middle.
            b2Vec2 p = 0.5f * (output->pointA + output->pointB);
            output->pointA = p;
            output->pointB = p;
            output->distance = 0.0f;
        }
    }
}

// GJK-raycast
// Algorithm by Gino van den Bergen.
// "Smooth Mesh Contacts with GJK" in Game Physics Pearls. 2010
static bool b2ShapeCast(b2ShapeCastOutput * output, const b2ShapeCastInput * input)
{
    output->iterations = 0;
    output->lambda = 1.0f;
    output->normal.SetZero();
    output->point.SetZero();
    
    const b2DistanceProxy* proxyA = &input->proxyA;
    const b2DistanceProxy* proxyB = &input->proxyB;
    
    float32 radiusA = b2Max(proxyA->m_radius, b2_polygonRadius);
    float32 radiusB = b2Max(proxyB->m_radius, b2_polygonRadius);
    float32 radius = radiusA + radiusB;
    
    b2Transform xfA = input->transformA;
    b2Transform xfB = input->transformB;
    
    b2Vec2 r = input->translationB;
    b2Vec2 n(0.0f, 0.0f);
    float32 lambda = 0.0f;
    
    // Initial simplex
    b2Simplex simplex;
    simplex.m_count = 0;
    
    // Get simplex vertices as an array.
    b2SimplexVertex* vertices = &simplex.m_v1;
    
    // Get support point in -r direction
    int32 indexA = proxyA->GetSupport(b2MulT(xfA.q, -r));
    b2Vec2 wA = b2Mul(xfA, proxyA->GetVertex(indexA));
    int32 indexB = proxyB->GetSupport(b2MulT(xfB.q, r));
    b2Vec2 wB = b2Mul(xfB, proxyB->GetVertex(indexB));
    b2Vec2 v = wA - wB;
    
    // Sigma is the target distance between polygons
    float32 sigma = b2Max(b2_polygonRadius, radius - b2_polygonRadius);
    const float32 tolerance = 0.5f * b2_linearSlop;
    
    // Main iteration loop.
    const int32 k_maxIters = 20;
    int32 iter = 0;
    while (iter < k_maxIters && b2Abs(v.Length() - sigma) > tolerance)
    {
        b2Assert(simplex.m_count < 3);
        
        output->iterations += 1;
        
        // Support in direction -v (A - B)
        indexA = proxyA->GetSupport(b2MulT(xfA.q, -v));
        wA = b2Mul(xfA, proxyA->GetVertex(indexA));
        indexB = proxyB->GetSupport(b2MulT(xfB.q, v));
        wB = b2Mul(xfB, proxyB->GetVertex(indexB));
        b2Vec2 p = wA - wB;
        
        // -v is a normal at p
        v.Normalize();
        
        // Intersect ray with plane
        float32 vp = b2Dot(v, p);
        float32 vr = b2Dot(v, r);
        if (vp - sigma > lambda * vr)
        {
            if (vr <= 0.0f)
            {
                return false;
            }
            
            lambda = (vp - sigma) / vr;
            if (lambda > 1.0f)
            {
                return false;
            }
            
            n = -v;
            simplex.m_count = 0;
        }
        
        // Reverse simplex since it works with B - A.
        // Shift by lambda * r because we want the closest point to the current clip point.
        // Note that the support point p is not shifted because we want the plane equation
        // to be formed in unshifted space.
        b2SimplexVertex* vertex = vertices + simplex.m_count;
        vertex->indexA = indexB;
        vertex->wA = wB + lambda * r;
        vertex->indexB = indexA;
        vertex->wB = wA;
        vertex->w = vertex->wB - vertex->wA;
        vertex->a = 1.0f;
        simplex.m_count += 1;
        
        switch (simplex.m_count)
        {
            case 1:
                break;
                
            case 2:
                simplex.Solve2();
                break;
                
            case 3:
                simplex.Solve3();
                break;
                
            default:
                b2Assert(false);
        }
        
        // If we have 3 points, then the origin is in the corresponding triangle.
        if (simplex.m_count == 3)
        {
            // Overlap
            return false;
        }
        
        // Get search direction.
        v = simplex.GetClosestPoint();
        
        // Iteration count is equated to the number of support point calls.
        ++iter;
    }
    
    // Prepare output.
    b2Vec2 pointA, pointB;
    simplex.GetWitnessPoints(&pointB, &pointA);
    
    if (v.LengthSquared() > 0.0f)
    {
        n = -v;
        n.Normalize();
    }
    
    output->point = pointA + radiusA * n;
    output->normal = n;
    output->lambda = lambda;
    output->iterations = iter;
    return true;
}

#endif /* b2CollisionHandyFunctions_h */
