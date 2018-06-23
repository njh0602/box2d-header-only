#ifndef b2GlobalValues_h
#define b2GlobalValues_h

template <typename Dummy> struct b2GlobalValueStaticBase
{
    // GJK using Voronoi regions (Christer Ericson) and Barycentric coordinates.
    static int32 gjkCalls;
    static int32 gjkIters;
    static int32 gjkMaxIters;
    
    static float32 toiTime;
    static float32 toiMaxTime;
    
    static int32 toiCalls;
    static int32 toiIters;
    static int32 toiMaxIters;
    
    static int32 toiRootIters;
    static int32 toiMaxRootIters;
    
    static bool blockSolve;
};

template <typename Dummy> int32 b2GlobalValueStaticBase<Dummy>::gjkCalls = 0;
template <typename Dummy> int32 b2GlobalValueStaticBase<Dummy>::gjkIters = 0;
template <typename Dummy> int32 b2GlobalValueStaticBase<Dummy>::gjkMaxIters = 0;

template <typename Dummy> float32 b2GlobalValueStaticBase<Dummy>::toiTime = 0.0f;
template <typename Dummy> float32 b2GlobalValueStaticBase<Dummy>::toiMaxTime = 0.0f;

template <typename Dummy> int32 b2GlobalValueStaticBase<Dummy>::toiCalls = 0;
template <typename Dummy> int32 b2GlobalValueStaticBase<Dummy>::toiIters = 0;
template <typename Dummy> int32 b2GlobalValueStaticBase<Dummy>::toiMaxIters = 0;

template <typename Dummy> int32 b2GlobalValueStaticBase<Dummy>::toiRootIters = 0;
template <typename Dummy> int32 b2GlobalValueStaticBase<Dummy>::toiMaxRootIters = 0;

template <typename Dummy> bool b2GlobalValueStaticBase<Dummy>::blockSolve = true;

struct b2GlobalValues : b2GlobalValueStaticBase<void> {};

#endif /* b2GlobalValues_h */
