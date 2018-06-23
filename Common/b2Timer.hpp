/*
* Copyright (c) 2011 Erin Catto http://box2d.org
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

#ifndef B2_TIMER_H
#define B2_TIMER_H

#include "b2Settings.hpp"

#if defined(_WIN32)

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <windows.h>

template <typename Dummy>
class b2TimerStaticBase
{
protected:
    static float64 s_invFrequency;
};

template <typename Dummy>
float64 b2TimerStaticBase<Dummy>::s_invFrequency = 0.0f;

/// Timer for profiling. This has platform specific code and may
/// not work on every platform.
class b2Timer : public b2TimerStaticBase<void>
{
public:

	/// Constructor
	b2Timer()
    {
        LARGE_INTEGER largeInteger;
        
        if (s_invFrequency == 0.0f)
        {
            QueryPerformanceFrequency(&largeInteger);
            s_invFrequency = float64(largeInteger.QuadPart);
            if (s_invFrequency > 0.0f)
            {
                s_invFrequency = 1000.0f / s_invFrequency;
            }
        }
        
        QueryPerformanceCounter(&largeInteger);
        m_start = float64(largeInteger.QuadPart);
    }

	/// Reset the timer.
	void Reset()
    {
        LARGE_INTEGER largeInteger;
        QueryPerformanceCounter(&largeInteger);
        m_start = float64(largeInteger.QuadPart);
    }

	/// Get the time since construction or the last reset.
	float32 GetMilliseconds() const
    {
        LARGE_INTEGER largeInteger;
        QueryPerformanceCounter(&largeInteger);
        float64 count = float64(largeInteger.QuadPart);
        float32 ms = float32(s_invFrequency * (count - m_start));
        return ms;
    }

private:

	float64 m_start;
    
};

#elif defined(__linux__) || defined (__APPLE__)

#include <sys/time.h>

class b2Timer
{
public:
    
    /// Constructor
    b2Timer()
    {
        Reset();
    }
    
    /// Reset the timer.
    void Reset()
    {
        timeval t;
        gettimeofday(&t, 0);
        m_start_sec = t.tv_sec;
        m_start_usec = t.tv_usec;
    }
    
    /// Get the time since construction or the last reset.
    float32 GetMilliseconds() const
    {
        timeval t;
        gettimeofday(&t, 0);
        time_t start_sec = m_start_sec;
        suseconds_t start_usec = m_start_usec;
        
        // http://www.gnu.org/software/libc/manual/html_node/Elapsed-Time.html
        if (t.tv_usec < start_usec)
        {
            int nsec = (start_usec - t.tv_usec) / 1000000 + 1;
            start_usec -= 1000000 * nsec;
            start_sec += nsec;
        }
        
        if (t.tv_usec - start_usec > 1000000)
        {
            int nsec = (t.tv_usec - start_usec) / 1000000;
            start_usec += 1000000 * nsec;
            start_sec -= nsec;
        }
        
        return 1000.0f * (t.tv_sec - start_sec) + 0.001f * (t.tv_usec - start_usec);
    }
    
private:
    
    unsigned long long m_start_sec;
    unsigned long long m_start_usec;
    
};

#else

class b2Timer
{
public:
    b2Timer(){}
    void Reset() {}
    float32 GetMilliseconds() const { return 0.0f; }
};

#endif

#endif
