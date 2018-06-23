Several code lines have been added and modified to separate dependencies between existing files and to only configure them as header files.

### USAGE

```cpp
#include "Box2D.hpp" // changed Box2D.h -> Box2D.hpp
int main()
{
	b2WorldImpl world; // changed (b2World -> b2WorldImpl)
    	// ...
	// same api (wow!)
    	// ...
}
```

It is not very meaningful, but it was made just to spend a boring weekend. I tested it on MacOS only. It will probably run on Windows, but I plan to test it soon.
