Several code lines have been added and modified to separate dependencies between existing files and to only configure them as header files.

### USAGE

```cpp
#include "Box2D.hpp" // changed Box2D.h -> Box2D.hpp
int main()
{
	b2WorldImpl world; // changed (b2World -> b2WorldImpl)
    	// ...
	// same api
    	// ...
}
```

It is not very meaningful, but it was made just to spend a boring weekend. I tested on MacOS, Windows, Linux.
