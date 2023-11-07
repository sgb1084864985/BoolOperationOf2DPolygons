# Bool Operation of 2D polygons
The class `BoolOperation` can handle the input polygons without holes, while the `BoolOperation2` can handle the polygons with holes.

In fact, the classes withs 2 (like `Polygon2`,`BoolOperation2`)are upgraded classes that are intended to handle the polygons with holes.

Example:
```c++
#include <cstddef>
#include <cstdio>
#include <iostream>
#include <string>
#include "bool.hpp"
#include "bool2.hpp"
using namespace std;

int main(){
    auto io=PolygonFileController2();
    auto p1=io.read("resources/polygon1.txt");

    auto p2=io.read("resources/polygon2.txt");

    auto bool_ops2=BoolOperation2(p1,p2);

    io.write("resources/polygon15.txt",bool_ops2.polygon_union()[0]);
    io.write("resources/polygon16.txt",bool_ops2.polygon_intersect()[0]);
    io.write("resources/polygon17.txt",bool_ops2.polygon_diff()[0]);

    return 0;
}
```