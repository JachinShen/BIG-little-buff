#include "Headers.h"

bool compareRect(const Rect& r1, const Rect& r2)
{
    if (r1.y < r2.y - r2.height) {
        return true;
    }
    return r1.x < r2.x;
}

bool compareRectX(const Rect& r1, const Rect& r2)
{
    return r1.x < r2.x;
}
