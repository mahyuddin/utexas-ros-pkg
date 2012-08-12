#define CPOINT_DISTANCE(a,b) sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z))
#define KEEP_POINT(a) (_minX < a.x && a.x < _maxX && _minY < a.y && a.y < _maxY && _minZ < a.z && a.z < _maxZ)

