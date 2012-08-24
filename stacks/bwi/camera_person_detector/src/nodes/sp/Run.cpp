#include "Run.h"

using namespace sp;

Run::Run() { colorID = 0; parent = 0; }

Run* Run::deepRoot() {
  Run* dr = root;
  while(dr != dr->root)
    dr = dr->root;
  return dr;
}

std::ostream &operator<<(std::ostream &stream, Run r)
{
  stream << "[" << r.left.x << "," << r.left.y <<"] --> [" << r.right.x << "," << r.right.y << "]";
  return stream;
}
