#ifndef RUN_H
#define RUN_H

#include <iostream>

#include "Point.h"

namespace sp {

  class Run {
    public:
      int colorID;
      Run();
      Point left;
      Point right;
      Run* parent;
      Run* root;
      Run* deepRoot();
  };
}
std::ostream &operator<<(std::ostream&,sp::Run);
#endif
