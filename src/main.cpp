#include <iostream>

#include "Geom.h"
#include "Layer.h"

int main()
{
  Geom::Point pt(10, 10);
  std::cout << pt.toJSON().dump() << std::endl;
  Geom::Transform tr(pt, 0, 1);
  std::cout << tr.toJSON().dump() << std::endl;
  Geom::Rect r(0, 0, 10, 10);
  std::cout << r.toJSON().dump() << std::endl;

  RouterDB::MetalLayer m;

  return 0;
}
