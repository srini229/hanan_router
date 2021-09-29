#include <iostream>

#include "Geom.h"
#include "Layer.h"


std::string parseArgs(const int argc, char* const argv[], const std::string& arg)
{
  std::string str;
  for (unsigned i = 0; i < argc; ++i) {
    if (std::string(argv[i]) == arg && i != (argc - 1)) {
      str = argv[i+1];
      break;
    }
  }
  return str;
}

int main(int argc, char* argv[])
{
  Geom::Point pt(10, 10);
  std::cout << pt.toJSON().dump() << std::endl;
  Geom::Transform tr(pt, 0, 1);
  std::cout << tr.toJSON().dump() << std::endl;
  Geom::Rect r(0, 0, 10, 10);
  std::cout << r.toJSON().dump() << std::endl;

  std::string layerJSONFile = parseArgs(argc, argv, "-l");

  DRC::LayerInfo linfo(layerJSONFile);

  return 0;
}
