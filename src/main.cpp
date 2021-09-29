#include <iostream>

#include "Geom.h"
#include "Layer.h"


std::string parseArgs(const int argc, char* const argv[], const std::string& arg)
{
  std::string str;
  for (int i = 0; i < argc; ++i) {
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

  std::string layerJSONFile = parseArgs(argc, argv, "-l");

  DRC::LayerInfo linfo(layerJSONFile);

  return 0;
}
