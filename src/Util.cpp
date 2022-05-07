#include "Util.h"
#include "Geom.h"

std::string parseArgs(const int argc, char* const argv[], const std::string& arg, std::string str)
{
  for (int i = 0; i < argc; ++i) {
    if (std::string(argv[i]) == arg && i != (argc - 1)) {
      str = argv[i+1];
      break;
    }
  }
  return str;
}

bool checkArg(const int argc, char* const argv[], const std::string& arg)
{
  for (int i = 0; i < argc; ++i) {
    if (std::string(argv[i]) == arg) {
      return true;
    }
  }
  return false;
}

const std::vector<std::string> LAYER_COLORS = {"red", "green", "blue", "cyan", "magenta", "black", "grey", "violet", "yellow", "orange", "black"};

std::vector<std::string> LAYER_NAMES;
std::string SEPARATOR = "/";
