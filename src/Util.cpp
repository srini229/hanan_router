#include "Util.h"
#include "Geom.h"
#include <sstream>

static thread_local std::ostream* t_threadLog = nullptr;
std::ostream& threadLog() { return t_threadLog ? *t_threadLog : std::cout; }
void setThreadLog(std::ostream* s) { t_threadLog = s; }

// Set once in main() before any routing (so it is race-free across worker threads).
static bool g_verbose = false;
bool verboseLog() { return g_verbose; }
void setVerboseLog(bool v) { g_verbose = v; }

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

std::set<std::string> splitString(const std::string& s, const char delim)
{
  std::set<std::string> strings;
  if (!s.empty()) {
    std::stringstream ss(s);
    std::string tmps;
    while(getline(ss, tmps, delim)) strings.insert(tmps);
  }
  return strings;
}

const std::vector<std::string> LAYER_COLORS = {"red", "green", "blue", "cyan", "magenta", "black", "grey", "violet", "yellow", "orange", "black"};

std::vector<std::string> LAYER_NAMES;
std::string SEPARATOR = "/";
