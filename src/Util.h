#ifndef UTIL_H_
#define UTIL_H_
#include <chrono>
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <set>

std::ostream& threadLog();
void setThreadLog(std::ostream* s);  // nullptr restores std::cout for this thread
#define COUT threadLog() //<< __PRETTY_FUNCTION__ << " -:- "
#define CERR std::cerr //<< __PRETTY_FUNCTION__ << " -:- "

class TimeMeasure {
  private:
    const std::string _name;
    std::chrono::nanoseconds* _rt;
    std::chrono::high_resolution_clock::time_point _begin;
  public:
    TimeMeasure(const std::string& name, std::chrono::nanoseconds* rt = nullptr) : _name(name), _rt(rt)
    {
      _begin = std::chrono::high_resolution_clock::now();
    }
    ~TimeMeasure()
    {
      auto difft = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - _begin);
      if (_rt) {
        (*_rt) += difft;
      } else {
        COUT << _name << " runtime : " << difft.count()/1.e9 << "(s)\n";
      }
    }
};
#define TIME_CONCAT_(a, b) a##b
#define TIME_CONCAT(a, b) TIME_CONCAT_(a, b)
#define TIME_MA(X) TimeMeasure TIME_CONCAT(__timer_, __LINE__)(__PRETTY_FUNCTION__, X)
#define TIME_M()  TimeMeasure TIME_CONCAT(__timer_, __LINE__)(__PRETTY_FUNCTION__)

class SaveRestoreStream {
  private:
    std::ofstream _ofs, _efs;
    std::streambuf *_ostream, *_estream;
  public:
    SaveRestoreStream(const std::string& logname, const std::string& errname = "err.log") : _ofs(logname), _efs(errname),
    _ostream(nullptr), _estream(nullptr)
    {
      if (_ofs) {
        _ostream = std::cout.rdbuf(_ofs.rdbuf());
      } else {
        _ofs.close();
      }
      if (_efs) {
        _estream = std::cerr.rdbuf(_efs.rdbuf());
      } else {
        _efs.close();
      }
    }
    ~SaveRestoreStream()
    {
      if (_ostream) {
        std::cout.rdbuf(_ostream);
      }
      if (_estream) {
        std::cerr.rdbuf(_estream);
      }
    }
};

extern const std::vector<std::string> LAYER_COLORS;

std::string parseArgs(const int argc, char* const argv[], const std::string& arg, std::string str = "");
bool checkArg(const int argc, char* const argv[], const std::string& arg);
std::set<std::string> splitString(const std::string& s, const char delim = ',');

extern std::vector<std::string> LAYER_NAMES;
extern std::string SEPARATOR;

inline const std::string& layerName(const int idx) {
    static const std::string unknown{"UNKNOWN"};
    return (idx >= 0 && static_cast<size_t>(idx) < LAYER_NAMES.size())
        ? LAYER_NAMES[idx] : unknown;
}

#endif
