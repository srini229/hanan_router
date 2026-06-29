#include "Util.h"
#include "Geom.h"
#include "Layer.h"
#include "Placement.h"
#include "Router.h"

int main(int argc, char* argv[])
{
  const std::string logfile = parseArgs(argc, argv, "-log", "route.log");
  if (argc <= 1) {
    std::cerr << "usage : " << argv[0] << "\n\t-d <layers.json>\n\t-p <placement file>\n\t-l <lef file>\n"
      << "\t-s <lef scaling>\n\t-uu <user units scaling>\n\t-ndr <ndr constraints.json> -o <output dir> -r <precision>\n"
      << "\t-reorder <N> (alternate net-ordering passes when nets remain unrouted; default 10)\n"
      << "\t-threads <N> (route non-overlapping nets in parallel using N worker threads; default 1)\n";
    exit(0);
  }
  SaveRestoreStream srs(logfile);
  TIME_M();
  std::string layerJSONFile = parseArgs(argc, argv, "-d");
  std::string plfile = parseArgs(argc, argv, "-p");
  std::string leffile = parseArgs(argc, argv, "-l");
  const bool uuflayer = checkArg(argc, argv, "-s");
  std::string ndrfile = parseArgs(argc, argv, "-ndr");
  std::string prec = parseArgs(argc, argv, "-r");
  SEPARATOR = parseArgs(argc, argv, "-sep", SEPARATOR);
  std::string interlefdir = parseArgs(argc, argv, "-uil");
  if (!interlefdir.empty() && interlefdir.back() != '/') {
    interlefdir += '/';
  }
  std::string outdir = parseArgs(argc, argv, "-o", "./");
  if (!outdir.empty() && outdir.back() != '/') {
    outdir += '/';
  }

  int uu{1};
  try {
    uu = std::stoi(parseArgs(argc, argv, "-uu"));
  } catch (const std::exception& e) {}
  COUT << "Using options : -d " << layerJSONFile << " -p " << plfile << " -l " << leffile;
  COUT << (uuflayer  ? " -s " : "") << " -uu " << uu;
  COUT << (!ndrfile.empty() ? (" -ndr " + ndrfile) : "");
  COUT << (!interlefdir.empty() ? (" -uil " + interlefdir) : "");
  COUT << (!outdir.empty() ? (" -o " + outdir) : "./") << std::endl;

  DRC::LayerInfo linfo(layerJSONFile, (uuflayer ? uu : 1));
  if (!linfo.populated())  {
    CERR << "missing or unable to read layers.json file argument" << std::endl;
    return 1;
  }
  try {
    Router::Router::_precision = prec.empty() ? 1 : std::stoi(prec);
  } catch (const std::exception& e) {
    CERR << "invalid -r precision '" << prec << "', using 1" << std::endl;
    Router::Router::_precision = 1;
  }
  Router::Router hrdb{linfo};
  const std::string rp = parseArgs(argc, argv, "-reorder");
  if (!rp.empty()) {
    try {
      const int n = std::stoi(rp);
      hrdb.setReorderPasses(n < 0 ? 0 : n);
    } catch (const std::exception& e) {
      CERR << "invalid -reorder value '" << rp << "', using default 10" << std::endl;
    }
  }
  const std::string tp = parseArgs(argc, argv, "-threads");
  if (!tp.empty()) {
    try {
      const int n = std::stoi(tp);
      hrdb.setThreads(n < 1 ? 1 : n);
    } catch (const std::exception& e) {
      CERR << "invalid -threads value '" << tp << "', using default 1" << std::endl;
    }
  }
  if (!plfile.empty() && !leffile.empty()) {
    Placement::Netlist netlist(plfile, leffile, linfo, uu, ndrfile, interlefdir);
    netlist.route(hrdb, outdir);
    netlist.checkShort();
  }

  return 0;
}
