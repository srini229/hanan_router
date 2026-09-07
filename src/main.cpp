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
      << "\t-replay <ATTEMPT_*.lef> (re-route one wire from a HANAN_DEBUG_WIRE dump; needs -d only)\n"
      << "\t-detour (with -replay: allow a large detour even without NDR saying so)\n"
      << "\t-rsmt (confine each net to a Borah Steiner corridor over its pins)\n"
      << "\t-threads <N> (route non-overlapping nets in parallel using N worker threads; default 1)\n"
      << "\t-relaxvia (in the final pass, for a net that still fails to route, retry its escape via with spacing relaxed to as close as 5 to, but never on, a shape -- source pins first, then also target pins if that alone isn't enough)\n"
      << "\t-v (verbose: emit high-volume per-element debug logging)\n";
    exit(0);
  }
  setVerboseLog(checkArg(argc, argv, "-v") || getenv("HANAN_VERBOSE"));
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
  COUT << "Using options :";
  for (int i = 1; i < argc; ++i) COUT << ' ' << argv[i];
  COUT << std::endl;

  // RSMT corridors use the Borah-Owens-Irwin edge-substitution heuristic
  // (IEEE TCAD 1994); see src/borah.h.
  const bool rsmtOpt = checkArg(argc, argv, "-rsmt");

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
  const bool relaxViaOpt = checkArg(argc, argv, "-relaxvia");
  COUT << "Effective settings :"
       << " -d " << (layerJSONFile.empty() ? "<none>" : layerJSONFile)
       << " -p " << (plfile.empty() ? "<none>" : plfile)
       << " -l " << (leffile.empty() ? "<none>" : leffile)
       << " -o " << outdir
       << " -uu " << uu
       << " -r " << Router::Router::_precision
       << " -sep " << SEPARATOR
       << " -reorder " << hrdb.reorderPasses()
       << " -threads " << hrdb.threads();
  if (!ndrfile.empty())     COUT << " -ndr " << ndrfile;
  if (!interlefdir.empty()) COUT << " -uil " << interlefdir;
  if (uuflayer)             COUT << " -s";
  if (relaxViaOpt)          COUT << " -relaxvia";
  if (rsmtOpt)              COUT << " -rsmt";
  if (checkArg(argc, argv, "-cornerescape")) COUT << " -cornerescape";
  if (verboseLog())         COUT << " -v";
  COUT << std::endl;
  hrdb.setRSMTCorridor(rsmtOpt);
  const std::string replayfile = parseArgs(argc, argv, "-replay");
  if (!replayfile.empty()) {
    hrdb.setCornerEscape(checkArg(argc, argv, "-cornerescape"));
    hrdb.setRelaxViaEscape(relaxViaOpt);
    return Router::replay(hrdb, replayfile, uu, checkArg(argc, argv, "-detour"), ndrfile, linfo) ? 0 : 1;
  }
  if (!plfile.empty() && !leffile.empty()) {
    Placement::Netlist netlist(plfile, leffile, linfo, uu, ndrfile, interlefdir);
    netlist.route(hrdb, outdir);
    const int open = netlist.totalUnrouted();
    if (open > 0) {
      COUT << "centre-track pin escape left " << open
           << " net(s) open; re-routing the design with corner pin-escape points\n";
      hrdb.setCornerEscape(true);
      hrdb.setDumpOpenNets(true);       // last pass: write a debug LEF for any net
                                        // still left open (pins/srcs/tgts/obstacles)
      if (relaxViaOpt) {
        hrdb.setRelaxViaEscape(true);   // last pass: for a net that's still unrouted,
                                        // retry its own escape via with spacing relaxed
                                        // to as close as 5 to (never on) a shape --
                                        // source pins first, then target pins too
      }
      Placement::Netlist netlist2(plfile, leffile, linfo, uu, ndrfile, interlefdir);
      netlist2.route(hrdb, outdir);
      netlist2.printRouteSummaries();   // one authoritative summary, final state
      netlist2.printWirelengths();
      netlist2.checkShort();
      netlist2.checkDRC(hrdb);
    } else {
      netlist.printRouteSummaries();
      netlist.printWirelengths();
      netlist.checkShort();
      netlist.checkDRC(hrdb);
    }
  }

  return 0;
}
