#include "Util.h"
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include "Geom.h"
#include "Layer.h"
#include "Placement.h"
#include "Router.h"

int main(int argc, char* argv[])
{
  const std::string logfile = parseArgs(argc, argv, "-log", "route.log");
  if (argc <= 1) {
    std::cerr << "usage : " << argv[0] << "\n\t-d <layers.json>\n\t-p <netlist.json> (netlist and placement)\n\t-l <lef file>\n"
      << "\t-s <lef scaling>\n\t-uu <user units scaling>\n\t-ndr <ndr constraints.json> -o <output dir> -r <precision>\n"
      << "\t-reorder <N> (alternate net-ordering passes when nets remain unrouted; default 10)\n"
      << "\t-maxexp <N> (A* node-expansion budget per search stage; default 100000)\n"
      << "\t-escapepitch <N> (thin pin-escape points to one per N wire pitches; 0 keeps them all; default 1)\n"
      << "\t-reorderbudget <N> (cap a block's reorder passes so base-route expansions x passes stays under N; 0 uncapped; default 15000000)\n"
      << "\t-keepblockedescapes (seed every pin-escape point, including ones no wire can start from; default is to drop them)\n"
      << "\t-viaalign (pick the via rotation whose pads run along the wires they join, instead of the first legal one)\n"
      << "\t-satfirst (route the nets whose pins the pre-route escape check could not clear before the rest)\n"
      << "\t-gridprune <percent of pitch: collapse grid coordinates closer than this>\n\t-abutescape (a shape already touching a pin does not block that pin's escape via on spacing; only a real overlap does)\n"
      << "\t-hopeless <N> (retire a net the escape check flagged once it has routed nothing in N whole attempts; 0 never; default 3)\n"
      << "\t-seedpolys <N> (for a pin split into more than N polygons, seed only the N nearest the other end; 0 seeds all; default 0)\n"
      << "\t-seedpolysalways (keep that limit on every attempt; default restores the rest for a net still open by attempt 3)\n"
      << "\t-replay <ATTEMPT_*.lef> (re-route one wire from a HANAN_DEBUG_WIRE dump; needs -d only)\n"
      << "\t-detour (with -replay: allow a large detour even without NDR saying so)\n"
      << "\t-rsmt (confine each net to a Borah Steiner corridor over its pins)\n"
      << "\t-padhalo (also put grid lines where a via pad, not just a wire, clears each obstacle; default: wire halo only)\n"
      << "\t-viacost <r|P|0> (r: a via costs the length of the narrower metal it joins with the same resistance, the\n"
      << "\t    default; P: at least P wire pitches of the cheaper metal; 0: its bare layer-file R)\n"
      << "\t-nohalofallback (do not re-route still-open hierarchies with -padhalo lines at the end)\n"
      << "\t-admissible (A* uses the admissible distance bound everywhere, not only under a corridor guide)\n"
      << "\t-nopattern (skip L/Z pattern routing; every wire goes to A*)\n"
      << "\t-softwires (other nets' wires are not obstacles, only their pins: the nets still open are pin-limited; output not legal)\n"
      << "\t-guidedsym (route a symmetric pair's second net under the guide only; by default, where the placement mirrored\n"
      << "\t    the pair's pins, the first net is routed clear of every other net's mirror too and the second is its exact mirror)\n"
      << "\t-satpoint (pre-route escape check takes escapes from points along each pin, as the search does)\n"
      << "\t-reserveexcluded (keep one escape clear for every pin of a do_not_route net, for a later pass)\n"
      << "\t-noviarotate (offer each single-cut via only as drawn, not also turned 90 degrees)\n"
      << "\t-noseed (do not seed grid lines inside corridor bands)\n"
      << "\t-threads <N> (route non-overlapping nets in parallel using N worker threads; default 1)\n"
      << "\t-relaxvia (in the final pass, for a net that still fails to route, retry its escape via with spacing relaxed to as close as 5 to, but never on, a shape -- source pins first, then also target pins if that alone isn't enough)\n"
      << "\t-v [N] (log verbosity: 0 results only (default), 1 per-net detail, 2 per-element dumps, 3 everything)\n";
    exit(0);
  }
  {
    // -v alone means level 1; -v <N> selects the level, as does HANAN_VERBOSE=<N>
    int level = LogLevel::RESULT;
    if (const char* e = getenv("HANAN_VERBOSE")) {
      level = (*e && isdigit(static_cast<unsigned char>(*e))) ? atoi(e) : LogLevel::NET;
    }
    if (checkArg(argc, argv, "-v")) {
      if (level < LogLevel::NET) level = LogLevel::NET;
      const std::string lv = parseArgs(argc, argv, "-v");
      if (!lv.empty() && lv.find_first_not_of("0123456789") == std::string::npos) {
        level = std::stoi(lv);
      }
    }
    setVerboseLevel(level);
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
  COUT << "Using options :";
  for (int i = 1; i < argc; ++i) COUT << ' ' << argv[i];
  COUT << std::endl;

  // RSMT corridors use the Borah-Owens-Irwin edge-substitution heuristic
  // (IEEE TCAD 1994); see src/borah.h.
  const bool rsmtOpt = checkArg(argc, argv, "-rsmt");

  const auto tStart = std::chrono::steady_clock::now();
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
  const std::string me = parseArgs(argc, argv, "-maxexp");
  if (!me.empty()) {
    try {
      const long long n = std::stoll(me);
      if (n > 0) hrdb.setMaxExpansions(static_cast<size_t>(n));
      else CERR << "invalid -maxexp value '" << me << "', using default" << std::endl;
    } catch (const std::exception& e) {
      CERR << "invalid -maxexp value '" << me << "', using default" << std::endl;
    }
  }
  const std::string rb = parseArgs(argc, argv, "-reorderbudget");
  if (!rb.empty()) {
    try {
      const long long n = std::stoll(rb);
      hrdb.setReorderBudget(n < 0 ? 0 : n);
    } catch (const std::exception& e) {
      CERR << "invalid -reorderbudget value '" << rb << "', using default" << std::endl;
    }
  }
  if (checkArg(argc, argv, "-keepblockedescapes")) hrdb.setPruneEscapes(false);
  if (checkArg(argc, argv, "-viaalign")) hrdb.setViaAlign(true);
  if (checkArg(argc, argv, "-padhalo")) hrdb.setPadHaloLines(true);
  if (checkArg(argc, argv, "-admissible")) hrdb.setAdmissibleBound(true);
  if (checkArg(argc, argv, "-nopattern")) hrdb.setNoPattern(true);
  if (checkArg(argc, argv, "-softwires")) hrdb.setSoftWires(true);
  if (checkArg(argc, argv, "-guidedsym")) hrdb.setHardSymmetry(false);
  if (checkArg(argc, argv, "-satpoint")) hrdb.setSatPoint(true);
  if (checkArg(argc, argv, "-reserveexcluded")) hrdb.setReserveExcluded(true);
  if (checkArg(argc, argv, "-noviarotate")) hrdb.setViaRotate(false);
  const std::string viacost = parseArgs(argc, argv, "-viacost");
  if (viacost.empty() || viacost == "r") hrdb.setViaCostFromResistance();
  else hrdb.setViaCostPitches(std::stod(viacost));
  if (checkArg(argc, argv, "-noseed")) hrdb.setSeedCorridor(false);
  if (checkArg(argc, argv, "-satfirst")) hrdb.setSatFirst(true);
  if (checkArg(argc, argv, "-abutescape")) hrdb.setAbutEscape(true);
  const std::string gp = parseArgs(argc, argv, "-gridprune");
  if (!gp.empty()) {
    try { hrdb.setGridPrune(std::stoi(gp)); }
    catch (const std::exception&) { CERR << "ignoring bad -gridprune " << gp << '\n'; }
  }
  const std::string ha = parseArgs(argc, argv, "-hopeless");
  if (!ha.empty()) {
    try { hrdb.setHopelessAfter(std::stoi(ha)); }
    catch (const std::exception& e) { CERR << "invalid -hopeless value '" << ha << "', using default" << std::endl; }
  }
  if (checkArg(argc, argv, "-seedpolysalways")) hrdb.setSeedPolysAlways(true);
  const std::string sp = parseArgs(argc, argv, "-seedpolys");
  if (!sp.empty()) {
    try {
      const int n = std::stoi(sp);
      hrdb.setMaxSeedPolys(n < 0 ? 0 : n);
    } catch (const std::exception& e) {
      CERR << "invalid -seedpolys value '" << sp << "', using default" << std::endl;
    }
  }
  const std::string ep = parseArgs(argc, argv, "-escapepitch");
  if (!ep.empty()) {
    try {
      const int n = std::stoi(ep);
      hrdb.setEscapePitchMul(n < 0 ? 0 : n);
    } catch (const std::exception& e) {
      CERR << "invalid -escapepitch value '" << ep << "', using default" << std::endl;
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
       << " -maxexp " << hrdb.maxExpansions()
       << " -escapepitch " << hrdb.escapePitchMul()
       << " -reorderbudget " << hrdb.reorderBudget()
       << (hrdb.pruneEscapes() ? "" : " -keepblockedescapes")
       << (hrdb.viaAlign() ? " -viaalign" : "")
       << (hrdb.padHaloLines() ? " -padhalo" : "")
       << (hrdb.admissibleBound() ? " -admissible" : "")
       << (hrdb.noPattern() ? " -nopattern" : "")
       << (hrdb.softWires() ? " -softwires" : "")
       << (hrdb.hardSymmetry() ? "" : " -guidedsym")
       << (hrdb.satPoint() ? " -satpoint" : "")
       << (hrdb.reserveExcluded() ? " -reserveexcluded" : "")
       << (hrdb.viaRotate() ? "" : " -noviarotate")
       << (checkArg(argc, argv, "-nohalofallback") ? " -nohalofallback" : "")
       << (viacost.empty() ? "" : " -viacost " + viacost)
       << (hrdb.seedCorridor() ? "" : " -noseed")
       << (hrdb.satFirst() ? " -satfirst" : "")
       << (hrdb.gridPrune() ? " -gridprune " + std::to_string(hrdb.gridPrune()) : "")
       << (hrdb.abutEscape() ? " -abutescape" : "")
       << (hrdb.hopelessAfter() ? " -hopeless " : "") << (hrdb.hopelessAfter() ? std::to_string(hrdb.hopelessAfter()) : "")
       << " -seedpolys " << hrdb.maxSeedPolys()
       << (hrdb.seedPolysAlways() ? " -seedpolysalways" : "")
       << " -threads " << hrdb.threads();
  if (!ndrfile.empty())     COUT << " -ndr " << ndrfile;
  if (!interlefdir.empty()) COUT << " -uil " << interlefdir;
  if (uuflayer)             COUT << " -s";
  if (relaxViaOpt)          COUT << " -relaxvia";
  if (rsmtOpt)              COUT << " -rsmt";
  if (checkArg(argc, argv, "-cornerescape")) COUT << " -cornerescape";
  if (verboseLevel())       COUT << " -v " << verboseLevel();
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
    const auto tRead = std::chrono::steady_clock::now();
    netlist.route(hrdb, outdir);
    int open = netlist.totalUnrouted();
    if (const int sym = netlist.symUnrouted()) {
      COUT << "hierarchies with symmetric pairs left " << sym
           << " net(s) open; re-routing them with the symmetry guide on planar distance only\n";
      const bool hard = hrdb.hardSymmetry();
      hrdb.setGuideLayers(false);       // the partner may leave the guide's layers to let other nets by
      hrdb.setHardSymmetry(false);      // and need not be the first net's exact mirror
      netlist.reroute(hrdb, outdir, true);
      if (netlist.symUnrouted() > sym) {
        COUT << "planar guide left " << netlist.symUnrouted() << " net(s) open against " << sym
             << "; re-routing those hierarchies as before\n";
        hrdb.setGuideLayers(true);
        hrdb.setHardSymmetry(hard);
        netlist.reroute(hrdb, outdir, true);
      }
      open = netlist.totalUnrouted();
    }
    if (open > 0) {
      COUT << "centre-track pin escape left " << open
           << " net(s) open; re-routing with corner pin-escape points\n";
      hrdb.setCornerEscape(true);
      hrdb.setDumpOpenNets(true);       // last pass: write a debug LEF for any net
                                        // still left open (pins/srcs/tgts/obstacles)
      if (relaxViaOpt) {
        hrdb.setRelaxViaEscape(true);   // last pass: for a net that's still unrouted,
                                        // retry its own escape via with spacing relaxed
                                        // to as close as 5 to (never on) a shape --
                                        // source pins first, then target pins too
      }
      netlist.reroute(hrdb, outdir);    // only the hierarchies that are still open
      open = netlist.totalUnrouted();
    }
    if (open > 0 && !hrdb.padHaloLines() && !checkArg(argc, argv, "-nohalofallback")) {
      COUT << "corner pin escape left " << open << " net(s) open; re-routing with via-pad halo grid lines\n";
      const int before = open;
      const bool corner = hrdb.cornerEscape();
      hrdb.setPadHaloLines(true);       // lines where a via pad clears an obstacle, paid for only where a net is open
      hrdb.setCornerEscape(false);      // halo lines with centre-track escapes: corner escapes on top lose nets here
      netlist.reroute(hrdb, outdir);
      open = netlist.totalUnrouted();
      if (open > before) {              // the denser grid lost more than it won: the pass before it, again
        COUT << "via-pad halo grid lines left " << open << " net(s) open against " << before
             << "; re-routing the open hierarchies as before\n";
        hrdb.setPadHaloLines(false);
        hrdb.setCornerEscape(corner);
        netlist.reroute(hrdb, outdir);
      }
    }
    const auto tRouted = std::chrono::steady_clock::now();
    netlist.printRouteSummaries();      // one authoritative summary, final state
    netlist.printWirelengths();
    netlist.checkShort();
    netlist.checkDRC(hrdb);
    // routing alone: from the inputs read to the last pass, less the modules' output writing and checks
    auto sec = [](const auto d) { return std::chrono::duration<double>(d).count(); };
    const double wr = sec(Placement::Module::writeTime), ck = sec(Placement::Module::checkTime);
    const auto tEnd = std::chrono::steady_clock::now();
    COUT << "RUNTIME read=" << sec(tRead - tStart) << " route=" << sec(tRouted - tRead) - wr - ck << " write=" << wr
         << " checks=" << ck + sec(tEnd - tRouted) << " total=" << sec(tEnd - tStart) << " (s)\n";
  }

  return 0;
}
