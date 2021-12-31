#include "Util.h"
#include "Geom.h"
#include "Layer.h"
#include "Placement.h"
#include "Router.h"

int main(int argc, char* argv[])
{
  const std::string logfile = parseArgs(argc, argv, "-log", "route.log");
  SaveRestoreStream srs(logfile);
  TIME_M();
  std::string layerJSONFile = parseArgs(argc, argv, "-d");
  std::string plfile = parseArgs(argc, argv, "-p");
  std::string leffile = parseArgs(argc, argv, "-l");
  const bool route = checkArg(argc, argv, "-r");
  int uu{1000};
  try {
    uu = std::stoi(parseArgs(argc, argv, "-uu"));
  } catch (const std::invalid_argument& ia) {}

  DRC::LayerInfo linfo(layerJSONFile);
  if (!linfo.populated())  {
    CERR << "missing or unable to read layers.json file argument" << std::endl;
    return 0;
  }
  Router::Router hrdb{linfo};
  if (!plfile.empty() && !leffile.empty()) {
    Placement::Netlist netlist(plfile, leffile, linfo, uu);
    if (route) {
      netlist.route(hrdb);
    }
    netlist.print();
    netlist.plot();
  }

  std::string stfile = parseArgs(argc, argv, "-s");
  if (!stfile.empty()) {
    hrdb.readDataFile(stfile);
    hrdb.findSol();
    hrdb.printSol();
    hrdb.plot();
  }

  return 0;
}
