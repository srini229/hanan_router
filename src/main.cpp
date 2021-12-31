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
  int uu{1000};
  try {
    uu = std::stoi(parseArgs(argc, argv, "-uu"));
  } catch (const std::invalid_argument& ia) {}

  if (layerJSONFile.empty())  {
    CERR << "missing layers.json file argument" << std::endl;
    return 0;
  }
  DRC::LayerInfo linfo(layerJSONFile);
  if (!plfile.empty() && !leffile.empty()) {
    Placement::Netlist netlist(plfile, leffile, linfo, uu);
    netlist.route();
    netlist.print();
    netlist.plot();
  }

  std::string stfile = parseArgs(argc, argv, "-s");
  Router::HananRouterDB hrdb{linfo};
  if (!stfile.empty()) hrdb.readDataFile(stfile);
  hrdb.findSol();
  hrdb.printSol();
  hrdb.plot();

  return 0;
}
