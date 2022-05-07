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
      << "\t-r <route on/off>\n\t-s <lef scaling>\n\t-uu <user units scaling>\n\t-ndr <ndr constraints.json>\n";
    exit(0);
  }
  SaveRestoreStream srs(logfile);
  TIME_M();
  std::string layerJSONFile = parseArgs(argc, argv, "-d");
  std::string plfile = parseArgs(argc, argv, "-p");
  std::string leffile = parseArgs(argc, argv, "-l");
  const bool route = checkArg(argc, argv, "-r");
  const bool uuflayer = checkArg(argc, argv, "-s");
  std::string ndrfile = parseArgs(argc, argv, "-ndr");
  SEPARATOR = parseArgs(argc, argv, "-sep", "+");

  int uu{2000};
  try {
    uu = std::stoi(parseArgs(argc, argv, "-uu"));
  } catch (const std::invalid_argument& ia) {}
  COUT << "Using options : -d " << layerJSONFile << " -p " << plfile << " -l " << leffile;
  COUT << (route ? " -r " : "") << (uuflayer  ? " -s " : "") << " -uu " << uu;
  COUT << (!ndrfile.empty() ? (" -ndr " + ndrfile) : "") << std::endl;

  DRC::LayerInfo linfo(layerJSONFile, (uuflayer ? uu : 1));
  if (!linfo.populated())  {
    CERR << "missing or unable to read layers.json file argument" << std::endl;
    return 0;
  }
  Router::Router hrdb{linfo};
  if (!plfile.empty() && !leffile.empty()) {
    Placement::Netlist netlist(plfile, leffile, linfo, uu, ndrfile);
    if (route) {
      netlist.route(hrdb);
    } else {
      netlist.writeDEF();
    }
    netlist.print();
    netlist.plot();
    netlist.checkShort();
  }

  /*std::string stfile = parseArgs(argc, argv, "-st");
  if (!stfile.empty()) {
    hrdb.readDataFile(stfile);
    hrdb.findSol();
    hrdb.plot();
    hrdb.printSol();
  }*/

  return 0;
}
