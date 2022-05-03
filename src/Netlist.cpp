#include <sstream>
#include "nlohmann/json.hpp"
#include "Util.h"
#include "Placement.h"

namespace Placement {
using json = nlohmann::json;
using ordered_json = nlohmann::ordered_json;
const auto& npos = std::string::npos;

Netlist::Netlist(const std::string& plfile, const::std::string& leffile, const DRC::LayerInfo& lf, const int uu, const std::string& ndrfile) : _uu(uu), _valid{1}
{
  if (plfile.empty()) {
    CERR<< "missing placement file" <<std::endl;
    _valid = 0;
    return;
  }
  std::ifstream ifs(plfile);
  if (!ifs) {
    CERR << "unable to open placement file " << plfile <<std::endl;
    _valid = 0;
    return;
  }
  ordered_json oj = json::parse(ifs);
  ifs.close();
  auto it = oj.find("leaves");
  if (it != oj.end()) {
    for (auto& l : *it) {
      auto lname = l.find("concrete_name");
      if (_modules.find(*lname) != _modules.end()) continue;
      if (lname != l.end()) {
        auto modu = new Module(*lname, 1, _uu);
        COUT << "adding leaf : " << *lname << '\n';
        auto terms = l.find("terminals");
        if (terms != l.end()) {
          for (auto& term : *terms) {
            auto p = modu->addPin(term["name"]);
            modu->addNet(term["name"]);
            modu->net(term["name"])->addPin(p);
          }
        }
        _modules[modu->name()] = modu;
      }
    }
  }
  it = oj.find("modules");
  if (it != oj.end()) {
    for (auto& m : *it) {
      auto mname = m.find("concrete_name");
      if (mname != m.end()) {
        if (_modules.find(*mname) != _modules.end()) continue;
        auto modu = new Module(*mname, 0, _uu);
        auto params = m.find("parameters");
        if (params != m.end()) {
          for (auto& p : *params) {
            modu->addPin(p);
          }
        }
        auto bbox = m.find("bbox");
        if (bbox != m.end()) {
          const auto& b = (*bbox);
          modu->setBBox(Geom::Rect(b[0], b[1], b[2], b[3]));
        }
        auto insts = m.find("instances");
        if (insts != m.end()) {
          for (auto& inst : *insts) {
            auto iname = inst.find("instance_name");
            auto mname = inst.find("concrete_template_name");
            auto tritr = inst.find("transformation");
            auto tr = (tritr == inst.end()) ? Geom::Transform() :
              Geom::Transform((*tritr)["oX"], (*tritr)["oY"], (*tritr)["sX"], (*tritr)["sY"]) ;
            Placement::Instance* instptr{nullptr};
            if (iname != inst.end() && mname != inst.end()) {
              instptr = modu->addInstance(*iname, *mname, tr);
            }
            if (instptr) {
              auto famap = inst.find("fa_map");
              if (famap != inst.end()) {
                for (auto& pm : *famap) {
                  const Net* n = &(modu->addNet(pm["actual"]));
                  modu->addTmpPin(n, instptr, pm["formal"]);
                }
              }
            } else {
              COUT << "instptr nullptr\n";
            }
          }
        }
        _modules[modu->name()] = modu;
      }
    }
  }
  loadLEF(leffile, lf);
  build();
  readNDR(ndrfile, lf);
}


Netlist::~Netlist()
{
  for (auto& m : _modules) delete m.second;
  _modules.clear();
}


void Netlist::print() const
{
  for (const auto& m : _modules) {
    COUT << "module : " << m.second->name() << '\n';
    m.second->print();
  }
}


void Netlist::build()
{
  for (auto& m : _modules) {
    for (auto& inst : m.second->instances()) {
      auto it = _modules.find(inst->moduleName());
      if (it != _modules.end()) {
        inst->setModule(it->second);
      }
    }
  }
  for (auto& m : _modules) m.second->build();
}


void Netlist::loadLEF(const std::string& leffile, const DRC::LayerInfo& lf)
{
  if (leffile.empty()) {
    CERR<< "missing leffile" <<std::endl;
    _valid = 0;
    return;
  }
  std::ifstream ifs(leffile);
  if (!ifs) {
    CERR << "unable to open leffile " << leffile <<std::endl;
    _valid = 0;
    return;
  }
  std::string line;
  bool inMacro{false}, inPin{false}, inObs{false}, inPort{false}, inUnits{false};
  Module* curr_module{nullptr};
  Pin* curr_pin{nullptr};
  std::string macroName, pinName;
  int layer{-1};
  double macroUnits{1.};
  int units = _uu;
  while (std::getline(ifs, line)) {
    std::string str;
    std::stringstream ss(line);
    if (line.find("MACRO") != npos) {
      ss >> str >> macroName;
      COUT << "macro " << macroName << '\n';
      auto it = _modules.find(macroName);
      if (it != _modules.end()) {
        curr_module =  it->second;
        COUT << "loading macro " << macroName << '\n';
      }
      inMacro = true;
      continue;
    }
    if (line.find("FOREIGN ") != npos) continue;
    if (line.find("END") != npos) {
      if (inUnits) {
        if (line.find("UNITS") != npos) {
          inUnits = false;
        }
      }
      if (inPort) {
        inPort = false;
      } else if (inPin) {
        if (line.find(pinName) != npos) {
          inPin = false;
          curr_pin = nullptr;
          pinName.clear();
        }
      } else if (inMacro) {
        if (line.find(macroName) != npos) {
          inMacro = false;
          curr_module = nullptr;
          macroName.clear();
        }
      } else if (inObs) {
        inObs = false;
        layer = -1;
      }
      continue;
    }
    if (inMacro && curr_module) {
      if (line.find("SIZE") != npos) {
        double w{0.}, h{0.};
        ss >> str >> w >> str >> h;
        curr_module->setBBox(Geom::Rect(0, 0, w * units, h * units));
      }
      if (line.find("PIN") != npos) {
        ss >> str >> pinName;
        curr_pin = curr_module->getPin(pinName);
        inPin = true;
        continue;
      }
    }
    if (inUnits && line.find("DATABASE") != npos) {
      ss >> str >> str >> str >> macroUnits;
      units /= macroUnits;
    }
    if (inPin && curr_pin && line.find("PORT") != npos) {
      inPort = true;
      layer = -1;
      continue;
    }
    if (line.find("OBS") != npos) {
      inObs = true;
      continue;
    }
    if (inPort && curr_pin) {
      if (line.find("LAYER") != npos) {
        ss >> str >> str;
        layer = lf.getLayerIndex(str);
        continue;
      }
      if (line.find("RECT") != npos) {
        double llx{0}, lly{0}, urx{0}, ury{0};
        ss >> str >> llx >> lly >> urx >> ury;
        if (layer > 0) {
          curr_pin->addRect(layer, Geom::Rect(llx * units, lly * units, urx * units, ury * units));
        }
        continue;
      }
    }
    if (inObs && curr_module) {
      if (line.find("LAYER") != npos) {
        ss >> str >> str;
        layer = lf.getLayerIndex(str);
        continue;
      }
      if (line.find("RECT") != npos) {
        double llx{0}, lly{0}, urx{0}, ury{0};
        ss >> str >> llx >> lly >> urx >> ury;
        if (layer > 0) {
          curr_module->addObstacle(layer, Geom::Rect(llx * units, lly * units, urx * units, ury * units));
        }
        continue;
      }
    }
  }
  ifs.close();
}

void Netlist::readNDR(const std::string& ndrfile, const DRC::LayerInfo& lf)
{
  if (!ndrfile.empty()) {
    std::ifstream ifs(ndrfile);
    if (!ifs) {
      CERR << "unable to open NDR file " << ndrfile <<std::endl;
      _valid = 0;
      return;
    }
    ordered_json oj = json::parse(ifs);
    for (auto& m : oj) {
      auto it = m.find("module");
      if (it != m.end()) {
        auto modit = _modules.find(*it);
        it = m.find("nets");
        if (modit != _modules.end() && it != m.end()) {
          for (auto& netiter : *it) {
            auto itnetname = netiter.find("name");
            const std::string wsd[] = {"widths", "spaces", "directions"};
            for (auto iwsd : {0, 1, 2}) {
              auto itwsd = netiter.find(wsd[iwsd]);
              if (itnetname != netiter.end() && itwsd != netiter.end()) {
                for (auto& el : (*itwsd).items()) {
                  auto layer = lf.getLayerIndex(el.key());
                  COUT << "ndr : " << wsd[iwsd] << ' ' << el.key() << ' ' << el.value() << '\n';
                  if (layer >= 0) {
                    switch (iwsd) {
                      default:
                      case 0: modit->second->addNDRWidth(*itnetname, layer, el.value());
                              break;
                      case 1: modit->second->addNDRSpace(*itnetname, layer, el.value());
                              break;
                      case 2: modit->second->addNDRDir(*itnetname, layer, el.value());
                              break;
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }
}

}
