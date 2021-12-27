#include "Placement.h"
#include <fstream>
#include <iostream>
#include <sstream>

namespace Placement {

const auto& npos = std::string::npos;

void Net::print() const
{
  std::cout << "pins :";
  for (const auto& p : _pins) {
    std::cout << " " << p->name();
  }
}


Module::~Module()
{
  for (auto& p : _pins) delete p.second;
  _pins.clear();
}


void Module::print() const
{
  for (const auto& p : _pins) {
    std::cout << "\tpin : " << p.first << '\n';
    for (const auto& l : p.second->shapes()) {
      std::cout << "\t\tlayer : " << l.first << '\n';
      for (const auto& r : l.second) {
        std::cout << "\t\t\t" << r.str() << '\n';
      }
    }
  }
  for (const auto& n : _nets) {
    std::cout << "\tnet : " << n.first << " : {";
    n.second.print();
    std::cout << "}\n";
  }
  for (const auto& inst : _instances) {
    std::cout << "\tinst : " ;
    inst->print("\t");
  }
  for (const auto& l : _obstacles) {
    std::cout << "\tobstacle : layer : " << l.first;
    for (const auto& r : l.second) {
      std::cout << "\t\t" << r.str() << '\n';
    }
  }
}

void Module::build()
{
  for (auto& i : _instances) i->build();
  for (auto& t : _tmpnetpins) {
    for (auto& instpin : t.second) {
      auto it = instpin.first->_pins.find(instpin.second);
      if (it != instpin.first->_pins.end()) {
        const_cast<Net*>(t.first)->addPin(it->second);
      }
    }
  }
  _tmpnetpins.clear();
}


void Module::route()
{
  if (!_routed) {
    for (auto& inst : _instances) {
      auto m = inst->module();
      if (!m->routed()) {
        const_cast<Module*>(m)->route();
      }
      for (const auto& l : m->obstacles()) {
        for (const auto& r : l.second) {
          _obstacles[l.first].push_back(inst->transform(r));
        }
      }
    }
    std::cout << " routing : " << _name << std::endl;
  }
  _routed = 1;
}


void Module::plot() const
{
}


Instance::~Instance()
{
  for (auto& p : _pins) delete p.second;
  _pins.clear();
}


void Instance::build()
{
  if (_m) {
    for (auto& p : _m->pins()) {
      auto ip = new Pin(_name + "/" + p.second->name());
      _pins[p.second->name()] = ip;
      for (const auto& ls : p.second->shapes()) {
        for (const auto& s : ls.second) {
          ip->addRect(ls.first, _tr.transform(s));
        }
      }
    }
  }
}


void Instance::print(const std::string& prefix) const
{
  std::cout << prefix << "name : " << _name << " module : " << _modname << '\n';
  std::cout << prefix << "\ttr : " << _tr.str() << '\n';
  for (const auto& p : _pins) {
    std::cout << prefix << "\tpin : " << p.first << '\n';
    for (const auto& l : p.second->shapes()) {
      std::cout << prefix << "\t\tlayer : " << l.first << '\n';
      for (const auto& r : l.second) {
        std::cout << prefix << "\t\t\t" << r.str() << '\n';
      }
    }
  }
}

using json = nlohmann::json;
using ordered_json = nlohmann::ordered_json;
Netlist::Netlist(const std::string& plfile, const::std::string& leffile, const DRC::LayerInfo& lf, const int uu) : _uu(uu)
{
  if (plfile.empty()) return;
  std::ifstream ifs(plfile);
  if (!ifs) return;
  ordered_json oj = json::parse(ifs);
  ifs.close();
  auto it = oj.find("leaves");
  if (it != oj.end()) {
    for (auto& l : *it) {
      auto lname = l.find("abstract_name");
      if (_modules.find(*lname) != _modules.end()) continue;
      if (lname != l.end()) {
        auto modu = new Module(*lname, 1);
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
      auto mname = m.find("abstract_name");
      if (mname != m.end()) {
        if (_modules.find(*mname) != _modules.end()) continue;
        auto modu = new Module(*mname, 0);
        auto params = m.find("parameters");
        if (params != m.end()) {
          for (auto& p : *params) {
            modu->addPin(p);
          }
        }
        auto insts = m.find("instances");
        if (insts != m.end()) {
          for (auto& inst : *insts) {
            auto iname = inst.find("instance_name");
            auto mname = inst.find("abstract_template_name");
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
              std::cout << "instptr nullptr\n";
            }
          }
        }
        _modules[modu->name()] = modu;
      }
    }
  }
  loadLEF(leffile, lf);
  build();
}


Netlist::~Netlist()
{
  for (auto& m : _modules) delete m.second;
  _modules.clear();
}


void Netlist::print() const
{
  for (const auto& m : _modules) {
    std::cout << "module : " << m.second->name() << '\n';
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
    std::cerr << "missing leffile" <<std::endl;
    return;
  }
  std::ifstream ifs(leffile);
  if (!ifs) {
    std::cerr << "unable to open leffile " << leffile <<std::endl;
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
      auto it = _modules.find(macroName);
      if (it != _modules.end()) {
        curr_module =  it->second;
      }
      inMacro = true;
      continue;
    }
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
    if (inMacro && curr_module && line.find("PIN") != npos) {
      ss >> str >> pinName;
      curr_pin = curr_module->getPin(pinName);
      inPin = true;
      continue;
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

};
