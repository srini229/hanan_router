#include "Util.h"
#include "Placement.h"
#include <sstream>

#include "nlohmann/json.hpp"

namespace Placement {

const auto& npos = std::string::npos;

inline void Net::print() const
{
  COUT << "pins :";
  for (const auto& p : _pins) {
    std::cout << " " << p->name();
  }
}


void Net::route(const Geom::LayerRects& l1, const Geom::LayerRects& l2, const Geom::LayerRects& l3)
{
  COUT << "routing net : " << _name << '\n';
  for (auto& pin : _pins) {
    Geom::MergeLayerRects(_routeshapes, pin->shapes(), &_bbox);
  }
}


Module::~Module()
{
  for (auto& p : _pins) delete p.second;
  for (auto& i : _instances) delete i;
  _pins.clear();
  _instances.clear();
}


void Module::print() const
{
  for (const auto& p : _pins) {
    COUT << "\tpin : " << p.first << '\n';
    for (const auto& l : p.second->shapes()) {
      std::cout << "\t\tlayer : " << l.first << '\n';
      for (const auto& r : l.second) {
        std::cout << "\t\t\t" << r.str() << '\n';
      }
    }
  }
  for (const auto& n : _nets) {
    COUT << "\tnet : " << n.first << " : {";
    n.second.print();
    std::cout << "}\n";
  }
  for (const auto& inst : _instances) {
    COUT << "\tinst : " ;
    inst->print("\t");
  }
  for (const auto& l : _obstacles) {
    COUT << "\tobstacle : layer : " << l.first;
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
    Geom::LayerRects _netObstaclesRouted, _netObstaclesUnrouted;
    for (auto it = _nets.begin(); it != _nets.end(); ++it) {
      _netObstaclesUnrouted.clear();
      for (auto itn = std::next(it); itn != _nets.end(); ++itn) {
        for (auto& p : itn->second.pins()) {
          Geom::MergeLayerRects(_netObstaclesUnrouted, p->shapes());
        }
      }
      it->second.route(_obstacles, _netObstaclesRouted, _netObstaclesUnrouted);
      Geom::MergeLayerRects(_netObstaclesRouted, it->second.routeShapes());
    }
    COUT << " routing : " << _name << '\n';
    for (auto& p : _pins) {
      auto itn = _nets.find(p.first);
      std::cout << "DEBUG pin name " << p.first << '\n';
      if (itn != _nets.end()) {
        std::cout << "DEBUG found net : " << itn->second.name() << '\n';
        p.second->copyRects(itn->second.routeShapes());
      }
    }
  }
  _routed = 1;
}


void Module::plot() const
{
  std::ofstream ofs(_name + ".gplt");
  if (ofs.is_open()) {
    COUT << "plotting module " << _name << " to " << _name << ".gplt\n";
    ofs << "unset key\n";
    ofs << "set title '" << _name << "' noenhanced\n";
    unsigned cnt{1};
    for (auto& l : _obstacles) {
      const auto& color = LAYER_COLORS[l.first % LAYER_COLORS.size()];
      for (auto& b : l.second) {
        if (b.valid() && b.width() && b.height()) {
          ofs << "set object " << cnt++ << " rect from ";
          ofs << b.xmin() << "," << b.ymin() << " to " << b.xmax() << "," << b.ymax() << " fillstyle transparent solid 0.5 fillcolor \"" << color << "\" behind\n";
        }
      }
    }
    for (const auto& p : _pins) {
      auto& b = p.second->bbox();
      if (b.valid()) {
        ofs << "set object " << cnt++ << " rect from ";
        ofs << b.xmin() << "," << b.ymin() << " to " << b.xmax() << "," << b.ymax() << " fillcolor 'black' fillstyle pattern " << ((cnt % 2) + 5) << " transparent behind\n";
        ofs << "set label \"" << p.second->name() << "\" at " << b.xcenter() << "," << b.ycenter() << " center noenhanced\n";
      }
    }
    for (auto& i : _instances) {
      auto& b = i->bbox();
      if (b.valid()) {
        ofs << "set label \"" << i->name() << "\" at " << b.xcenter() << "," << b.ycenter() << " center tc lt 3 font \",15\" noenhanced\n";
      }
      for (const auto& p : i->pins()) {
        auto& b = p.second->bbox();
        if (b.valid()) {
          ofs << "set object " << cnt++ << " rect from ";
          ofs << b.xmin() << "," << b.ymin() << " to " << b.xmax() << "," << b.ymax() << " fillcolor 'black' fillstyle pattern 2 transparent behind\n";
          ofs << "set label \"" << p.second->name() << "\" at " << b.xcenter() << "," << b.ycenter() << " center noenhanced\n";
        }
      }
    }
    auto& b = _bbox;
    ofs << "plot[:][:] '-' using 1:2 w l lt -1 lw 2 lc -1, '-' using 1:2 w l lt 1 lw 2 lc 1\n";
    if (b.valid()) {
      ofs << b.xmin() << " " << b.ymin() << "\n";
      ofs << b.xmax() << " " << b.ymin() << "\n";
      ofs << b.xmax() << " " << b.ymax() << "\n";
      ofs << b.xmin() << " " << b.ymax() << "\n";
      ofs << b.xmin() << " " << b.ymin() << "\n\n";
    }
    for (auto& i : _instances) {
      auto& b = i->bbox();
      if (b.valid()) {
        ofs << b.xmin() << " " << b.ymin() << "\n";
        ofs << b.xmax() << " " << b.ymin() << "\n";
        ofs << b.xmax() << " " << b.ymax() << "\n";
        ofs << b.xmin() << " " << b.ymax() << "\n";
        ofs << b.xmin() << " " << b.ymin() << "\n\n";
      }
    }
    ofs << "EOF\n";
    for (auto& n : _nets) {
      for (auto& p : n.second.pins()) {
        auto& b = p->bbox();
        if (b.valid()) {
          ofs << b.xcenter() << " " << b.ycenter() << "\n";
        }
      }
      ofs << "\n";
    }
    ofs << "EOF\n";
    ofs << "set size ratio GPVAL_DATA_Y_MAX/GPVAL_DATA_X_MAX\nrepl\npause -1";
  }
  ofs.close();
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
  COUT << prefix << "name : " << _name << " module : " << _modname << '\n';
  COUT << prefix << "\ttr : " << _tr.str() << '\n';
  for (const auto& p : _pins) {
    COUT << prefix << "\tpin : " << p.first << '\n';
    for (const auto& l : p.second->shapes()) {
      COUT << prefix << "\t\tlayer : " << l.first << '\n';
      for (const auto& r : l.second) {
        COUT << prefix << "\t\t\t" << r.str() << '\n';
      }
    }
  }
}


void Instance::setModule(const Module* m)
{
  _m = m;
  _bbox = _tr.transform(m->bbox());
}


using json = nlohmann::json;
using ordered_json = nlohmann::ordered_json;
Netlist::Netlist(const std::string& plfile, const::std::string& leffile, const DRC::LayerInfo& lf, const int uu) : _uu(uu)
{
  if (plfile.empty()) {
    CERR<< "missing placement file" <<std::endl;
    return;
  }
  std::ifstream ifs(plfile);
  if (!ifs) {
    CERR << "unable to open placement file " << plfile <<std::endl;
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
        auto modu = new Module(*lname, 1);
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
        auto modu = new Module(*mname, 0);
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
    return;
  }
  std::ifstream ifs(leffile);
  if (!ifs) {
    CERR << "unable to open leffile " << leffile <<std::endl;
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

};
