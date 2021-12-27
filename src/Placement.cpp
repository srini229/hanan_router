#include "Placement.h"
#include <fstream>
#include <iostream>

namespace Placement {

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
  std::cout << "\tpins : [";
  for (const auto& p : _pins) {
    std::cout << " " << p.first;
  }
  std::cout << " ]\n";
  for (const auto& n : _nets) {
    std::cout << "\tnet : " << n.first << " : {";
    n.second.print();
    std::cout << "}\n";
  }
  for (const auto& inst : _instances) {
    std::cout << "\tinst : " << inst->name() << ' ' << inst->moduleName() << ' ' << inst->transform().str() << '\n';
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


using json = nlohmann::json;
using ordered_json = nlohmann::ordered_json;
Netlist::Netlist(const std::string& plfile)
{
  if (plfile.empty()) return;
  std::ifstream ifs(plfile);
  if (!ifs) return;
  ordered_json oj = json::parse(ifs);
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

};
