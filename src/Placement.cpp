#include "Util.h"
#include "Placement.h"
#include "Escape.h"

#include <algorithm>

namespace Placement {

void Port::print() const
{
  COUT << "port : " << _name << '\n';
  for (const auto& l : _shapes) {
    COUT << "\tlayer : " << l.first << '\n';
    for (const auto& r : l.second) {
      COUT << "\t\t" << r.str() << '\n';
    }
  }
}

void Port::addRect(const int layer, const Geom::Rect& r)
{
  auto it = _shapes.find(layer);
  if (it != _shapes.end()) {
    bool pushed{false};
    for (auto& s : it->second) {
      if (r.contains(s)) {
        pushed = true;
        s = r;
        break;
      }
      if (s.contains(r)) {
        pushed = true;
        break;
      }
      if (r.overlaps(s)) {
        if (r.xmin() == s.xmin() && r.xmax() == s.xmax()) {
          s.ymin() = std::min(s.ymin(), r.ymin());
          s.ymax() = std::max(s.ymax(), r.ymax());
          pushed = true;
        } else if (r.ymin() == s.ymin() && r.ymax() == s.ymax()) {
          s.xmin() = std::min(s.xmin(), r.xmin());
          s.xmax() = std::max(s.xmax(), r.xmax());
          pushed = true;
        }
      }
    }
    if (!pushed) it->second.push_back(r);
  } else {
    _shapes[layer].push_back(r);
  }
  _bbox.merge(r);
}

Port* Port::getTransformedPort(const Geom::Transform& tr) const
{
  Port* port = new Port(_name);
  for (const auto& ls : _shapes) {
    for (const auto& s : ls.second) {
      port->_shapes[ls.first].push_back(tr.transform(s));
    }
  }
  port->_bbox = tr.transform(_bbox);
  return port;
}

void Pin::print() const
{
  COUT << "pin : " << _name << '\n';
  for (auto& p : _ports) p->print();
}

Module::~Module()
{
  for (auto& p : _pins) delete p.second;
  for (auto& i : _instances) delete i;
  _pins.clear();
  _instances.clear();
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

void Module::route(Router::Router& router, const std::string& outdir)
{
  TIME_M();
  if (!_routed) {
    router.setuu(_uu);
    //writeDEF("_before");
    router.clearObstacles();
    router.clearObstacles(true);
    for (auto& inst : _instances) {
      auto m = inst->module();
      if (!m->routed()) {
        const_cast<Module*>(m)->route(router, outdir);
      }
      inst->build(true);
      for (const auto& l : m->obstacles()) {
        for (const auto& r : l.second) {
          _obstacles[l.first].push_back(inst->transform(r));
        }
      }
      for (const auto& l : m->internalroutes()) {
        for (const auto& r : l.second) {
          _obstacles[l.first].push_back(inst->transform(r));
          _internalroutes[l.first].push_back(inst->transform(r));
        }
      }
    }
    updateNets();
    {
      std::set<const Pin*> connectedPins;
      for (auto& n : _nets) {
        for (auto& p : n.second.pins()) connectedPins.insert(p);
      }
      for (auto& inst : _instances) {
        for (auto& pp : inst->_pins) {
          const Pin* pin = pp.second;
          if (connectedPins.count(pin)) continue;
          for (auto& port : pin->ports()) {
            for (auto& l : port->shapes()) {
              for (auto& r : l.second) {
                COUT << "protecting unconnected pin " << pin->name() << " : adding obstacle layer "
                     << l.first << ' ' << r.str() << '\n';
                _obstacles[l.first].push_back(r);
              }
            }
          }
        }
      }
    }
    NetsVec nets;
    for (auto &n : _nets) {
      if (std::find(_routeorder.begin(), _routeorder.end(), &n.second) == _routeorder.end()) {
        nets.push_back(&n.second);
      }
    }
    std::sort(nets.begin(), nets.end(), [](const Net* a, const Net* b) -> bool
        { return a->halfpm() < b->halfpm(); });
    nets.insert(nets.begin(), _routeorder.begin(), _routeorder.end());
    COUT << " routing : " << _name << "; num nets : " << nets.size() << "; use pin width : " << ((_usepinwidth == 1) ? 1 : 0) << '\n';
    //router.addObstacles(_obstacles);
    router.setModName(_name);
    COUT << "setting module name : " << _name << '\n';
    router.setusepinwidth((_usepinwidth == 1) ? true : false);
    static std::set<std::string> debugnet(splitString((getenv("HANAN_DEBUG_NET") ? std::string(getenv("HANAN_DEBUG_NET")) : std::string("")), ','));
    static const int m1Layer = []() {
      for (int i = 0; i < static_cast<int>(LAYER_NAMES.size()); ++i) {
        if (LAYER_NAMES[i] == "M1") return i;
      }
      return -1;
    }();

    auto routeAllNets = [&](const bool addAdjObstacles) -> bool {
      Geom::LayerRects netObstaclesRouted, netObstaclesUnrouted;
      bool anyUnrouted{false};
      for (auto it = nets.begin(); it != nets.end(); ++it) {
        netObstaclesUnrouted.clear();
        for (auto itn = nets.begin(); itn != it; ++itn) {
          if ((*itn)->excluded()) {
            for (auto virt : {true, false}) {
              const auto& pins = virt ? (*itn)->virtualpins() : (*itn)->pins();
              for (auto& pin : pins) {
                for (auto& p : pin->ports()) {
                  Geom::MergeLayerRects(netObstaclesUnrouted, p->shapes());
                }
              }
            }
          }
        }
        for (auto itn = std::next(it); itn != nets.end(); ++itn) {
          for (auto virt : {true, false}) {
            const auto& pins = virt ? (*itn)->virtualpins() : (*itn)->pins();
            for (auto& pin : pins) {
              for (auto& p : pin->ports()) {
                Geom::MergeLayerRects(netObstaclesUnrouted, p->shapes());
              }
            }
          }
        }
        if (addAdjObstacles && m1Layer >= 0 && layerName(m1Layer + 1)[0] == 'M') {
          auto itm1 = netObstaclesUnrouted.find(m1Layer);
          if (itm1 != netObstaclesUnrouted.end() && !itm1->second.empty()) {
            auto& adj = netObstaclesUnrouted[m1Layer + 1];
            adj.insert(adj.end(), itm1->second.begin(), itm1->second.end());
          }
        }
        router.setNetName((*it)->name());
        if (debugnet.find(_name + "__" + (*it)->name()) != debugnet.end()
            || debugnet.find((*it)->name()) != debugnet.end()
            || debugnet.find(_name) != debugnet.end()) {
          router.setEnableDebug(true);
        } else {
          router.setEnableDebug(false);
        }
        (*it)->route(router, netObstaclesRouted, netObstaclesUnrouted, _obstacles, true, _uu, _bbox, _name);
        //writeDEF("_" + (*it)->name(), (*it)->name());
        if ((*it)->unrouted()) anyUnrouted = true;
        Geom::MergeLayerRects(netObstaclesRouted, (*it)->routeShapesWithPins());
      }
      return anyUnrouted;
    };

    {
      std::vector<Escape::Pin> epins;
      int netid = 0;
      for (auto& nv : nets) {
        if (!nv->excluded() && nv->pins().size() >= 2) {
          for (auto& pin : nv->pins()) {
            Escape::Pin ep;
            ep.name = _name + SEPARATOR + pin->name();
            ep.net = netid;
            for (auto& port : pin->ports()) {
              for (auto& l : port->shapes()) {
                for (auto& r : l.second) ep.shapes[l.first].push_back(r);
              }
            }
            if (!ep.shapes.empty()) epins.push_back(std::move(ep));
          }
        }
        ++netid;
      }
      if (!epins.empty()) {
        Escape::LayerModel lm;
        lm.minLayer = router.minLayer();
        lm.maxLayer = router.maxLayer();
        lm.width   = [&router](int z) { return std::max(router.baseWidthX(z), router.baseWidthY(z)); };
        lm.space   = [&router](int z) { return std::max(router.baseSpaceX(z), router.baseSpaceY(z)); };
        lm.canUp   = [&router](int z) { return router.canViaUp(z); };
        lm.canDown = [&router](int z) { return router.canViaDown(z); };
        std::vector<std::string> blocked;
        std::string reason;
        if (Escape::feasible(epins, _obstacles, lm, &blocked, &reason)) {
          COUT << "pin escape SAT : all " << epins.size() << " pins in " << _name
               << " have a guaranteed escape\n";
        } else {
          COUT << "pin escape SAT : " << _name << " is infeasible (" << reason << ")\n";
          for (auto& b : blocked) COUT << "  no escape for pin : " << b << '\n';
        }
      }
    }

    for (auto& n : _nets) n.second.snapshotRoutes();
    if (routeAllNets(false)) {
      COUT << "module " << _name << " has unrouted nets; retrying with adjacent-layer pin obstacles\n";
      for (auto& n : _nets) n.second.clearRoutes();
      routeAllNets(true);
    }
    router.clearObstacles();
    std::set<std::string> _addednets;
    for (auto& p : _pins) {
      auto itn = _nets.find(p.first);
      //COUT << "DEBUG pin name " << p.first << '\n';
      if (itn != _nets.end()) {
        _addednets.insert(itn->first);
        //COUT << "DEBUG found net : " << itn->second.name() << ' ' << itn->second.routeShapesWithPins().size() << '\n';
        if (!itn->second.excluded()) p.second->copyRects(itn->second.routeShapesWithPins());
        else {
          COUT << "excluded : " << itn->second.name() << "\n";
          for (auto& pin : itn->second.pins()) {
            COUT << "pin : " << pin->name() << '\n';
            for (auto& port : pin->ports()) {
              COUT << "port : " << port->name() << '\n';
              p.second->copyRects(port->shapes(), true);
            }
          }
        }
      }
    }
    for (auto& n : _nets) {
      if (_addednets.find(n.first) == _addednets.end()) {
        //COUT << "unadded net : " << n.first << '\n';
        Geom::MergeLayerRects(_internalroutes, n.second.routeShapesWithPins());
      }
    }
    writeDEF(outdir);
  }
  if (!_leaf) {
    writeLEF(outdir);
  }
  _routed = 1;
  checkShort();
}

void Module::checkShort() const
{
  COUT << "Checking SHORTS for module : " << _name << '\n';
  for (auto it1 = _nets.begin(); it1 != _nets.end(); ++it1) {
    for (auto it2 = std::next(it1); it2 != _nets.end(); ++it2) {
      auto& s1 = it1->second.routeShapesWithPins();
      auto& s2 = it2->second.routeShapesWithPins();
      for (auto& l : s1) {
        auto its2 = s2.find(l.first);
        if (its2 == s2.end()) continue;
        for (auto& o1 : l.second) {
          for (auto& o2 : its2->second) {
            if (o1.overlaps(o2) && o1 != o2) {
              COUT << "SHORT (router or pin) between " << it1->second.name() << " & " << it2->second.name() << " @ layer : " << l.first << '\n';
              COUT << o1.str() << ' ' << o2.str() << '\n';
            }
          }
        }
      }
    }
  }
  for (auto it1 = _nets.begin(); it1 != _nets.end(); ++it1) {
    auto& s1 = it1->second.routeShapesWithPins();
    auto& s2 = _obstacles;
    for (auto& l : s1) {
      auto its2 = s2.find(l.first);
      if (its2 == s2.end()) continue;
      for (auto& o2 : its2->second) {
        bool obsPinOverlapping{false};
        for (auto& pin : _pins) {
          for (auto& p : pin.second->ports()) {
            const auto& s3 = p->shapes();
            auto its3 = s3.find(l.first);
            if (its3 == s3.end()) continue;
            for (auto& o3 : its3->second) {
              if (o3.overlaps(o2)) {
                obsPinOverlapping = true;
                break;
              }
            }
          }
        }
        if (obsPinOverlapping) continue;
        for (auto& o1 : l.second) {
          if (o1.overlaps(o2) && o1 != o2) {
            COUT << "SHORT between " << it1->second.name() << " & obstacle @ layer : " << l.first << '\n';
            COUT << o1.str() << ' ' << o2.str() << '\n';
          }
        }
      }
    }
  }
}

void Module::writeDEF(const std::string& outdir, const std::string& nstr, const std::string& netname) const
{
  std::ofstream ofs(outdir + _name + nstr + ".def");
  if (ofs.is_open()) {
    ofs << "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN " << _name << " ;\n";
    ofs << "UNITS DISTANCE MICRONS " << _uu << " ;\n";
    ofs << "DIEAREA ( " << _bbox.xmin() << ' ' << _bbox.ymin() << " ) ( " << _bbox.xmax() << ' ' << _bbox.ymax() << " ) ; \n\n";
    if (!_instances.empty() || !netname.empty()) {
      ofs << "COMPONENTS " << (_instances.size() + !netname.empty()) << " ;\n";
      for (auto& inst : _instances) {
        auto& tr = inst->transform();
        ofs << "- " << inst->name() << ' ' << inst->moduleName();
        ofs << " + PLACED ( ";
        ofs << ((tr.sX() > 0 ) ? tr.x() : (tr.x() - inst->bbox().width()))  << ' ';
        ofs << ((tr.sY() > 0 ) ? tr.y() : (tr.y() - inst->bbox().height())) << " ) " << tr.orient() << " ;\n";
      }
      if (!netname.empty()) {
        ofs << "- " << _name << '_' << netname << "_0 " << _name << '_' << netname;
        ofs << " + PLACED ( 0 0 ) N ;\n";
      }
      ofs << "END COMPONENTS\n\n";
    }
    if (!_nets.empty()) {
      ofs << "NETS " << _nets.size() << " ;\n ";
      for (auto& n : _nets) {
        ofs << "- " << n.first << "\n";
        for (auto& p : n.second.pins()) {
          std::string instname = p->name();
          std::string pinname  = p->name();
          auto ppos = p->name().rfind('+');
          if (ppos != std::string::npos) {
            instname = p->name().substr(0, ppos);
            pinname  = p->name().substr(ppos + 1);
          }
          ofs << " ( " << instname << ' ' << pinname << " )";
        }
        auto& routeShapes = n.second.routeShapes();
        if (!routeShapes.empty()) {
          ofs << "\n";
          for (auto& l : routeShapes) {
            for (auto& r : l.second) {
              ofs << "  + RECT " << layerName(l.first);
              ofs << " ( " << r.xmin() << ' ' << r.ymin() << " ) ( " << r.xmax() << ' ' << r.ymax() << " )\n";
            }
          }
        } else {
          ofs << "\n";
          for (auto& pin : n.second.pins()) {
            for (auto& p : pin->ports()) {
              const auto& shapes = p->shapes();
              if (!shapes.empty()) {
                for (auto& l : shapes) {
                  if (layerName(l.first)[0] == 'M') {
                    ofs << "  + RECT " << layerName(l.first);
                    for (auto& r : l.second) {
                      ofs << " ( " << r.xmin() << ' ' << r.ymin() << " ) ( " << r.xmax() << ' ' << r.ymax() << " )\n";
                      break;
                    }
                    break;
                  }
                }
              }
            }
          }
        }
        ofs << " ;\n";
      }
      ofs << "END NETS\n\n";
    }
    /*if (!_internalroutes.empty()) {
      ofs << "FILLS " << _internalroutes.size() << " ;\n ";
      for (auto& l : _internalroutes) {
        ofs << "  - LAYER " << layerName(l.first) << "\n";
        for (unsigned i = 0; i < l.second.size(); ++i) {
          auto& r = l.second[i];
          ofs << "    RECT ( " << r.xmin() << ' ' << r.ymin() << " ) ( " << r.xmax() << ' ' << r.ymax() << " )";
          if (i == l.second.size() - 1) ofs << " ;\n";
          else ofs << "\n";
        }
      }
      ofs << "END FILLS\n\n";
    }*/
    ofs << "END DESIGN\n";
  }
}

void Module::writeLEF(const std::string& outdir) const
{
  std::ofstream ofs(outdir + _name + "_interim_hier.lef");
  if (ofs.is_open()) {
    ofs << "MACRO " << _name << "\n";
    ofs << "  UNITS\n    DISTANCE MICRONS " << _uu << ";\n  END UNITS\n";
    ofs << "  ORIGIN "  << _bbox.xmin()  << ' ' << _bbox.ymin() << " ;\n";
    ofs << "  FOREIGN " << _name << ' '  << (1.*_bbox.xmin()/_uu) << ' ' << (1.*_bbox.ymin()/_uu) << " ;\n";
    ofs << "  SIZE "    << (1.*_bbox.width()/_uu) << " BY " << (1.* _bbox.height()/_uu) << " ;\n";
    if (!_pins.empty()) {
      for (auto& pin : _pins) {
        ofs << "  PIN " << pin.first << "\n    DIRECTION INOUT ;\n    USE SIGNAL ;\n";
        for (auto& p : pin.second->ports()) {
          const auto& shapes = p->shapes();
          if (!shapes.empty()) {
            ofs << "    PORT\n";
            for (auto& l : shapes) {
              ofs << "      LAYER " << layerName(l.first) << " ;\n";
              for (auto& r : l.second) {
                ofs << "        RECT " << (r.xmin()*1.0/_uu) << ' ' << (1.*r.ymin()/_uu) << ' ' << (1.*r.xmax()/_uu) << ' ' << (1.*r.ymax()/_uu) << " ;\n";
              }
            }
            ofs << "    END\n";
          }
        }
        ofs << "  END " << pin.first << '\n';
      }
    }
    if (!_obstacles.empty() || !_internalroutes.empty()) {
      ofs << "    OBS\n";
      for (auto& l : _obstacles) {
        ofs << "      LAYER " << layerName(l.first) << " ;\n";
        for (auto& r : l.second) {
          ofs << "        RECT " << (1.*r.xmin()/_uu) << ' ' << (1.*r.ymin()/_uu) << ' ' << (1.*r.xmax()/_uu) << ' ' << (1.*r.ymax()/_uu) << " ;\n";
        }
      }
      for (auto& l : _internalroutes) {
        ofs << "      LAYER " << layerName(l.first) << " ;\n";
        for (auto& r : l.second) {
          ofs << "        RECT " << (1.*r.xmin()/_uu) << ' ' << (1.*r.ymin()/_uu) << ' ' << (1.*r.xmax()/_uu) << ' ' << (1.*r.ymax()/_uu) << " ;\n";
        }
      }
      ofs << "    END\n";
    }
    ofs << "END " << _name << "\n";
  }
}

}
