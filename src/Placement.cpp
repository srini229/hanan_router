#include "Util.h"
#include "Placement.h"

#include <algorithm>

namespace Placement {

inline void Net::print() const
{
  COUT << "pins :";
  for (const auto& p : _pins) {
    std::cout << " " << p->name();
  }
}

PinCVec Net::reorderPins() const
{
  PinCVec pins(_pins.begin(), _pins.end());
  return pins;
}

void Net::route(Router::Router& router, const Geom::LayerRects& l1, const Geom::LayerRects& l2)
{
  TIME_M();
  _unroute = 0;
  for (auto& p : _pins) {
    Geom::MergeLayerRects(_routeshapes, p->shapes(), &_bbox);
  }
  if (_pins.size() > 1) {
    COUT << "routing net : " << _name << ' ' << halfpm() << '\n';
    /*for (int i : {0, 1}) {
      for (auto& l : (i ? l1 : l2)) {
        for (auto& o : l.second) {
          COUT << "obs : " << l.first << ' ' << o.str() << '\n';
        }
      }
    }*/
    router.addObstacles(l1, true);
    router.addObstacles(l2, true);
    auto pins = reorderPins();
    auto it1 = pins.rbegin();
    auto it2 = std::next(it1);
    while (it2 != pins.rend()) {
      router.clearSourceTargets();
      COUT << "routing pins : " << (*it1)->name() << ' ' << (*it2)->name() << '\n';
      router.setName(_name + "__" + (*it1)->name() + "__" + (*it2)->name());
      const auto& p1 = (*it1)->shapes();
      const auto& p2 = (*it2)->shapes();
      for (auto src : {true, false}) {
        for (auto& l : (src ? p1 : p2)) {
          if (l.first > router.maxLayer() || l.first < router.minLayer()) continue;
          for (auto& s : l.second) {
            if (src) {
              router.addSource(s, l.first);
            } else {
              router.addTarget(s, l.first);
            }
          }
        }
      }
      for (auto& l : p1) {
        auto it = p2.find(l.first);
        if (it != p2.end()) {
          for (auto& s1 : l.second) {
            for (auto& s2 : it->second) {
              if (s1.xmin() < s2.xmax() && s1.xmax() > s2.xmin()) {
                int xmin(std::max(s1.xmin(), s2.xmin())), xmax(std::min(s1.xmax(), s2.xmax()));
                if (xmax - xmin >= router.widthy(l.first)) {
                  router.addSource(Geom::Rect(xmin, s1.ymin(), xmax, s1.ymax()), l.first);
                  router.addTarget(Geom::Rect(xmin, s2.ymin(), xmax, s2.ymax()), l.first);
                }
              } else if (s1.ymin() < s2.ymax() && s1.ymax() > s2.ymin()) {
                int ymin(std::max(s1.ymin(), s2.ymin())), ymax(std::min(s1.ymax(), s2.ymax()));
                if (ymax - ymin >= router.widthx(l.first)) {
                  router.addSource(Geom::Rect(s1.xmin(), ymin, s1.xmax(), ymax), l.first);
                  router.addTarget(Geom::Rect(s2.xmin(), ymin, s2.xmax(), ymax), l.first);
                }
              }
            }
          }
        }
      }
      auto sol = router.findSol();
      if (!sol.empty()) {
        Geom::MergeLayerRects(_routeshapes, sol, &_bbox);
      } else {
        _unroute = 1;
      }
      Geom::MergeLayerRects(const_cast<Geom::LayerRects&>((*it1)->shapes()), sol, &_bbox);
      Geom::MergeLayerRects(const_cast<Geom::LayerRects&>((*it2)->shapes()), sol, &_bbox);
      it1 = it2;
      ++it2;
    }
    router.clearObstacles(true);
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

void Module::route(Router::Router& router)
{
  if (!_routed) {
    writeDEF("_before");
    router.clearObstacles();
    router.clearObstacles(true);
    for (auto& inst : _instances) {
      auto m = inst->module();
      if (!m->routed()) {
        const_cast<Module*>(m)->route(router);
      }
      inst->build(true);
      for (const auto& l : m->obstacles()) {
        for (const auto& r : l.second) {
          _obstacles[l.first].push_back(inst->transform(r));
        }
      }
    }
    updateNets();
    NetsVec nets;
    for (auto &n : _nets) nets.push_back(&n.second);
    std::sort(nets.begin(), nets.end(), [](const Net* a, const Net* b) -> bool
        { return a->halfpm() < b->halfpm(); });
    COUT << " routing : " << _name << '\n';
    router.addObstacles(_obstacles);
    Geom::LayerRects _netObstaclesRouted, _netObstaclesUnrouted;
    for (auto it = nets.begin(); it != nets.end(); ++it) {
      _netObstaclesUnrouted.clear();
      for (auto itn = std::next(it); itn != nets.end(); ++itn) {
        for (auto& p : (*itn)->pins()) {
          Geom::MergeLayerRects(_netObstaclesUnrouted, p->shapes());
        }
      }
      (*it)->route(router, _netObstaclesRouted, _netObstaclesUnrouted);
      Geom::MergeLayerRects(_netObstaclesRouted, (*it)->routeShapes());
    }
    router.clearObstacles();
    for (auto& p : _pins) {
      auto itn = _nets.find(p.first);
      std::cout << "DEBUG pin name " << p.first << '\n';
      if (itn != _nets.end()) {
        std::cout << "DEBUG found net : " << itn->second.name() << ' ' << itn->second.routeShapes().size() << '\n';
        p.second->copyRects(itn->second.routeShapes());
      }
    }
  }
  if (!_leaf) {
    writeDEF();
    writeLEF();
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
          ofs << b.xmin() << "," << b.ymin() << " to " << b.xmax() << "," << b.ymax() << " fillstyle transparent solid 0.25 fillcolor \"" << color << "\" behind\n";
        }
      }
    }
    for (const auto& p : _pins) {
      if (p.second->shapes().empty()) {
        auto& b = p.second->bbox();
        if (b.valid()) {
          ofs << "set object " << cnt++ << " rect from ";
          ofs << b.xmin() << "," << b.ymin() << " to " << b.xmax() << "," << b.ymax() << " fillcolor 'black' fillstyle pattern " << ((cnt % 2) + 5) << " transparent behind\n";
          ofs << "set label \"" << p.second->name() << "\" at " << b.xcenter() << "," << b.ycenter() << " center noenhanced\n";
        }
      }
    }
    for (auto& n : _nets) {
      for (auto& l : n.second.routeShapes()) {
        for (auto& b : l.second) {
          const auto& color = LAYER_COLORS[l.first % LAYER_COLORS.size()];
          if (b.valid()) {
            //ofs << b.xmin() << " " << b.ymin() << "\n";
            //ofs << b.xmax() << " " << b.ymin() << "\n";
            //ofs << b.xmax() << " " << b.ymax() << "\n";
            //ofs << b.xmin() << " " << b.ymax() << "\n";
            //ofs << b.xmin() << " " << b.ymin() << "\n\n";
            ofs << "set object " << cnt++ << " rect from ";
            ofs << b.xmin() << "," << b.ymin() << " to " << b.xmax() << "," << b.ymax() << " fillstyle transparent solid 0.25 fillcolor \"" << color << "\" behind\n";
          }
        }
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
      //if (!n.second.open()) continue;
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


void Module::checkShort() const
{
  for (auto it1 = _nets.begin(); it1 != _nets.end(); ++it1) {
    for (auto it2 = std::next(it1); it2 != _nets.end(); ++it2) {
      auto& s1 = it1->second.routeShapes();
      auto& s2 = it2->second.routeShapes();
      for (auto& l : s1) {
        auto its2 = s2.find(l.first);
        if (its2 == s2.end()) continue;
        for (auto& o1 : l.second) {
          for (auto& o2 : its2->second) {
            if (o1.overlaps(o2)) {
              COUT << "SHORT between " << it1->second.name() << " & " << it2->second.name() << " @ layer : " << l.first << '\n';
              COUT << o1.str() << ' ' << o2.str() << '\n';
            }
          }
        }
      }
    }
  }
  for (auto it1 = _nets.begin(); it1 != _nets.end(); ++it1) {
    auto& s1 = it1->second.routeShapes();
    auto& s2 = _obstacles;
    for (auto& l : s1) {
      auto its2 = s2.find(l.first);
      if (its2 == s2.end()) continue;
      for (auto& o1 : l.second) {
        for (auto& o2 : its2->second) {
          if (o1.overlaps(o2)) {
            COUT << "SHORT between " << it1->second.name() << " & obstacle @ layer : " << l.first << '\n';
            COUT << o1.str() << ' ' << o2.str() << '\n';
          }
        }
      }
    }
  }
}

void Module::writeDEF(const std::string& nstr) const
{
  std::ofstream ofs(_name + nstr + ".def");
  if (ofs.is_open()) {
    ofs << "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN " << _name << " ;\n";
    ofs << "UNITS DISTANCE MICRONS " << _uu << " ;\n";
    ofs << "DIEAREA ( " << _bbox.xmin() << ' ' << _bbox.ymin() << " ) ( " << _bbox.xmax() << ' ' << _bbox.ymax() << " ) ; \n\n";
    if (!_instances.empty()) {
      ofs << "COMPONENTS " << _instances.size() << " ;\n";
      for (auto& inst : _instances) {
        auto& tr = inst->transform();
        ofs << "- " << inst->name() << ' ' << inst->moduleName();
        ofs << " + PLACED ( ";
        ofs << ((tr.sX() > 0 ) ? tr.x() : (tr.x() - inst->bbox().width()))  << ' ';
        ofs << ((tr.sY() > 0 ) ? tr.y() : (tr.y() - inst->bbox().height())) << " ) " << tr.orient() << " ;\n";
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
              ofs << "  + RECT " << LAYER_NAMES[l.first];
              ofs << " ( " << r.xmin() << ' ' << r.ymin() << " ) ( " << r.xmax() << ' ' << r.ymax() << " )\n";
            }
          }
        }
        ofs << " ;\n";
      }
      ofs << "END NETS\n\n";
    }
    ofs << "END DESIGN\n";
  }
}

void Module::writeLEF() const
{
  std::ofstream ofs(_name + "_interim_hier.lef");
  if (ofs.is_open()) {
    ofs << "MACRO " << _name << "\n";
    ofs << "  UNITS\n    DISTANCE MICRONS " << _uu << ";\n  END UNITS\n";
    ofs << "  ORIGIN "  << _bbox.xmin()  << ' ' << _bbox.ymin() << " ;\n";
    ofs << "  FOREIGN " << _name << ' '  << (1.*_bbox.xmin()/_uu) << ' ' << (1.*_bbox.ymin()/_uu) << " ;\n";
    ofs << "  SIZE "    << (1.*_bbox.width()/_uu) << " BY " << (1.* _bbox.height()/_uu) << " ;\n";
    if (!_pins.empty()) {
      for (auto& p : _pins) {
        ofs << "  PIN " << p.first << "\n    DIRECTION INOUT ;\n    USE SIGNAL ;\n";
        auto& shapes = p.second->shapes();
        if (!shapes.empty()) {
          ofs << "    PORT\n";
          for (auto& l : shapes) {
            ofs << "      LAYER " << LAYER_NAMES[l.first] << " ;\n";
            for (auto& r : l.second) {
              ofs << "        RECT " << (r.xmin()*1.0/_uu) << ' ' << (1.*r.ymin()/_uu) << ' ' << (1.*r.xmax()/_uu) << ' ' << (1.*r.ymax()/_uu) << " ;\n";
            }
          }
          ofs << "    END\n";
        }
        ofs << "  END " << p.first << '\n';
      }
    }
    if (!_obstacles.empty()) {
      ofs << "    OBS\n";
      for (auto& l : _obstacles) {
        ofs << "      LAYER " << LAYER_NAMES[l.first] << " ;\n";
        for (auto& r : l.second) {
          ofs << "        RECT " << (1.*r.xmin()/_uu) << ' ' << (1.*r.ymin()/_uu) << ' ' << (1.*r.xmax()/_uu) << ' ' << (1.*r.ymax()/_uu) << " ;\n";
        }
      }
      ofs << "    END\n";
    }
    ofs << "END " << _name << "\nEND LIBRARY\n";
  }
}


Instance::~Instance()
{
  for (auto& p : _pins) delete p.second;
  _pins.clear();
}


void Instance::build(const bool rebuild)
{
  if (_m) {
    for (auto& p : _m->pins()) {
      Pin* ip{nullptr};
      if (rebuild) {
        auto it = _pins.find(p.second->name());
        if (it != _pins.end()) {
          ip = it->second;
          const_cast<Geom::LayerRects&>(ip->shapes()).clear();
        }
      } else {
        ip = new Pin(_name + "+" + p.second->name());
        _pins[p.second->name()] = ip;
      }
      if (!ip) continue;
      for (const auto& ls : p.second->shapes()) {
        for (const auto& s : ls.second) {
          ip->addRect(ls.first, _tr.transform(s));
        }
      }
    }
    _bbox = _tr.transform(_m->bbox());
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
}
