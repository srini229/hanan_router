#include "Util.h"
#include "Placement.h"

namespace Placement {

inline void Net::print() const
{
  COUT << "pins :";
  for (const auto& p : _pins) {
    std::cout << " " << p->name();
  }
}


void Net::route(Router::Router& router, const Geom::LayerRects& l1, const Geom::LayerRects& l2)
{
  _unroute = 0;
  for (auto& p : _pins) {
    Geom::MergeLayerRects(_routeshapes, p->shapes(), &_bbox);
  }
  if (_pins.size() > 1) {
    COUT << "routing net : " << _name << '\n';
    /*for (int i : {0, 1}) {
      for (auto& l : (i ? l1 : l2)) {
        for (auto& o : l.second) {
          COUT << "obs : " << l.first << ' ' << o.str() << '\n';
        }
      }
    }*/
    router.addObstacles(l1, true);
    router.addObstacles(l2, true);
    auto it1 = _pins.rbegin();
    auto it2 = std::next(it1);
    while (it2 != _pins.rend()) {
      router.clearSourceTargets();
      COUT << "routing pins : " << (*it1)->name() << ' ' << (*it2)->name() << '\n';
      const auto& p1 = (*it1)->shapes();
      const auto& p2 = (*it2)->shapes();
      for (auto src : {true, false}) {
        for (auto& l : (src ? p1 : p2)) {
          if (l.first > router.maxLayer() || l.first < router.minLayer()) continue;
          for (auto& s : l.second) {
            if (src) {
              router.addSource(s.xcenter(), s.ycenter(), l.first);
            } else {
              router.addTarget(s.xcenter(), s.ycenter(), l.first);
            }
          }
        }
      }
      router.setName(_name + "__" + (*it1)->name() + "__" + (*it2)->name());
      router.writeSTO();
      auto sol = router.findSol();
      if (!sol.empty()) {
        Geom::MergeLayerRects(_routeshapes, sol, &_bbox);
      } else {
        _unroute = 1;
      }
      router.plot();
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
    router.clearObstacles();
    router.clearObstacles(true);
    for (auto& inst : _instances) {
      auto m = inst->module();
      if (!m->routed()) {
        const_cast<Module*>(m)->route(router);
      }
      for (const auto& l : m->obstacles()) {
        for (const auto& r : l.second) {
          _obstacles[l.first].push_back(inst->transform(r));
        }
      }
    }
    COUT << " routing : " << _name << '\n';
    router.addObstacles(_obstacles);
    Geom::LayerRects _netObstaclesRouted, _netObstaclesUnrouted;
    for (auto it = _nets.begin(); it != _nets.end(); ++it) {
      _netObstaclesUnrouted.clear();
      for (auto itn = std::next(it); itn != _nets.end(); ++itn) {
        for (auto& p : itn->second.pins()) {
          Geom::MergeLayerRects(_netObstaclesUnrouted, p->shapes());
        }
      }
      it->second.route(router, _netObstaclesRouted, _netObstaclesUnrouted);
      Geom::MergeLayerRects(_netObstaclesRouted, it->second.routeShapes());
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
    /*for (auto& l : _obstacles) {
      const auto& color = LAYER_COLORS[l.first % LAYER_COLORS.size()];
      for (auto& b : l.second) {
        if (b.valid() && b.width() && b.height()) {
          ofs << "set object " << cnt++ << " rect from ";
          ofs << b.xmin() << "," << b.ymin() << " to " << b.xmax() << "," << b.ymax() << " fillstyle transparent solid 0.25 fillcolor \"" << color << "\" behind\n";
        }
      }
    }*/
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
      if (!n.second.open()) continue;
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
  auto it1 = _nets.begin();
  auto it2 = std::next(it1);
  while (it2 != _nets.end()) {
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

    it1 = it2;
    ++it2;
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
      auto ip = new Pin(_name + "+" + p.second->name());
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
}
