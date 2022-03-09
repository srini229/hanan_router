#include "Util.h"
#include "Placement.h"

#include <algorithm>

namespace Placement {


auto DBLMAX = std::numeric_limits<double>::max();
inline void Net::print() const
{
  COUT << "pins :";
  for (const auto& p : _pins) {
    std::cout << " " << p->name();
  }
}


double Net::findMST(PinPairs& porder) const
{
  PinCVec pins(_pins.begin(), _pins.end());
  pins.insert(pins.end(), _vpins.begin(), _vpins.end());
  std::sort(pins.begin(), pins.end(),
      [](const Pin* p1, const Pin* p2) -> bool { return p1->bbox().halfpm() > p2->bbox().halfpm(); });
  std::vector<std::vector<double>> pinpairdist(pins.size(), std::vector<double>(pins.size(), 0));
  double mindist{DBLMAX};
  int idx1{-1}, idx2{-1};
  for (unsigned i = 0; i < pins.size(); ++i) {
    auto& p1 = pins[i];
    auto& s1 = p1->shapes();
    for (unsigned j = i + 1; j < pins.size(); ++j) {
      auto& p2 = pins[j];
      double dist = Geom::Dist(p1->bbox(), p2->bbox());
      auto& s2 = p2->shapes();
      for (auto& l : s1) {
        auto its2 = s2.find(l.first);
        if (its2 != s2.end()) {
          for (auto& r1 : l.second) {
            for (auto& r2 : its2->second) {
              dist = std::min(Geom::Dist(r1, r2), dist);
            }
          }
        }
      }
      pinpairdist[i][j] = dist;
      pinpairdist[j][i] = dist;
      if (mindist > dist) {
        mindist = dist;
        idx1 = static_cast<int>(i);
        idx2 = static_cast<int>(j);
      }
    }
  }
  std::vector<std::pair<int, int>> primorder;
  primorder.reserve(pins.size() - 1);
  primorder.push_back(std::make_pair(idx1, idx2));
  std::vector<int> selected(pins.size(), 0);
  selected[idx1] = 1;
  selected[idx2] = 1;
  double cost{pinpairdist[idx1][idx2]};
  while (primorder.size() < pins.size() - 1) {
    double mindist{DBLMAX};
    int minidx2 = -1, minidx1 = -1;
    for (int i = 0; i < static_cast<int>(pins.size()); ++i) {
      if (!selected[i]) continue;
      for (int j = 0; j < static_cast<int>(pins.size()); ++j) {
        if (!selected[j] && pinpairdist[i][j] < mindist) {
          mindist = pinpairdist[i][j];
          minidx1 = i;
          minidx2 = j;
        }
      }
    }
    if (minidx2 >= 0) {
      primorder.push_back(std::make_pair(minidx1, minidx2));
      selected[minidx2] = 1;
      cost += mindist;
    }
  }

  std::sort(primorder.begin(), primorder.end(), [pinpairdist](const std::pair<int, int>& a, const std::pair<int, int>& b) -> bool
      { return pinpairdist[a.first][a.second] < pinpairdist[b.first][b.second]; });

  for (auto& pp : primorder) porder.emplace_back(pins[pp.first], pins[pp.second]);
  return cost;
}

PinPairs Net::findSteiners()
{
  Geom::PointSet pinpoints;
  std::set<int> xpts, ypts;
  int minl{INT_MAX};
  for (auto& p : _pins) {
    auto xc(p->bbox().xcenter()), yc(p->bbox().ycenter());
    pinpoints.insert(Geom::Point(xc, yc));
    xpts.insert(xc);
    ypts.insert(yc);
    minl = std::min(minl, p->minLayer());
  }

  PinPairs porder;
  double cost = findMST(porder);
  double minCost{cost};
  for (int ctr = 0; ctr < static_cast<int>(_pins.size()) - 2; ++ctr) {
    Pin* vp = new Pin(_name + "+" + std::to_string(ctr), 0);
    _vpins.push_back(vp);
    Geom::Point ptstore;
    for (auto& xp : xpts) {
      for (auto& yp : ypts) {
        if (pinpoints.find(Geom::Point(xp, yp)) == pinpoints.end()) {
          vp->clearRects();
          vp->addRect(minl, Geom::Rect(xp - 16, yp - 16, xp + 16, yp + 16));
          PinPairs tmporder;
          cost = findMST(tmporder);
          //COUT << "MST cost : " << cost << ' ' << minCost << " (" << xp << ',' << yp << ") " << minl << '\n';
          if (cost < minCost) {
            minCost = cost;
            porder = tmporder;
            ptstore.set(xp, yp);
          }
        }
      }
    }
    if (ptstore.valid()) {
      pinpoints.insert(ptstore);
      vp->clearRects();
      vp->addRect(minl, Geom::Rect(ptstore.x() - 16, ptstore.y() - 16, ptstore.x() + 16, ptstore.y() + 16));
      COUT << "adding vpin at : " << ptstore.x() << ',' << ptstore.y() << ' ' << vp->name() << '\n';
    } else {
      delete vp;
      _vpins.pop_back();
      break;
    }
  }
  return porder;
}

/*PinPairs Net::reorderPins() const
{
  PinPairs porder;
  PinCVec pins(_pins.begin(), _pins.end());
  if (pins.size() == 2) {
    porder.emplace_back(pins.front(), pins.back());
  } else {
    std::sort(pins.begin(), pins.end(),
        [](const Pin* p1, const Pin* p2) -> bool { return p1->bbox().halfpm() > p2->bbox().halfpm(); });
    std::vector<std::vector<double>> pinpairdist(pins.size(), std::vector<double>(pins.size(), 0));
    double mindist{DBLMAX};
    int idx1{-1}, idx2{-1};
    for (unsigned i = 0; i < pins.size(); ++i) {
      auto& p1 = pins[i];
      auto& s1 = p1->shapes();
      for (unsigned j = i + 1; j < pins.size(); ++j) {
        auto& p2 = pins[j];
        double dist = Geom::Dist(p1->bbox(), p2->bbox());
        auto& s2 = p2->shapes();
        for (auto& l : s1) {
          auto its2 = s2.find(l.first);
          if (its2 != s2.end()) {
            for (auto& r1 : l.second) {
              for (auto& r2 : its2->second) {
                dist = std::min(Geom::Dist(r1, r2), dist);
              }
            }
          }
        }
        pinpairdist[i][j] = dist;
        pinpairdist[j][i] = dist;
        COUT << "pins dist : " << pins[i]->name() << ' ' << pins[j]->name() << ' ' << dist << '\n';
        if (mindist > dist) {
          mindist = dist;
          idx1 = static_cast<int>(i);
          idx2 = static_cast<int>(j);
        }
      }
    }
    std::vector<std::pair<int, int>> primorder;
    primorder.reserve(pins.size() - 1);
    primorder.push_back(std::make_pair(idx1, idx2));
    std::vector<int> selected(pins.size(), 0);
    selected[idx1] = 1;
    selected[idx2] = 1;
    double cost{pinpairdist[idx1][idx2]};
    while (primorder.size() < pins.size() - 1) {
      double mindist{DBLMAX};
      int minidx2 = -1, minidx1 = -1;
      for (int i = 0; i < static_cast<int>(pins.size()); ++i) {
        if (!selected[i]) continue;
        for (int j = 0; j < static_cast<int>(pins.size()); ++j) {
          if (!selected[j] && pinpairdist[i][j] < mindist) {
            mindist = pinpairdist[i][j];
            minidx1 = i;
            minidx2 = j;
          }
        }
      }
      if (minidx2 >= 0) {
        cost += mindist;
        primorder.push_back(std::make_pair(minidx1, minidx2));
        COUT << "pins to route order : " << pins[minidx1]->name() << ' ' << pins[minidx2]->name() << '\n';
        selected[minidx2] = 1;
      }
    }

    std::sort(primorder.begin(), primorder.end(), [pinpairdist](const std::pair<int, int>& a, const std::pair<int, int>& b) -> bool
        { return pinpairdist[a.first][a.second] < pinpairdist[b.first][b.second]; });


    for (auto& pp : primorder) porder.emplace_back(pins[pp.first], pins[pp.second]);
  }

  return porder;
}*/

void Net::route(Router::Router& router, const Geom::LayerRects& l1, const Geom::LayerRects& l2, const Geom::LayerRects& l3, const bool update)
{
  //TIME_M();
#if DEBUG
  SaveRestoreStream src(_name + "_route.log");
#endif
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
    auto pinpairs = findSteiners();
    for (auto& p : _vpins) {
      Geom::MergeLayerRects(_routeshapes, p->shapes(), &_bbox);
    }
    for (auto& pp : pinpairs) {
      const auto& pin1 = pp.first;
      const auto& pin2 = pp.second;
      router.clearSourceTargets();
      COUT << "routing pins : " << pin1->name() << ' ' << pin2->name() << '\n';
      router.setName(_name + "__" + pin1->name() + "__" + pin2->name());
      const auto& p1 = pin1->shapes();
      const auto& p2 = pin2->shapes();
      for (auto src : {true, false}) {
        for (auto& l : (src ? p1 : p2)) {
          if (l.first > router.maxLayer() || l.first < router.minLayer()) continue;
          for (auto& s : l.second) {
            if (src) {
              router.addSourceShapes(s, l.first);
            } else {
              router.addTargetShapes(s, l.first);
            }
          }
        }
      }
      router.updatendr(update);
#if DEBUG
      COUT << "adding line of sight nodes if they exist\n";
#endif
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
      router.addObstacles(l1, true);
      router.addObstacles(l2, true);
      router.addObstacles(l3, true);
      auto sol = router.findSol();
      if (!sol.empty()) {
#if DEBUG
        for (auto& l : sol) {
          for (auto& s : l.second) {
            COUT << "sol : " << l.first << ' ' << s.str() << ' ' << s.width() << ' ' << s.height() << '\n';
          }
        }
#endif
        Geom::MergeLayerRects(_routeshapes, sol, &_bbox);
      } else {
        _unroute = 1;
      }
      //Geom::MergeLayerRects(const_cast<Geom::LayerRects&>(pin1->shapes()), sol, &_bbox);
      //Geom::MergeLayerRects(const_cast<Geom::LayerRects&>(pin2->shapes()), sol, &_bbox);
      router.clearObstacles(true);
    }
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
  TIME_M();
  if (!_routed) {
    router.setModName(_name);
    router.setuu(_uu);
    //writeDEF("_before");
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
    COUT << " routing : " << _name << "; num nets : " << nets.size() << '\n';
    //router.addObstacles(_obstacles);
    Geom::LayerRects _netObstaclesRouted, _netObstaclesUnrouted;
    for (auto it = nets.begin(); it != nets.end(); ++it) {
      _netObstaclesUnrouted.clear();
      for (auto itn = std::next(it); itn != nets.end(); ++itn) {
        for (auto& p : (*itn)->pins()) {
          Geom::MergeLayerRects(_netObstaclesUnrouted, p->shapes());
        }
      }
      /*if (_name == "mixer_first_rx_0") {
        COUT << "net : " << (*it)->name() << '\n';
        if ((*it)->name().find("VCMBIAS") == std::string::npos)  continue;
      }*/
      router.setNetName((*it)->name());
      (*it)->route(router, _netObstaclesRouted, _netObstaclesUnrouted, _obstacles, true);
      //writeDEF("_" + (*it)->name(), (*it)->name());
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
  checkShort();
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
  COUT << "Checking SHORT for module : " << _name << '\n';
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

void Module::writeDEF(const std::string& nstr, const std::string& netname) const
{
  std::ofstream ofs(_name + nstr + ".def");
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
              ofs << "  + RECT " << LAYER_NAMES[l.first];
              ofs << " ( " << r.xmin() << ' ' << r.ymin() << " ) ( " << r.xmax() << ' ' << r.ymax() << " )\n";
            }
          }
        }
        for (auto& p : n.second.vpins()) {
          for (auto& l : p->shapes()) {
            for (auto& r : l.second) {
              ofs << "  + RECT " << LAYER_NAMES[l.first] << "_vp";
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
