#include "Util.h"
#include "Placement.h"
#include "Router.h"
#include "flute.h"
#include <mutex>

using namespace boost::polygon::operators;
static const int RSMT_CORRIDOR_PITCHES = 4;
static const int RSMT_EDGE_WIDTH = 5;   // drawn width of the corridor outline

#include <algorithm>
#include <mutex>

namespace Placement {

PortPairs Net::reorderPorts() const
{
  PortCVec ports;
  for (auto virt : {true, false}) {
    auto& pins = virt ? _vpins : _pins;
    for (auto& p : pins) {
      ports.insert(ports.end(), p->ports().begin(), p->ports().end());
    }
  }
  PortPairs porder;
  // Need at least 2 ports to form any pair; e.g. a net whose other pin(s)
  // contributed zero ports (LEF PIN with no PORT geometry) can leave exactly
  // one port here, which the pairwise-distance loop below can't process.
  if (ports.size() < 2) return porder;
  std::sort(ports.begin(), ports.end(),
      [](const Port* p1, const Port* p2) -> bool
      {
      const auto& b1 = p1->bbox();
      const auto& b2 = p2->bbox();
      if (b1.halfpm() == b2.halfpm()) {
      if (b1.ymin() == b2.ymin()) {
      if (b1.xmin() == b2.xmin()) return p1->name() < p2->name();  // stable tie-break: ports
      return b1.xmin() > b2.xmin();                                // are gathered from a
      }                                                            // pointer-ordered set, so a
      return b1.ymin() > b2.ymin();                                // unique-name fallback keeps
      }                                                            // the route order deterministic
      return b1.halfpm() > b2.halfpm();                            // across builds/runs.
      }
      );
  std::vector< std::vector<double> > portpairdist(ports.size(), std::vector<double>(ports.size(), 0));
  double mindist{1e30};
  int idx1{-1}, idx2{-1};
  Geom::Rect netbbox;
  for (auto& p : ports) {
    netbbox.merge(p->bbox());
  }
  double nethpwl{(netbbox.width() + netbbox.height()) * 1.};
  if (nethpwl < 1.0) nethpwl = 1.0;  // guard against zero-area bbox (division by zero)
  for (unsigned i = 0; i < ports.size(); ++i) {
    auto& p1 = ports[i];
    auto& s1 = p1->shapes();
    for (unsigned j = i + 1; j < ports.size(); ++j) {
      auto& p2 = ports[j];
      double dist{1.e30};// = Geom::Dist(p1->bbox(), p2->bbox()) / nethpwl;
      auto& s2 = p2->shapes();
      for (auto& l1 : s1) {
        for (auto& r1 : l1.second) {
          for (auto& l2 : s2) {
            for (auto& r2 : l2.second) {
              dist = std::min(Geom::Dist(r1, r2)/nethpwl + std::abs(l1.first - l2.first) * 0.03, dist);
              /*if (ports[i]->isVirtualPort() || ports[j]->isVirtualPort()) {
                COUT << ports[i]->name() << ' ' << ports[j]->name() << ' ' <<  Geom::Dist(r1, r2) << ' ' << l1.first << ' ' << l2.first << ' ' << r1.str() << ' ' << r2.str() << ' ' << dist << '\n';
              }*/
            }
          }
        }
      }
      /*for (auto& l1 : s1) {
        auto its2 = s2.find(l1.first);
        if (its2 != s2.end()) {
          for (auto& r1 : l.second) {
            for (auto& r2 : its2->second) {
              dist = std::min(Geom::Dist(r1, r2)/nethpwl, dist);
            }
          }
        }
      }*/
      if (ports[i]->isVirtualPort() || ports[j]->isVirtualPort()) dist /= 10;
      portpairdist[i][j] = dist;
      portpairdist[j][i] = dist;
      COUT << "ports dist : " << ports[i]->name() << ' ' << ports[j]->name() << ' ' << dist << '\n';
      if (mindist > dist) {
        mindist = dist;
        idx1 = static_cast<int>(std::min(i, j));
        idx2 = static_cast<int>(std::max(i, j));
      } else if (mindist == dist) {
        if (idx1 > static_cast<int>(i)) {
          idx1 = static_cast<int>(std::min(i, j));
          idx2 = static_cast<int>(std::max(i, j));
        }
      }
    }
  }
  // Guard: if no valid pair was found (all port pairs had no common-layer shapes),
  // there is nothing to route — return an empty order rather than crashing.
  if (idx1 < 0 || idx2 < 0) return porder;
  std::vector<std::pair<int, int>> primorder;
  primorder.reserve(ports.size() - 1);
  primorder.push_back(std::make_pair(idx1, idx2));
  COUT << "ports to route order : " << ports[idx1]->name() << ' ' << ports[idx2]->name() << '\n';
  std::vector<int> selected(ports.size(), 0);
  selected[idx1] = 1;
  selected[idx2] = 1;
  while (primorder.size() < ports.size() - 1) {
    double mindist{1e30};
    int minidx2 = -1, minidx1 = -1;
    for (int i = 0; i < static_cast<int>(ports.size()); ++i) {
      if (!selected[i]) continue;
      for (int j = 0; j < static_cast<int>(ports.size()); ++j) {
        if (!selected[j] && portpairdist[i][j] < mindist) {
          mindist = portpairdist[i][j];
          minidx1 = i;
          minidx2 = j;
        }
      }
    }
    if (minidx2 >= 0) {
      primorder.push_back(std::make_pair(minidx1, minidx2));
      COUT << "ports to route order : " << ports[minidx1]->name() << ' ' << ports[minidx2]->name() << '\n';
      selected[minidx2] = 1;
    }
  }

  std::sort(primorder.begin(), primorder.end(), [&portpairdist](const std::pair<int, int>& a, const std::pair<int, int>& b) -> bool
      { return portpairdist[a.first][a.second] < portpairdist[b.first][b.second]; });

  
  for (auto& pp : primorder) porder.emplace_back(ports[pp.first], ports[pp.second]);

  return porder;
}

PortPairs Net::clockRouteOrder() const
{
  PortPairs porder;
  COUT << "clock net : " << _name << " with driver : " << _driver << '\n';
  const Pin* driver {nullptr};
  for (auto& p : _pins) {
    if (p->name() == _driver) {
      driver = p;
      break;
    }
  }
  if (driver != nullptr && !driver->ports().empty()) {
    for (unsigned j = 1; j < driver->ports().size(); ++j) {
      porder.push_back(std::make_pair(driver->ports()[0], driver->ports()[j]));
    }
    for (auto virt : {true, false}) {
      auto& pins = virt ? _vpins : _pins;
      for (auto& p : pins) {
        if (p == driver) continue;
        else {
          for (auto& port : p->ports()) {
            porder.push_back(std::make_pair(driver->ports()[0], port));
          }
        }
      }
    }
  }
  std::sort(porder.begin(), porder.end(),
      [](const PortPair& p1, const PortPair& p2) -> bool
      { return Geom::Dist(p1.first->bbox(), p1.second->bbox()) > Geom::Dist(p2.first->bbox(), p2.second->bbox()); }
      );
  for (auto& p : porder) {
    COUT << "ports to route order : " << p.first->name() << ' ' << p.second->name() << ' ' << Geom::Dist(p.first->bbox(), p.second->bbox()) << '\n';
  }
  return porder;
}


// FLUTE's lookup tables are loaded once and shared; construction is not thread
// safe, so the first caller builds the state under a lock. The tables are
// compiled into the binary, hence the null paths.
static Flute::FluteState* fluteState()
{
  static std::mutex m;
  static Flute::FluteState* st = nullptr;
  static bool tried = false;
  std::lock_guard<std::mutex> g(m);
  if (!tried) {
    tried = true;
    st = Flute::flute_init(nullptr, nullptr);
    if (!st) CERR << "WARNING: FLUTE tables failed to load; RSMT corridor disabled\n";
  }
  return st;
}

// A rectilinear Steiner tree over the net's pin centres, each branch turned into
// a band `margin` wide. Confining the maze search to this corridor stops a net
// wandering far outside the region its own pins occupy -- the detours that show
// up as long excursions on an expensive layer -- while still leaving the Steiner
// topology's own freedom inside the band.

// Trace the outline of a rectilinear region and return each edge as a thin box.
// Drawing the corridor as its boundary rather than as the slabs it decomposes
// into keeps the picture readable -- the slabs tile the interior and bury the
// routing underneath them.
static Geom::Rects outlineBoxes(const Geom::Rects& region, const int w, int* nholes = nullptr)
{
  Geom::Rects edges;
  if (region.empty()) return edges;
  PolySet ps;
  for (const auto& r : region) ps.insert(PRect(r.xmin(), r.ymin(), r.xmax(), r.ymax()));
  PPolyWHs polys;
  ps.get(polys);
  const int lo = w / 2, hi = w - lo;
  auto wall = [&](auto first, auto last) {
    std::vector<Geom::Point> pts;
    for (auto it = first; it != last; ++it) {
      pts.emplace_back(static_cast<int>(bp::x(*it)), static_cast<int>(bp::y(*it)));
    }
    for (size_t i = 0; i < pts.size(); ++i) {
      const auto& a = pts[i];
      const auto& b = pts[(i + 1) % pts.size()];
      if (a.y() == b.y()) {
        edges.emplace_back(std::min(a.x(), b.x()) - lo, a.y() - lo,
                           std::max(a.x(), b.x()) + hi, a.y() + hi);
      } else if (a.x() == b.x()) {
        edges.emplace_back(a.x() - lo, std::min(a.y(), b.y()) - lo,
                           a.x() + hi, std::max(a.y(), b.y()) + hi);
      }
    }
  };
  for (const auto& poly : polys) {
    wall(poly.begin(), poly.end());
    for (auto ith = poly.begin_holes(); ith != poly.end_holes(); ++ith) {
      wall(ith->begin(), ith->end());
      if (nholes) ++*nholes;
    }
  }
  return edges;
}

Geom::Rects Net::rsmtCorridor(const int margin) const
{
  Geom::Rects corridor;
  std::vector<Geom::Rect> boxes;
  for (auto virt : {true, false}) {
    for (auto& p : (virt ? _vpins : _pins)) {
      Geom::Rect b;
      for (auto& port : p->ports()) if (port->bbox().valid()) b.merge(port->bbox());
      if (b.valid()) boxes.push_back(b);
    }
  }
  if (boxes.empty()) return corridor;
  for (auto& b : boxes) corridor.push_back(b.bloatby(margin, margin));
  if (boxes.size() < 2) return corridor;

  auto band = [&](const int x1, const int y1, const int x2, const int y2) {
    corridor.emplace_back(std::min(x1, x2) - margin, y1 - margin,
                          std::max(x1, x2) + margin, y1 + margin);
    corridor.emplace_back(x2 - margin, std::min(y1, y2) - margin,
                          x2 + margin, std::max(y1, y2) + margin);
  };

  std::vector<int> cx, cy;
  cx.reserve(boxes.size()); cy.reserve(boxes.size());
  for (auto& b : boxes) { cx.push_back(b.xcenter()); cy.push_back(b.ycenter()); }

  if (boxes.size() == 2) {
    band(cx[0], cy[0], cx[1], cy[1]);       // the single branch FLUTE would lay
    _rsmtlen = std::abs(cx[0] - cx[1]) + std::abs(cy[0] - cy[1]);
  } else if (boxes.size() == 3) {
    std::vector<int> sx(cx), sy(cy);
    std::nth_element(sx.begin(), sx.begin() + 1, sx.end());
    std::nth_element(sy.begin(), sy.begin() + 1, sy.end());
    const int mx = sx[1], my = sy[1];       // median point : the Steiner point for 3
    _rsmtlen = 0;
    for (size_t i = 0; i < cx.size(); ++i) {
      band(cx[i], cy[i], mx, my);
      _rsmtlen += std::abs(cx[i] - mx) + std::abs(cy[i] - my);
    }
  } else {
    auto* st = fluteState();
    if (!st) return corridor;               // no tables: corridor stays the pins
    std::vector<FLUTE_DTYPE> xs(cx.begin(), cx.end()), ys(cy.begin(), cy.end());
    Flute::Tree t = Flute::flute(st, static_cast<int>(xs.size()), xs.data(), ys.data(), FLUTE_ACCURACY);
    _rsmtlen = t.length;
    const int nb = 2 * t.deg - 2;
    for (int i = 0; i < nb; ++i) {
      const int j = t.branch[i].n;
      if (j < 0 || j >= nb) continue;
      band(t.branch[i].x, t.branch[i].y, t.branch[j].x, t.branch[j].y);
    }
    Flute::free_tree(st, t);
  }

  // The bands overlap heavily where branches meet. Merge them with
  // boost::polygon so the corridor is one rectilinear polygon rather than a pile
  // of boxes, and return its rectangle decomposition -- that is what gets
  // subtracted from the net's bbox and what gets drawn in the LEF.
  PolySet ps;
  for (const auto& r : corridor) ps.insert(PRect(r.xmin(), r.ymin(), r.xmax(), r.ymax()));
  PRects merged;
  get_rectangles(merged, ps);
  Geom::Rects out;
  out.reserve(merged.size());
  for (const auto& m : merged) out.emplace_back(bp::xl(m), bp::yl(m), bp::xh(m), bp::yh(m));
  return out;
}

Geom::LayerRects Net::dropSameNetObstacles(const Geom::LayerRects& obs) const
{
  Geom::LayerRects kept;
  // Seed from all of this net's metal -- its pins and anything already routed --
  // so obstacle shapes continuous with either are recognised as ours.
  Geom::LayerRects pinshapes;
  for (auto virt : {true, false}) {
    for (auto& p : (virt ? _vpins : _pins)) {
      for (auto& port : p->ports()) {
        for (auto& l : port->shapes()) {
          for (auto& r : l.second) pinshapes[l.first].push_back(r);
        }
      }
    }
  }
  for (const auto& l : _routeshapeswithpins) {
    for (const auto& r : l.second) pinshapes[l.first].push_back(r);
  }
  for (const auto& lo : obs) {
    const auto ip = pinshapes.find(lo.first);
    if (ip == pinshapes.end() || lo.second.empty()) { kept[lo.first] = lo.second; continue; }
    PolySet pins, obsps;
    for (const auto& r : ip->second) pins.insert(PRect(r.xmin(), r.ymin(), r.xmax(), r.ymax()));
    for (const auto& r : lo.second)  obsps.insert(PRect(r.xmin(), r.ymin(), r.xmax(), r.ymax()));
    PolySet all(pins);
    all += obsps;
    PPolys polys;
    all.get(polys);                       // connected components of pin+obstacle metal
    PolySet connected;
    for (const auto& poly : polys) {
      PolySet P;
      P.insert(poly);
      PolySet touch(P);
      touch &= pins;
      PRects probe;
      get_rectangles(probe, touch);
      if (!probe.empty()) connected += P;  // this polygon contains one of our pins
    }
    PolySet keep(obsps);
    keep -= connected;
    PRects krects;
    get_rectangles(krects, keep);
    auto& out = kept[lo.first];
    out.reserve(krects.size());
    for (const auto& k : krects) out.emplace_back(bp::xl(k), bp::yl(k), bp::xh(k), bp::yh(k));
    const size_t dropped = lo.second.size() ? (lo.second.size() - std::min(lo.second.size(), out.size())) : 0;
    if (!krects.empty() || !lo.second.empty()) {
      if (out.size() != lo.second.size()) {
        COUT << "same-net trace : net " << _name << " layer " << lo.first << " : "
             << lo.second.size() << " obstacle(s) -> " << out.size()
             << " after dropping metal continuous with our own pins";
        if (dropped) COUT << " (" << dropped << " fewer)";
        COUT << '\n';
      }
    }
  }
  return kept;
}

void Net::route(Router::Router& router, const Geom::LayerRects& l1, const Geom::LayerRects& l2, const Geom::LayerRects& l3, const bool update, const int uu, const Geom::Rect& bbox, const std::string& modname)
{
  //TIME_M();
#if DEBUG
  // SaveRestoreStream swaps the *global* std::cout/std::cerr rdbuf, which is
  // not safe for concurrent -threads routing (each worker thread would race
  // on the same global streambuf pointer); serialize this DEBUG-only per-net
  // log redirection rather than plumb thread-awareness into Net::route().
  static std::mutex debugLogMutex;
  std::lock_guard<std::mutex> debugLogGuard(debugLogMutex);
  SaveRestoreStream src(_name + "_route.log");
#endif
  _unroute = 0;
  _wirelen = 0;
  _openwires.clear();
  std::vector<const Pin*> sortedpins(_pins.begin(), _pins.end());
  std::sort(sortedpins.begin(), sortedpins.end(),
      [](const Pin* a, const Pin* b) { return a->name() < b->name(); });
  for (auto& pin : sortedpins) {
    for (auto& p : pin->ports()) {
      Geom::MergeLayerRects(_routeshapeswithpins, p->shapes(), &_bbox);
    }
  }
  if (router.debug()) {
    const Geom::LayerRects *obs[] = {&l1, &l2, &l3};
    writeLEF(modname, uu, bbox, obs);
  }
  if (_exclude) {
    COUT << "excluding net : " << _name << " from routing\n";
    return;
  }
  COUT << "net : " << _name << " num pins : " << _pins.size() << '\n';
  if (_pins.size() > 1) {
    COUT << "routing net : " << _name << ' ' << halfpm() << '\n';
    /*for (int i : {0, 1}) {
      for (auto& l : (i ? l1 : l2)) {
        for (auto& o : l.second) {
          COUT << "obs : " << l.first << ' ' << o.str() << '\n';
        }
      }
    }*/
    const bool tracing = router.traceSameNetObstacles();
    const Geom::LayerRects l3t = tracing ? dropSameNetObstacles(l3) : Geom::LayerRects();
    const Geom::LayerRects obt = tracing ? dropSameNetObstacles(_obstacles) : Geom::LayerRects();

    PortPairs ppairs = (_driver.empty() ? reorderPorts() : clockRouteOrder());

    // Confine this net to the neighbourhood of its own Steiner tree: build the
    // FLUTE corridor over the pin centres, then hand the maze search everything
    // OUTSIDE it as an obstacle. The MST pair routing below is unchanged -- it
    // simply can no longer wander far from the tree that connects the pins.
    Geom::LayerRects keepout;
    if (router.rsmtCorridor()) {
      int pitch = 0;
      for (int z = router.minLayer(); z <= router.maxLayer(); ++z) {
        pitch = std::max(pitch, std::max(router.baseWidthX(z), router.baseWidthY(z))
                              + std::max(router.baseSpaceX(z), router.baseSpaceY(z)));
      }
      auto corridor = rsmtCorridor(pitch * RSMT_CORRIDOR_PITCHES);
      if (!corridor.empty() && bbox.valid()) {
        // Clip to the module before taking the outline. The bands are bloated well
        // past the die on a small block, and an outline that sits outside the
        // routable area is a wall nothing can reach -- which is exactly why the
        // corridor appeared to be ignored.
        PolySet cs;
        for (const auto& r : corridor) cs.insert(PRect(r.xmin(), r.ymin(), r.xmax(), r.ymax()));
        PolySet box;
        box.insert(PRect(bbox.xmin(), bbox.ymin(), bbox.xmax(), bbox.ymax()));
        cs &= box;
        PRects crects;
        get_rectangles(crects, cs);
        corridor.clear();
        for (const auto& c : crects) corridor.emplace_back(bp::xl(c), bp::yl(c), bp::xh(c), bp::yh(c));
      }
      _corridor = corridor;
      int nholes = 0;
      _corridorEdges = outlineBoxes(corridor, RSMT_EDGE_WIDTH, &nholes);
      // The obstacle is the corridor's boundary itself -- the same thin walls the
      // LEF draws -- on every routing layer. A wall plus its spacing halo is
      // enough to stop the search crossing it, and it is a handful of rectangles
      // instead of a filled complement.
      if (!_corridorEdges.empty()) {
        for (const auto& r : _corridorEdges) {
          for (int z = router.minLayer(); z <= router.maxLayer(); ++z) keepout[z].push_back(r);
        }
        COUT << "RSMT corridor for net " << _name << " : " << corridor.size()
             << " band(s), " << nholes << " hole(s), boundary " << _corridorEdges.size() << " wall(s) on layers "
             << router.minLayer() << ".." << router.maxLayer() << '\n';
      }
    }

    for (auto& pp : ppairs) {
      const auto& port1 = pp.first;
      const auto& port2 = pp.second;
      router.clearSourceTargets();
      COUT << "routing ports : " << port1->name() << ' ' << port2->name() << '\n';
      router.setName(_name + "__" + port1->name() + "__" + port2->name());
      router.setMBox(bbox);
      const auto& p1 = port1->shapes();
      const auto& p2 = port2->shapes();
      //bool preflayersrctgt{true};
      Geom::LayerRects samenetobst;
      auto addSrcTgtShapes = [&]() {
      samenetobst.clear();
      for (auto src : {true, false}) {
        bool preflayer{false};
        for (auto& l : _preflayers) {
          const auto& p = (src ? p1 : p2);
          auto it = p.find(l);
          if (it != p.end() && !it->second.empty()) {
            preflayer = true;
            break;
          }
        }
        //preflayersrctgt &= preflayer;
        if (!_preflayers.empty()) {
          COUT << "pref layer pin" << (preflayer ? "" : " not") << " found for " << (src ? port1->name() : port2->name()) << '\n';
        }
        for (auto& l : (src ? p1 : p2)) {
          if (l.first > router.maxLayer() || l.first < router.minLayer()) {
            for (auto& s : l.second) samenetobst[l.first].push_back(s);
            continue;
          }
          if (preflayer && _preflayers.find(l.first) == _preflayers.end()) continue;
          //COUT << "port : " << (src ? port1->name() : port2->name()) << " layer " << l.first << '\n';
          for (auto& s : l.second) {
            if (src) {
              router.addSourceShapes(s, l.first);
            } else {
              router.addTargetShapes(s, l.first);
            }
          }
        }
      }
      };
      addSrcTgtShapes();
      router.updatendr(update, _ndrwidths, _ndrspaces, _ndrdirs, _preflayers, _ndrvias);
#if DEBUG
      COUT << "adding line of sight nodes if they exist\n";
#endif
      /*for (auto& l : p1) {
        auto it = p2.find(l.first);
        if (preflayersrctgt && _preflayers.find(l.first) == _preflayers.end()) continue;
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
      }*/
      if (_detour) router.allowDetour();
      router.addObstacles(l1, true);
      router.addObstacles(l2, true);
      router.addObstacles(tracing ? l3t : l3, true);
      router.addObstacles(tracing ? obt : _obstacles, true);
      router.addObstacles(samenetobst, true);
      router.addObstacles(keepout, true);
      auto sol = router.findSol();
      // The corridor is a heuristic bound, never a hard constraint: if a pair
      // cannot be routed inside it, drop it and try again unconfined rather than
      // leave the net open. Only the corridor is removed -- every real obstacle is
      // put back exactly as it was.
      if (!router.lastSolutionFound() && !keepout.empty()) {
        COUT << "RSMT corridor blocked " << port1->name() << " -> " << port2->name()
             << " ; retrying without it\n";
        router.clearObstacles(true);
        router.clearSourceTargets();
        router.setName(_name + "__" + port1->name() + "__" + port2->name());
        router.setMBox(bbox);
        addSrcTgtShapes();
        router.updatendr(update, _ndrwidths, _ndrspaces, _ndrdirs, _preflayers, _ndrvias);
        if (_detour) router.allowDetour();
        router.addObstacles(l1, true);
        router.addObstacles(l2, true);
        router.addObstacles(tracing ? l3t : l3, true);
        router.addObstacles(tracing ? obt : _obstacles, true);
        router.addObstacles(samenetobst, true);
        sol = router.findSol();
      }
      // Not sol.empty(): a source and target that already coincide need zero
      // additional shapes and legitimately return an empty sol on success.
      if (router.lastSolutionFound()) {
        _wirelen += router.solLength();
#if DEBUG
        for (auto& l : sol) {
          for (auto& s : l.second) {
            COUT << "sol : " << l.first << ' ' << s.str() << ' ' << s.width() << ' ' << s.height() << '\n';
          }
        }
#endif
        if (port1->isVirtualPort() || port2->isVirtualPort()) {
          for (auto i : {true, false}) {
            auto& port = i ? port1 : port2;
            if (port->isVirtualPort()) {
              auto& shapes = port->shapes();
              bool merged{false};
              for (auto& l : sol) {
                auto it = shapes.find(l.first);
                if (it != shapes.end()) {
                  for (auto& r : l.second) {
                    for (auto& rp : it->second) {
                      if (rp.overlaps(r)) {
                        if ((rp.ymin() == r.ymin() && rp.ymax() == r.ymax())
                            || (rp.xmin() == r.xmin() && rp.xmax() == r.xmax())) {
                          r.merge(rp);
                          merged = true;
                          break;
                        }
                      }
                    }
                  }
                  if (merged) break;
                }
                if (merged) break;
              }
            }
          }
        }
        Geom::MergeLayerRects(_routeshapeswithpins, sol, &_bbox);
        Geom::MergeLayerRects(_routeshapes, sol, &_bbox);
      } else {
        _unroute = 1;
        addOpenWire(router.name());
      }
      if (!port1->isVirtualPort() || !_driver.empty()) {
        COUT << "Adding routes to " << port1->name() << ' ' << sol.size() << std::endl;
        Geom::MergeLayerRects(const_cast<Geom::LayerRects&>(port1->shapes()), sol, &_bbox);
        if (port2->isVirtualPort()) {
          Geom::MergeLayerRects(const_cast<Geom::LayerRects&>(port1->shapes()), port2->shapes(), &_bbox);
          Geom::MergeLayerRects(_routeshapeswithpins, port2->shapes(), &_bbox);
          Geom::MergeLayerRects(_routeshapes, port2->shapes(), &_bbox);
        }
      }
      if (!port2->isVirtualPort() || !_driver.empty()) {
        COUT << "Adding routes to " << port2->name() << ' ' << sol.size() << std::endl;
        Geom::MergeLayerRects(const_cast<Geom::LayerRects&>(port2->shapes()), sol, &_bbox);
        if (port1->isVirtualPort()) {
          Geom::MergeLayerRects(const_cast<Geom::LayerRects&>(port2->shapes()), port1->shapes(), &_bbox);
          Geom::MergeLayerRects(_routeshapeswithpins, port1->shapes(), &_bbox);
          Geom::MergeLayerRects(_routeshapes, port1->shapes(), &_bbox);
        }
      }
      router.clearObstacles(true);
    }
  }
  // Final-pass diagnostics: if this net was left open, dump a debug LEF holding
  // its pins (the sources/targets) and the three obstacle sets the A* search
  // faced (routed nets, unrouted-net pins, module obstacles) so the blockage can
  // be inspected. Single-pin (non-routable) and excluded nets never reach here
  // as open, so only genuine failures are written.
  if (router.dumpOpenNets() && _unroute) {
    const Geom::LayerRects* obs[] = {&l1, &l2, &l3};
    writeLEF(modname, uu, bbox, obs);
    COUT << "wrote open-net debug LEF : net_" << modname << '_' << _name << ".lef\n";
  }
}

void Net::writeLEF(const std::string& modname, const int uu, const Geom::Rect& bbox, const Geom::LayerRects* obs[]) const
{
  auto name(("net_" + modname + "_" + _name + ".lef"));
  COUT << "writing LEF file : " << name << "\n";
  std::ofstream ofs(name);
  if (ofs.is_open()) {
    ofs << "MACRO " << modname << "\n";
    ofs << "  UNITS\n    DISTANCE MICRONS " << uu << ";\n  END UNITS\n";
    ofs << "  ORIGIN "  << bbox.xmin()  << ' ' << bbox.ymin() << " ;\n";
    ofs << "  FOREIGN " << name << ' '  << (1.*bbox.xmin()/uu) << ' ' << (1.*bbox.ymin()/uu) << " ;\n";
    ofs << "  SIZE "    << (1.*bbox.width()/uu) << " BY " << (1.* bbox.height()/uu) << " ;\n";
    for (auto& v : {0, 1}) {
      for (auto& p : (v ? _pins : _vpins)) {
        ofs << "  PIN " << p->name() <<"\n    DIRECTION INOUT ;\n    USE SIGNAL ;\n";
        for (auto& pp : p->ports()) {
          ofs << "    PORT\n";
          for (auto& l : pp->shapes()) {
            ofs << "      LAYER " << layerName(l.first) << "_PIN ;\n";
            for (auto& r : l.second) {
              ofs << "        RECT " << (1.*r.xmin()/uu) << ' ' << (1.*r.ymin()/uu) << ' ' << (1.*r.xmax()/uu) << ' ' << (1.*r.ymax()/uu) << " ;\n";
            }
          }
          ofs << "      END\n";
        }
        ofs <<"  END " << p->name() << '\n';
      }
    }
    ofs << "  OBS\n";
    for (unsigned i = 0; i < 4; ++i) {
      if (i == 3 || obs[i]) {
        for (auto& l : (i < 3 ? *obs[i] : _obstacles)) {
          ofs << "      LAYER " << layerName(l.first) << " ;\n";
          for (auto r : l.second) {
            //if (r.AND(_bbox)) {
              ofs << "        RECT " << (1.*r.xmin()/uu) << ' ' << (1.*r.ymin()/uu) << ' ' << (1.*r.xmax()/uu) << ' ' << (1.*r.ymax()/uu) << " ;\n";
            //}
          }
        }
      }
    }
    ofs << "    LAYER BBOX ;\n";
    ofs << "      RECT " << (1.*_bbox.xmin()/uu) << ' ' << (1.*_bbox.ymin()/uu) << ' ' << (1.*_bbox.xmax()/uu) << ' ' << (1.*_bbox.ymax()/uu) << " ;\n";
    ofs << "  END\n";
    ofs << "END " << modname << "\nEND LIBRARY\n";
  }
}

}
