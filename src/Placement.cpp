#include "Util.h"
#include "Placement.h"
#include "Escape.h"
#include <unordered_map>
#include <tuple>

#include <algorithm>
#include <thread>
#include <mutex>
#include <queue>
#include <sstream>
#include <iostream>
#include <pthread.h>

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

int Module::mergeCoincidentNets()
{
  // Index every real-pin footprint as (layer, rect). When a footprint is already
  // owned by a different net, the two nets occupy the same physical point and are
  // electrically one -- union them (union-find over Net*).
  std::map<std::tuple<int, int, int, int, int>, Net*> owner;
  std::map<Net*, Net*> uf;
  for (auto& np : _nets) uf[&np.second] = &np.second;
  auto find = [&](Net* n) -> Net* {
    Net* r = n;
    while (uf[r] != r) r = uf[r];
    while (uf[n] != r) { Net* nx = uf[n]; uf[n] = r; n = nx; }  // path-halve
    return r;
  };
  for (auto& np : _nets) {
    Net* net = &np.second;
    for (auto pin : net->pins())
      for (auto port : pin->ports())
        for (auto& l : port->shapes())
          for (auto& r : l.second) {
            auto key = std::make_tuple(l.first, r.xmin(), r.ymin(), r.xmax(), r.ymax());
            auto it = owner.find(key);
            if (it == owner.end()) { owner.emplace(key, net); continue; }
            Net* a = find(it->second), *b = find(net);
            if (a != b) uf[b] = a;
          }
  }
  // Move every merged net's pins into its representative and empty the originals
  // so they route as a single connected net (and cannot short each other). Skip
  // pins whose footprint the representative already covers -- two pins on the
  // exact same point would otherwise become a degenerate (zero-length) routing
  // pair that the maze router cannot solve.
  using FP = std::tuple<int, int, int, int, int>;
  auto pinFootprints = [](const Pin* p, std::set<FP>& s) {
    for (auto port : p->ports())
      for (auto& l : port->shapes())
        for (auto& r : l.second)
          s.emplace(l.first, r.xmin(), r.ymin(), r.xmax(), r.ymax());
  };
  int merges = 0;
  for (auto& np : _nets) {
    Net* net = &np.second;
    Net* rep = find(net);
    if (rep == net) continue;
    std::set<FP> have;
    for (auto pin : rep->pins()) pinFootprints(pin, have);
    COUT << "WARNING: net " << net->name() << " has pin(s) coincident with net "
         << rep->name() << "; merging them into one connected net\n";
    for (auto pin : net->pins()) {
      std::set<FP> fp;
      pinFootprints(pin, fp);
      bool dup = false;
      for (const auto& f : fp) if (have.count(f)) { dup = true; break; }
      if (!dup) { rep->addPin(pin); have.insert(fp.begin(), fp.end()); }
    }
    net->clearPins();
    ++merges;
  }
  if (merges)
    COUT << "merged " << merges << " coincident-pin net(s) in module " << _name << '\n';
  return merges;
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
    mergeCoincidentNets();   // warn + merge nets whose pins sit on the same point
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

    int parPad = 0;
    for (int z = router.minLayer(); z <= router.maxLayer(); ++z) {
      parPad = std::max(parPad,
          std::max(router.baseWidthX(z), router.baseWidthY(z))
        + std::max(router.baseSpaceX(z), router.baseSpaceY(z)));
    }
    parPad *= 2;

    auto pinBBox = [](const Net* n) -> Geom::Rect {
      Geom::Rect b;
      for (auto virt : {true, false}) {
        const auto& pins = virt ? n->virtualpins() : n->pins();
        for (auto& pin : pins)
          for (auto& p : pin->ports())
            for (auto& l : p->shapes())
              for (auto& r : l.second) b.merge(r);
      }
      return b;
    };

    auto buildUnrouted = [&](size_t pi, bool addAdj) -> Geom::LayerRects {
      Geom::LayerRects u;
      for (size_t k = 0; k < nets.size(); ++k) {
        if (k == pi) continue;
        if (k < pi && !nets[k]->excluded()) continue;  // already in the routed set
        for (auto virt : {true, false}) {
          const auto& pins = virt ? nets[k]->virtualpins() : nets[k]->pins();
          for (auto& pin : pins)
            for (auto& p : pin->ports())
              Geom::MergeLayerRects(u, p->shapes());
        }
      }
      if (addAdj && m1Layer >= 0 && layerName(m1Layer + 1)[0] == 'M') {
        auto itm1 = u.find(m1Layer);
        if (itm1 != u.end() && !itm1->second.empty()) {
          auto& adj = u[m1Layer + 1];
          adj.insert(adj.end(), itm1->second.begin(), itm1->second.end());
        }
      }
      return u;
    };

    auto applyDebug = [&](Router::Router& r, size_t i) {
      if (debugnet.find(_name + "__" + nets[i]->name()) != debugnet.end()
          || debugnet.find(nets[i]->name()) != debugnet.end()
          || debugnet.find(_name) != debugnet.end())
        r.setEnableDebug(true);
      else
        r.setEnableDebug(false);
    };

    auto buildBatches = [&]() -> std::vector<std::vector<size_t>> {
      std::vector<std::vector<size_t>> batches;
      const size_t N = nets.size();
      const size_t pinned = _routeorder.size();
      if (router.threads() <= 1) {
        for (size_t i = 0; i < N; ++i) batches.push_back({i});
        return batches;
      }
      auto mustSingleton = [&](size_t i) {
        return i < pinned || nets[i]->excluded() || nets[i]->isDetour()
            || !nets[i]->routable();
      };
      std::vector<Geom::Rect> boxOf(N);
      for (size_t i = 0; i < N; ++i) {
        Geom::Rect b = pinBBox(nets[i]);
        if (b.valid()) b = b.bloatby(b.width() / 2 + parPad, b.height() / 2 + parPad);
        boxOf[i] = b;
      }
      size_t i = 0;
      while (i < N) {
        if (mustSingleton(i)) { batches.push_back({i}); ++i; continue; }
        std::vector<size_t> batch{i};
        size_t j = i + 1;
        while (j < N && !mustSingleton(j)) {
          bool disjoint = true;
          for (size_t idx : batch)
            if (boxOf[j].overlaps(boxOf[idx])) { disjoint = false; break; }
          if (!disjoint) break;
          batch.push_back(j);
          ++j;
        }
        batches.push_back(std::move(batch));
        i = j;
      }
      return batches;
    };

    auto routeBatch = [&](const std::vector<size_t>& batch,
                          const Geom::LayerRects& routedSnapshot,
                          const std::vector<Geom::LayerRects>& unroutedSets,
                          bool /*addAdj*/) {
      if (batch.size() == 1 || router.threads() <= 1) {
        for (size_t bi = 0; bi < batch.size(); ++bi) {
          size_t i = batch[bi];
          router.setNetName(nets[i]->name());
          applyDebug(router, i);
          nets[i]->route(router, routedSnapshot, unroutedSets[bi], _obstacles,
                         true, _uu, _bbox, _name);
        }
        return;
      }

      std::queue<size_t> q;                 // positions within 'batch'
      for (size_t bi = 0; bi < batch.size(); ++bi) q.push(bi);
      pthread_mutex_t qlock;
      pthread_mutex_init(&qlock, nullptr);
      std::mutex logmtx;                     // guards the real log sink

      const int nworkers = std::min<int>(router.threads(),
                                         static_cast<int>(batch.size()));
      std::vector<std::thread> workers;
      for (int w = 0; w < nworkers; ++w) {
        workers.emplace_back([&]() {
          std::ostringstream tlog;
          setThreadLog(&tlog);
          auto flush = [&]() {
            std::lock_guard<std::mutex> lk(logmtx);
            std::cout << tlog.str();
            tlog.str("");
          };
          {
            Router::Router myrouter(router.layerInfo());
            myrouter.setuu(_uu);
            myrouter.setModName(_name);
            myrouter.setusepinwidth((_usepinwidth == 1) ? true : false);
            flush();
            for (;;) {
              pthread_mutex_lock(&qlock);
              if (q.empty()) { pthread_mutex_unlock(&qlock); break; }
              size_t bi = q.front();
              q.pop();
              pthread_mutex_unlock(&qlock);

              size_t i = batch[bi];
              myrouter.setNetName(nets[i]->name());
              applyDebug(myrouter, i);
              nets[i]->route(myrouter, routedSnapshot, unroutedSets[bi], _obstacles, true, _uu, _bbox, _name);
              flush();
            }
          }
          flush();
          setThreadLog(nullptr);
        });
      }
      for (auto& t : workers) t.join();
      pthread_mutex_destroy(&qlock);
    };

    auto routeAllNets = [&](const bool addAdjObstacles) -> bool {
      Geom::LayerRects netObstaclesRouted;
      bool anyUnrouted{false};
      std::vector<std::vector<size_t>> batches = buildBatches();
      for (auto& batch : batches) {
        std::vector<Geom::LayerRects> unroutedSets(batch.size());
        for (size_t bi = 0; bi < batch.size(); ++bi)
          unroutedSets[bi] = buildUnrouted(batch[bi], addAdjObstacles);

        routeBatch(batch, netObstaclesRouted, unroutedSets, addAdjObstacles);

        for (size_t bi = 0; bi < batch.size(); ++bi) {
          size_t i = batch[bi];
          if (nets[i]->routable() && nets[i]->unrouted()) anyUnrouted = true;
          Geom::MergeLayerRects(netObstaclesRouted, nets[i]->routeShapesWithPins());
        }
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

    // single-pin (and excluded) nets have nothing to route, so they never count.
    auto countUnrouted = [&]() -> int {
      int c = 0;
      for (auto& n : _nets)
        if (!n.second.excluded() && n.second.routable() && n.second.unrouted()) ++c;
      return c;
    };

    // one full routing attempt on the current 'nets' ordering: route once, and
    // if anything is open retry with adjacent-layer pin obstacles. Always starts
    // from a clean (snapshot-restored) state so attempts are independent.
    auto attempt = [&]() -> int {
      for (auto& n : _nets) n.second.clearRoutes();
      if (routeAllNets(false)) {
        COUT << "module " << _name << " has unrouted nets; retrying with adjacent-layer pin obstacles\n";
        for (auto& n : _nets) n.second.clearRoutes();
        routeAllNets(true);
      }
      return countUnrouted();
    };

    int bestUnrouted = attempt();

    // If nets remain open, the net ordering is usually to blame: a net routed
    // early lays wire across a resource a later net needs, leaving it blocked.
    // Iterate (rip-up and reorder), each pass giving the nets that were left open
    // a higher priority so they route earlier, and keep the order that leaves the
    // fewest nets open.
    const int passes = router.reorderPasses();
    const size_t pinned = _routeorder.size();
    if (bestUnrouted > 0 && passes > 0 && nets.size() > pinned + 1) {
      COUT << "module " << _name << " has " << bestUnrouted
           << " unrouted net(s); promoting blocked nets up the routing order (up to "
           << passes << " pass(es))\n";
      NetsVec bestOrder = nets;
      std::unordered_map<const Net*, int> blockCount;   // times a net was left open
      std::unordered_map<const Net*, int> baseIdx;      // original tail position (tie-break)
      for (size_t i = pinned; i < nets.size(); ++i) baseIdx[nets[i]] = static_cast<int>(i);

      for (int pass = 0; pass < passes && bestUnrouted > 0; ++pass) {
        // raise the priority of every net the latest attempt left open: a net
        // that stays blocked keeps climbing until it routes before the rest.
        bool any = false;
        for (size_t i = pinned; i < nets.size(); ++i) {
          Net* v = nets[i];
          if (!v->excluded() && v->routable() && v->unrouted()) { ++blockCount[v]; any = true; }
        }
        if (!any) break;

        // re-sort descending block priority (original HPWL order breaks ties):
        // nets that have been left open rise toward the front
        // so they get first claim on the contested resources next time, ahead of
        // the nets that previously routed before them. This is priority promotion,
        // not an explicit blocker lookup -- the blocked nets simply outrank the
        // ones that aren't (yet) blocked.
        NetsVec newOrder = nets;
        std::stable_sort(newOrder.begin() + pinned, newOrder.end(),
          [&](const Net* a, const Net* b) {
            const int ba = blockCount[a], bb = blockCount[b];
            if (ba != bb) return ba > bb;
            return baseIdx[a] < baseIdx[b];
          });
        if (newOrder == nets) break;              // order already converged; no progress

        nets = std::move(newOrder);
        const int u = attempt();
        COUT << "  reorder pass " << (pass + 1) << "/" << passes << " : "
             << u << " unrouted (best so far " << std::min(u, bestUnrouted) << ")\n";
        if (u < bestUnrouted) { bestUnrouted = u; bestOrder = nets; }
      }
      // reproduce the best ordering found unless the last attempt already was it.
      if (nets != bestOrder) {
        COUT << "module " << _name << " : re-routing with best ordering ("
             << bestUnrouted << " unrouted)\n";
        nets = bestOrder;
        attempt();
      }
    }

    // Authoritative per-module routing result. Emitted as one structured line so
    // that downstream consumers (e.g. the smoke harness) can read the final
    // unrouted count directly instead of scraping per-attempt "sol not found"
    // lines, which over-count across the base/adj sub-passes and reorder passes.
    COUT << "ROUTE_SUMMARY module=" << _name << " nets=" << nets.size()
         << " unrouted=" << countUnrouted() << '\n';

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
        // Emit the pins in a stable, name-sorted order. Net::pins() is a
        // std::set keyed by Pin pointer, whose address ordering varies between
        // runs (ASLR), so iterating it directly makes the DEF non-deterministic.
        std::vector<const Pin*> orderedPins(n.second.pins().begin(), n.second.pins().end());
        std::sort(orderedPins.begin(), orderedPins.end(),
            [](const Pin* a, const Pin* b) { return a->name() < b->name(); });
        for (auto& p : orderedPins) {
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
          for (auto& pin : orderedPins) {
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
