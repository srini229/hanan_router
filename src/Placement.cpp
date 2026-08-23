#include "Util.h"
#include "Placement.h"
#include "Escape.h"
#include "Router.h"
#include <unordered_map>
#include <unordered_set>
#include <tuple>
#include <exception>
#include <functional>

#include <algorithm>
#include <random>
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
      if (!m) {
        CERR << "ERROR: instance '" << inst->name() << "' in module '"
             << _name << "' has no resolved template; skipping\n";
        continue;
      }
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

    // Pin bbox of each net, cached: pins never move, so it is computed once here
    // (ports are still pin-only) and reused by every batching / obstacle-filter
    // pass instead of being recomputed for each net on each reorder attempt.
    auto computePinBBox = [](const Net* n) -> Geom::Rect {
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
    std::unordered_map<const Net*, Geom::Rect> pinBoxCache;
    pinBoxCache.reserve(nets.size());
    for (auto n : nets) pinBoxCache.emplace(n, computePinBBox(n));
    auto pinBBox = [&](const Net* n) -> const Geom::Rect& { return pinBoxCache.at(n); };

    // ---- symmetric_nets : route the first of a pair, then mirror its solution as
    // a guide that biases the A* search of the second so the pair appears mirrored.
    std::unordered_set<const Net*> symNets;                 // every net in any pair
    std::unordered_map<const Net*, const SymPair*> secondToPair;  // second net -> pair
    std::unordered_map<const Net*, const SymPair*> netToPair;     // either net -> its pair
    for (auto& sp : _sympairs) {
      symNets.insert(sp.first);
      symNets.insert(sp.second);
      secondToPair[sp.second] = &sp;
      netToPair[sp.first] = &sp;
      netToPair[sp.second] = &sp;
    }
    auto mirrorRect = [](const Geom::Rect& r, const bool vert, const int pos) -> Geom::Rect {
      if (vert) return Geom::Rect(2 * pos - r.xmax(), r.ymin(), 2 * pos - r.xmin(), r.ymax());
      return Geom::Rect(r.xmin(), 2 * pos - r.ymax(), r.xmax(), 2 * pos - r.ymin());
    };
    auto mirrorShapes = [&](const Geom::LayerRects& in, const bool vert, const int pos) -> Geom::LayerRects {
      Geom::LayerRects out;
      for (auto& l : in)
        for (auto& r : l.second) out[l.first].push_back(mirrorRect(r, vert, pos));
      return out;
    };
    // Mirror axis for a pair: explicit override, else inferred from the two nets'
    // pin-bbox centres (the larger centre offset picks the axis orientation).
    auto resolveAxis = [&](const SymPair& sp, bool& vert, int& pos) {
      if (sp.orient == 1) { vert = true;  pos = sp.pos; return; }
      if (sp.orient == 2) { vert = false; pos = sp.pos; return; }
      const Geom::Rect& ba = pinBBox(sp.first);
      const Geom::Rect& bb = pinBBox(sp.second);
      const long dx = std::labs(static_cast<long>(ba.xcenter()) - bb.xcenter());
      const long dy = std::labs(static_cast<long>(ba.ycenter()) - bb.ycenter());
      if (dx >= dy) { vert = true;  pos = (ba.xcenter() + bb.xcenter()) / 2; }
      else          { vert = false; pos = (ba.ycenter() + bb.ycenter()) / 2; }
    };
    // Keep each pair's first net ahead of its second in the routing order so the
    // guide (the first net's mirrored route) exists when the second is routed.
    auto enforceSymOrder = [&](NetsVec& order) {
      for (auto& sp : _sympairs) {
        auto ita = std::find(order.begin(), order.end(), sp.first);
        auto itb = std::find(order.begin(), order.end(), sp.second);
        if (ita == order.end() || itb == order.end() || itb > ita) continue;
        Net* b = *itb;
        order.erase(itb);
        ita = std::find(order.begin(), order.end(), sp.first);
        order.insert(ita + 1, b);
      }
    };

    // Generous super-set of a net's A* search box (pin bbox +50% + bloat*4); the
    // parPad term covers the bloat with room to spare, so an obstacle/pin outside
    // this box provably cannot overlap the search and is safe to drop.
    auto searchBox = [&](size_t i) -> Geom::Rect {
      Geom::Rect b = pinBBox(nets[i]);
      if (b.valid()) b = b.bloatby(b.width() + parPad * 8, b.height() + parPad * 8);
      return b;
    };

    Geom::LayerRects escapeReserve;
    {
      std::vector<Escape::Pin> epins;
      std::vector<char> isPort;
      int netid = 0;
      for (auto& nv : nets) {
        const bool outwardPort = (nv->pins().size() == 1 && _pins.count(nv->name()) > 0);
        if (!nv->excluded() && (nv->pins().size() >= 2 || outwardPort)) {
          for (auto& pin : nv->pins()) {
            Escape::Pin ep;
            ep.name = _name + SEPARATOR + pin->name();
            ep.net = netid;
            for (auto& port : pin->ports())
              for (auto& l : port->shapes())
                for (auto& r : l.second) ep.shapes[l.first].push_back(r);
            if (!ep.shapes.empty()) {
              epins.push_back(std::move(ep));
              isPort.push_back(outwardPort ? 1 : 0);
            }
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
        std::vector<Escape::Chosen> chosen;
        if (Escape::feasible(epins, _obstacles, lm, nullptr, nullptr, &chosen)) {
          for (const auto& c : chosen) {
            if (c.pin < isPort.size() && isPort[c.pin]) {
              // Reserve only what one escape actually needs, not the whole pin: a
              // via-up candidate's footprint is the entire pin rect on the layer
              // above, and holding all of that hostage costs far more routing
              // resource than the port needs and pushes other nets onto higher
              // layers. A landing box one routing width across, centred in the
              // footprint, keeps exactly one escape alive and frees the rest.
              Geom::Rect fp = c.fp;
              const int wz = std::max(router.baseWidthX(c.layer), router.baseWidthY(c.layer));
              if (fp.width() > wz || fp.height() > wz) {
                const int hx = std::min(fp.width(),  wz) / 2;
                const int hy = std::min(fp.height(), wz) / 2;
                const int cx = fp.xcenter(), cy = fp.ycenter();
                fp = Geom::Rect(cx - hx, cy - hy, cx + hx, cy + hy);
              }
              escapeReserve[c.layer].push_back(fp);
              COUT << "reserving escape for port " << epins[c.pin].name
                   << " : layer " << c.layer << ' ' << fp.str()
                   << " (of " << c.fp.str() << ")\n";
            }
          }
        }
      }
    }
    // What this module's own nets must keep clear: the real obstacles plus the
    // escapes reserved above. _obstacles itself is left untouched.
    Geom::LayerRects routeObstacles = _obstacles;
    Geom::MergeLayerRects(routeObstacles, escapeReserve);

    std::map<int, Geom::RTree2D> obsTree;
    for (auto& lr : routeObstacles) obsTree.emplace(lr.first, lr.second);
    auto queryObs = [&](const Geom::Rect& box) -> Geom::LayerRects {
      Geom::LayerRects out;
      for (auto& lt : obsTree) {
        Geom::Rects hits;
        lt.second.search(hits, box);
        if (!hits.empty()) out[lt.first] = std::move(hits);
      }
      return out;
    };
    auto netObstacles = [&](size_t i) -> Geom::LayerRects {
      Geom::Rect box = searchBox(i);
      if (nets[i]->isDetour() || !nets[i]->routable() || !box.valid())
        return routeObstacles;
      Geom::LayerRects out = queryObs(box);
      auto pit = netToPair.find(nets[i]);
      if (pit != netToPair.end()) {
        bool vert; int pos;
        resolveAxis(*pit->second, vert, pos);
        Geom::Rect mbox = mirrorRect(box, vert, pos);
        Geom::MergeLayerRects(out, mirrorShapes(queryObs(mbox), vert, pos));
      }
      return out;
    };

    // All pins indexed once (pin-only ports, before any routing) so the per-net
    // unrouted-pin obstacle set can be gathered by an R-tree query of the net's
    // search box instead of scanning every other net (O(N) per net -> O(N^2)).
    // Equivalent to the exact scan: pins outside the box never overlap the
    // search, and a net already routed has its pins in the routed set (l1) too.
    Geom::LayerRects allPins;
    for (auto n : nets)
      for (auto virt : {true, false})
        for (auto& pin : (virt ? n->virtualpins() : n->pins()))
          for (auto& p : pin->ports())
            for (auto& l : p->shapes())
              for (auto& r : l.second) allPins[l.first].push_back(r);
    std::map<int, Geom::RTree2D> pinTree;
    for (auto& lr : allPins) pinTree.emplace(lr.first, lr.second);

    // Exact full scan (used for detour nets, whose search box is not bounded by
    // the pin bbox, and as a fallback): pins of every net after pi, plus excluded
    // nets before it.
    auto exactUnrouted = [&](size_t pi, Geom::LayerRects& u) {
      for (size_t k = 0; k < nets.size(); ++k) {
        if (k == pi) continue;
        if (k < pi && !nets[k]->excluded()) continue;  // already in the routed set
        for (auto virt : {true, false})
          for (auto& pin : (virt ? nets[k]->virtualpins() : nets[k]->pins()))
            for (auto& p : pin->ports())
              Geom::MergeLayerRects(u, p->shapes());
      }
    };

    auto buildUnrouted = [&](size_t pi, bool addAdj) -> Geom::LayerRects {
      Geom::LayerRects u;
      Geom::Rect box = searchBox(pi);
      // The adjacency retry projects this set's M1 pins onto M1+1, which would
      // amplify the (otherwise harmless) redundant pins the spatial query adds,
      // so use the exact set whenever that projection is in play.
      if (nets[pi]->isDetour() || addAdj || !box.valid()) {
        exactUnrouted(pi, u);
      } else {
        // own pins must not become obstacles for the net routing to them
        std::set<std::tuple<int, int, int, int, int>> self;
        for (auto virt : {true, false})
          for (auto& pin : (virt ? nets[pi]->virtualpins() : nets[pi]->pins()))
            for (auto& p : pin->ports())
              for (auto& l : p->shapes())
                for (auto& r : l.second)
                  self.emplace(l.first, r.xmin(), r.ymin(), r.xmax(), r.ymax());
        for (auto& lt : pinTree) {
          Geom::Rects hits;
          lt.second.search(hits, box);
          for (auto& r : hits)
            if (!self.count(std::make_tuple(lt.first, r.xmin(), r.ymin(), r.xmax(), r.ymax())))
              u[lt.first].push_back(r);
        }
      }
      if (addAdj && m1Layer >= 0 && layerName(m1Layer + 1)[0] == 'M') {
        auto itm1 = u.find(m1Layer);
        if (itm1 != u.end() && !itm1->second.empty()) {
          const int adjSpace = std::max(router.baseSpaceX(m1Layer + 1),
                                        router.baseSpaceY(m1Layer + 1));
          Geom::Rects own;      // this net's own M1 pin footprints = its landing pads
          for (auto virt : {true, false})
            for (auto& pin : (virt ? nets[pi]->virtualpins() : nets[pi]->pins()))
              for (auto& p : pin->ports()) {
                auto sit = p->shapes().find(m1Layer);
                if (sit != p->shapes().end())
                  own.insert(own.end(), sit->second.begin(), sit->second.end());
              }
          auto& adj = u[m1Layer + 1];
          for (auto& r : itm1->second) {
            bool blocksOwnVia = false;
            for (auto& o : own) {
              if (o.bloatby(adjSpace).overlaps(r, true)) { blocksOwnVia = true; break; }
            }
            if (!blocksOwnVia) adj.push_back(r);
          }
        }
      }
      return u;
    };

    auto applyDebug = [&](Router::Router& r, size_t i) {
      r.setEnableDebug(debugnet.count(_name + "__" + nets[i]->name())
                       || debugnet.count(nets[i]->name())
                       || debugnet.count(_name));
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
            || !nets[i]->routable() || symNets.count(nets[i]);
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
                          const std::vector<Geom::LayerRects>& obsSets) {
      // A singleton batch (the only kind when threads <= 1) routes inline on the
      // shared main router, preserving exact sequential semantics.
      if (batch.size() == 1) {
        size_t i = batch[0];
        router.setNetName(nets[i]->name());
        applyDebug(router, i);
        // If this net is the second of a symmetric pair and its partner routed,
        // install the mirrored partner route as an A* guide.
        bool vert = true; int pos = 0; bool guided = false;
        auto sit = secondToPair.find(nets[i]);
        if (sit != secondToPair.end() && !sit->second->first->routeShapes().empty()) {
          resolveAxis(*sit->second, vert, pos);
          router.setGuide(mirrorShapes(sit->second->first->routeShapes(), vert, pos),
                          _devweight * router.baseUnitCost());
          guided = true;
          COUT << "symmetric net : routing " << nets[i]->name() << " guided by "
               << sit->second->first->name() << " mirrored about "
               << (vert ? "V:" : "H:") << pos << '\n';
        }
        nets[i]->route(router, routedSnapshot, unroutedSets[0], obsSets[0],
                       true, _uu, _bbox, _name);
        if (guided) {
          double maxdev = 0, sumdev = 0; long cnt = 0;
          for (auto& l : nets[i]->routeShapes())
            for (auto& r : l.second) {
              double d = router.guideDeviation(r.xcenter(), r.ycenter(), l.first);
              maxdev = std::max(maxdev, d); sumdev += d; ++cnt;
            }
          COUT << "SYMMETRY module=" << _name << " pair=" << sit->second->first->name()
               << ',' << nets[i]->name() << " axis=" << (vert ? "V:" : "H:") << pos
               << " maxdev=" << maxdev << " meandev=" << (cnt ? sumdev / cnt : 0.0) << '\n';
          router.clearGuide();
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
      // Each worker catches its own exceptions rather than letting them escape
      // the thread entry function (which would call std::terminate()); the
      // first one seen is rethrown on the main thread once every worker has
      // been joined.
      std::vector<std::exception_ptr> workerExceptions(nworkers);
      try {
        for (int w = 0; w < nworkers; ++w) {
          workers.emplace_back([&, w]() {
            std::ostringstream tlog;
            setThreadLog(&tlog);
            auto flush = [&]() {
              std::lock_guard<std::mutex> lk(logmtx);
              std::cout << tlog.str();
              tlog.str("");
            };
            try {
              {
                Router::Router myrouter(router.layerInfo());
                myrouter.setuu(_uu);
                myrouter.setModName(_name);
                myrouter.setusepinwidth((_usepinwidth == 1) ? true : false);
                myrouter.setCornerEscape(router.cornerEscape());
                myrouter.setRelaxViaEscape(router.relaxViaEscape());
                myrouter.setDumpOpenNets(router.dumpOpenNets());
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
                  nets[i]->route(myrouter, routedSnapshot, unroutedSets[bi], obsSets[bi], true, _uu, _bbox, _name);
                  flush();
                }
              }
              flush();
            } catch (...) {
              workerExceptions[w] = std::current_exception();
            }
            setThreadLog(nullptr);
          });
        }
      } catch (...) {
        // std::thread construction itself failed partway through the loop
        // (e.g. the OS thread-count ceiling); join every already-started
        // thread before propagating so none is destroyed while joinable.
        for (auto& t : workers) if (t.joinable()) t.join();
        pthread_mutex_destroy(&qlock);
        throw;
      }
      for (auto& t : workers) t.join();
      pthread_mutex_destroy(&qlock);
      for (auto& e : workerExceptions) {
        if (e) std::rethrow_exception(e);
      }
    };

    auto routeAllNets = [&](const bool addAdjObstacles) -> bool {
      if (!_sympairs.empty()) enforceSymOrder(nets);
      COUT << "ROUTING_ORDER attempt=" << router.attemptNo()
           << " phase=" << (addAdjObstacles ? "adj-retry" : "base")
           << ' ' << Router::Router::attemptModeName(router.attemptNo()) << " : ";
      for (size_t i = 0; i < nets.size(); ++i) {
        if (i) COUT << ", ";
        COUT << nets[i]->name();
      }
      if (nets.empty()) COUT << "(none)";
      COUT << '\n';
      Geom::LayerRects netObstaclesRouted;
      bool anyUnrouted{false};
      std::vector<std::vector<size_t>> batches = buildBatches();
      for (auto& batch : batches) {
        std::vector<Geom::LayerRects> unroutedSets(batch.size());
        std::vector<Geom::LayerRects> obsSets(batch.size());
        for (size_t bi = 0; bi < batch.size(); ++bi) {
          unroutedSets[bi] = buildUnrouted(batch[bi], addAdjObstacles);
          obsSets[bi] = netObstacles(batch[bi]);
        }

        routeBatch(batch, netObstaclesRouted, unroutedSets, obsSets);

        for (size_t bi = 0; bi < batch.size(); ++bi) {
          size_t i = batch[bi];
          if (nets[i]->routable() && nets[i]->unrouted()) anyUnrouted = true;
          Geom::MergeLayerRects(netObstaclesRouted, nets[i]->routeShapesWithPins());
        }
      }
      return anyUnrouted;
    };

    auto escapeCheck = [&](const char* when, const Geom::LayerRects* extra,
                           const bool portsOnly) {
      std::vector<Escape::Pin> epins;
      int netid = 0;
      for (auto& nv : nets) {
        const bool outwardPort = (nv->pins().size() == 1 && _pins.count(nv->name()) > 0);
        const bool want = portsOnly ? outwardPort : (nv->pins().size() >= 2 || outwardPort);
        if (!nv->excluded() && want) {
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
        Geom::LayerRects obs = _obstacles;
        if (extra) Geom::MergeLayerRects(obs, *extra);
        if (Escape::feasible(epins, obs, lm, &blocked, &reason)) {
          COUT << "pin escape SAT (" << when << ") : all " << epins.size() << " pins in "
               << _name << " have a guaranteed escape\n";
        } else {
          COUT << "pin escape SAT (" << when << ") : " << _name << " is infeasible ("
               << reason << ")\n";
          for (auto& b : blocked) COUT << "  no escape for pin : " << b << '\n';
        }
      }
    };
    escapeCheck("pre-route", nullptr, false);


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
    int attemptNo = 0;
    auto attempt = [&]() -> int {
      router.setAttemptNo(++attemptNo);
      // Open-net dumping (when enabled) must fire only on the LAST routing of this
      // attempt -- the adjacent-obstacle retry if it runs, else the base route --
      // so an open net gets exactly one debug LEF reflecting its final context.
      const bool wantDump = router.dumpOpenNets();
      router.setDumpOpenNets(false);
      for (auto& n : _nets) n.second.clearRoutes();
      if (routeAllNets(false)) {
        COUT << "module " << _name << " has unrouted nets; retrying with adjacent-layer pin obstacles\n";
        for (auto& n : _nets) n.second.clearRoutes();
        router.setDumpOpenNets(wantDump);
        routeAllNets(true);
      }
      router.setDumpOpenNets(wantDump);
      return countUnrouted();
    };

    // Open-net debug dumping is deferred to a single final attempt (below), so an
    // intermediate reorder ordering that leaves a net open -- but a later ordering
    // routes -- does not write a stale debug LEF. Suppress it during the search.
    const bool dumpOpen = router.dumpOpenNets();
    const bool rsmtOn = router.rsmtCorridor();
    router.setDumpOpenNets(false);

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

      auto normalize = [&](NetsVec o) -> NetsVec {
        if (!_sympairs.empty()) enforceSymOrder(o);
        return o;
      };
      auto openTail = [&]() -> NetsVec {          // nets the live routes left open
        NetsVec o;
        for (size_t i = pinned; i < nets.size(); ++i) {
          Net* v = nets[i];
          if (!v->excluded() && v->routable() && v->unrouted()) o.push_back(v);
        }
        return o;
      };

      typedef std::pair<int, NetsVec> OrderKey;   // (attempt mode, ordering)
      auto keyFor = [&](const NetsVec& o) -> OrderKey {
        // the attempt this ordering would be routed as is the next one
        return OrderKey(Router::Router::attemptMode(attemptNo + 1), o);
      };
      std::map<OrderKey, NetsVec> tried;          // (mode, ordering) -> nets it left open
      NetsVec openNets = openTail();              // open nets of the latest ordering
      NetsVec routedOrder = nets;                 // ordering the live routes came from
      tried[OrderKey(Router::Router::attemptMode(attemptNo), nets)] = openNets;

      std::mt19937 rng(0);
      bool randomOrder = false;
      const int maxIters = passes * 8;
      const int maxShuffles = 64;
      int pass = 0;
      for (int iter = 0; pass < passes && iter < maxIters && bestUnrouted > 0; ++iter) {
        if (pass == Router::Router::RSMT_RELAX_AFTER_PASS && router.rsmtCorridor()) {
          COUT << "module " << _name << " : relaxing RSMT corridors after "
               << pass << " reorder pass(es)\n";
          router.setRSMTCorridor(false);
        }
        if (openNets.empty()) break;
        for (Net* v : openNets) ++blockCount[v];

        if (!randomOrder) {
          NetsVec newOrder = nets;
          std::stable_sort(newOrder.begin() + pinned, newOrder.end(),
            [&](const Net* a, const Net* b) {
              const int ba = blockCount[a], bb = blockCount[b];
              if (ba != bb) return ba > bb;
              return baseIdx[a] < baseIdx[b];
            });
          newOrder = normalize(std::move(newOrder));
          if (newOrder == nets) {
            randomOrder = true;
            COUT << "module " << _name << " : promotion converged after " << pass
                 << " pass(es); trying random net orderings (seed 0) for the rest\n";
          } else {
            nets = std::move(newOrder);
            auto it = tried.find(keyFor(nets));
            if (it != tried.end()) {
              openNets = it->second;
              continue;
            }
          }
        }
        if (randomOrder) {
          NetsVec cand;
          bool found = false;
          for (int t = 0; t < maxShuffles && !found; ++t) {
            cand = nets;
            std::shuffle(cand.begin() + pinned, cand.end(), rng);
            cand = normalize(std::move(cand));
            found = (tried.find(keyFor(cand)) == tried.end());
          }
          if (!found) break;      // nothing left that has not already been routed
          nets = std::move(cand);
        }

        const OrderKey key = keyFor(nets);
        const int u = attempt();
        routedOrder = nets;
        openNets = openTail();
        tried[key] = openNets;
        ++pass;
        COUT << "  reorder pass " << pass << "/" << passes << " : "
             << u << " unrouted (best so far " << std::min(u, bestUnrouted) << ")\n";
        if (u < bestUnrouted) { bestUnrouted = u; bestOrder = nets; }
      }
      const bool reroute = (routedOrder != bestOrder);
      nets = bestOrder;
      if (reroute) {
        COUT << "module " << _name << " : re-routing with best ordering ("
             << bestUnrouted << " unrouted)\n";
        attempt();
      }
    }

    // Final-pass diagnostics: if any net is still open, reproduce the final
    // ordering once more with open-net dumping on, so only nets open in the FINAL
    // result get a debug LEF (pins/srcs/tgts/obstacles). 'nets' already holds the
    // best (final) ordering, so this attempt is deterministic and identical.
    if (dumpOpen && countUnrouted() > 0) {
      router.setDumpOpenNets(true);
      attempt();
      router.setDumpOpenNets(false);
    }
    router.setRSMTCorridor(rsmtOn);     // restore for sibling hierarchies
    router.setDumpOpenNets(dumpOpen);   // restore for sibling hierarchies

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
    {
      Geom::LayerRects laid;
      for (auto& n : _nets)
        if (!n.second.excluded()) Geom::MergeLayerRects(laid, n.second.routeShapesWithPins());
      escapeCheck("post-route", &laid, true);
    }
    writeDEF(outdir);
  }
  checkDRC(router);
  if (!_leaf) {
    writeLEF(outdir);
  }
  _routed = 1;
  checkShort();
}

void Module::checkShort() const
{
  COUT << "Checking SHORTS for module : " << _name << '\n';
  using namespace boost::polygon::operators;
  std::map<int, std::map<const Net*, PolySet>> byLayer;
  for (auto& n : _nets) {
    if (n.second.excluded()) continue;
    for (auto& l : n.second.routeShapesWithPins()) {
      auto& ps = byLayer[l.first][&n.second];
      for (auto& r : l.second) ps.insert(PRect(r.xmin(), r.ymin(), r.xmax(), r.ymax()));
    }
  }

  int shorts = 0;
  // net vs net
  for (auto& lv : byLayer) {
    const int z = lv.first;
    std::vector<std::pair<const Net*, const PolySet*>> v;
    v.reserve(lv.second.size());
    for (auto& kv : lv.second) v.emplace_back(kv.first, &kv.second);
    for (size_t i = 0; i < v.size(); ++i) {
      for (size_t j = i + 1; j < v.size(); ++j) {
        PolySet x(*v[i].second);
        x &= *v[j].second;
        PRects rs;
        get_rectangles(rs, x);
        for (const auto& r : rs) {
          const Geom::Rect o(bp::xl(r), bp::yl(r), bp::xh(r), bp::yh(r));
          COUT << "SHORT (router or pin) between " << v[i].first->name() << " & "
               << v[j].first->name() << " @ layer : " << z << '\n';
          COUT << o.str() << '\n';
          ++shorts;
        }
      }
    }
  }

  for (auto& n : _nets) {
    if (n.second.excluded()) continue;
    auto lit = n.second.routeShapesWithPins();
    if (lit.empty()) continue;
    const Geom::LayerRects obs = n.second.dropSameNetObstacles(_obstacles);
    for (auto& lv : byLayer) {
      const int z = lv.first;
      auto nit = lv.second.find(&n.second);
      if (nit == lv.second.end()) continue;
      auto oit = obs.find(z);
      if (oit == obs.end() || oit->second.empty()) continue;
      PolySet ops;
      for (const auto& r : oit->second) ops.insert(PRect(r.xmin(), r.ymin(), r.xmax(), r.ymax()));
      ops &= nit->second;
      PRects rs;
      get_rectangles(rs, ops);
      for (const auto& r : rs) {
        const Geom::Rect o(bp::xl(r), bp::yl(r), bp::xh(r), bp::yh(r));
        COUT << "SHORT between " << n.second.name() << " & obstacle @ layer : " << z << '\n';
        COUT << o.str() << '\n';
        ++shorts;
      }
    }
  }
  if (shorts) COUT << "SHORT_SUMMARY module " << _name << " : " << shorts << " short(s)\n";
}

int Module::checkDRC(const Router::Router& router) const
{
  if (_leaf) return 0;
  // route() is re-entered for a module already routed as someone's child, so this
  // can run more than once: start from a clean slate rather than accumulating.
  _drcmarkers.clear();
  _drccount = 0;
  _drcplacement = 0;
  struct Shape {
    Geom::Rect r;
    const void* owner;   // net identity; shapes of one net never conflict
    bool routed;
  };
  std::map<int, std::vector<Shape>> byLayer;
  for (auto& n : _nets) {
    if (n.second.excluded()) continue;
    const void* id = &n.second;
    for (auto& l : n.second.routeShapes())
      for (auto& r : l.second) byLayer[l.first].push_back({r, id, true});
    // A net's own pins are input geometry, but carry its identity so that a
    // route landing on the pin it is connecting to is never reported.
    for (auto& pin : n.second.pins())
      for (auto& p : pin->ports())
        for (auto& l : p->shapes())
          for (auto& r : l.second) byLayer[l.first].push_back({r, id, false});
  }
  // Only pairs with a routed side are ever reported, so input geometry that is
  // nowhere near a route can neither raise a violation nor bridge one polygon to
  // another that we would compare. Bound each layer by the routed extent, grown
  // by the spacing, and drop everything outside -- without this the union and the
  // pair scan both run over every obstacle in the module and become the dominant
  // cost on a large hierarchy.
  std::map<int, Geom::Rect> routedBox;
  for (auto& lv : byLayer) {
    Geom::Rect b;
    for (auto& s : lv.second) if (s.routed) b.merge(s.r);
    if (b.valid()) {
      const int pad = std::max(router.drcSpaceX(lv.first), router.drcSpaceY(lv.first)) + 1;
      routedBox[lv.first] = b.bloatby(pad, pad);
    }
  }
  auto nearRoutes = [&](int l, const Geom::Rect& r) {
    auto it = routedBox.find(l);
    return it != routedBox.end() && it->second.overlaps(r);
  };
  for (auto& l : _obstacles)
    for (auto& r : l.second)
      if (nearRoutes(l.first, r)) byLayer[l.first].push_back({r, nullptr, false});

  // Rectangles that touch or overlap are one piece of metal, so spacing between
  // them is meaningless -- a wire and the via pad it lands on, or a pin and the
  // route that connects to it, are a single polygon however many rects describe
  // it. Merge per layer with boost::polygon first and only ever measure across
  // merged polygons; comparing raw rectangles fires on two halves of one shape.
  //
  // Per routed polygon P: anything that lies within the spacing of P but is not
  // part of P is a violation. Grow P by the spacing, intersect with all the metal
  // on the layer, and subtract P itself -- what survives is exactly the intruding
  // metal, and it doubles as the marker geometry.
  using namespace boost::polygon::operators;
  int count = 0, placementCount = 0;
  std::set<std::tuple<int, int, int, int, int>> seen;
  for (auto& lv : byLayer) {
    const int z = lv.first;
    if (z < router.minLayer() || z > router.maxLayer()) continue;
    const int sx = router.drcSpaceX(z), sy = router.drcSpaceY(z);
    if (sx <= 0 && sy <= 0) continue;
    // Merge each net's own metal (its routes plus the pins they land on) into
    // polygons. Merging is deliberately per net: the obstacle set is a bag of
    // unrelated metal from many cells, so unioning everything chains a route to
    // half the layer through shapes it has nothing to do with and hides real
    // violations.
    std::map<const void*, PolySet> byOwner;
    std::set<const void*> routedOwners;
    for (const auto& s : lv.second) {
      byOwner[s.owner].insert(PRect(s.r.xmin(), s.r.ymin(), s.r.xmax(), s.r.ymax()));
      if (s.routed) routedOwners.insert(s.owner);
    }
    for (const void* id : routedOwners) {
      PolySet pinps;
      for (const auto& sh : lv.second)
        if (sh.owner == id && !sh.routed)
          pinps.insert(PRect(sh.r.xmin(), sh.r.ymin(), sh.r.xmax(), sh.r.ymax()));
      PRects pinr;
      get_max_rectangles(pinr, pinps);
      auto preExisting = [&](const Geom::Rect& bb) {
        for (const auto& pr : pinr) {
          const Geom::Rect q(bp::xl(pr), bp::yl(pr), bp::xh(pr), bp::yh(pr));
          if (q.overlaps(bb)) return false;
          const int qdx = std::max(0, std::max(bb.xmin() - q.xmax(), q.xmin() - bb.xmax()));
          const int qdy = std::max(0, std::max(bb.ymin() - q.ymax(), q.ymin() - bb.ymax()));
          if (qdx == 0 && qdy == 0) continue;
          if (qdx < sx && qdy < sy) return true;
        }
        return false;
      };
      PRects selfr;
      get_max_rectangles(selfr, byOwner[id]);   // this net as merged polygons
      for (const auto& s : lv.second) {
        if (s.owner == id) continue;            // same net: never a spacing check
        const Geom::Rect& b = s.r;
        bool touching = false;
        for (const auto& pr : selfr) {
          const Geom::Rect a(bp::xl(pr), bp::yl(pr), bp::xh(pr), bp::yh(pr));
          if (a.overlaps(b)) { touching = true; break; }   // one polygon, or a short
        }
        if (touching) continue;
        for (const auto& pr : selfr) {
          const Geom::Rect a(bp::xl(pr), bp::yl(pr), bp::xh(pr), bp::yh(pr));
          const int dx = std::max(0, std::max(b.xmin() - a.xmax(), a.xmin() - b.xmax()));
          const int dy = std::max(0, std::max(b.ymin() - a.ymax(), a.ymin() - b.ymax()));
          if (dx == 0 && dy == 0) continue;               // overlap: checkShort's job
          if (dx >= sx || dy >= sy) continue;             // clear on at least one axis
          Geom::Rect m(a);
          m.merge(b);
          if (!seen.emplace(z, m.xmin(), m.ymin(), m.xmax(), m.ymax()).second) continue;
          const bool placement = !s.routed && preExisting(b);
          COUT << "DRC spacing " << (placement ? "(placement) " : "(router) ")
               << _name << " layer " << layerName(z) << " : "
               << a.str() << " vs " << b.str()
               << " dx=" << dx << " dy=" << dy << " need " << sx << 'x' << sy << '\n';
          _drcmarkers[z].push_back(m);
          if (placement) ++placementCount; else ++count;
        }
      }
    }
  }
  if (count || placementCount)
    COUT << "DRC module " << _name << " : " << count << " router-caused, "
         << placementCount << " placement spacing violation(s)\n";
  _drccount = count;
  _drcplacement = placementCount;
  return count;
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
          auto ppos = p->name().rfind(SEPARATOR);
          if (ppos != std::string::npos) {
            instname = p->name().substr(0, ppos);
            pinname  = p->name().substr(ppos + SEPARATOR.size());
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
      for (auto& n : _nets) {
        if (n.second.excluded()) continue;
        for (auto& l : n.second.routeShapes()) {
          if (l.second.empty()) continue;
          ofs << "      LAYER SOL_" << layerName(l.first) << '_' << n.first << " ;\n";
          for (auto& r : l.second) {
            ofs << "        RECT " << (1.*r.xmin()/_uu) << ' ' << (1.*r.ymin()/_uu) << ' '
                << (1.*r.xmax()/_uu) << ' ' << (1.*r.ymax()/_uu) << " ;\n";
          }
        }
      }
      for (auto& n : _nets) {
        if (n.second.corridorEdges().empty()) continue;
        ofs << "      LAYER RSMT_" << n.first << " ;\n";
        for (auto& r : n.second.corridorEdges()) {
          ofs << "        RECT " << (1.*r.xmin()/_uu) << ' ' << (1.*r.ymin()/_uu) << ' '
              << (1.*r.xmax()/_uu) << ' ' << (1.*r.ymax()/_uu) << " ;\n";
        }
      }
      for (auto& l : _drcmarkers) {
        ofs << "      LAYER DRC_" << layerName(l.first) << " ;\n";
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
