#include "Escape.h"
#include "Sat.h"
#include <chrono>
#include <functional>
#include <algorithm>

namespace Escape {

namespace {

struct Cand {
  int layer;
  Geom::Rect fp;
  int var{0};
  int srcLayer{-1};
  Geom::Rect srcFp;
};

bool clashes(const Cand& a, const Cand& b, const std::function<int(int)>& space)
{
  auto hit = [&](int la, const Geom::Rect& ra, int lb, const Geom::Rect& rb) {
    return la >= 0 && la == lb && ra.bloatby(space(la)).overlaps(rb, true);
  };
  return hit(a.layer, a.fp, b.layer, b.fp) || hit(a.srcLayer, a.srcFp, b.layer, b.fp) ||
         hit(a.layer, a.fp, b.srcLayer, b.srcFp) || hit(a.srcLayer, a.srcFp, b.srcLayer, b.srcFp);
}

// spacing: minimum required clearance on `layer` -- an obstacle merely within
// this distance of `r` blocks the candidate, not just one that literally
// overlaps it, matching the real router's isViaValid spacing checks.
// `pin` is the shape the escape belongs to. When abutEscape is on, an obstacle
// already touching that pin is judged on interior overlap with the candidate
// alone -- the same relaxation the router makes in isViaValid.
bool hitsObstacle(const Geom::Rect& r, const int layer, const Geom::LayerRects& obs,
                  const int spacing, const Geom::Rect* pin = nullptr)
{
  auto it = obs.find(layer);
  if (it == obs.end()) return false;
  const auto rr = r.bloatby(spacing);
  for (const auto& o : it->second) {
    if (pin && o.overlaps(*pin, false)) {
      if (o.overlaps(r, true)) return true;
      continue;
    }
    if (rr.overlaps(o, true)) return true;
  }
  return false;
}

bool ownShape(const Geom::Rect& o, const int layer, const Geom::LayerRects& own)
{
  auto it = own.find(layer);
  if (it == own.end()) return false;
  for (const auto& r : it->second)
    if (r.xmin() == o.xmin() && r.ymin() == o.ymin() && r.xmax() == o.xmax() && r.ymax() == o.ymax()) return true;
  return false;
}

// hitsObstacle, but a pin's own shapes never block its own escape
bool hitsOther(const Geom::Rect& r, const int layer, const Geom::LayerRects& obs, const int spacing,
               const Geom::LayerRects& own)
{
  auto it = obs.find(layer);
  if (it == obs.end()) return false;
  const auto rr = r.bloatby(spacing);
  for (const auto& o : it->second)
    if (rr.overlaps(o, true) && !ownShape(o, layer, own)) return true;
  return false;
}

// escape points along a shape's centre line, step apart, both ends included
std::vector<Geom::Point> samples(const Geom::Rect& r, const int step, const int w)
{
  std::vector<Geom::Point> out;
  const bool horiz = r.width() >= r.height();
  const int c = horiz ? r.ycenter() : r.xcenter();
  const int lo = (horiz ? r.xmin() : r.ymin()) + w / 2, hi = (horiz ? r.xmax() : r.ymax()) - w / 2;
  const int mid = horiz ? r.xcenter() : r.ycenter();
  std::vector<int> t;
  if (hi < lo) t.push_back(mid);
  else {
    t.push_back(lo); t.push_back(hi);
    for (int k = 0; step > 0 && mid + k * step <= hi; ++k) {
      t.push_back(mid + k * step);
      if (k && mid - k * step >= lo) t.push_back(mid - k * step);
    }
  }
  std::sort(t.begin(), t.end());
  t.erase(std::unique(t.begin(), t.end()), t.end());
  for (const int v : t) out.push_back(horiz ? Geom::Point(v, c) : Geom::Point(c, v));
  return out;
}

} // namespace

bool feasible(const std::vector<Pin>& pins, const Geom::LayerRects& obstacles,
              const LayerModel& lm, std::vector<std::string>* blocked, std::string* reason,
              std::vector<Chosen>* chosen, Stats* stats)
{
  const auto t0 = std::chrono::steady_clock::now();
  auto done = [&](const int result, const Sat::Solver& sat, const int nblocked) {
    if (!stats) return;
    stats->pins = static_cast<int>(pins.size());
    stats->vars = sat.numVars();
    stats->clauses = sat.numClauses();
    stats->blockedPins = nblocked;
    stats->result = result;
    stats->seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  };
  // Static blockers an escape may not land on: real obstacles plus every pin's
  // own fixed shapes (a pin cannot escape on top of another fixed pin). The
  // pin's own shape never invalidates its own escapes: vias go to a different
  // layer and same-layer stubs only abut it (interior-overlap test).
  Geom::LayerRects blockers = obstacles;
  for (const auto& p : pins)
    for (const auto& l : p.shapes)
      for (const auto& r : l.second) blockers[l.first].push_back(r);

  Sat::Solver sat;
  std::vector<std::vector<Cand>> pinCands(pins.size());
  bool anyHardBlocked = false;
  int nblocked = 0;

  const bool pointModel = lm.viaPads && lm.step;
  for (size_t pi = 0; pi < pins.size(); ++pi) {
    const Pin& p = pins[pi];
    auto& cands = pinCands[pi];
    for (const auto& l : p.shapes) {
      if (!pointModel) continue;
      const int L = l.first;
      if (L < lm.minLayer || L > lm.maxLayer) continue;
      const int w = lm.width(L), s = lm.space(L);
      for (const auto& r : l.second) {
        const auto pts = samples(r, lm.step(L), w);
        const bool horiz = r.width() >= r.height();
        for (size_t k = 0; k < pts.size(); ++k) {
          const int x = pts[k].x(), y = pts[k].y();
          for (const bool up : {true, false}) {
            const int D = up ? L + 1 : L - 1;
            if (D < lm.minLayer || D > lm.maxLayer || !(up ? lm.canUp(L) : lm.canDown(L))) continue;
            for (const auto& pp : lm.viaPads(L, up)) {
              const Geom::Rect src(pp.first.xmin() + x, pp.first.ymin() + y, pp.first.xmax() + x, pp.first.ymax() + y);
              const Geom::Rect dst(pp.second.xmin() + x, pp.second.ymin() + y, pp.second.xmax() + x, pp.second.ymax() + y);
              if (!hitsOther(src, L, blockers, s, p.shapes) && !hitsOther(dst, D, blockers, lm.space(D), p.shapes)) {
                cands.push_back({D, dst, 0, L, src});
                break;
              }
            }
          }
          // stubs across the shape from every point, along it only from its two ends
          std::vector<Geom::Rect> stubs;
          if (horiz) {
            stubs.emplace_back(x - w / 2, r.ymax(), x + w / 2, r.ymax() + s + w);
            stubs.emplace_back(x - w / 2, r.ymin() - s - w, x + w / 2, r.ymin());
            if (k == 0) stubs.emplace_back(r.xmin() - s - w, y - w / 2, r.xmin(), y + w / 2);
            if (k + 1 == pts.size()) stubs.emplace_back(r.xmax(), y - w / 2, r.xmax() + s + w, y + w / 2);
          } else {
            stubs.emplace_back(r.xmax(), y - w / 2, r.xmax() + s + w, y + w / 2);
            stubs.emplace_back(r.xmin() - s - w, y - w / 2, r.xmin(), y + w / 2);
            if (k == 0) stubs.emplace_back(x - w / 2, r.ymin() - s - w, x + w / 2, r.ymin());
            if (k + 1 == pts.size()) stubs.emplace_back(x - w / 2, r.ymax(), x + w / 2, r.ymax() + s + w);
          }
          for (const auto& st : stubs)
            if (!hitsOther(st, L, blockers, s, p.shapes)) cands.push_back({L, st, 0});
        }
      }
    }
    for (const auto& l : p.shapes) {
      if (pointModel) continue;
      const int L = l.first;
      if (L < lm.minLayer || L > lm.maxLayer) continue;
      const int w = lm.width(L), s = lm.space(L);
      for (const auto& r : l.second) {
        // via up / via down: the via lands within the pin, so reserve the pin
        // footprint on the destination layer.
        const Geom::Rect* ap = lm.abutEscape ? &r : nullptr;
        if (L + 1 <= lm.maxLayer && lm.canUp(L) && !hitsObstacle(r, L + 1, blockers, lm.space(L + 1), ap))
          cands.push_back({L + 1, r, 0});
        if (L - 1 >= lm.minLayer && lm.canDown(L) && !hitsObstacle(r, L - 1, blockers, lm.space(L - 1), ap))
          cands.push_back({L - 1, r, 0});
        // same-layer stubs: reach the next track (one space + one width) in each
        // of the four directions.
        const Geom::Rect stubs[4] = {
          Geom::Rect(r.xmax(),         r.ymin(),         r.xmax() + s + w, r.ymax()),
          Geom::Rect(r.xmin() - s - w, r.ymin(),         r.xmin(),         r.ymax()),
          Geom::Rect(r.xmin(),         r.ymax(),         r.xmax(),         r.ymax() + s + w),
          Geom::Rect(r.xmin(),         r.ymin() - s - w, r.xmax(),         r.ymin())
        };
        for (const auto& st : stubs)
          if (!hitsObstacle(st, L, blockers, s, ap)) cands.push_back({L, st, 0});
      }
    }
    for (auto& c : cands) c.var = sat.newVar();
    if (cands.empty()) {
      anyHardBlocked = true;
      ++nblocked;
      if (blocked) blocked->push_back(p.name);
      continue;
    }
    // exactly one escape per pin; at-least-one alone is equivalent, and the point model keeps only that
    std::vector<int> atLeastOne;
    atLeastOne.reserve(cands.size());
    for (const auto& c : cands) atLeastOne.push_back(c.var);
    sat.addClause(atLeastOne);
    for (size_t a = 0; !pointModel && a < cands.size(); ++a)
      for (size_t b = a + 1; b < cands.size(); ++b)
        sat.addClause({-cands[a].var, -cands[b].var});
  }

  auto greedy = [&]() {
    if (!chosen || !lm.greedy) return;
    std::vector<std::pair<int, const Cand*>> taken;
    for (size_t pi = 0; pi < pinCands.size(); ++pi) {
      for (const auto& c : pinCands[pi]) {
        bool clash = false;
        for (const auto& t : taken)
          if (t.first != pins[pi].net && clashes(*t.second, c, lm.space)) { clash = true; break; }
        if (clash) continue;
        chosen->push_back({pi, c.layer, c.fp, c.srcLayer, c.srcFp});
        taken.emplace_back(pins[pi].net, &c);
        break;
      }
    }
  };
  if (anyHardBlocked) {
    if (reason) *reason = "pin(s) with no possible escape";
    done(-2, sat, nblocked);
    greedy();
    return false;
  }

  // escapes of different nets that share a layer and would clash (within
  // spacing) cannot both be chosen.
  for (size_t i = 0; i < pins.size(); ++i) {
    for (size_t j = i + 1; j < pins.size(); ++j) {
      if (pins[i].net == pins[j].net) continue;
      for (const auto& ci : pinCands[i]) {
        for (const auto& cj : pinCands[j]) {
          if (clashes(ci, cj, lm.space)) sat.addClause({-ci.var, -cj.var});
        }
      }
    }
  }

  const int res = sat.solve();
  done(res, sat, 0);
  if (res == 1) {
    if (chosen) {
      for (size_t pi = 0; pi < pinCands.size(); ++pi) {
        for (const auto& c : pinCands[pi]) {
          if (sat.value(c.var)) { chosen->push_back({pi, c.layer, c.fp, c.srcLayer, c.srcFp}); break; }
        }
      }
    }
    return true;
  }
  if (res == -1) {   // gave up: don't block routing on an unproven instance
    if (reason) *reason = "sat budget exceeded; assuming feasible";
    greedy();
    return true;
  }
  if (reason) *reason = "escapes mutually conflict (unsat)";
  greedy();
  return false;
}

}
