#include "Escape.h"
#include "Sat.h"

namespace Escape {

namespace {

struct Cand {
  int layer;
  Geom::Rect fp;
  int var{0};
};

// spacing: minimum required clearance on `layer` -- an obstacle merely within
// this distance of `r` blocks the candidate, not just one that literally
// overlaps it, matching the real router's isViaValid spacing checks.
bool hitsObstacle(const Geom::Rect& r, const int layer, const Geom::LayerRects& obs, const int spacing)
{
  auto it = obs.find(layer);
  if (it == obs.end()) return false;
  const auto rr = r.bloatby(spacing);
  for (const auto& o : it->second) if (rr.overlaps(o, true)) return true;
  return false;
}

} // namespace

bool feasible(const std::vector<Pin>& pins, const Geom::LayerRects& obstacles,
              const LayerModel& lm, std::vector<std::string>* blocked, std::string* reason)
{
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

  for (size_t pi = 0; pi < pins.size(); ++pi) {
    const Pin& p = pins[pi];
    auto& cands = pinCands[pi];
    for (const auto& l : p.shapes) {
      const int L = l.first;
      if (L < lm.minLayer || L > lm.maxLayer) continue;
      const int w = lm.width(L), s = lm.space(L);
      for (const auto& r : l.second) {
        // via up / via down: the via lands within the pin, so reserve the pin
        // footprint on the destination layer.
        if (L + 1 <= lm.maxLayer && lm.canUp(L) && !hitsObstacle(r, L + 1, blockers, lm.space(L + 1)))
          cands.push_back({L + 1, r, 0});
        if (L - 1 >= lm.minLayer && lm.canDown(L) && !hitsObstacle(r, L - 1, blockers, lm.space(L - 1)))
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
          if (!hitsObstacle(st, L, blockers, s)) cands.push_back({L, st, 0});
      }
    }
    for (auto& c : cands) c.var = sat.newVar();
    if (cands.empty()) {
      anyHardBlocked = true;
      if (blocked) blocked->push_back(p.name);
      continue;
    }
    // exactly one escape per pin: at-least-one + pairwise at-most-one.
    std::vector<int> atLeastOne;
    atLeastOne.reserve(cands.size());
    for (const auto& c : cands) atLeastOne.push_back(c.var);
    sat.addClause(atLeastOne);
    for (size_t a = 0; a < cands.size(); ++a)
      for (size_t b = a + 1; b < cands.size(); ++b)
        sat.addClause({-cands[a].var, -cands[b].var});
  }

  if (anyHardBlocked) {
    if (reason) *reason = "pin(s) with no possible escape";
    return false;
  }

  // escapes of different nets that share a layer and would clash (within
  // spacing) cannot both be chosen.
  for (size_t i = 0; i < pins.size(); ++i) {
    for (size_t j = i + 1; j < pins.size(); ++j) {
      if (pins[i].net == pins[j].net) continue;
      for (const auto& ci : pinCands[i]) {
        for (const auto& cj : pinCands[j]) {
          if (ci.layer != cj.layer) continue;
          if (ci.fp.bloatby(lm.space(ci.layer)).overlaps(cj.fp, true))
            sat.addClause({-ci.var, -cj.var});
        }
      }
    }
  }

  const int res = sat.solve();
  if (res == 1) return true;
  if (res == -1) {   // gave up: don't block routing on an unproven instance
    if (reason) *reason = "sat budget exceeded; assuming feasible";
    return true;
  }
  if (reason) *reason = "escapes mutually conflict (unsat)";
  return false;
}

}
