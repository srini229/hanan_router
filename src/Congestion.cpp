#include "Congestion.h"
#include "Sat.h"
#include <algorithm>

namespace Congestion {

namespace {

// at-most-k over 'lits' using the Sinz sequential-counter encoding.
void atMostK(Sat::Solver& sat, const std::vector<int>& lits, const int k)
{
  const int n = static_cast<int>(lits.size());
  if (k >= n) return;                                  // always satisfiable
  if (k == 0) { for (const int l : lits) sat.addClause({-l}); return; }
  std::vector<std::vector<int>> s(n, std::vector<int>(k));
  for (int i = 0; i < n; ++i) for (int j = 0; j < k; ++j) s[i][j] = sat.newVar();
  sat.addClause({-lits[0], s[0][0]});
  for (int j = 1; j < k; ++j) sat.addClause({-s[0][j]});
  for (int i = 1; i < n; ++i) {
    sat.addClause({-lits[i], s[i][0]});
    sat.addClause({-s[i - 1][0], s[i][0]});
    for (int j = 1; j < k; ++j) {
      sat.addClause({-lits[i], -s[i - 1][j - 1], s[i][j]});
      sat.addClause({-s[i - 1][j], s[i][j]});
    }
    sat.addClause({-lits[i], -s[i - 1][k - 1]});
  }
}

// length of [lo,hi] covered by obstacles on layer z that cross the cut line
// (a vertical cut at x=fixed, or a horizontal cut at y=fixed).
int coveredLen(const bool vcut, const int fixed, const int lo, const int hi,
               const int z, const Geom::LayerRects* obs)
{
  if (!obs) return 0;
  auto it = obs->find(z);
  if (it == obs->end()) return 0;
  std::vector<std::pair<int, int>> iv;
  for (const auto& o : it->second) {
    if (vcut) {
      if (o.xmin() <= fixed && fixed <= o.xmax()) {
        const int a = std::max(o.ymin(), lo), b = std::min(o.ymax(), hi);
        if (a < b) iv.emplace_back(a, b);
      }
    } else {
      if (o.ymin() <= fixed && fixed <= o.ymax()) {
        const int a = std::max(o.xmin(), lo), b = std::min(o.xmax(), hi);
        if (a < b) iv.emplace_back(a, b);
      }
    }
  }
  std::sort(iv.begin(), iv.end());
  int total = 0, curlo = 0, curhi = -1;
  for (const auto& p : iv) {
    if (p.first > curhi) { if (curhi > curlo) total += curhi - curlo; curlo = p.first; curhi = p.second; }
    else curhi = std::max(curhi, p.second);
  }
  if (curhi > curlo) total += curhi - curlo;
  return total;
}

// free track capacity of a cut, summed over routing layers, after removing
// obstacle-blocked length.
int cutCapacity(const bool vcut, const int fixed, const int lo, const int hi, const Model& m)
{
  int cap = 0;
  for (int z = m.minLayer; z <= m.maxLayer; ++z) {
    const int pitch = vcut ? m.pitchY(z) : m.pitchX(z);
    if (pitch <= 0) continue;
    const int freeLen = std::max(0, (hi - lo) - coveredLen(vcut, fixed, lo, hi, z, m.obstacles));
    cap += freeLen / pitch;
  }
  return cap;
}

} // namespace

bool feasible(const std::vector<Demand>& demands, const Model& m, std::string* reason)
{
  const int W = m.bbox.width(), H = m.bbox.height();
  if (demands.size() < 2 || W <= 0 || H <= 0) return true;

  // largest routing pitch sets a sane cell size: keep cells several tracks wide
  // so a single channel has capacity > 1 and the grid stays small.
  int maxPitch = 1;
  for (int z = m.minLayer; z <= m.maxLayer; ++z)
    maxPitch = std::max({maxPitch, m.pitchX(z), m.pitchY(z)});
  const int cellTarget = std::max(1, maxPitch * 2);
  const int NX = std::min(16, std::max(2, (W + cellTarget - 1) / cellTarget));
  const int NY = std::min(16, std::max(2, (H + cellTarget - 1) / cellTarget));
  const int cellW = std::max(1, W / NX), cellH = std::max(1, H / NY);

  auto col = [&](int x) { return std::min(NX - 1, std::max(0, (x - m.bbox.xmin()) / cellW)); };
  auto row = [&](int y) { return std::min(NY - 1, std::max(0, (y - m.bbox.ymin()) / cellH)); };
  auto veId = [&](int i, int j) { return j * NX + i; };            // i in [0,NX-1)
  auto heId = [&](int i, int j) { return NX * NY + j * NX + i; };  // j in [0,NY-1)
  const int nEdges = 2 * NX * NY;

  Sat::Solver sat;
  std::vector<std::vector<int>> edgeLits(nEdges);     // candidate vars crossing edge
  std::vector<std::vector<int>> edgeDemand(nEdges);   // parallel demand index
  std::vector<int> edgeCap(nEdges, 0);

  auto addCross = [&](int eid, int var, int dem) {
    edgeLits[eid].push_back(var);
    edgeDemand[eid].push_back(dem);
  };

  bool anyDemand = false;
  for (size_t d = 0; d < demands.size(); ++d) {
    const auto& b = demands[d].box;
    int i1 = col(b.xmin()), i2 = col(b.xmax());
    int j1 = row(b.ymin()), j2 = row(b.ymax());
    if (i1 > i2) std::swap(i1, i2);
    if (j1 > j2) std::swap(j1, j2);
    if (i1 == i2 && j1 == j2) continue;   // net stays within one cell

    std::vector<int> candVars;
    auto buildCand = [&](const std::vector<int>& edges) {
      const int v = sat.newVar();
      candVars.push_back(v);
      for (const int e : edges) addCross(e, v, static_cast<int>(d));
    };
    // monotone routes that bend at column c (horiz @ j1, vert @ c, horiz @ j2)
    for (int c = i1; c <= i2; ++c) {
      std::vector<int> e;
      for (int i = i1; i < c;  ++i) e.push_back(veId(i, j1));
      for (int j = j1; j < j2; ++j) e.push_back(heId(c, j));
      for (int i = c;  i < i2; ++i) e.push_back(veId(i, j2));
      buildCand(e);
    }
    // monotone routes that bend at row r (vert @ i1, horiz @ r, vert @ i2)
    if (j1 < j2 && i1 < i2) {
      for (int r = j1; r <= j2; ++r) {
        std::vector<int> e;
        for (int j = j1; j < r;  ++j) e.push_back(heId(i1, j));
        for (int i = i1; i < i2; ++i) e.push_back(veId(i, r));
        for (int j = r;  j < j2; ++j) e.push_back(heId(i2, j));
        buildCand(e);
      }
    }
    if (candVars.empty()) continue;
    anyDemand = true;
    sat.addClause(candVars);                                   // at least one route
    for (size_t a = 0; a < candVars.size(); ++a)               // at most one route
      for (size_t bb = a + 1; bb < candVars.size(); ++bb)
        sat.addClause({-candVars[a], -candVars[bb]});
  }
  if (!anyDemand) return true;

  int worstEdge = -1, worstOver = 0;
  for (int e = 0; e < nEdges; ++e) {
    if (edgeLits[e].empty()) continue;
    // obstacle-aware capacity for this specific cut
    int cap;
    if (e < NX * NY) {   // vertical cut VE(i,j)
      const int i = e % NX, j = e / NX;
      const int x = m.bbox.xmin() + (i + 1) * cellW;
      const int y0 = m.bbox.ymin() + j * cellH, y1 = m.bbox.ymin() + (j + 1) * cellH;
      cap = cutCapacity(true, x, y0, y1, m);
    } else {             // horizontal cut HE(i,j)
      const int e2 = e - NX * NY, i = e2 % NX, j = e2 / NX;
      const int y = m.bbox.ymin() + (j + 1) * cellH;
      const int x0 = m.bbox.xmin() + i * cellW, x1 = m.bbox.xmin() + (i + 1) * cellW;
      cap = cutCapacity(false, y, x0, x1, m);
    }
    edgeCap[e] = cap;
    std::vector<int> distinct(edgeDemand[e]);
    std::sort(distinct.begin(), distinct.end());
    distinct.erase(std::unique(distinct.begin(), distinct.end()), distinct.end());
    const int dem = static_cast<int>(distinct.size());
    if (dem <= cap) continue;                          // can never overflow -> skip
    if (dem - cap > worstOver) { worstOver = dem - cap; worstEdge = e; }
    atMostK(sat, edgeLits[e], cap);
  }

  const int res = sat.solve();
  if (res == 1 || res == -1) return true;             // sat or inconclusive
  if (reason) {
    const bool ve = worstEdge < NX * NY;
    *reason = "global routing congestion (unsat); tightest channel needs ~"
            + std::to_string(worstOver) + " more "
            + (ve ? "horizontal" : "vertical") + " track(s)";
  }
  return false;
}

}
