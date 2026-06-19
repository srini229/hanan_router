#include "CpRoute.h"
#include <algorithm>
#include <map>
#include <string>

namespace CpRoute {

#ifndef USE_ORTOOLS

bool available() { return false; }

bool route(const std::vector<Net>&, const Geom::LayerRects&, const LayerModel&,
           double, std::vector<Geom::LayerRects>&, std::string* msg)
{
  if (msg) *msg = "CP-SAT routing not built (rebuild with: make CPSAT=1)";
  return false;
}

#else

} // namespace CpRoute (reopened below after the OR-Tools includes)

#include "ortools/sat/cp_model.h"

namespace CpRoute {

using operations_research::Domain;
using namespace operations_research::sat;

namespace {

struct Edge { int a, b; int layer; bool via; };   // grid nodes a,b

bool pointBlocked(int x, int y, int layer, const Geom::LayerRects& obs)
{
  auto it = obs.find(layer);
  if (it == obs.end()) return false;
  for (const auto& r : it->second)
    if (x > r.xmin() && x < r.xmax() && y > r.ymin() && y < r.ymax()) return true;
  return false;
}

} // namespace

bool available() { return true; }

bool route(const std::vector<Net>& nets, const Geom::LayerRects& obstacles,
           const LayerModel& lm, double timeLimitSec,
           std::vector<Geom::LayerRects>& routes, std::string* msg)
{
  routes.assign(nets.size(), Geom::LayerRects());
  const int W = lm.bbox.width(), H = lm.bbox.height();
  const int NL = lm.maxLayer - lm.minLayer + 1;
  if (W <= 0 || H <= 0 || NL <= 0) { if (msg) *msg = "empty module"; return false; }

  // choose a grid: aim for ~10 cells/side, then coarsen until the model is small
  int NX = std::min(14, std::max(2, W / std::max(1, W / 10)));
  int NY = std::min(14, std::max(2, H / std::max(1, H / 10)));
  NX = std::max(2, std::min(NX, 12));
  NY = std::max(2, std::min(NY, 12));
  while (static_cast<long>(nets.size()) * NX * NY * NL > 9000 && (NX > 2 || NY > 2)) {
    if (NX >= NY && NX > 2) --NX; else if (NY > 2) --NY; else break;
  }
  const int cellW = std::max(1, W / NX), cellH = std::max(1, H / NY);
  auto col = [&](int x) { return std::min(NX - 1, std::max(0, (x - lm.bbox.xmin()) / cellW)); };
  auto row = [&](int y) { return std::min(NY - 1, std::max(0, (y - lm.bbox.ymin()) / cellH)); };
  auto cx  = [&](int i) { return lm.bbox.xmin() + i * cellW + cellW / 2; };
  auto cy  = [&](int j) { return lm.bbox.ymin() + j * cellH + cellH / 2; };
  auto nid = [&](int i, int j, int l) { return ((l - lm.minLayer) * NY + j) * NX + i; };
  const int N = NX * NY * NL;

  // terminals: map each pin to the grid node of its (first shape's) centre
  std::vector<std::vector<int>> termsOf(nets.size());
  std::vector<int> termCount(N, 0);
  for (size_t t = 0; t < nets.size(); ++t) {
    std::vector<int> ts;
    for (const auto& pin : nets[t].pins) {
      if (pin.empty()) continue;
      const auto& l = *pin.begin();
      int L = std::min(lm.maxLayer, std::max(lm.minLayer, l.first));
      if (l.second.empty()) continue;
      const auto& r = l.second.front();
      ts.push_back(nid(col(r.xcenter()), row(r.ycenter()), L));
    }
    std::sort(ts.begin(), ts.end());
    ts.erase(std::unique(ts.begin(), ts.end()), ts.end());
    termsOf[t] = ts;
    for (int v : ts) ++termCount[v];
  }

  // blocked nodes (obstacle covers the cell centre) -- pins always allowed
  std::vector<char> blocked(N, 0);
  for (int l = lm.minLayer; l <= lm.maxLayer; ++l)
    for (int j = 0; j < NY; ++j)
      for (int i = 0; i < NX; ++i)
        if (pointBlocked(cx(i), cy(j), l, obstacles)) blocked[nid(i, j, l)] = 1;
  for (auto& ts : termsOf) for (int v : ts) blocked[v] = 0;

  // grid edges between available nodes
  std::vector<Edge> edges;
  std::vector<std::vector<int>> incident(N);   // node -> edge indices
  auto addEdge = [&](int a, int b, int layer, bool via) {
    if (blocked[a] || blocked[b]) return;
    const int e = static_cast<int>(edges.size());
    edges.push_back({a, b, layer, via});
    incident[a].push_back(e);
    incident[b].push_back(e);
  };
  for (int l = lm.minLayer; l <= lm.maxLayer; ++l) {
    for (int j = 0; j < NY; ++j) for (int i = 0; i < NX; ++i) {
      if (i + 1 < NX) addEdge(nid(i, j, l), nid(i + 1, j, l), l, false);
      if (j + 1 < NY) addEdge(nid(i, j, l), nid(i, j + 1, l), l, false);
      if (l + 1 <= lm.maxLayer && lm.canUp(l)) addEdge(nid(i, j, l), nid(i, j, l + 1), l, true);
    }
  }

  // ---- CP-SAT model ----
  CpModelBuilder cp;
  // y[t][e] : net t uses edge e ; nodeUsed[t][v] : net t occupies node v
  std::vector<std::map<int, BoolVar>> nodeUsed(nets.size());
  std::vector<std::vector<BoolVar>> Y(nets.size());
  std::vector<LinearExpr> obj1;   // collected objective terms
  LinearExpr objective;
  bool anyNet = false;

  for (size_t t = 0; t < nets.size(); ++t) {
    const auto& ts = termsOf[t];
    if (ts.size() < 2) continue;
    anyNet = true;
    const int K = static_cast<int>(ts.size()) - 1;
    auto& y = Y[t];
    y.resize(edges.size());
    std::vector<IntVar> fwd(edges.size()), bwd(edges.size());
    for (size_t e = 0; e < edges.size(); ++e) {
      y[e] = cp.NewBoolVar();
      fwd[e] = cp.NewIntVar(Domain(0, K));
      bwd[e] = cp.NewIntVar(Domain(0, K));
      cp.AddLessOrEqual(fwd[e] + bwd[e], y[e] * K);            // flow only on used edges
      objective += y[e] * (edges[e].via ? 3 : 1);              // wirelength + via penalty
    }
    auto nu = [&](int v) -> BoolVar {
      auto it = nodeUsed[t].find(v);
      if (it != nodeUsed[t].end()) return it->second;
      BoolVar b = cp.NewBoolVar();
      nodeUsed[t].emplace(v, b);
      return b;
    };
    // y[e] => both endpoints occupied
    for (size_t e = 0; e < edges.size(); ++e) {
      cp.AddImplication(y[e], nu(edges[e].a));
      cp.AddImplication(y[e], nu(edges[e].b));
    }
    // flow conservation: in - out = demand (root supplies K, other terminals take 1)
    const int root = ts[0];
    std::vector<int> demand(N, 0);
    demand[root] = -K;
    for (size_t k = 1; k < ts.size(); ++k) demand[ts[k]] = 1;
    for (int v = 0; v < N; ++v) {
      if (incident[v].empty() && demand[v] == 0) continue;
      LinearExpr bal;
      for (int e : incident[v]) {
        if (edges[e].a == v) bal += bwd[e] - fwd[e];          // into v = b->a, out = a->b
        else                 bal += fwd[e] - bwd[e];          // into v = a->b, out = b->a
      }
      cp.AddEquality(bal, demand[v]);
    }
    for (int v : ts) cp.AddEquality(nu(v), 1);                 // terminals occupied
  }

  if (!anyNet) { if (msg) *msg = "no multi-pin nets"; return true; }

  // node-disjoint: at most one net occupies each node (skip nodes shared by
  // terminals of several nets -- a grid-resolution artifact)
  for (int v = 0; v < N; ++v) {
    if (termCount[v] > 1) continue;
    std::vector<BoolVar> users;
    for (size_t t = 0; t < nets.size(); ++t) {
      auto it = nodeUsed[t].find(v);
      if (it != nodeUsed[t].end()) users.push_back(it->second);
    }
    if (users.size() > 1) cp.AddAtMostOne(users);
  }

  cp.Minimize(objective);

  Model model;
  SatParameters params;
  params.set_max_time_in_seconds(timeLimitSec > 0 ? timeLimitSec : 10.0);
  params.set_num_search_workers(8);
  model.Add(NewSatParameters(params));
  const CpSolverResponse resp = SolveCpModel(cp.Build(), &model);

  if (resp.status() != CpSolverStatus::OPTIMAL && resp.status() != CpSolverStatus::FEASIBLE) {
    if (msg) *msg = "CP-SAT found no routing (status " + std::to_string(resp.status()) + ")";
    return false;
  }

  // ---- decode solution into rectangles ----
  for (size_t t = 0; t < nets.size(); ++t) {
    if (Y[t].empty()) continue;
    auto& out = routes[t];
    const int hwDefault = std::max(1, lm.width(lm.minLayer) / 2);
    for (size_t e = 0; e < edges.size(); ++e) {
      if (!SolutionBooleanValue(resp, Y[t][e])) continue;
      const Edge& ed = edges[e];
      const int ia = ed.a % NX, ja = (ed.a / NX) % NY, la = ed.a / (NX * NY) + lm.minLayer;
      const int ib = ed.b % NX, jb = (ed.b / NX) % NY, lb = ed.b / (NX * NY) + lm.minLayer;
      if (ed.via) {
        const int hw = std::max(hwDefault, lm.width(la) / 2);
        out[la].emplace_back(cx(ia) - hw, cy(ja) - hw, cx(ia) + hw, cy(ja) + hw);
        out[lb].emplace_back(cx(ib) - hw, cy(jb) - hw, cx(ib) + hw, cy(jb) + hw);
      } else {
        const int hw = std::max(1, lm.width(la) / 2);
        if (ja == jb)   // horizontal
          out[la].emplace_back(std::min(cx(ia), cx(ib)), cy(ja) - hw, std::max(cx(ia), cx(ib)), cy(ja) + hw);
        else            // vertical
          out[la].emplace_back(cx(ia) - hw, std::min(cy(ja), cy(jb)), cx(ia) + hw, std::max(cy(ja), cy(jb)));
      }
    }
    // connect each pin to its grid-cell centre and keep the pin shape
    for (const auto& pin : nets[t].pins) {
      if (pin.empty()) continue;
      const auto& l = *pin.begin();
      if (l.second.empty()) continue;
      const int L = std::min(lm.maxLayer, std::max(lm.minLayer, l.first));
      const int hw = std::max(1, lm.width(L) / 2);
      const auto& r = l.second.front();
      const int pcx = r.xcenter(), pcy = r.ycenter();
      const int tx = cx(col(pcx)), ty = cy(row(pcy));
      out[L].push_back(r);
      out[L].emplace_back(std::min(pcx, tx), pcy - hw, std::max(pcx, tx), pcy + hw);
      out[L].emplace_back(tx - hw, std::min(pcy, ty), tx + hw, std::max(pcy, ty));
    }
  }
  if (msg) *msg = "CP-SAT routed " + std::to_string(nets.size()) + " nets on a "
               + std::to_string(NX) + "x" + std::to_string(NY) + "x" + std::to_string(NL) + " grid";
  return true;
}

#endif

}
