#include "borah.h"

#include <algorithm>
#include <chrono>
#include <limits>
#include <numeric>

namespace rsmt {
namespace {

int64_t Clamp(int64_t v, int64_t lo, int64_t hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// Closest rectilinear distance between two rectangles; 0 if they touch/overlap.
int64_t Dist(const Rect &a, const Rect &b) {
  const int64_t dx = std::max(std::max(a.xlo - b.xhi, b.xlo - a.xhi), int64_t{0});
  const int64_t dy = std::max(std::max(a.ylo - b.yhi, b.ylo - a.yhi), int64_t{0});
  return dx + dy;
}

void ClosestPair(const Rect &a, const Rect &b, Point *pa, Point *pb) {
  if (a.xhi < b.xlo)      { pa->x = a.xhi; pb->x = b.xlo; }
  else if (b.xhi < a.xlo) { pa->x = a.xlo; pb->x = b.xhi; }
  else { const int64_t x = std::max(a.xlo, b.xlo); pa->x = x; pb->x = x; }
  if (a.yhi < b.ylo)      { pa->y = a.yhi; pb->y = b.ylo; }
  else if (b.yhi < a.ylo) { pa->y = a.ylo; pb->y = b.yhi; }
  else { const int64_t y = std::max(a.ylo, b.ylo); pa->y = y; pb->y = y; }
}

Point ClosestOnBox(const Rect &p, const Rect &box, int64_t *dist) {
  Point q;
  if (p.xhi < box.xlo) q.x = box.xlo;
  else if (p.xlo > box.xhi) q.x = box.xhi;
  else q.x = Clamp(p.xlo, box.xlo, box.xhi);
  if (p.yhi < box.ylo) q.y = box.ylo;
  else if (p.ylo > box.yhi) q.y = box.yhi;
  else q.y = Clamp(p.ylo, box.ylo, box.yhi);
  *dist = Dist(p, Rect(q));
  return q;
}

}  // namespace

BorahTree BorahOwens(const std::vector<Rect> &raw) {
  using namespace std::chrono;
  const auto t0 = steady_clock::now();

  BorahTree t;
  std::vector<Rect> pts = raw;
  std::sort(pts.begin(), pts.end());
  pts.erase(std::unique(pts.begin(), pts.end()), pts.end());
  const int d = static_cast<int>(pts.size());
  t.nodes = pts;
  if (d <= 1) return t;

  // Rectilinear MST over the terminals (Prim; d is small).
  {
    std::vector<bool> in(d, false);
    std::vector<int64_t> best(d, std::numeric_limits<int64_t>::max());
    std::vector<int> from(d, -1);
    best[0] = 0;
    for (int it = 0; it < d; ++it) {
      int v = -1;
      for (int k = 0; k < d; ++k) {
        if (!in[k] && (v < 0 || best[k] < best[v])) v = k;
      }
      in[v] = true;
      if (from[v] >= 0) t.edges.push_back(std::make_pair(from[v], v));
      for (int k = 0; k < d; ++k) {
        if (in[k]) continue;
        const int64_t dd = Dist(pts[v], pts[k]);
        if (dd < best[k]) { best[k] = dd; from[k] = v; }
      }
    }
  }
  for (size_t i = 0; i < t.edges.size(); ++i) {
    t.mst_length += Dist(t.nodes[t.edges[i].first], t.nodes[t.edges[i].second]);
  }

  for (int iter = 0; iter < 4 * d + 16; ++iter) {
    const int n = static_cast<int>(t.nodes.size());
    std::vector<std::vector<std::pair<int, int> > > adj(n);
    for (size_t e = 0; e < t.edges.size(); ++e) {
      adj[t.edges[e].first].push_back(std::make_pair(t.edges[e].second, static_cast<int>(e)));
      adj[t.edges[e].second].push_back(std::make_pair(t.edges[e].first, static_cast<int>(e)));
    }

    std::vector<std::pair<Point, Point> > epts(t.edges.size());
    for (size_t e = 0; e < t.edges.size(); ++e) {
      ClosestPair(t.nodes[t.edges[e].first], t.nodes[t.edges[e].second],
                  &epts[e].first, &epts[e].second);
    }

    int64_t best_gain = 0;
    int best_p = -1, best_e = -1, best_drop = -1;
    Point best_s;

    for (size_t e = 0; e < t.edges.size(); ++e) {
      const int u = t.edges[e].first, v = t.edges[e].second;
      std::vector<int64_t> max_on_path(n, -1);
      std::vector<int> max_edge(n, -1);
      std::vector<int> stack;
      stack.push_back(u);
      max_on_path[u] = 0;
      max_edge[u] = -1;
      while (!stack.empty()) {
        const int a = stack.back();
        stack.pop_back();
        for (size_t k = 0; k < adj[a].size(); ++k) {
          const int b = adj[a][k].first, eid = adj[a][k].second;
          if (eid == static_cast<int>(e) || max_on_path[b] >= 0) continue;
          const int64_t len = Dist(t.nodes[a], t.nodes[b]);
          if (len > max_on_path[a]) { max_on_path[b] = len; max_edge[b] = eid; }
          else { max_on_path[b] = max_on_path[a]; max_edge[b] = max_edge[a]; }
          stack.push_back(b);
        }
      }

      const Rect box(std::min(epts[e].first.x, epts[e].second.x),
                     std::min(epts[e].first.y, epts[e].second.y),
                     std::max(epts[e].first.x, epts[e].second.x),
                     std::max(epts[e].first.y, epts[e].second.y));
      const int64_t uv = Dist(t.nodes[u], t.nodes[v]);
      for (int p = 0; p < n; ++p) {
        if (max_on_path[p] <= 0 || max_edge[p] < 0) continue;
        int64_t cost = 0;
        const Point s = ClosestOnBox(t.nodes[p], box, &cost);
        const Rect srect(s);
        const int64_t split = Dist(t.nodes[u], srect) + Dist(srect, t.nodes[v]) - uv;
        const int64_t gain = max_on_path[p] - cost - split;
        if (gain > best_gain) {
          best_gain = gain;
          best_p = p;
          best_e = static_cast<int>(e);
          best_drop = max_edge[p];
          best_s = s;
        }
      }
    }
    if (best_gain <= 0) break;

    const int u = t.edges[best_e].first, v = t.edges[best_e].second;
    const Rect srect(best_s);
    int s_idx = -1;
    for (int i = 0; i < static_cast<int>(t.nodes.size()); ++i) {
      if (t.nodes[i] == srect) { s_idx = i; break; }
    }
    if (s_idx < 0) {
      s_idx = static_cast<int>(t.nodes.size());
      t.nodes.push_back(srect);
    }

    std::vector<std::pair<int, int> > next;
    for (int i = 0; i < static_cast<int>(t.edges.size()); ++i) {
      if (i == best_e || i == best_drop) continue;
      next.push_back(t.edges[i]);
    }
    if (u != s_idx) next.push_back(std::make_pair(u, s_idx));
    if (s_idx != v) next.push_back(std::make_pair(s_idx, v));
    if (best_p != s_idx) next.push_back(std::make_pair(best_p, s_idx));
    t.edges.swap(next);
  }

  bool changed = true;
  while (changed) {
    changed = false;
    std::vector<int> deg(t.nodes.size(), 0);
    for (size_t i = 0; i < t.edges.size(); ++i) { ++deg[t.edges[i].first]; ++deg[t.edges[i].second]; }
    std::vector<std::pair<int, int> > next;
    for (size_t i = 0; i < t.edges.size(); ++i) {
      const bool drop = (deg[t.edges[i].first] == 1 && t.edges[i].first >= d) ||
                        (deg[t.edges[i].second] == 1 && t.edges[i].second >= d);
      if (drop) changed = true;
      else next.push_back(t.edges[i]);
    }
    t.edges.swap(next);
  }
  t.length = 0;
  t.edge_pts.clear();
  t.edge_pts.reserve(t.edges.size());
  for (size_t i = 0; i < t.edges.size(); ++i) {
    const Rect &a = t.nodes[t.edges[i].first], &b = t.nodes[t.edges[i].second];
    Point pa, pb;
    ClosestPair(a, b, &pa, &pb);
    t.edge_pts.push_back(std::make_pair(pa, pb));
    t.length += Dist(a, b);
  }

  t.seconds = duration<double>(steady_clock::now() - t0).count();
  return t;
}

}  // namespace rsmt
