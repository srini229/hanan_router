#include "borah.h"

#include <algorithm>
#include <chrono>
#include <limits>
#include <numeric>
#include <queue>

namespace rsmt {
namespace {

int64_t Abs64(int64_t v) { return v < 0 ? -v : v; }
int64_t Dist(const Point &a, const Point &b) {
  return Abs64(a.x - b.x) + Abs64(a.y - b.y);
}

Point ClosestOnBox(const Point &p, const Point &a, const Point &b,
                   int64_t *dist) {
  const int64_t xlo = std::min(a.x, b.x), xhi = std::max(a.x, b.x);
  const int64_t ylo = std::min(a.y, b.y), yhi = std::max(a.y, b.y);
  Point q{std::min(std::max(p.x, xlo), xhi), std::min(std::max(p.y, ylo), yhi)};
  *dist = Dist(p, q);
  return q;
}

}

BorahTree BorahOwens(const std::vector<Point> &raw) {
  using namespace std::chrono;
  const auto t0 = steady_clock::now();

  BorahTree t;
  std::vector<Point> pts = raw;
  std::sort(pts.begin(), pts.end());
  pts.erase(std::unique(pts.begin(), pts.end()), pts.end());
  const int d = static_cast<int>(pts.size());
  t.nodes = pts;
  if (d <= 1) return t;

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
      if (from[v] >= 0) t.edges.push_back({from[v], v});
      for (int k = 0; k < d; ++k) {
        if (in[k]) continue;
        const int64_t dd = Dist(pts[v], pts[k]);
        if (dd < best[k]) { best[k] = dd; from[k] = v; }
      }
    }
  }

  for (int iter = 0; iter < 4 * d + 16; ++iter) {
    const int n = static_cast<int>(t.nodes.size());
    std::vector<std::vector<std::pair<int, int>>> adj(n);
    for (size_t e = 0; e < t.edges.size(); ++e) {
      adj[t.edges[e].first].push_back({t.edges[e].second, static_cast<int>(e)});
      adj[t.edges[e].second].push_back({t.edges[e].first, static_cast<int>(e)});
    }

    int64_t best_gain = 0;
    int best_p = -1, best_e = -1, best_drop = -1;
    Point best_s{};

    for (size_t e = 0; e < t.edges.size(); ++e) {
      const int u = t.edges[e].first, v = t.edges[e].second;
      std::vector<int64_t> max_on_path(n, -1);
      std::vector<int> max_edge(n, -1);
      std::vector<int> stack{u};
      max_on_path[u] = 0;
      max_edge[u] = -1;
      while (!stack.empty()) {
        const int a = stack.back();
        stack.pop_back();
        for (const auto &nbr : adj[a]) {
          const int b = nbr.first, eid = nbr.second;
          if (eid == static_cast<int>(e) || max_on_path[b] >= 0) continue;
          const int64_t len = Dist(t.nodes[a], t.nodes[b]);
          if (len > max_on_path[a]) {
            max_on_path[b] = len;
            max_edge[b] = eid;
          } else {
            max_on_path[b] = max_on_path[a];
            max_edge[b] = max_edge[a];
          }
          stack.push_back(b);
        }
      }

      for (int p = 0; p < n; ++p) {
        if (max_on_path[p] <= 0 || max_edge[p] < 0) continue;
        int64_t cost = 0;
        const Point s = ClosestOnBox(t.nodes[p], t.nodes[u], t.nodes[v], &cost);
        const int64_t gain = max_on_path[p] - cost;
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
    int s_idx = -1;
    for (int i = 0; i < static_cast<int>(t.nodes.size()); ++i) {
      if (t.nodes[i].x == best_s.x && t.nodes[i].y == best_s.y) { s_idx = i; break; }
    }
    if (s_idx < 0) {
      s_idx = static_cast<int>(t.nodes.size());
      t.nodes.push_back(best_s);
    }

    std::vector<std::pair<int, int>> next;
    for (int i = 0; i < static_cast<int>(t.edges.size()); ++i) {
      if (i == best_e || i == best_drop) continue;
      next.push_back(t.edges[i]);
    }
    auto add = [&](int a, int b) {
      if (a != b) next.push_back({a, b});
    };
    add(u, s_idx);
    add(s_idx, v);
    add(best_p, s_idx);
    t.edges.swap(next);
  }

  bool changed = true;
  while (changed) {
    changed = false;
    std::vector<int> deg(t.nodes.size(), 0);
    for (const auto &e : t.edges) { ++deg[e.first]; ++deg[e.second]; }
    std::vector<std::pair<int, int>> next;
    for (const auto &e : t.edges) {
      const bool drop = (deg[e.first] == 1 && e.first >= d) ||
                        (deg[e.second] == 1 && e.second >= d);
      if (drop) changed = true;
      else next.push_back(e);
    }
    t.edges.swap(next);
  }
  t.length = 0;
  for (const auto &e : t.edges) t.length += Dist(t.nodes[e.first], t.nodes[e.second]);

  t.seconds = duration<double>(steady_clock::now() - t0).count();
  return t;
}

}
