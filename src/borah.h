// Borah-Owens-Irwin edge-substitution heuristic for the rectilinear Steiner
// minimal tree (Borah, Owens & Irwin, "An edge-based heuristic for Steiner
// routing", IEEE TCAD 1994).
//
// Start from the rectilinear MST of the terminals, then repeatedly consider
// (node, edge) pairs: connecting node p to the bounding box of tree edge (u,v)
// creates a cycle, and the longest edge on that cycle can be dropped. The gain
// is (longest edge on the path) - (cost of the new connection); apply the best
// positive-gain move and repeat.
#ifndef RSMT_BORAH_H
#define RSMT_BORAH_H

#include <cstdint>
#include <utility>
#include <vector>

namespace rsmt {

struct Point {
  int64_t x = 0;
  int64_t y = 0;
  bool operator==(const Point &o) const { return x == o.x && y == o.y; }
  bool operator<(const Point &o) const { return x != o.x ? x < o.x : y < o.y; }
};

struct Rect {
  int64_t xlo = 0, ylo = 0, xhi = 0, yhi = 0;
  Rect() = default;
  Rect(int64_t a, int64_t b, int64_t c, int64_t d) : xlo(a), ylo(b), xhi(c), yhi(d) {}
  explicit Rect(const Point &p) : xlo(p.x), ylo(p.y), xhi(p.x), yhi(p.y) {}
  bool operator==(const Rect &o) const {
    return xlo == o.xlo && ylo == o.ylo && xhi == o.xhi && yhi == o.yhi;
  }
  bool operator<(const Rect &o) const {
    if (xlo != o.xlo) return xlo < o.xlo;
    if (ylo != o.ylo) return ylo < o.ylo;
    if (xhi != o.xhi) return xhi < o.xhi;
    return yhi < o.yhi;
  }
};

struct BorahTree {
  int64_t length = 0;
  int64_t mst_length = 0;                    // the rectilinear MST it started from
  std::vector<Rect> nodes;                   // terminals first, then Steiner points
  std::vector<std::pair<int, int>> edges;    // indices into nodes
  std::vector<std::pair<Point, Point> > edge_pts;
  double seconds = 0.0;
};

// Deduplicates the input. Every Steiner point it introduces is a Hanan point.
BorahTree BorahOwens(const std::vector<Rect> &terms);

}  // namespace rsmt

#endif  // RSMT_BORAH_H
