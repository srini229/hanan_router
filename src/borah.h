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
#include <vector>


namespace rsmt {
struct Point {
  int64_t x = 0;
  int64_t y = 0;
  bool operator==(const Point &o) const { return x == o.x && y == o.y; }
  bool operator<(const Point &o) const { return x != o.x ? x < o.x : y < o.y; }
};
}  // namespace rsmt

namespace rsmt {

struct BorahTree {
  int64_t length = 0;
  std::vector<Point> nodes;                  // terminals first, then Steiner
  std::vector<std::pair<int, int>> edges;    // indices into nodes
  double seconds = 0.0;
};

// Deduplicates the input. Every Steiner point it introduces is a Hanan point.
BorahTree BorahOwens(const std::vector<Point> &pts);

}  // namespace rsmt

#endif  // RSMT_BORAH_H
