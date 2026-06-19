#ifndef CONGESTION_H_
#define CONGESTION_H_

#include <vector>
#include <string>
#include <functional>
#include "Geom.h"

// Coarse global-routing congestion feasibility, encoded as SAT. The module is
// cut into a grid of global cells; each net becomes a demand (the bounding box
// it must span) that may take one of several monotone routes, each crossing a
// set of cell edges. Every edge carries a track capacity (its span divided by
// the routing pitch, summed over routing layers) enforced by an at-most-k
// cardinality constraint. If the instance is UNSAT the design cannot be routed
// without overflowing some channel -- congestion the per-pin escape check
// cannot see. This is a coarse necessary check: it is deliberately optimistic
// (many candidate routes, capacity summed over all layers) so it does not cry
// wolf on routable designs.
namespace Congestion {

struct Demand {
  std::string net;
  Geom::Rect box;   // the bounding box the net must span
};

struct Model {
  Geom::Rect bbox;
  int minLayer{0}, maxLayer{0};
  std::function<int(int)> pitchX;  // vertical-track pitch on layer z (step in x)
  std::function<int(int)> pitchY;  // horizontal-track pitch on layer z (step in y)
  const Geom::LayerRects* obstacles{nullptr};  // tracks blocked by these don't count
};

// Returns true if a non-overflowing global assignment exists (or the check is
// inconclusive). On a proven overflow returns false and fills 'reason'.
bool feasible(const std::vector<Demand>& demands, const Model& m,
              std::string* reason = nullptr);

}
#endif
