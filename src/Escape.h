#ifndef ESCAPE_H_
#define ESCAPE_H_

#include <vector>
#include <string>
#include <functional>
#include "Geom.h"

// SAT-based pin escape feasibility check. Before any net is routed we verify
// that every pin can be assigned an "escape" -- either a via to an adjacent
// metal layer, or a same-layer wire stub onto a neighbouring track -- such that
// the escapes chosen for pins of different nets do not physically clash. If the
// instance is UNSAT (or some pin has no escape at all), routing is bound to
// leave a pin stranded, and we surface that up front.
namespace Escape {

struct Pin {
  std::string name;
  int net;
  Geom::LayerRects shapes;
};

struct LayerModel {
  int minLayer{0}, maxLayer{0};
  std::function<int(int)> width;    // routing width on layer z
  std::function<int(int)> space;    // required spacing on layer z
  std::function<bool(int)> canUp;   // a via connects z and z+1
  std::function<bool(int)> canDown; // a via connects z and z-1
  // both set: escapes from points step(z) apart, as the search takes them (via pads on z / other layer)
  std::function<std::vector<std::pair<Geom::Rect, Geom::Rect>>(int, bool)> viaPads;
  std::function<int(int)> step;
  bool greedy{false};   // not satisfiable: still choose, pin by pin, escapes that clash with no earlier choice
  // Mirrors the router's -abutescape rule: a shape already touching the pin has
  // no spacing to it left to protect, so it blocks an escape only by running
  // into it. Without this the check disagrees with the router it is predicting.
  bool abutEscape{false};
};

struct Stats {
  int pins{0}, vars{0}, clauses{0}, blockedPins{0};
  int result{0};      // 1 sat, 0 unsat, -1 inconclusive, -2 a pin with no candidate
  double seconds{0};
};

struct Chosen {
  size_t pin{0};
  int layer{0};
  Geom::Rect fp;
  int srcLayer{-1};      // a via's pad on the pin's own layer, when the point model chose one
  Geom::Rect srcFp;
};

bool feasible(const std::vector<Pin>& pins, const Geom::LayerRects& obstacles,
              const LayerModel& lm, std::vector<std::string>* blocked = nullptr,
              std::string* reason = nullptr, std::vector<Chosen>* chosen = nullptr,
              Stats* stats = nullptr);

}
#endif
