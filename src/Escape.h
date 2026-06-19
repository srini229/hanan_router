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
  int net;                  // pins of the same net never conflict with each other
  Geom::LayerRects shapes;  // the pin's physical shapes (ports), in module coords
};

struct LayerModel {
  int minLayer{0}, maxLayer{0};
  std::function<int(int)> width;    // routing width on layer z
  std::function<int(int)> space;    // required spacing on layer z
  std::function<bool(int)> canUp;   // a via connects z and z+1
  std::function<bool(int)> canDown; // a via connects z and z-1
};

// Returns true when every pin is guaranteed an escape. On failure 'blocked'
// lists pins that have no possible escape at all and 'reason' describes why.
bool feasible(const std::vector<Pin>& pins, const Geom::LayerRects& obstacles,
              const LayerModel& lm, std::vector<std::string>* blocked = nullptr,
              std::string* reason = nullptr);

}
#endif
