#ifndef CPROUTE_H_
#define CPROUTE_H_

#include <vector>
#include <string>
#include <functional>
#include "Geom.h"

// An optional alternative router that formulates the whole module as a single
// CP-SAT (Google OR-Tools) problem instead of routing nets one by one with A*.
//
// Formulation: a coarse 3-D routing grid (gcells x layers). Every net is a
// multi-terminal commodity that must form a connected tree on the grid
// (single-commodity flow from one pin to the others). Two different nets may
// not occupy the same grid node (node-disjoint -> no shorts), obstacle nodes
// are forbidden, and total wirelength (with a via penalty) is minimised. The
// model is handed to CP-SAT; the optimal/feasible solution is decoded back into
// per-net wire/via rectangles.
//
// Built only when compiled with -DUSE_ORTOOLS (make CPSAT=1); otherwise the
// implementation is a stub that returns false so the rest of the tool builds
// without the OR-Tools dependency.
namespace CpRoute {

struct Net {
  std::string name;
  std::vector<Geom::LayerRects> pins;   // each pin = its physical shapes
};

struct LayerModel {
  Geom::Rect bbox;
  int minLayer{0}, maxLayer{0};
  std::function<int(int)> width;    // wire width on layer z
  std::function<bool(int)> canUp;   // a via connects z and z+1
};

// 'routes[i]' receives the routed shapes for nets[i] (empty if that net was not
// routed). Returns true if a solution was produced. 'msg' gets a short status.
bool route(const std::vector<Net>& nets, const Geom::LayerRects& obstacles,
           const LayerModel& lm, double timeLimitSec,
           std::vector<Geom::LayerRects>& routes, std::string* msg);

bool available();   // true if compiled with OR-Tools support

}
#endif
