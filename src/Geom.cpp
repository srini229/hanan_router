#include "Geom.h"
#include <cmath>

namespace Geom {

double Dist(const Geom::Rect& r1, const Geom::Rect& r2, const bool manh)
{
  double dist{0.};
  if (manh) {
    if (!r1.overlaps(r2)) {
      auto xdist{std::min(abs(r1.xmin() - r2.xmax()), abs(r1.xmax() - r2.xmin()))};
      if (r1.xmin() <= r2.xmax() && r2.xmin() <= r1.xmax()) xdist = 0;
      auto ydist{std::min(abs(r1.ymin() - r2.ymax()), abs(r1.ymax() - r2.ymin()))};
      if (r1.ymin() <= r2.ymax() && r2.ymin() <= r1.ymax()) ydist = 0;
      dist = (xdist + ydist);
    }
  } else {
    if (!r1.overlaps(r2)) {
      auto xdist{std::min(abs(r1.xmin() - r2.xmax()), abs(r1.xmax() - r2.xmin()))};
      if (r1.xmin() <= r2.xmax() && r2.xmin() <= r1.xmax()) xdist = 0;
      auto ydist{std::min(abs(r1.ymin() - r2.ymax()), abs(r1.ymax() - r2.ymin()))};
      if (r1.ymin() <= r2.ymax() && r2.ymin() <= r1.ymax()) ydist = 0;
      dist = sqrt(xdist * xdist * 1. + ydist * ydist * 1.);
    }
  }
  return dist;
}

void MergeLayerRects(Geom::LayerRects& l1, const Geom::LayerRects& l2, Geom::Rect* b)
{
  for (auto& l : l2) {
    l1[l.first].insert(l1[l.first].end(), l.second.begin(), l.second.end());
  }
  if (b != nullptr) {
    for (const auto& l : l2) {
      for (const auto& r : l.second) {
        b->merge(r);
      }
    }
  }
}
}
