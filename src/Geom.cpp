#include "Geom.h"

namespace Geom {
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
