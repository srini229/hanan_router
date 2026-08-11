#include "Router.h"
#include <algorithm>

namespace Router {
#if DEBUG
size_t Node::_nodectr = 0;
#endif
#define NUM_POINTS 1000

int Router::_precision = 1;

using namespace boost::polygon::operators;

Via::Via(const Via& via, const Geom::Point& p) : _l{via._l}, _u{via._u}, _c{via._c}
{
  _center = via._center.trans(p);
  _lb     = via._lb.trans(p);
  _ub     = via._ub.trans(p);
  _cut    = via._cut.trans(p);
  _bbox   = via._bbox.trans(p);
  for (auto& c : via._cuts) {
    _cuts.emplace_back(c.trans(p));
  }
}

void Via::addCuts(const Geom::Point& o, const int wx, const int wy, const int nrow, const int ncol, const int sx, const int sy)
{
  _cuts.clear();
  Geom::Rect r{o.x(), o.y(), o.x() + wx, o.y() + wy};
  if (nrow == 1 && ncol == 1) {
    _cut = r;
    _bbox.merge(_cut);
  } else {
    for (int i = 0; i < nrow; ++i) {
      for (int j = 0; j < ncol; ++j) {
        _cuts.push_back(r.trans(Geom::Point(i * (sx + wx), j * (sy + wy))));
        _cut.merge(_cuts.back());
        _bbox.merge(_cuts.back());
      }
    }
  }
}

std::string Via::str() const
{
  return "l: " + std::to_string(_l) + " u: " + std::to_string(_u) + " c: " + std::to_string(_c) +
    " center: " + _center.str() + " lb: " + _lb.str() + " ub: " + _ub.str() + " cut: " + _cut.str() +
    " bbox: " + _bbox.str();
}

void Via::addShapes(Geom::LayerRects& lr) const
{
  lr[_l].push_back(_lb);
  lr[_u].push_back(_ub);
  if (_cuts.empty()) {
    lr[_c].push_back(_cut);
  } else {
    for (auto& c : _cuts) {
      lr[_c].push_back(c);
    }
  }
}

CostFn::CostFn(const DRC::LayerInfo& lf)
{
  auto& layers = lf.layers();
  _layerHCost.resize(layers.size(), COST_MAX);
  _layerVCost.resize(layers.size(), COST_MAX);
  _bendCost.resize(layers.size(), 0);
  for (unsigned i = 0; i < layers.size(); ++i) {
    if (layers[i]->isMetal()) {
      auto r = std::max(1, static_cast<int>(layers[i]->meanR()));
      if (static_cast<DRC::MetalLayer*>(layers[i])->isHorizontal()) {
        _layerHCost[i] = r;
      }
      if (static_cast<DRC::MetalLayer*>(layers[i])->isVertical()) {
        _layerVCost[i] = r;
      }
      {
        auto ml = static_cast<DRC::MetalLayer*>(layers[i]);
        _bendCost[i] = static_cast<CostType>(r) * (ml->width() + ml->space()) * BEND_PITCHES;
      }
      _topRoutingLayer = static_cast<int>(i);
    }
  }
  _layerPairCost.resize(_layerHCost.size(), std::vector<CostType>(_layerHCost.size(), COST_MAX));
  for (unsigned i = 0; i < layers.size(); ++i) {
    if (layers[i]->isVia()) {
      auto l = lf.getLayers(static_cast<DRC::ViaLayer*>(layers[i]));
      auto r = std::max(1, static_cast<int>(layers[i]->meanR()));
      if (l.first >= 0 && l.second >= 0) {
        _layerPairCost[l.first][l.second] = r;
        _layerPairCost[l.second][l.first] = r;
      }
    }
  }
  _baseLayerPairCost = _layerPairCost;
  _minMetalCost = COST_MAX;
  for (int i = 0; i <= _topRoutingLayer; ++i) {
    _minMetalCost = std::min(_minMetalCost, std::min(_layerHCost[i], _layerVCost[i]));
  }
  if (_minMetalCost >= COST_MAX) _minMetalCost = 1;
  for (int i = 0; i <= _topRoutingLayer; ++i) {
    COUT << "layer : " << i << " cost : " << _layerHCost[i] << ' ' << _layerVCost[i] << '\n';
  }
  COUT << "cheapest metal cost (used inside pin neighbourhoods) : " << _minMetalCost << '\n';
  for (int i = 0; i <= _topRoutingLayer; ++i) {
    if (i > 0) {
      COUT << "layerPairCost : " << i << ' ' << i - 1 << ' ' << _layerPairCost[i][i-1] << '\n';
    }
    if (i < _topRoutingLayer) {
      COUT << "layerPairCost : " << i << ' ' << i + 1 << ' ' << _layerPairCost[i][i+1] << '\n';
    }
  }
}

CostType CostFn::deltaCost(const Node& n1, const Node& n2) const
{
  CostType dc{0};

  if (n1.z() == n2.z()) {
    CostType bendCost{0};
    if (n1.parent() == &n2 && n2.parent() && n2.parent()->z() == n2.z()) {
      if ((n1.x() == n2.x() && n2.parent()->x() != n1.x())
          || (n1.y() == n2.y() && n2.parent()->y() != n1.y())){
        bendCost = _bendCost[n1.z()];
        /*COUT << "bend cost : " << bendCost << ' ' << std::min(_layerVCost[n1.z()], _layerHCost[n1.z()]) << '\n' ;
        n1.print("bend1 : ");
        n2.print("bend2 : ");
        n2.parent()->print("bend3 : ");*/
      }
    }
    if (n1.x() == n2.x() && _layerVCost[n1.z()] < COST_MAX) {
      const CostType c = relaxed(_layerVCost[n1.z()], n1.z(), n1.x(), n1.y(), n2.x(), n2.y());
      return (bendCost + c * std::abs(n1.y() - n2.y()));
    }
    if (n1.y() == n2.y() && _layerHCost[n1.z()] < COST_MAX) {
      const CostType c = relaxed(_layerHCost[n1.z()], n1.z(), n1.x(), n1.y(), n2.x(), n2.y());
      return (bendCost + c * std::abs(n1.x() - n2.x()));
    }
  }
  if (n1.x() == n2.x() && n1.y() == n2.y()) {
    if (abs(n2.z() - n1.z()) == 1) {
      return _layerPairCost[n1.z()][n2.z()];
    }
  }
  auto minz = std::min(n1.z(), n2.z());
  auto maxz = std::max(n1.z(), n2.z());
  CostType minHCost(COST_MAX), minVCost(COST_MAX);
  //if (true)
  if (_layerHCost[minz] != _layerVCost[minz]) {
    if (minz < _topRoutingLayer) maxz = minz + 1;
    else minz -= 1;
  }
  for (int i = minz; i <= maxz; ++i) {
    minHCost = std::min(minHCost, _layerHCost[i]);
    minVCost = std::min(minVCost, _layerVCost[i]);
  }
  if (n1.z() == n2.z()) {
    minHCost = relaxed(minHCost, n1.z(), n1.x(), n1.y(), n2.x(), n2.y());
    minVCost = relaxed(minVCost, n1.z(), n1.x(), n1.y(), n2.x(), n2.y());
  }
  dc += (minHCost * std::abs(n1.x() - n2.x())) + (minVCost * std::abs(n1.y() - n2.y()));
  if (std::abs(n1.x() - n2.x()) && std::abs(n1.y() - n2.y())) {
    for (int i = minz; i <= maxz; ++i) {
      if (_layerHCost[i] == minHCost && minHCost == minVCost && _layerVCost[i] == _layerHCost[i]) {
        dc += (i < _topRoutingLayer) ? _layerPairCost[i][i + 1] / 2 : _layerPairCost[i][i - 1] / 2;
        break;
      }
    }
  }
  for (int i = std::min(n1.z(), minz); i < std::max(n1.z(), minz); ++i) {
    dc += _layerPairCost[i][i+1];
  }
  for (int i = minz; i < maxz; ++i) {
    dc += _layerPairCost[i][i+1];
  }
  for (int i = std::min(n2.z(), maxz); i < std::max(n2.z(), maxz); ++i) {
    dc += _layerPairCost[i][i+1];
  }
  /*
  {
    std::set<int> minHCostLayers, minVCostLayers;
    for (int i = 0; i <= _topRoutingLayer; ++i) {
      if (_layerHCost[i] == minHCost) minHCostLayers.insert(i);
      if (_layerVCost[i] == minVCost) minVCostLayers.insert(i);
    }
    for (int i = 0; i <= _topRoutingLayer; ++i) {
      minHCost = std::min(minHCost, _layerHCost[i]);
      minVCost = std::min(minVCost, _layerVCost[i]);
    }
    auto hit{std::lower_bound(minHCostLayers.begin(), minHCostLayers.end(), minz - 1)};
    auto vit{std::lower_bound(minVCostLayers.begin(), minVCostLayers.end(), minz - 1)};
    bool hinbetween{hit != minHCostLayers.end() && *hit <= maxz};
    bool vinbetween{vit != minVCostLayers.end() && *vit <= maxz};
    dc += (minHCost * std::abs(n1.x() - n2.x())) + (minVCost * std::abs(n1.y() - n2.y()));
    //case : minz <= minHCostLayers, minVCostLayers <= maxz
    for (int i = minz; i < maxz; ++i) {
      dc += _layerPairCost[i][i+1];
    }
    if (hinbetween && !vinbetween) {
      if (!minVCostLayers.empty()) {
        if (*minVCostLayers.rbegin() < minz) {
          for (int i = *minVCostLayers.rbegin(); i < minz; ++i) {
            dc += 2 * _layerPairCost[i][i+1];
          }
        } else if (*minVCostLayers.begin() > maxz) {
          for (int i = maxz; i < *minVCostLayers.begin(); ++i) {
            dc += 2 * _layerPairCost[i][i+1];
          }
        }
      }
    } else if (!hinbetween && vinbetween) {
      if (!minHCostLayers.empty()) {
        if (*minHCostLayers.rbegin() < minz) {
          for (int i = *minHCostLayers.rbegin(); i < minz; ++i) {
            dc += 2 * _layerPairCost[i][i+1];
          }
        } else if (*minHCostLayers.begin() > maxz) {
          for (int i = maxz; i < *minHCostLayers.begin(); ++i) {
            dc += 2 * _layerPairCost[i][i+1];
          }
        }
      }
    } else if (!hinbetween && !vinbetween) {
      if (!minHCostLayers.empty() && !minVCostLayers.empty()) {
        if (*minVCostLayers.rbegin() < minz && *minHCostLayers.rbegin() < minz) {
          for (int i = std::min(*minVCostLayers.rbegin(), *minHCostLayers.rbegin()); i < minz; ++i) {
            dc += 2 * _layerPairCost[i][i+1];
          }
        } else if (*minVCostLayers.rbegin() < minz && *minHCostLayers.begin() > maxz) {
          for (int i = *minVCostLayers.rbegin(); i < minz; ++i) {
            dc += 2 * _layerPairCost[i][i+1];
          }
          for (int i = maxz; i < *minHCostLayers.begin(); ++i) {
            dc += 2 * _layerPairCost[i][i+1];
          }
        } else if (*minHCostLayers.rbegin() < minz && *minVCostLayers.begin() > maxz) {
          for (int i = *minHCostLayers.rbegin(); i < minz; ++i) {
            dc += 2 * _layerPairCost[i][i+1];
          }
          for (int i = maxz; i < *minVCostLayers.begin(); ++i) {
            dc += 2 * _layerPairCost[i][i+1];
          }
        } else if (*minHCostLayers.begin() > maxz && *minVCostLayers.begin() > maxz) {
          for (int i = maxz; i < std::max(*minHCostLayers.begin(), *minVCostLayers.begin()); ++i) {
            dc += 2 * _layerPairCost[i][i+1];
          }
        }
      }
    }
  }*/

  return dc;
}

void CostFn::updatendr(const std::map<int, DRC::Direction>& ndrdir, const std::set<int>& preflayers)
{
  _savedLayerHCost = _layerHCost;
  _savedLayerVCost = _layerVCost;
  _preflayers = preflayers;
  for (auto& it : ndrdir) {
    auto mincost{std::min(_layerHCost[it.first], _layerVCost[it.first])};
    if (it.second == DRC::Direction::ORTHOGONAL) {
      _layerVCost[it.first] = mincost;
      _layerHCost[it.first] = mincost;
    } else if (it.second == DRC::Direction::VERTICAL) {
      _layerHCost[it.first] = COST_MAX;
      _layerVCost[it.first] = mincost;
    } else if (it.second == DRC::Direction::HORIZONTAL) {
      _layerVCost[it.first] = COST_MAX;
      _layerHCost[it.first] = mincost;
    }
    COUT << "ndr dir : " << it.first << ' ' << _layerVCost[it.first] << ' ' << _layerHCost[it.first] << '\n';
  }
  if (!preflayers.empty()) {
    for (int i = 0; i <= _topRoutingLayer; ++i) {
      if (preflayers.find(i) != preflayers.end()) {
        if (_layerHCost[i] < _layerVCost[i]) {
          _layerHCost[i] = 1;
        } else if (_layerVCost[i] < _layerHCost[i]) {
          _layerVCost[i] = 1;
        } else {
          _layerHCost[i] = 1;
          _layerVCost[i] = 1;
        }
      } else {
        _layerHCost[i] = static_cast<long>(COST_MAX);
        _layerVCost[i] = static_cast<long>(COST_MAX);
      }
    }
  }
  if (!preflayers.empty() || !ndrdir.empty()) {
    for (int i = 0; i <= _topRoutingLayer; ++i) {
      COUT << "layer cost (" << i << ") : " << _layerHCost[i] << ' ' << _layerVCost[i] << '\n';
    }
  }
}

Router::Router(const DRC::LayerInfo& lf) : _cf{lf}, _sol{nullptr}, _minLayer{INT_MAX}, _maxLayer{0}, _maxRoutingLayer{0}, _name{}, _lf{lf}
{
  auto& layers = lf.layers();
  _widthx.reserve(layers.size());
  _widthy.reserve(layers.size());
  _spacex.reserve(layers.size());
  _spacey.reserve(layers.size());
  _ndrwidthx.reserve(layers.size());
  _ndrwidthy.reserve(layers.size());
  _ndrspacex.reserve(layers.size());
  _ndrspacey.reserve(layers.size());
  for (unsigned i = 0; i < layers.size(); ++i) {
    if (layers[i]->isMetal()) {
      auto mlayer = static_cast<DRC::MetalLayer*>(layers[i]);
      _widthx.push_back(mlayer->width());
      _widthy.push_back(mlayer->width());
      _spacex.push_back(mlayer->space());
      _spacey.push_back(mlayer->space());
      const int drcs = std::max(mlayer->space(), mlayer->minSpace());
      _drcspacex.push_back(drcs);
      _drcspacey.push_back(drcs);
      if (mlayer->minSpace() > mlayer->space()) {
        CERR << "WARNING: layer " << mlayer->name() << " pitch gives spacing "
             << mlayer->space() << " but MinSpacing is " << mlayer->minSpace()
             << "; routing will pack to " << mlayer->space()
             << " and the final DRC check will flag it (pitch should be >= "
             << (mlayer->width() + mlayer->minSpace()) << ")\n";
      }
      COUT << "layer : " << i << " width : " << _widthx.back() << " space : " << _spacex.back() << " v : " << _cf.isVert(i) << " h : " << _cf.isHor(i) << '\n';
    }
  }
  _aboveViaLayer.resize(_widthx.size(), -1);
  _belowViaLayer.resize(_widthx.size(), -1);
  for (unsigned i = 0; i < layers.size(); ++i) {
    if (layers[i]->isVia()) {
      auto vlayer = static_cast<DRC::ViaLayer*>(layers[i]);
      auto l = lf.getLayers(vlayer);
      if (l.first >= 0) _aboveViaLayer[l.first]  = i;
      if (l.second >= 0) _belowViaLayer[l.second] = i;
      _widthx.push_back(vlayer->widthx());
      _widthy.push_back(vlayer->widthy());
      _spacex.push_back(vlayer->spacex());
      _spacey.push_back(vlayer->spacey());
      _drcspacex.push_back(vlayer->spacex());
      _drcspacey.push_back(vlayer->spacey());
    }
  }
  for (unsigned i = 0; i < _aboveViaLayer.size(); ++i) {
      COUT << "layer : " << i << " above : " << _aboveViaLayer[i] << " below : " << _belowViaLayer[i] << '\n';
  }
  _minLayer = lf.signalBottomLayer();
  _maxLayer = lf.signalTopLayer();
  {
    CostType floorc = COST_MAX;
    for (int z = _minLayer; z <= _maxLayer; ++z) {
      floorc = std::min(floorc, std::min(_cf.hcost(z), _cf.vcost(z)));
    }
    _cf.setRelaxFloor(floorc);
    COUT << "pin-neighbourhood relaxed cost floor : " << floorc << '\n';
  }
  _maxRoutingLayer = static_cast<int>(_widthx.size()) - 1;
  COUT << "min routing layer : " << LAYER_NAMES[_minLayer] << " max routing layer : " << LAYER_NAMES[_maxLayer] << '\n';
  _nodes.clear();
  _nodes.resize(_maxLayer + 1);
  constructVias();
}

void Router::setGuide(const Geom::LayerRects& g, const CostType weight)
{
  _guide = g;
  _guideByLayer.clear();
  _guideAll.clear();
  for (auto& l : _guide) {
    auto& v = _guideByLayer[l.first];
    for (auto& r : l.second) {
      v.push_back(r);
      _guideAll.push_back(r);
    }
  }
  _guideWeight = weight;
  _hasGuide = (weight > 0 && !_guideAll.empty());
}

CostType Router::baseUnitCost() const
{
  CostType m = CostTypeMax;
  for (int i = 0; i <= _cf.topRoutingLayer(); ++i) {
    m = std::min(m, std::min(_cf.hcost(i), _cf.vcost(i)));
  }
  return (m == CostTypeMax || m <= 0) ? 1 : m;
}

CostType Router::guideDeviation(const int x, const int y, const int z) const
{
  // Manhattan distance from (x,y) to the nearest rect in 'rs' (0 if inside one).
  auto nearest = [&](const Geom::Rects& rs) -> long {
    long best = -1;
    for (auto& r : rs) {
      long dx = std::max(std::max(r.xmin() - x, x - r.xmax()), 0);
      long dy = std::max(std::max(r.ymin() - y, y - r.ymax()), 0);
      long d = dx + dy;
      if (best < 0 || d < best) best = d;
      if (best == 0) break;
    }
    return best;
  };
  auto it = _guideByLayer.find(z);
  if (it != _guideByLayer.end() && !it->second.empty()) {
    long d = nearest(it->second);
    if (d >= 0) return static_cast<CostType>(d);
  }
  long d = nearest(_guideAll);
  return d < 0 ? 0 : static_cast<CostType>(d);
}

Node* Router::createNode(const int x, const int y, const int z,
    const Node* parent, const int fcost, const int tcost)
{
  auto coord = std::make_pair(x,y);
  auto it = _nodes[z].find(coord);
  Node* n = nullptr;
  if (z < _minLayer || z > _maxLayer) COUT << "ERROR in layer no : " << z << '\n';
  if (it == _nodes[z].end()) {
    n = new Node(x, y, z, fcost, tcost, parent);
    auto itr = _nodes[z].emplace(std::make_pair(coord, n));
    it = itr.first;
    if (!itr.second) {
      COUT << "ERROR adding node to nz "; n->print("n : ");
      delete n;
      n = itr.first->second;
#if DEBUG
    } else {
      _nodeset.insert(n);
#endif
    }
#if DEBUG
    COUT << "creating new node : " << x << ',' << y << ',' << z << '\n';
#endif
  }
  return (it != _nodes[z].end()) ? it->second : nullptr;
}

Geom::PointWidthSet Router::findValidPoints(const Geom::Rect& r, const int z, const Direction dir) const
{
  auto vert = _cf.isVert(z);
  auto hor = _cf.isHor(z);
  auto x = roundup(r.xcenter()), y = roundup(r.ycenter()); 
  Geom::PointWidthSet points;
  COUT << _name << " points : \n";
  //COUT << "x : " << x << " y : " << y << "\n";

  if (dir == DOWN || dir == UP) {
    auto adj = z;
    if (dir == UP) {
      if (z < _maxLayer) adj = z + 1;
    } else {
      if (z > _minLayer) adj = z - 1;
    }

    if (adj != z) {
      auto wx = widthx(adj), wy = widthy(adj);
      Geom::Rect padbox, zpadbox;
      if (dir == UP && !_upVias[z].empty()) {
        padbox = _upVias[z][0]->upad();
        zpadbox = _upVias[z][0]->lpad();
      }
      if (dir == DOWN && !_dnVias[z].empty()) {
        padbox = _dnVias[z][0]->lpad();
        zpadbox = _dnVias[z][0]->upad();
      }
      if (vert) {
        int space = roundup(std::max(spacey(adj) + ((wy % 2 == 0) ? wy/2 : (wy/2 + 1)), r.height()/NUM_POINTS));
        if (padbox.valid()) {
          space = std::max(space, padbox.width());
        }
        points.insert(std::make_pair(Geom::Point(x,y), widthy(z)));
        auto dimy{std::max(zpadbox.height(), widthy(z))/2};
        int yn = r.ymax() - dimy;
        while (yn >= (r.ymin() + dimy)) {
          points.insert(std::make_pair(Geom::Point(x,yn), widthy(z)));
          COUT << "pointv : " << x << ',' << yn << ',' << widthy(z) << '\n';
          yn -= space;
        }
      }
      if (hor) {
        int space = roundup(std::max(spacex(adj) + ((wx % 2 == 0) ? wx/2 : (wx/2 + 1)), r.width()/NUM_POINTS));
        if (padbox.valid()) {
          space = std::max(space, padbox.height());
        }
        points.insert(std::make_pair(Geom::Point(x,y), widthx(z)));
        auto dimx{std::max(zpadbox.width(), widthx(z))/2};
        int xn = r.xmax() - dimx;
        while (xn >= (r.xmin() + dimx)) {
          points.insert(std::make_pair(Geom::Point(xn,y), widthx(z)));
          COUT << "pointh : " << xn << ',' << y << ',' << widthx(z) << '\n';
          xn -= space;
        }
      }
    }
  } else if (dir == EAST || dir == WEST) {
    if (hor) {
      auto width = widthx(z);
#if DEBUG
      COUT << "we : " << width << '\n';
#endif
      if (!_cornerEscape) {
        int space = std::max(1, roundup(std::max(spacex(z) + ((width % 2 == 0) ? width/2 : (width/2 + 1)), r.width()/NUM_POINTS)));
        int lo = r.xmin() + width/2, hi = r.xmax() - width/2;
        if (hi < lo) {
          points.insert(std::make_pair(Geom::Point(x, y), width));
        } else {
          for (int xn = x; xn <= hi; xn += space)
            points.insert(std::make_pair(Geom::Point(xn, y), width));
          for (int xn = x - space; xn >= lo; xn -= space)
            points.insert(std::make_pair(Geom::Point(xn, y), width));
        }
      }
      // Corner/edge escape points: always used in the design-wide corner-escape
      // pass, and ALSO added here -- on top of whatever the centre track just
      // produced -- when that left very few candidates. A pin with only 1-2
      // centre-track points is often structurally blocked there regardless of
      // net order, so retrying it through the (powerless) reorder loop and then
      // waiting for a full second design pass is pure waste; offering the
      // corner points immediately lets the first attempt succeed instead.
      if (_cornerEscape || points.size() <= MIN_ESCAPE_POINTS) {
        const int fxlo = r.xmin() + width/2, fxhi = r.xmax() - width/2;
        const bool fitx = (fxhi >= fxlo);
        const bool cx = centreEscapeOnly(r.width(), width);
        const int bxlo = (cx || !fitx) ? roundup(r.xcenter()) : roundup(fxlo);
        const int bxhi = (cx || !fitx) ? roundup(r.xcenter()) : roundup(fxhi);
        if (width < r.height()) {
          int space = roundup(std::max(spacex(z) + ((width % 2 == 0) ? width/2 : (width/2 + 1)), r.height()/NUM_POINTS));
          for (auto right : {true, false}) {
            int ex = (right ? bxhi : bxlo);
            int yn = y;
            while (yn < r.ymax()) { points.insert(std::make_pair(Geom::Point(ex,yn), width)); yn += space; }
            yn = r.ycenter();
            while (yn <= (r.ymax() - width/2)) { points.insert(std::make_pair(Geom::Point(ex,yn), width)); yn += space; }
            yn = r.ycenter();
            while (yn >= (r.ymin() + width/2)) { points.insert(std::make_pair(Geom::Point(ex,yn), width)); yn -= space; }
          }
          int yn = roundup(r.ycenter());
          while (yn <= (r.ymax() - width/2)) {
            points.insert(std::make_pair(Geom::Point(bxlo,yn), width));
            points.insert(std::make_pair(Geom::Point(bxhi,yn), width));
            yn += space;
          }
          yn = roundup(r.ycenter());
          while (yn >= (r.ymin() + width/2)) {
            points.insert(std::make_pair(Geom::Point(bxlo,yn), width));
            points.insert(std::make_pair(Geom::Point(bxhi,yn), width));
            yn -= space;
          }
        }
        if (width != r.width()) {
          points.insert(std::make_pair(Geom::Point(bxlo,r.ycenter()), width));
          points.insert(std::make_pair(Geom::Point(bxhi,r.ycenter()), width));
        }
      }
    }
  } else if (dir == NORTH || dir == SOUTH) {
    if (vert) {
      auto width = widthy(z);
#if DEBUG
      COUT << "wn : " << width << '\n';
#endif
      if (!_cornerEscape) {
        int space = std::max(1, roundup(std::max(spacey(z) + ((width % 2 == 0) ? width/2 : (width/2 + 1)), r.height()/NUM_POINTS)));
        int lo = r.ymin() + width/2, hi = r.ymax() - width/2;
        if (hi < lo) {
          points.insert(std::make_pair(Geom::Point(x, y), width));
        } else {
          for (int yn = y; yn <= hi; yn += space)
            points.insert(std::make_pair(Geom::Point(x, yn), width));
          for (int yn = y - space; yn >= lo; yn -= space)
            points.insert(std::make_pair(Geom::Point(x, yn), width));
        }
      }
      // Corner/edge escape points: always used in the design-wide corner-escape
      // pass, and ALSO added here when the centre track left very few
      // candidates (see the EAST/WEST branch above for the rationale).
      if (_cornerEscape || points.size() <= MIN_ESCAPE_POINTS) {
        // See the EAST/WEST branch: pull the edge candidates in by half a width so
        // the escape is flush with the pin rather than hanging outside it.
        const int fylo = r.ymin() + width/2, fyhi = r.ymax() - width/2;
        const bool fity = (fyhi >= fylo);
        const bool cy = centreEscapeOnly(r.height(), width);
        const int bylo = (cy || !fity) ? roundup(r.ycenter()) : roundup(fylo);
        const int byhi = (cy || !fity) ? roundup(r.ycenter()) : roundup(fyhi);
        if (width < r.width()) {
          int space = roundup(std::max(spacey(z) + ((width % 2 == 0) ? width/2 : (width/2 + 1)), r.width() / NUM_POINTS));
          int xn = roundup(r.xcenter());
          while (xn <= (r.xmax() - width/2)) {
            points.insert(std::make_pair(Geom::Point(xn,bylo), width));
            points.insert(std::make_pair(Geom::Point(xn,byhi), width));
            xn += space;
          }
          xn = roundup(r.xcenter());
          while (xn >= (r.xmin() + width/2)) {
            points.insert(std::make_pair(Geom::Point(xn,bylo), width));
            points.insert(std::make_pair(Geom::Point(xn,byhi), width));
            xn -= space;
          }
        }
        if (width != r.height()) {
          points.insert(std::make_pair(Geom::Point(r.xcenter(),bylo), width));
          points.insert(std::make_pair(Geom::Point(r.xcenter(),byhi), width));
        }
      }
    }
  }

  return points;
}

void Router::addSourceTargetShapes(const Geom::Rect& r, const int z, const bool src)
{
  if (z >= _minLayer && z <= _maxLayer) {
    // Charge this layer at the cheapest metal rate within RELAX_PIN_SPACINGS
    // spacings of the pin, so a short hop to a neighbouring pin stays on-layer
    // instead of paying for a via pair to escape an expensive layer.
    _cf.addRelaxZone(z, r.bloatby(RELAX_PIN_SPACINGS * spacex(z),
                                  RELAX_PIN_SPACINGS * spacey(z)));
    auto& shapes = (src ? _sourceshapes : _targetshapes);
    if (src) {
      _psources[z] += PRect(r.xmin(), r.ymin(), r.xmax(), r.ymax());
    } else {
      _ptargets[z] += PRect(r.xmin(), r.ymin(), r.xmax(), r.ymax());
    }
    bool inserted{false};
    for (auto it = shapes[z].begin(); it != shapes[z].end(); ++it) {
      auto s = *it;
      if (s.contains(r)) {
        inserted = true;
        break;
      } else if (r.contains(s)) {
        shapes[z].erase(it);
        shapes[z].insert(r);
        inserted = true;
        break;
      }
    }
    if (!inserted) shapes[z].insert(r);
  }
}

void Router::addSourceTarget(const Geom::Rect& r, const int z, const bool src)
{
  if (z < _minLayer || z > _maxLayer) return;
  auto& shapes = (src ? _sourceshapes : _targetshapes);
  auto& pshapes = (src ? _psources : _ptargets);
  auto& dest   = (src ? _sources : _targets);
  int fcost    = (src ? 0 : -1);
  int tcost    = (src ? -1 : 0);
  bool inserted{false};
  for (auto it = shapes[z].begin(); it != shapes[z].end(); ++it) {
    auto s = *it;
    if (s.contains(r)) {
      inserted = true;
      break;
    } else if (r.contains(s)) {
      shapes[z].erase(it);
      shapes[z].insert(r);
      inserted = true;
      break;
    }
  }
  if (!inserted) shapes[z].insert(r);
  pshapes[z] += PRect(r.xmin(), r.ymin(), r.xmax(), r.ymax());
  //COUT << "shape : " << r.str() << " z : " << z << '\n';
  for (auto dir : {UP, DOWN, EAST, NORTH}) {
    auto points = findValidPoints(r, z, dir);
    //COUT << "num points : " << points.size() << ' ' << "dir : " << dir << '\n';
    for (auto& pp : points) {
      auto& p = pp.first;
      const int offc = std::abs(p.x() - r.xcenter()) + std::abs(p.y() - r.ycenter());
      const int pen = static_cast<int>(_cf.offCentreEscapeCost(offc));
      auto n = createNode(p.x(), p.y(), z, nullptr,
                          src ? fcost + pen : fcost,
                          src ? tcost : tcost + pen);
      n->expand(dir, true);
      if ((dir == UP && z >= _maxLayer) || (dir == DOWN && z <= _minLayer)) {
        n->expand(dir, false);
      }
      if (dir == EAST) {
        n->sethwx(pp.second/2);
        n->expand(WEST, true);
      }
      if (dir == NORTH) {
        n->sethwy(pp.second/2);
        n->expand(SOUTH, true);
      }
#if DEBUG
      n->print((src ? "src" : "tgt"));
#endif
      dest.insert(n);
      _bbox.merge(n->x(), n->y(), n->x(), n->y());
    }
  }
}

void Router::insertToPQ(const Node* n)
{
  setexpand(const_cast<Node*>(n), n->parent());
  _pq.insert(n);
#if DEBUG
  n->print("\tadding to pq :");
  if (n->parent()) n->parent()->print("\t\tparent:");
#endif
}

void Router::setexpand(Node* newn, const Node* parent) const
{
  if (parent) {
    newn->setexpand();
    if (newn->z() == parent->z()) {
      if (newn->x() == parent->x()) {
        if (newn->y() < parent->y()) {
          newn->expand(NORTH, false);
        } else if (newn->y() > parent->y()) {
          newn->expand(SOUTH, false);
        }
      } else if (newn->y() == parent->y()) {
        if (newn->x() < parent->x()) {
          newn->expand(EAST, false);
        } else if (newn->x() > parent->x()) {
          newn->expand(WEST, false);
        }
      }
    }
    if (newn->z() < parent->z() || newn->z() >= _maxLayer || newn->z() <_minLayer) newn->expand(UP, false);
    else {
      if (newn->upVia() == nullptr) {
        auto v = isViaValid(newn, true);
        if (v) {
          newn->upVia(v);
        } else {
          newn->expand(UP, false);
        }
      }
    }
    if (newn->z() > parent->z() || newn->z() <= _minLayer || newn->z() > _maxLayer) newn->expand(DOWN, false);
    else {
      if (newn->dnVia() == nullptr) {
        auto v = isViaValid(newn, false);
        if (v) {
          newn->dnVia(v);
        } else {
          newn->expand(DOWN, false);
        }
      }
    }
  } else {
    newn->setexpand();
    if (newn->z() >= _maxLayer)  newn->expand(UP, false);
    else if (newn->z() >= _minLayer && newn->z() < _maxLayer) {
      newn->expand(UP, false);
      auto v = isViaValid(newn, true);
      if (v) {
        newn->upVia(v);
        newn->expand(UP, true);
      }
    }
    if (newn->z() <= _minLayer) newn->expand(DOWN, false);
    else if (newn->z() > _minLayer && newn->z() <= _maxLayer) {
      newn->expand(DOWN, false);
      auto v = isViaValid(newn, false);
      if (v) {
        newn->dnVia(v);
        newn->expand(DOWN, true);
      }
    }
  }
  if (_cf.hcost(newn->z()) == _cf.vcost(newn->z()) && _cf.hcost(newn->z()) == COST_MAX) {
    newn->expand(NORTH, false);
    newn->expand(SOUTH, false);
    newn->expand(EAST, false);
    newn->expand(WEST, false);
  }
}

void Router::checkAndInsert(Node* newn, const Node* n)
{
  if (_sources.find(newn) != _sources.end()) {
#if DEBUG
    newn->print("trying to add source node : ");
#endif
    return;
  }
#if DEBUG
  newn->print("newn bef :");
  if (newn->parent()) {
    newn->parent()->print("newn parent bef :");
  }
#endif
  if (newn->cost() < 0) {
    if (newn->parent() != n) newn->setParent(n);
    evalCost(newn);
    insertToPQ(newn);
    if (n->z() == newn->z()) {
      if (n->x() == newn->x()) {
        newn->sethwy(n->hwy());
      } else {
        newn->sethwx(n->hwx());
      }
    }
  } else if (newn->parent() != n) {
    auto oldfcost = newn->fcost();
    auto oldparent = newn->parent();
    auto it = _pq.find(newn);
    newn->setParent(n);
    if (n->z() == newn->z()) {
      if (n->x() == newn->x()) {
        newn->sethwy(n->hwy());
      } else {
        newn->sethwx(n->hwx());
      }
    }
    evalFCost(newn);
    if (newn->fcost() > oldfcost) {
      newn->setParent(oldparent);
      newn->setFCost(oldfcost);
    } else if (it != _pq.end()) {
      _pq.erase(it);
      insertToPQ(newn);
    }
  }
#if DEBUG
  newn->print("newn aft :");
  if (newn->parent()) {
    newn->parent()->print("newn parent aft :");
  }
#endif
}

int Router::snap(const Node* n, const bool vert, const bool up) const
{
  int snapc = (vert ? (up ? _bbox.ymin() : _bbox.ymax())
      : (up ? _bbox.xmin() : _bbox.xmax()));
  const auto& grid = (vert ? _hanangridv[n->z()] : _hanangridh[n->z()]);
  if (!grid.empty()) {
    int pos = n->y(), lkp = n->x();
    if (vert) {
      std::swap(pos, lkp);
    }
    auto itp = grid.find(pos);
    if (itp != grid.end()) {
      for (const auto& r : itp->second) {
        if (lkp >= r.first && lkp <= r.second) {
          snapc = (up ? r.second : r.first);
          break;
        }
      }
    } else {
      snapc = (vert ? n->y() : n->x());
      //snapc = (vert ? (up ? _bbox.ymax() : _bbox.ymin())
      //    : (up ? _bbox.xmax() : _bbox.xmin()));
    }
  }
  return snapc;
}

void Router::getTargetGrid(std::set<int>& s, const Node* n, const bool vert, const int snapc)
{
  for (auto& t : _targets) {
    if (vert) {
      auto maxc = std::max(n->y(), snapc);
      auto minc = std::min(n->y(), snapc);
      if (t->y() > minc && t->y() < maxc) s.insert(t->y());
    } else {
      auto maxc = std::max(n->x(), snapc);
      auto minc = std::min(n->x(), snapc);
      if (t->x() > minc && t->x() < maxc) s.insert(t->x());
    }
  }
}

void Router::getAdjacentGrid(std::set<int>& s, const Node* n, const bool above, const bool up, const int snapc)
{
  int adjLayer = (above ? (n->z() < _maxLayer ? n->z() + 1 : -1) : (n->z() > _minLayer ? n->z() - 1 : -1));
  if (adjLayer >= 0) {
    auto vert = _cf.isVert(n->z());
    const auto& grid = (vert ? _hanangridh[adjLayer] : _hanangridv[adjLayer]);
    if (!grid.empty()) {
      auto coord = (vert ? n->y() : n->x());
      for (auto& pos : grid) {
        if ((up && pos.first > coord && pos.first < snapc) || (!up && pos.first < coord && pos.first > snapc)) {
          s.insert(pos.first);
        }
      }
    }
  }
}

void Router::getCrossGrid(std::set<int>& s, const Node* n, const bool vert, const int snapc)
{
  const auto& grid = vert ? _hanangridh[n->z()] : _hanangridv[n->z()];
  if (grid.empty()) return;
  const int from = vert ? n->y() : n->x();
  const int lkp = vert ? n->x() : n->y();
  const int lo = std::min(from, snapc);
  const int hi = std::max(from, snapc);
  for (auto it = grid.lower_bound(lo); it != grid.end() && it->first < hi; ++it) {
    if (it->first <= lo) continue;
    for (const auto& r : it->second) {
      if (lkp >= r.first && lkp <= r.second) { s.insert(it->first); break; }
    }
  }
}

void Router::expandNode(const Node* n1)
{
  auto n = const_cast<Node*>(n1);
  if (n->closed()) return;
#if DEBUG
  n->print("expanding node :");
#endif

  Node* newn{nullptr};
  if (n->viadown()) {
#if DEBUG
    COUT << "\texpanding via down\n";
#endif
    newn = createNode(n->x(), n->y(), n->z() - 1, n);
    checkAndInsert(newn, n);
  }
  if (n->viaup()) {
#if DEBUG
    COUT << "\texpanding via up\n";
#endif
    newn = createNode(n->x(), n->y(), n->z() + 1, n);
    checkAndInsert(newn, n);
  }

  const bool horiz = _cf.isHor(n->z());
  const bool vert = _cf.isVert(n->z());
  if (horiz) {
    if (n->x() < _bbox.xmin()) {
      n->expand(EAST, false);
    }
    if (n->x() > _bbox.xmax()) {
      n->expand(WEST, false);
    }
  } else {
    n->expand(EAST, false);
    n->expand(WEST, false);
  }
  if (vert) {
    if (n->y() < _bbox.ymin()) {
      n->expand(NORTH, false);
    }
    if (n->y() > _bbox.ymax()) {
      n->expand(SOUTH, false);
    }
  } else {
    n->expand(NORTH, false);
    n->expand(SOUTH, false);
  }

  std::set<int> gridpos;
  if (n->expandwest()) {
#if DEBUG
    COUT << "\texpanding left\n";
#endif
    int snapc = snap(n, false, false);
#if DEBUG
    COUT << "\t\tsnapcd : " << snapc << '\n';
#endif
    newn = nullptr;
    if (snapc < n->x()) {
      newn = createNode(snapc, n->y(), n->z(), n);
    }
    if (newn) checkAndInsert(newn, n);
    getAdjacentGrid(gridpos, n, true, false, snapc);
    getAdjacentGrid(gridpos, n, false, false, snapc);
    getTargetGrid(gridpos, n, false, snapc);
    getCrossGrid(gridpos, n, false, snapc);
    for (auto &pos : gridpos) {
#if DEBUG
      COUT << "\t\tgrid pos : " << pos << '\n';
#endif
      if (pos < n->x()) {
        newn = createNode(pos, n->y(), n->z(), n);
        if (newn) checkAndInsert(newn, n);
      }
    }
    gridpos.clear();
  }
  if (n->expandeast()) {
#if DEBUG
    COUT << "\texpanding right\n";
#endif
    Node* newn{nullptr};
    int snapc = snap(n, false, true);
#if DEBUG
    COUT << "\t\tsnapcu : " << snapc << '\n';
#endif
    newn = nullptr;
    if (snapc > n->x()) {
      newn = createNode(snapc, n->y(), n->z(), n);
      if (newn) checkAndInsert(newn, n);
    }
    getAdjacentGrid(gridpos, n, true, true, snapc);
    getAdjacentGrid(gridpos, n, false, true, snapc);
    getTargetGrid(gridpos, n, false, snapc);
    getCrossGrid(gridpos, n, false, snapc);
    for (auto &pos : gridpos) {
#if DEBUG
      COUT << "\t\tgrid pos : " << pos << '\n';
#endif
      if (pos > n->x()) {
        newn = createNode(pos, n->y(), n->z(), n);
        if (newn) checkAndInsert(newn, n);
      }
    }
    gridpos.clear();
  }
  if (n->expandsouth()) {
#if DEBUG
    COUT << "\texpanding down\n";
#endif
    int snapc = snap(n, true, false);
#if DEBUG
    COUT << "\t\tsnapcd : " << snapc << '\n';
#endif
    newn = nullptr;
    if (snapc < n->y()) {
      newn = createNode(n->x(), snapc, n->z(), n);
    }
    if (newn) checkAndInsert(newn, n);
    getAdjacentGrid(gridpos, n, true, false, snapc);
    getAdjacentGrid(gridpos, n, false, false, snapc);
    getTargetGrid(gridpos, n, true, snapc);
    getCrossGrid(gridpos, n, true, snapc);
    for (auto &pos : gridpos) {
#if DEBUG
      COUT << "\t\tgrid pos : " << pos << '\n';
#endif
      if (pos < n->y()) {
        newn = createNode(n->x(), pos, n->z(), n);
        if (newn) checkAndInsert(newn, n);
      }
    }
    gridpos.clear();
  }
  if (n->expandnorth()) {
#if DEBUG
    COUT << "\texpanding up\n";
#endif
    Node* newn{nullptr};
    int snapc = snap(n, true, true);
#if DEBUG
    COUT << "\t\tsnapcu : " << snapc << '\n';
#endif
    newn = nullptr;
    if (snapc > n->y()) {
      newn = createNode(n->x(), snapc, n->z(), n);
    }
    if (newn) checkAndInsert(newn, n);
    getAdjacentGrid(gridpos, n, true, true, snapc);
    getAdjacentGrid(gridpos, n, false, true, snapc);
    getTargetGrid(gridpos, n, true, snapc);
    getCrossGrid(gridpos, n, true, snapc);
    for (auto &pos : gridpos) {
#if DEBUG
      COUT << "\t\tgrid pos : " << pos << '\n';
#endif
      if (pos > n->y()) {
        newn = createNode(n->x(), pos, n->z(), n);
        if (newn) checkAndInsert(newn, n);
      }
    }
    gridpos.clear();
  }
  n->resetexpand();
}

void Router::insertRange(IntRangeSet& s, const IntPair& r)
{
  IntPair rc = r;
  std::vector<IntRangeSet::iterator> overlapit;
#if DEBUG
  COUT << "insert range : " << rc.first << ' ' << rc.second << '\n';
#endif
  for (auto it = s.begin(); it != s.end(); ++it) {
    if (it->first > rc.second) {
      break;
    } else if (it->first <= rc.second && it->second >= rc.first) {
      rc.first = std::min(it->first, rc.first);
      rc.second = std::max(it->second, rc.second);
      overlapit.push_back(it);
    }
  }
  for (auto& i : overlapit) s.erase(i);
  s.insert(rc);
}

void Router::invertRange(IntRangeSet& s, const bool vert)
{
  IntRangeSet sout;
  int start = _bbox.xmin(), end = _bbox.xmax();
  if (vert) {
    start = _bbox.ymin();
    end = _bbox.ymax();
  }
  for (auto &r : s) {
#if DEBUG
    COUT << "r : " << r.first << ' ' << r.second << '\n';
#endif
    if (r.first >= start) {
      sout.insert(std::make_pair(start, r.first));
    }
    start = r.second;
  }
  sout.insert(std::make_pair(start, end));
  s = sout;
}

void splitRects(Geom::Rects& rects, const PolySet& ps, const Geom::Rect& bbox, const int sx, const int sy)
{
  PRects prects;
  get_max_rectangles(prects, ps);
  rects.reserve(prects.size() + rects.size());
  for (auto& r : prects) {
    Geom::Rect rect(bp::xl(r) - sx, bp::yl(r) - sy, bp::xh(r) + sx, bp::yh(r) + sy);
    if (rect.overlaps(bbox)) {
      rects.emplace_back(rect);
    }
  }
}

Geom::Rects splitRects(const Geom::Rects& tobs)
{
  PolySet ps;
  for (auto& obs : tobs) {
    auto r = obs.bloatby(-1);
    ps.insert(PRect(r.xmin(), r.ymin(), r.xmax(), r.ymax()));
  }
  PRects prects;
  get_max_rectangles(prects, ps);
  Geom::Rects rects;
  rects.reserve(prects.size());
  for (auto& r : prects) {
    rects.emplace_back(bp::xl(r) - 1, bp::yl(r) - 1, bp::xh(r) + 1, bp::yh(r) + 1);
  }
  return rects;
}

/*static bool overlaps(const PPoly& p1, const PPoly& p2)
{
  PolySet ps1;
  ps1 = p1;
  PRects rects1;
  get_rectangles(rects1, ps1);
  PolySet ps2;
  ps2 = p2;
  PRects rects2;
  get_rectangles(rects2, ps2);
  for (auto& r1 : rects1) {
    for (auto& r2 : rects2) {
      if (intersects(r1, r2)) return true;
    }
  }
  return false;
}*/

void coverHoles(PolySet& ps, const PolySet& src, const PolySet& tgt)
{
  PPolyWHs pwhs;
  ps.get(pwhs);
  PRect bboxs, bboxt;
  bool valids = extents(bboxs, src);
  bool validh = extents(bboxt, tgt);
  for (auto& pwh : pwhs) {
    for (auto ith = pwh.begin_holes(); ith != pwh.end_holes(); ++ith) {
      PRect bbox;
      if (extents(bbox, *ith) && (!valids || !intersects(bbox, bboxs)) && (!validh || !intersects(bbox, bboxt))) {
        ps += bbox;
      }
    }
  }
}


void Router::generateHananGrid()
{
  for (auto& l : _ptobstacles) {
    auto its = _psources.find(l.first);
    auto ith = _ptargets.find(l.first);
    coverHoles(l.second, ((its != _psources.end()) ? its->second : PolySet()), 
        ((ith != _ptargets.end()) ? ith->second : PolySet()));
    if (its != _psources.end()) {
      l.second -= its->second;
    }
    if (ith != _ptargets.end()) {
      l.second -= ith->second;
    }
  }
  _hanangridh.clear();
  _hanangridh.resize(_maxLayer + 1);
  _hanangridv.clear();
  _hanangridv.resize(_maxLayer + 1);
  std::set<int> xcoords, ycoords;
  int bloat(0);
  for (auto l = _minLayer; l <= _maxLayer; ++l) {
    bloat = std::max(_spacex[l], bloat);
    bloat = std::max(_spacey[l], bloat);
  }
  _bbox.expand(bloat * 4);
  _bbox.AND(_mbox);
  _ltree.clear();
  _tobstacles.clear();
  for (auto l = _minLayer; l <= _maxLayer; ++l) {
    auto box = _bbox;
    //box.bloat(_bbox.width(), _bbox.height());
    _tobstacles[l].push_back(Geom::Rect(box.xmin() - 100, box.ymin(), box.xmin() - 50, box.ymax()));
    _tobstacles[l].push_back(Geom::Rect(box.xmin(), box.ymin() - 100, box.xmax(), box.ymin() - 50));
    _tobstacles[l].push_back(Geom::Rect(box.xmax() + 50, box.ymin(), box.xmax() + 100, box.ymax()));
    _tobstacles[l].push_back(Geom::Rect(box.xmin(), box.ymax() + 50, box.xmax(), box.ymax() + 100));
  }
  for (auto& l : _ptobstacles) {
    int sx{0}, sy{0};
    auto& layer = l.first;
    if (layer < static_cast<int>(_widthx.size())) {
      sx = spacex(layer) + (layer <= _maxLayer ? ((widthy(layer) % 2 == 0) ? widthy(layer)/2 : (widthy(layer)/2 + 1)) : 0);
      sy = spacey(layer) + (layer <= _maxLayer ? ((widthx(layer) % 2 == 0) ? widthx(layer)/2 : (widthx(layer)/2 + 1)) : 0);
    }
    splitRects(_tobstacles[l.first], l.second, _bbox, sx, sy);
    _tobstacles[l.first] = splitRects(_tobstacles[l.first]);
  }
  for (auto& it: _tobstacles) {
    _ltree.emplace(it.first, Geom::RTree2D(it.second));
  }
  for (auto& l : _tobstacles) {
    if (l.first > _maxLayer) continue;
    for (auto& o : l.second) {
      if (!o.overlaps(_bbox)) continue;
      auto osnapped{o};
      osnapped.snap(_precision);
      xcoords.insert(osnapped.xmin());
      xcoords.insert(osnapped.xmax());
      ycoords.insert(osnapped.ymin());
      ycoords.insert(osnapped.ymax());
    }
  }
  for (bool src : {true, false}) {
    for (auto& s : (src ? _sources : _targets)) {
      xcoords.insert(s->x());
      ycoords.insert(s->y());
    }
  }
  for (auto& l : _tobstacles) {
    if (l.first > _maxLayer) continue;
    for (auto i : {0, 1}) {
      if ((i == 0 && !_cf.isVert(l.first)) || (i == 1 && !_cf.isHor(l.first))) continue;
      std::map<int, IntRangeSet> tmpranges;
      for (auto& x : ((i == 0) ? xcoords : ycoords)) {
        if ((i == 0 && x >= _bbox.xmin() && x <= _bbox.xmax()) || 
            (i == 1 && x >= _bbox.ymin() && x <= _bbox.ymax())) {
          tmpranges[x].clear();
        }
      }

      for (auto& o : l.second) {
        const int lo = (i == 0) ? o.xmin() : o.ymin();
        const int hi = (i == 0) ? o.xmax() : o.ymax();
        const IntPair rng = (i == 0)
          ? std::make_pair(std::max(o.ymin(), _bbox.ymin() - 1), std::min(o.ymax(), _bbox.ymax()))
          : std::make_pair(std::max(o.xmin(), _bbox.xmin() - 1), std::min(o.xmax(), _bbox.xmax()));
        for (auto it = tmpranges.upper_bound(lo); it != tmpranges.end() && it->first < hi; ++it) {
          insertRange(it->second, rng);
        }
      }
      for (auto& r : tmpranges) {
        invertRange(r.second, i == 0);
      }
      if (i == 0) _hanangridv[l.first] = tmpranges;
      else _hanangridh[l.first] = tmpranges;
    }
  }
}

void Router::buildSol(Geom::LayerRects& sol)
{
  if (!_sol) return;
  const Node* n = _sol;
  while (n) {
    auto parent = n->parent();
    if (parent) {
      if (parent->z() == n->z()) {
        const bool vert = (n->x() == parent->x());
        while (true) {
          auto p = parent->parent();
          const bool pv = (p ? p->x() == parent->x() : !vert);
          if (p && p->z() == parent->z() && pv == vert) {
            parent = p;
          } else {
            break;
          }
        }
        auto hwx1 = (n->hwx() == 0 ? widthx(n->z())/2 : n->hwx());
        auto hwx2 = (n->hwx() == 0 ? (widthx(n->z()) - hwx1) : n->hwx());
        auto hwy1 = (n->hwy() == 0 ? widthy(n->z())/2 : n->hwy());
        auto hwy2 = (n->hwy() == 0 ? (widthy(n->z()) - hwy1) : n->hwy());
        auto extnx1 = hwx1;
        auto extnx2 = hwx2;
        auto extny1 = hwy1;
        auto extny2 = hwy2;
        if (n->y() > parent->y()) {
          extnx1 = hwy1;
          extnx2 = hwy2;
          if (isTarget(n)) extny2 = 0;
          else {
            auto it = _endextnymax.find(n);
            if (it != _endextnymax.end()) extny2 = it->second;
          }
          if (isSource(parent)) extny1 = 0;
          else {
            auto it = _endextnymin.find(parent);
            if (it != _endextnymin.end()) extny1 = it->second;
          }
        } else if (n->y() < parent->y()) {
          extnx1 = hwy1;
          extnx2 = hwy2;
          if (isSource(parent)) extny2 = 0;
          else {
            auto it = _endextnymax.find(parent);
            if (it != _endextnymax.end()) extny2 = it->second;
          }
          if (isTarget(n)) extny1 = 0;
          else {
            auto it = _endextnymin.find(n);
            if (it != _endextnymin.end()) extny1 = it->second;
          }
        }
        if (n->x() > parent->x()) {
          extny1 = hwx1;
          extny2 = hwx2;
          if (isTarget(n)) extnx2 = 0;
          else {
            auto it = _endextnxmax.find(n);
            if (it != _endextnxmax.end()) extnx2 = it->second;
          }
          if (isSource(parent)) extnx1 = 0;
          else {
            auto it = _endextnxmin.find(parent);
            if (it != _endextnxmin.end()) extnx1 = it->second;
          }
        } else if (n->x() < parent->x()) {
          extny1 = hwx1;
          extny2 = hwx2;
          if (isSource(parent)) extnx2 = 0;
          else {
            auto it = _endextnxmax.find(parent);
            if (it != _endextnxmax.end()) extnx2 = it->second;
          }
          if (isTarget(n)) extnx1 = 0;
          else {
            auto it = _endextnxmin.find(n);
            if (it != _endextnxmin.end()) extnx1 = it->second;
          }
        }
        sol[n->z()].push_back(Geom::Rect(n->x(), n->y(), parent->x(), parent->y()).bloatby(extnx1, extny1, extnx2, extny2));
#if DEBUG
        COUT << "sol : " << n->z() << ' ' << sol[n->z()].back().str() << ' ' << n->x() << ' ' << n->y() << ' ' << parent->x() << ' ' << parent->y() << '\n';
#endif
      } else {
        if (parent->z() > n->z() && parent->dnVia()) {
          parent->dnVia()->addShapes(sol);
        } else if (parent->z() < n->z() && parent->upVia()) {
          parent->upVia()->addShapes(sol);
        } else {
          if (parent->z() < n->z()) {
            if (!_dnVias[n->z()].empty()) {
              Via v(*(_dnVias[n->z()][0]), Geom::Point(n->x(), n->y()));
              v.addShapes(sol);
            }
          } else if (parent->z() > n->z()) {
            if (!_upVias[n->z()].empty()) {
              Via v(*(_upVias[n->z()][0]), Geom::Point(n->x(), n->y()));
              v.addShapes(sol);
            }
          }
        }
      }
    }
    n = parent;
  }
}

Geom::LayerRects Router::findSol()
{
  TIME_M();
  COUT << "routing : " << _name << '\n';
  _relaxSrcViaEscape = false;
  _relaxTgtViaEscape = false;
  static std::string debugplot{getenv("HANAN_DEBUG_WIRE") ? getenv("HANAN_DEBUG_WIRE") : ""};
  if (!debugplot.empty() && debugplot == _name) {
    COUT << "debug plot enabled for wire : " << debugplot << '\n';
  }
  Geom::LayerRects sol;
  if (!_sources.empty() && !_targets.empty()) {
    size_t minExpansions{1000000};
    for (auto attempt : {0, 1}) {
      if (_targets.size() < _sources.size()) {
        std::swap(_sources,_targets);
        std::swap(_sourceshapes,_targetshapes);
        std::swap(_psources,_ptargets);
      }
      if (attempt == 1) {
        // retry in the reverse direction with fresh state : intermediate nodes
        // from attempt 0 carry costs measured from the old sources
        std::swap(_sourceshapes,_targetshapes);
        std::swap(_psources,_ptargets);
        _sources.clear();
        _targets.clear();
        flushNodes();
        createSourceTargetNodes();
      }
      COUT << "num src : " << _sources.size() << " tgt : " << _targets.size() << std::endl;
      if (verboseLog()) {
        for (auto& s : _sources) {
          s->print("source : ");
        }
        for (auto& t : _targets) {
          t->print("targets : ");
        }
      }
      for (auto& s : _sources) {
        _bbox.merge(s->x(), s->y(), s->x(), s->y());
#if DEBUG
        s->print("src : ");
#endif
      }
      for (auto& t : _targets) {
        _bbox.merge(t->x(), t->y(), t->x(), t->y());
#if DEBUG
        t->print("tgt : ");
#endif
      }

      //for (auto& l : _tobstacles) {
      //  for (auto& o : l.second) {
      //    _bbox.merge(o.xmin(), o.ymin(), o.xmax(), o.ymax());
      //  }
      //}
      _bbox.expand(_bbox.width() / 2, _bbox.height() / 2);

      /*for (auto& l : _tobstacles) {
        COUT << "layer before : " << l.first << '\n';
        for (auto& o : l.second) {
        COUT << "o : " << o.str() << '\n';
        }
        }*/
      // merge _pobstacles and _ptobstacles
      if (attempt == 0) {
          for (auto &l : _pobstacles) {
              _ptobstacles[l.first] |= l.second;
          }
      }
//#if DEBUG
//#else
//      if (!debugplot.empty() && (debugplot == "1" || debugplot == _name)) 
//#endif
        //writeSTO();
      /*for (auto& l : _tobstacles) {
        COUT << "layer after : " << l.first << '\n';
        for (auto& o : l.second) {
        COUT << "o : " << o.str() << '\n';
        }
        }*/
      generateHananGrid();
      for (auto& s : _sources) {
        evalTCost(s);
        if (!s->closed()) insertToPQ(s);
      }
#if DEBUG
#else
      if (!debugplot.empty() && (debugplot == "1" || debugplot == _name || _debugplot))
#endif
        writeLEF(attempt ? "ATTEMPT_1" : "ATTEMPT_0");

#if DEBUG
      for (unsigned l = 0; l < _hanangridh.size(); ++l) {
        COUT << "layerh : " << l << '\n';
        for (auto& pos : _hanangridh[l]) {
          COUT << "pos : " << pos.first << " : ";
          for (auto& r : pos.second) {
            COUT << "[" << r.first << ',' << r.second << "] ";
          }
          COUT << '\n';
        }
      }
      for (unsigned l = 0; l < _hanangridv.size(); ++l) {
        COUT << "layerv : " << l << '\n';
        for (auto& pos : _hanangridv[l]) {
          COUT << "pos : " << pos.first << " : ";
          for (auto& r : pos.second) {
            COUT << "[" << r.first << ',' << r.second << "] ";
          }
          COUT << '\n';
        }
      }
#endif

      std::vector<unsigned> layerExpansions(_maxLayer + 1, 0);
      while (!_pq.empty()) {
        auto t = const_cast<Node*>(*_pq.begin());
        if (_targets.find(t) != _targets.end()) {
          _sol = t;
          COUT << "sol found with " << _expansions << " expansions!" << std::endl;
          for (unsigned i = 0; i < layerExpansions.size(); ++i) {
            COUT << "\texpanded : " << i << ' ' << layerExpansions[i] << '\n';
          }
          break;
        }
        _pq.erase(_pq.begin());
        ++layerExpansions[t->z()];
        expandNode(t);
        ++_expansions;
        if (_expansions >= _maxExpansions) break;
      }
      if (!_sol) {
        COUT << "search failed in pass " << attempt << " for " << _name << " after " << _expansions << " expansions!\n";
        for (unsigned i = 0; i < layerExpansions.size(); ++i) {
          COUT << "\texpanded : " << i << ' ' << layerExpansions[i] << '\n';
        }
      }
      minExpansions = std::min(minExpansions, _expansions);
      _pq.clear();
      _hanangridv.clear();
      _hanangridh.clear();
      _expansions = 0;
      if (_sol) break;
    }
    if (!_sol && _usepinwidth && minExpansions < 1000) {
      _bbox.expand(_bbox.width(), _bbox.height());
      COUT << "retrying " << _name << " with pin width escape\n";
#if DEBUG
#else
      if (!debugplot.empty() && (debugplot == "1" || debugplot == _name || _debugplot))
#endif
        writeLEF("ATTEMPT_2");
      auto savedNdrWidthx = _ndrwidthx;
      auto savedNdrWidthy = _ndrwidthy;
      for (auto src : {true, false}) {
        for (auto& l : (src ? _sourceshapes : _targetshapes)) {
          auto z = l.first;
          if (z < 0 || z >= static_cast<int>(_ndrwidthx.size())) continue;
          for (auto& r : l.second) {
            if (_cf.isVert(z)) {
              auto cur = (_ndrwidthy[z] != INT_MAX ? _ndrwidthy[z] : _widthy[z]);
              _ndrwidthy[z] = std::min(cur, r.width());
            }
            if (_cf.isHor(z)) {
              auto cur = (_ndrwidthx[z] != INT_MAX ? _ndrwidthx[z] : _widthx[z]);
              _ndrwidthx[z] = std::min(cur, r.height());
            }
          }
        }
      }
      _sources.clear();
      _targets.clear();
      flushNodes();
      _bbox = Geom::Rect();
      createSourceTargetNodes();
      for (auto& s : _sources) _bbox.merge(s->x(), s->y(), s->x(), s->y());
      for (auto& t : _targets) _bbox.merge(t->x(), t->y(), t->x(), t->y());
      _bbox.expand(_bbox.width() / 2, _bbox.height() / 2);
      generateHananGrid();
      for (auto& s : _sources) {
        evalTCost(s);
        if (!s->closed()) insertToPQ(s);
      }
      std::vector<unsigned> escLayerExpansions(_maxLayer + 1, 0);
      while (!_pq.empty()) {
        auto t = const_cast<Node*>(*_pq.begin());
        if (_targets.find(t) != _targets.end()) {
          _sol = t;
          COUT << "sol found with pin width for " << _name << " after " << _expansions << " expansions!\n";
          for (unsigned i = 0; i < escLayerExpansions.size(); ++i) {
            COUT << "\texpanded : " << i << ' ' << escLayerExpansions[i] << '\n';
          }
          break;
        }
        _pq.erase(_pq.begin());
        ++escLayerExpansions[t->z()];
        expandNode(t);
        ++_expansions;
        if (_expansions >= _maxExpansions) break;
      }
      if (!_sol) {
        COUT << "pin width also failed for " << _name << " after " << _expansions << " expansions!\n";
      }
      _pq.clear();
      _hanangridv.clear();
      _hanangridh.clear();
      _expansions = 0;
      if (_sol) buildSol(sol);
      _ndrwidthx = savedNdrWidthx;
      _ndrwidthy = savedNdrWidthy;
    } else if (_sol) {
      buildSol(sol);
    }
    if (!_sol && _relaxViaEscape) {
      for (auto stage : {0, 1}) {
        if (stage == 0) {
          _relaxSrcViaEscape = true;
          COUT << "retrying " << _name << " with via escape relaxed at source (min spacing "
               << MIN_ESCAPE_SPACE << ")\n";
        } else {
          _relaxTgtViaEscape = true;
          COUT << "via escape relaxed at source failed for " << _name
               << ", also relaxing at target\n";
        }
        _sources.clear();
        _targets.clear();
        flushNodes();
        _bbox = Geom::Rect();
        createSourceTargetNodes();
        for (auto& s : _sources) _bbox.merge(s->x(), s->y(), s->x(), s->y());
        for (auto& t : _targets) _bbox.merge(t->x(), t->y(), t->x(), t->y());
        _bbox.expand(_bbox.width() / 2, _bbox.height() / 2);
        generateHananGrid();
        for (auto& s : _sources) {
          evalTCost(s);
          if (!s->closed()) insertToPQ(s);
        }
        std::vector<unsigned> relaxLayerExpansions(_maxLayer + 1, 0);
        while (!_pq.empty()) {
          auto t = const_cast<Node*>(*_pq.begin());
          if (_targets.find(t) != _targets.end()) {
            _sol = t;
            COUT << "sol found with via escape relaxed for " << _name << " after " << _expansions << " expansions!\n";
            for (unsigned i = 0; i < relaxLayerExpansions.size(); ++i) {
              COUT << "\texpanded : " << i << ' ' << relaxLayerExpansions[i] << '\n';
            }
            break;
          }
          _pq.erase(_pq.begin());
          ++relaxLayerExpansions[t->z()];
          expandNode(t);
          ++_expansions;
          if (_expansions >= _maxExpansions) break;
        }
        if (!_sol) {
          COUT << "via escape relax stage failed for " << _name << " after " << _expansions << " expansions!\n";
          for (unsigned i = 0; i < relaxLayerExpansions.size(); ++i) {
            COUT << "\texpanded : " << i << ' ' << relaxLayerExpansions[i] << '\n';
          }
        }
        _pq.clear();
        _hanangridv.clear();
        _hanangridh.clear();
        _expansions = 0;
        if (_sol) break;
      }
      if (_sol) {
        buildSol(sol);
      } else {
        COUT << "via escape relaxed at source and target still failed for " << _name << "!\n";
      }
      _relaxSrcViaEscape = false;
      _relaxTgtViaEscape = false;
    }
    if (!_sol) {
      COUT << "sol not found for " << _name << "!\n";
    }
  } else {
    COUT << "source or target empty! " << _sources.empty() << ' ' << _targets.empty() << '\n';
  }
#if DEBUG
#else
  if (!debugplot.empty() && (debugplot == "1" || debugplot == _name))
#endif
  {
    plot();
    printSol();
    writeLEF("SOL", &sol);
  }
  _lastSolFound = (_sol != nullptr);
  clearSourceTargets();
  return sol;
}

void Router::printSol() const
{
  for (auto& s : _sources) {
    s->print("source : ");
  }
  for (auto& t : _targets) {
    t->print("targets : ");
  }
  for (auto& l : _tobstacles) {
    for (auto& o : l.second) {
      COUT << "obs : " << o.str() << ' ' << l.first << '\n';
    }
  }
  if (_sol) {
    const Node* n = _sol;
    while (n) {
      n->print("sol");
      n = n->parent();
    }
  }
}


void Router::plot() const
{
  std::ofstream ofs(_name + "_route.gplt");
  if (ofs.is_open()) {
    COUT << "plotting route to " << _name << "_route.gplt\n";
    ofs << "unset key\n";
    unsigned cnt{1};
    for (auto& l : _tobstacles) {
      const auto& color = LAYER_COLORS[l.first % LAYER_COLORS.size()];
      for (auto& b : l.second) {
        if (b.valid() && b.width() && b.height()) {
          ofs << "set object " << cnt++ << " rect from ";
          ofs << b.xmin() << "," << b.ymin() << " to " << b.xmax() << "," << b.ymax() << " fillstyle transparent solid 0.5 fillcolor \"" << color << "\" behind # " << l.first << '\n';
        }
      }
    }
    int dx{std::max(2, _bbox.width()/100)};
    int dy{std::max(2, _bbox.height()/100)};
    if (_sol) {
      const Node* n = _sol;
      const Node* prev = _sol->parent();
      while (prev) {
        if (prev->z() != n->z()) {
          Geom::Rect b(n->x() - dx, n->y() - dy, n->x() + dx, n->y() + dy);
          const auto& color = LAYER_COLORS[n->z() % LAYER_COLORS.size()];
          ofs << "set object " << cnt++ << " circle at ";
          ofs << n->x() << "," << n->y() << " size " << dx << " fillstyle transparent solid 0.5 fillcolor \"" << color << "\" behind\n";
        }
        n = prev;
        prev = prev->parent();
      }
    }
    ofs << "plot[:][:] '-' using 1:2 w filledcurves lt -1 lw 2 lc 'red', '-' using 1:2 w filledcurves lt -1 lw 2 lc 'blue', '-' using 1:2 w l lt -1 lw 3 lc 6\n";
    for (auto& s : _sources) {
      Geom::Rect b(s->x() - dx, s->y() - dy, s->x() + dx, s->y() + dy);
      ofs << b.xmin() << " " << b.ymin() << "\n";
      ofs << b.xmax() << " " << b.ymin() << "\n";
      ofs << b.xmax() << " " << b.ymax() << "\n";
      ofs << b.xmin() << " " << b.ymax() << "\n";
      ofs << b.xmin() << " " << b.ymin() << "\n\n";
    }
    ofs << "EOF\n";
    for (auto& t : _targets) {
      Geom::Rect b(t->x() - dx, t->y() - dy, t->x() + dx, t->y() + dy);
      ofs << b.xmin() << " " << b.ymin() << "\n";
      ofs << b.xmax() << " " << b.ymin() << "\n";
      ofs << b.xmax() << " " << b.ymax() << "\n";
      ofs << b.xmin() << " " << b.ymax() << "\n";
      ofs << b.xmin() << " " << b.ymin() << "\n\n";
    }
    ofs << "EOF\n\n";
    if (_sol) {
      const Node* n = _sol;
      const Node* prev = _sol->parent();
      while (prev) {
        ofs << n->x() << ' ' << n->y() << ' ' << n->z() << '\n';
        n = prev;
        prev = prev->parent();
        if (!prev && n) {
          ofs << n->x() << ' ' << n->y() << ' ' << n->z() << '\n';
        }
      }
    }
    ofs << "EOF\n\n";
    ofs << "set size ratio GPVAL_DATA_Y_MAX/GPVAL_DATA_X_MAX\nrepl\npause -1";

  }
}


/*void Router::writeSTO() const
{
  std::ofstream ofs(_name + "_route.sto");
  COUT << "writing sto to " << _name << "_route.sto\n";
  if (ofs.is_open()) {
    for (auto& s : _sources) {
      ofs << "Source " << s->x() << ' ' << s->y() << ' ' << s->z() << '\n';
    }
    for (auto& t : _targets) {
      ofs << "Target " << t->x() << ' ' << t->y() << ' ' << t->z() << '\n';
    }
    for (auto& l : _tobstacles) {
      for (auto& o : l.second) {
        ofs << "Obstacle " << o.xmin() << ' ' << o.ymin() << ' ' << o.width() << ' ' << o.height() << ' ' << l.first << '\n';
      }
    }
  }
}*/

void Router::addObstacles(const Geom::LayerRects& lr, const bool temp)
{
  std::set<int> uselayers;
  bool srcInPref{false}, tgtInPref{false};
  if (!_preflayers.empty()) {
    for (auto& l : _sourceshapes) {
      if (_preflayers.find(l.first) != _preflayers.end()) {
        srcInPref = true;
        break;
      }
    }
    for (auto& l : _targetshapes) {
      if (_preflayers.find(l.first) != _preflayers.end()) {
        tgtInPref = true;
        break;
      }
    }
    if (srcInPref && tgtInPref) {
      for (auto& l : _preflayers) {
        uselayers.insert(l);
        if (l >= 0 && l <= _cf.topRoutingLayer()) {
          if (l == *_preflayers.begin()) {
            uselayers.insert(_aboveViaLayer[l]);
          } else if (l == *_preflayers.rbegin()) {
            uselayers.insert(_belowViaLayer[l]);
          } else {
            uselayers.insert(_aboveViaLayer[l]);
            uselayers.insert(_belowViaLayer[l]);
          }
        }
      }
    }
  }
  /*if (_preflayers.empty() || !srcInPref || !tgtInPref) {
    int minLayer{_cf.topRoutingLayer()}, maxLayer{0};
    for (auto src : {true, false}) {
      for (auto& l : (src ? _sourceshapes : _targetshapes)) {
        minLayer = std::min(minLayer, l.first);
        maxLayer = std::max(maxLayer, l.first);
      }
    }
    if (!_preflayers.empty()) {
      minLayer = std::min(minLayer, *_preflayers.begin());
      maxLayer = std::max(maxLayer, *_preflayers.rbegin());
    }
    for (int l = minLayer; l <= maxLayer; ++l) {
      uselayers.insert(l);
      if (l == minLayer) {
        uselayers.insert(_aboveViaLayer[l]);
      } else if (l == maxLayer) {
        uselayers.insert(_belowViaLayer[l]);
      } else {
        uselayers.insert(_aboveViaLayer[l]);
        uselayers.insert(_belowViaLayer[l]);
      }
    }
  }*/
  for (auto& l : lr) {
    const auto& layer = l.first;
    if (!uselayers.empty() && uselayers.find(l.first) == uselayers.end()) continue;
    for (const auto& r : l.second) {
      /*bool olsrcortgt{false};
      for (auto src : {true, false}) {
        const auto& shapes = src ? _sourceshapes : _targetshapes;
        const auto it = shapes.find(layer);
        if (it != shapes.end()) {
          for (auto& sr : it->second) {
            if (sr.overlaps(r, true)) {
              olsrcortgt = true;
              break;
            }
          }
        }
        if (olsrcortgt) break;
      }*/
      //if (!olsrcortgt) {
        if (temp) {
          _ptobstacles[layer] += PRect(r.xmin(), r.ymin(), r.xmax(), r.ymax());
        } else {
          _pobstacles[layer] += PRect(r.xmin(), r.ymin(), r.xmax(), r.ymax());
        }
      //}
    }
  }
}

void Router::updatendr(const bool usendr, const std::map<int, int>& ndrwidths,
    const std::map<int, int>& ndrspaces, const std::map<int, DRC::Direction>& ndrdirs,
    const std::set<int>& preflayers, const std::map<int, DRC::ViaArray>& ndrvias)
{
  _ndrwidthx.clear(); _ndrwidthy.clear();
  _ndrspacex.clear(); _ndrspacey.clear();
  _ndrwidthx.resize(_widthx.size(), INT_MAX);
  _ndrwidthy.resize(_widthy.size(), INT_MAX);
  _ndrspacex.resize(_spacex.size(), INT_MAX);
  _ndrspacey.resize(_spacey.size(), INT_MAX);
  _preflayers = preflayers;
  if (usendr) {
    if (!ndrdirs.empty() || !preflayers.empty()) _cf.updatendr(ndrdirs, preflayers);
    else _cf.resetdirs();
    if (!ndrspaces.empty()) {
      _ndrspacex = _spacex;
      _ndrspacey = _spacey;
      for (auto& it : ndrspaces) {
        _ndrspacex[it.first] = it.second;
        _ndrspacey[it.first] = it.second;
      }
    }
    if (!ndrwidths.empty()) {
      _ndrwidthx = _widthx;
      _ndrwidthy = _widthy;
      for (auto& it : ndrwidths) {
        _ndrwidthx[it.first] = it.second;
        _ndrwidthy[it.first] = it.second;
      }
    } else if (preflayers.empty() && _usepinwidth) {
      if (_sourceshapes.size() == 1) {
        auto itsrc = _sourceshapes.begin();
        auto ittgt = _targetshapes.find(itsrc->first);

        if (ittgt != _targetshapes.end() 
            && ittgt->second.size() == 1
            && itsrc->second.size() == 1) {
          auto z = itsrc->first;
          if ((itsrc->second.begin()->width() < 10 * itsrc->second.begin()->height()) 
              && (itsrc->second.begin()->height() < 10 * itsrc->second.begin()->width())
              && (ittgt->second.begin()->width() < 10 * ittgt->second.begin()->height())
              && (ittgt->second.begin()->height() < 10 * ittgt->second.begin()->width())) {
            if (_cf.isVert(z)) {
              _ndrwidthy[z] = std::min(_ndrwidthy[z], itsrc->second.begin()->width());
              _ndrwidthy[z] = std::min(_ndrwidthy[z], ittgt->second.begin()->width());
            }
            if (_cf.isHor(z)) {
              _ndrwidthx[z] = std::min(_ndrwidthx[z], itsrc->second.begin()->height());
              _ndrwidthx[z] = std::min(_ndrwidthx[z], ittgt->second.begin()->height());
            }
          } else {
            if (_cf.isVert(z) && _cf.isHor(z)) {
              _ndrwidthy[z] = std::min(_ndrwidthy[z], itsrc->second.begin()->width());
              _ndrwidthy[z] = std::min(_ndrwidthy[z], ittgt->second.begin()->width());
              _ndrwidthy[z] = std::min(_ndrwidthy[z], itsrc->second.begin()->height());
              _ndrwidthy[z] = std::min(_ndrwidthy[z], ittgt->second.begin()->height());
              _ndrwidthx[z] = _ndrwidthy[z];
            }
          }
        }
      }
      if (_targetshapes.size() == 1) {
        auto ittgt = _targetshapes.begin();
        auto itsrc = _sourceshapes.find(ittgt->first);
        if (itsrc != _sourceshapes.end() 
            && ittgt->second.size() == 1
            && itsrc->second.size() == 1) {
          auto z = ittgt->first;
          if ((itsrc->second.begin()->width() < 10 * itsrc->second.begin()->height()) 
              && (itsrc->second.begin()->height() < 10 * itsrc->second.begin()->width())
              && (ittgt->second.begin()->width() < 10 * ittgt->second.begin()->height())
              && (ittgt->second.begin()->height() < 10 * ittgt->second.begin()->width())) {
            if (_cf.isVert(z)) {
              _ndrwidthy[z] = std::min(_ndrwidthy[z], itsrc->second.begin()->width());
              _ndrwidthy[z] = std::min(_ndrwidthy[z], ittgt->second.begin()->width());
            }
            if (_cf.isHor(z)) {
              _ndrwidthx[z] = std::min(_ndrwidthx[z], itsrc->second.begin()->height());
              _ndrwidthx[z] = std::min(_ndrwidthx[z], ittgt->second.begin()->height());
            }
          } else {
            if (_cf.isVert(z) && _cf.isHor(z)) {
              _ndrwidthy[z] = std::min(_ndrwidthy[z], itsrc->second.begin()->width());
              _ndrwidthy[z] = std::min(_ndrwidthy[z], ittgt->second.begin()->width());
              _ndrwidthy[z] = std::min(_ndrwidthy[z], itsrc->second.begin()->height());
              _ndrwidthy[z] = std::min(_ndrwidthy[z], ittgt->second.begin()->height());
              _ndrwidthx[z] = _ndrwidthy[z];
            }
          }
        }
      }
    }
    for (unsigned i = 0; i < _ndrwidthx.size(); ++i) {
      COUT << "after updatendr layer : " << i << " width : " << _widthx[i] << ' ' << _widthy[i];
      if (_ndrwidthx[i] != INT_MAX) {
        COUT << " ndr widthx : " << _ndrwidthx[i] ;
      }
      if (_ndrwidthy[i] != INT_MAX) {
        COUT << " ndr widthy : " << _ndrwidthy[i];
      }
      if (_ndrspacex[i] != INT_MAX) {
        COUT << " ndr spacex : " << _ndrspacex[i] ;
      }
      if (_ndrspacey[i] != INT_MAX) {
        COUT << " ndr spacey : " << _ndrspacey[i];
      }
      COUT << '\n';
    }
  }
//#if DEBUG
  for (unsigned i = 0; i < _ndrwidthx.size(); ++i) {
    COUT << "layer : " << i << " width : " << _widthx[i] << ' ' << _widthy[i];
    if (_ndrwidthx[i] != INT_MAX) {
      COUT << " ndr widthx : " << _ndrwidthx[i] ;
    }
    if (_ndrwidthy[i] != INT_MAX) {
      COUT << " ndr widthy : " << _ndrwidthy[i];
    }
    COUT << '\n';
  }
//#endif
  /*if (_targetshapes.size() == 1) {
    auto it = _targetshapes.begin();
    if (it->second.size() == 1) {
      auto z = it->first;
      if (_cf.isVert(z)) {
        _ndrwidthy[z] = std::min(_ndrwidthy[z], it->second.begin()->width());
      }
      if (_cf.isHor(z)) {
        _ndrwidthx[z] = std::min(_ndrwidthx[z], it->second.begin()->height());
      }
    }
  }*/
  constructVias(&ndrvias);
  createSourceTargetNodes();
}

void Router::createSourceTargetNodes()
{
  for (const bool src : {true, false}) {
    bool prefLayerShape{false};
    if (!_preflayers.empty()) {
      for (const auto& l : (src ? _sourceshapes : _targetshapes)) {
        if (_preflayers.find(l.first) != _preflayers.end()) {
          prefLayerShape = true;
          break;
        }
      }
    }
    for (const auto& l : (src ? _psources : _ptargets)) {
      if (prefLayerShape && _preflayers.find(l.first) == _preflayers.end()) continue;
      PRects prects;
      get_rectangles(prects, l.second);
      for (auto& pr : prects) {
        Geom::Rect r(bp::xl(pr), bp::yl(pr), bp::xh(pr), bp::yh(pr));
        addSourceTarget(r, l.first, src);
      }
    }
  }
}

const Via* Router::isViaValid(const Node* n, const bool up) const
{
  Via* via{nullptr};
  if (up) {
    if (n->z() < _maxLayer && !_upVias[n->z()].empty()) {
      auto aboveLayer = _aboveViaLayer[n->z()];
      if (aboveLayer >= 0) {
        for (auto& v : _upVias[n->z()]) {
          delete via;
          via = new Via(*v, Geom::Point(n->x(), n->y()));
          auto it = _ltree.find(aboveLayer);
          if (it != _ltree.end()) {
            Geom::Rects nbrs;
            it->second.search(nbrs, via->bbox().bloatby(_lf.spacex(aboveLayer), _lf.spacey(aboveLayer)));
            for (auto& o : nbrs) {
              for (auto& c : via->cuts()) {
                if (o.bloatby(_lf.spacex(aboveLayer), _lf.spacey(aboveLayer)).overlaps(c, false)) {
                  delete via;
                  via = nullptr;
                  break;
                }
              }
              if (via == nullptr) break;
            }
          }
          if (via) {
            //COUT << "verifying via : " << via->l() << ',' << via->u() << " lpad : " << via->lpad().str() << " upad : " << via->upad().str() << " cuts : ";
            //for (auto& c : via->cuts()) {
            //    COUT << c.str() << ' ';
            //}
            //COUT << '\n';
            //n->print("verifying : ");
            // last-resort escape: relax spacing to MIN_ESCAPE_SPACE for BOTH pads of
            // this one via candidate -- not just the pad on the layer the check
            // happens to be looking at -- whenever the via connects to a registered
            // source/target pin on either of its two layers (covers escaping FROM
            // that pin as well as landing ON it from the adjacent layer). Never
            // applies to a mid-path via.
            const bool relaxThisVia =
                (_relaxSrcViaEscape && (isSourceAt(n->x(), n->y(), via->l()) || isSourceAt(n->x(), n->y(), via->u()))) ||
                (_relaxTgtViaEscape && (isTargetAt(n->x(), n->y(), via->l()) || isTargetAt(n->x(), n->y(), via->u())));
            for (bool lower : {true, false}) {
              auto l = (lower ? via->l() : via->u());
              it = _ltree.find(l);
              if (it != _ltree.end()) {
                Geom::Rect p = (lower ? via->lpad() : via->upad());
                Geom::Rects nbrs;
                it->second.search(nbrs, p.bloatby(spacex(l), spacey(l)));
                //p.expand(-std::min(widthy(l), p.width())/2, -std::min(widthx(l), p.height())/2);
                for (auto o : nbrs) {
                  //COUT << "via up pad check: obs_raw=" << o.str() << " pad=" << p.str() << " layer=" << LAYER_NAMES[l] << '\n';
                  o.expand(-widthy(l)/2, -widthx(l)/2);
                  if (relaxThisVia) {
                    o.expand(-std::max(0, spacex(l) - MIN_ESCAPE_SPACE),
                             -std::max(0, spacey(l) - MIN_ESCAPE_SPACE));
                  }
                  //COUT << "  obs_shrunk=" << o.str() << " overlaps=" << o.overlaps(p, true) << '\n';
                  if (o.overlaps(p, true)) {
                    //COUT << "obs viapad up : " << o.str() << ' ' << p.str() << ' ' << lower << ' ' << LAYER_NAMES[l] << '\n';
                    delete via;
                    via = nullptr;
                    break;
                  }
                }
              }
              if (via == nullptr) break;
            }
          }
          if (via != nullptr) break;
        }
      }
    }
  } else {
    if (n->z() > _minLayer && !_dnVias[n->z()].empty()) {
      auto belowLayer = _belowViaLayer[n->z()];
      if (belowLayer >= 0) {
        for (auto& v : _dnVias[n->z()]) {
          delete via;
          via = new Via(*v, Geom::Point(n->x(), n->y()));
          auto it = _ltree.find(belowLayer);
          if (it != _ltree.end()) {
            Geom::Rects nbrs;
            it->second.search(nbrs, via->bbox().bloatby(_lf.spacex(belowLayer), _lf.spacey(belowLayer)));
            for (auto& o : nbrs) {
              for (auto& c : via->cuts()) {
                if (o.bloatby(_lf.spacex(belowLayer), _lf.spacey(belowLayer)).overlaps(c, false)) {
                  delete via;
                  via = nullptr;
                  break;
                }
              }
              if (via == nullptr) break;
            }
          }
          if (via) {
            //COUT << "verifying via : " << via->l() << ',' << via->u() << " lpad : " << via->lpad().str() << " upad : " << via->upad().str() << " cuts : ";
            //for (auto& c : via->cuts()) {
            //    COUT << c.str() << ' ';
            //}
            //COUT << '\n';
            //n->print("verifying : ");
            // See the "up" branch above for why this is computed once per via,
            // covering both pads, rather than per pad/layer.
            const bool relaxThisVia =
                (_relaxSrcViaEscape && (isSourceAt(n->x(), n->y(), via->l()) || isSourceAt(n->x(), n->y(), via->u()))) ||
                (_relaxTgtViaEscape && (isTargetAt(n->x(), n->y(), via->l()) || isTargetAt(n->x(), n->y(), via->u())));
            for (bool lower : {true, false}) {
              auto l = (lower ? via->l() : via->u());
              it = _ltree.find(l);
              if (it != _ltree.end()) {
                Geom::Rect p = (lower ? via->lpad() : via->upad());
                Geom::Rects nbrs;
                it->second.search(nbrs, p.bloatby(spacex(l), spacey(l)));
                for (auto o : nbrs) {
                  //COUT << "via down pad check: obs_raw=" << o.str() << " pad=" << p.str() << " layer=" << LAYER_NAMES[l] << '\n';
                  o.expand(-widthy(l)/2, -widthx(l)/2);
                  if (relaxThisVia) {
                    o.expand(-std::max(0, spacex(l) - MIN_ESCAPE_SPACE),
                             -std::max(0, spacey(l) - MIN_ESCAPE_SPACE));
                  }
                  //COUT << "  obs_shrunk=" << o.str() << " overlaps=" << o.overlaps(p, true) << '\n';
                  if (o.overlaps(p, true)) {
                    //COUT << "obs viapad down : " << o.str() << ' ' << p.str() << ' ' << lower << ' ' << LAYER_NAMES[l] << '\n';
                    delete via;
                    via = nullptr;
                    break;
                  }
                }
              }
              if (via == nullptr) break;
            }
          }
          if (via != nullptr) break;
        }
      }
    }
  }
  return via;
}

void Router::constructVias(const std::map<int, DRC::ViaArray>* ndrvias)
{
  _upVias.clear();
  _upVias.resize(_widthx.size());
  _dnVias.clear();
  _vias.clear();
  _dnVias.resize(_widthx.size());
  for (auto &v : _lf.layers()) {
    if (v->isVia()) {
      auto vl = static_cast<DRC::ViaLayer*>(v);
      auto lp = vl->layers();
      if (lp.first && lp.second) {
        if (ndrvias && ndrvias->find(v->index()) != ndrvias->end()) {
          const auto& va = ndrvias->find(v->index())->second;
          auto via = std::make_shared<Via>(lp.first->index(), lp.second->index(), v->index());
          const auto& wx = va._sw._width.first;
          const auto& wy = va._sw._width.second;
          const auto& sx = va._sw._space.first;
          const auto& sy = va._sw._space.second;
          Geom::Point c(-wx/2, -wy/2);
          if (va._nx > 1) {
            c.x() = -((va._nx - 1) * (wx + sx) + wx)/2;
          }
          if (va._ny > 1) {
            c.y() = -((va._ny - 1) * (wy + sy) + wy)/2;
          }
          via->addCuts(c, wx, wy, va._nx, va._ny, sx, sy);
          Geom::Rect cbbox = via->bbox();
          auto lx = cbbox.width() + 2 * vl->coverlx();
          auto ly = cbbox.height() + 2 * vl->coverly();
          auto ux = cbbox.width() + 2 * vl->coverux();
          auto uy = cbbox.height() + 2 * vl->coveruy();
          via->setLB(Geom::Rect(-lx/2, -ly/2, lx/2, ly/2));
          via->setUB(Geom::Rect(-ux/2, -uy/2, ux/2, uy/2));
          _vias.push_back(via);
          _upVias[lp.first->index()].push_back(via);
          _dnVias[lp.second->index()].push_back(via);
        } else {
          auto vas = vl->viaArray();
          if (vas.empty()) {
            auto via = std::make_shared<Via>(lp.first->index(), lp.second->index(), v->index());
            auto wx = vl->widthx(), wy = vl->widthy();
            auto lx = wx + 2 * vl->coverlx();
            auto ly = wy + 2 * vl->coverly();
            auto ux = wx + 2 * vl->coverux();
            auto uy = wy + 2 * vl->coveruy();
            via->setLB(Geom::Rect(-lx/2, -ly/2, lx/2, ly/2));
            via->setUB(Geom::Rect(-ux/2, -uy/2, ux/2, uy/2));
            via->addCuts(Geom::Point(-wx/2, -wy/2), wx, wy);
            _vias.push_back(via);
            _upVias[lp.first->index()].push_back(via);
            _dnVias[lp.second->index()].push_back(via);
          } else {
            for (auto& va : vas) {
              auto via = std::make_shared<Via>(lp.first->index(), lp.second->index(), v->index());
              const auto& wx = va._sw._width.first;
              const auto& wy = va._sw._width.second;
              const auto& sx = va._sw._space.first;
              const auto& sy = va._sw._space.second;
              Geom::Point c(-wx/2, -wy/2);
              if (va._nx > 1) {
                c.x() = -((va._nx - 1) * (wx + sx) + wx)/2;
              }
              if (va._ny > 1) {
                c.y() = -((va._ny - 1) * (wy + sy) + wy)/2;
              }
              via->addCuts(c, wx, wy, va._nx, va._ny, sx, sy);
              Geom::Rect cbbox = via->bbox();
              auto lx = cbbox.width() + 2 * vl->coverlx();
              auto ly = cbbox.height() + 2 * vl->coverly();
              auto ux = cbbox.width() + 2 * vl->coverux();
              auto uy = cbbox.height() + 2 * vl->coveruy();
              via->setLB(Geom::Rect(-lx/2, -ly/2, lx/2, ly/2));
              via->setUB(Geom::Rect(-ux/2, -uy/2, ux/2, uy/2));
              _vias.push_back(via);
              _upVias[lp.first->index()].push_back(via);
              _dnVias[lp.second->index()].push_back(via);
            }
          }
        }
      }
    }
  }

  for (auto& v : _vias) {
    COUT << "via : " << v->str() << '\n';
  }

  for (int z = _minLayer; z < _maxLayer; ++z) {
    if (_upVias[z].empty()) continue;
    const Via* v = _upVias[z][0].get();
    const auto& lp = v->lpad();
    const auto& up = v->upad();
    const CostType lo = std::min(_cf.hcost(z),     _cf.vcost(z));
    const CostType hi = std::min(_cf.hcost(z + 1), _cf.vcost(z + 1));
    const int lex = std::max(0, std::max(lp.width(), lp.height()) - widthx(z));
    const int uex = std::max(0, std::max(up.width(), up.height()) - widthx(z + 1));
    CostType extra = 0;
    if (lo < COST_MAX) extra += lo * lex;
    if (hi < COST_MAX) extra += hi * uex;
    if (extra > 0) {
      _cf.addViaPadCost(z, z + 1, extra);
      COUT << "via pad cost : layers " << z << '-' << (z + 1)
           << " pads " << lp.str() << ' ' << up.str()
           << " -> +" << extra << '\n';
    }
  }
}

void Router::writeLEF(const std::string& prefix, const Geom::LayerRects* sol) const
{
  auto name(prefix + "_" + _modname + "_" + _name);
  std::replace(name.begin(), name.end(), '/', '+');
  COUT << "writing LEF file : " << name << ".lef\n";
  std::ofstream ofs(name + (sol ? "_sol.lef" : ".lef"));
  if (ofs.is_open()) {
    ofs << "MACRO " << name << "\n";
    ofs << "  UNITS\n    DISTANCE MICRONS " << _uu << ";\n  END UNITS\n";
    ofs << "  ORIGIN "  << _bbox.xmin()  << ' ' << _bbox.ymin() << " ;\n";
    ofs << "  FOREIGN " << name << ' '  << (1.*_bbox.xmin()/_uu) << ' ' << (1.*_bbox.ymin()/_uu) << " ;\n";
    ofs << "  SIZE "    << (1.*_bbox.width()/_uu) << " BY " << (1.* _bbox.height()/_uu) << " ;\n";
    ofs << "    PIN SRC\n      DIRECTION INOUT ;\n      USE SIGNAL ;\n      PORT\n";
    for (auto& l : _sourceshapes) {
      ofs << "        LAYER " << LAYER_NAMES[l.first] << "_SRC ;\n";
      for (auto& r : l.second) {
        ofs << "          RECT " << (1.*r.xmin()/_uu) << ' ' << (1.*r.ymin()/_uu) << ' ' << (1.*r.xmax()/_uu) << ' ' << (1.*r.ymax()/_uu) << " ;\n";
      }
    }
    ofs << "      END\n    END SRC\n";
    ofs << "    PIN TGT\n      DIRECTION INOUT ;\n      USE SIGNAL ;\n      PORT\n";
    for (auto& l : _targetshapes) {
      ofs << "      LAYER " << LAYER_NAMES[l.first] << "_TGT ;\n";
      for (auto& r : l.second) {
        ofs << "        RECT " << (1.*r.xmin()/_uu) << ' ' << (1.*r.ymin()/_uu) << ' ' << (1.*r.xmax()/_uu) << ' ' << (1.*r.ymax()/_uu) << " ;\n";
      }
    }
    ofs << "      END\n    END TGT\n";
    ofs << "    PIN SRCNODES\n      DIRECTION INOUT ;\n      USE SIGNAL ;\n      PORT\n";
    for (auto& s : _sources) {
      ofs << "        LAYER " << LAYER_NAMES[s->z()] << "_SRC ;\n";
      ofs << "          RECT " << (1.*(s->x() - 10)/_uu) << ' ' << (1.*(s->y() - 10)/_uu) << ' ' << (1.*(s->x() + 10)/_uu) << ' ' << (1.*(s->y() + 10)/_uu) << " ;\n";
    }
    ofs << "      END\n    END SRCNODES\n";
    ofs << "    PIN TGTNODES\n      DIRECTION INOUT ;\n      USE SIGNAL ;\n      PORT\n";
    for (auto& s : _targets) {
      ofs << "        LAYER " << LAYER_NAMES[s->z()] << "_TGT ;\n";
      ofs << "          RECT " << (1.*(s->x() - 10)/_uu) << ' ' << (1.*(s->y() - 10)/_uu) << ' ' << (1.*(s->x() + 10)/_uu) << ' ' << (1.*(s->y() + 10)/_uu) << " ;\n";
    }
    ofs << "      END\n    END TGTNODES\n";
    if (sol) {
      ofs << "    PIN SOL\n      DIRECTION INOUT ;\n      USE SIGNAL ;\n      PORT\n";
      for (auto& l : *sol) {
        ofs << "        LAYER " << LAYER_NAMES[l.first] << "_SOL ;\n";
        for (auto& r : l.second) {
          ofs << "          RECT " << (1.*r.xmin()/_uu) << ' ' << (1.*r.ymin()/_uu) << ' ' << (1.*r.xmax()/_uu) << ' ' << (1.*r.ymax()/_uu) << " ;\n";
        }
      }
      ofs << "      END\n    END SOL\n";
    }
    ofs << "    PIN GRID\n      DIRECTION INOUT ;\n      USE SIGNAL ;\n      PORT\n";
    for (auto vert : {true, false}) {
      for (unsigned l = 0; l < _hanangridh.size(); ++l) {
        ofs << "        LAYER " << LAYER_NAMES[l] << "_GRID ;\n";
        for (auto& pos : (vert ? _hanangridv[l] : _hanangridh[l])) {
          for (auto& r : pos.second) {
            if (vert) {
              ofs << "          RECT " << (1.*(pos.first - 1)/_uu) << ' ' << (1.*r.first/_uu) << ' ' << (1.*(pos.first + 1)/_uu) << ' ' << (1.*r.second/_uu) << " ;\n";
            } else {
              ofs << "          RECT " << (1.*r.first/_uu) << ' ' << (1.*(pos.first - 1)/_uu) << ' ' << (1.*r.second/_uu) << ' ' << (1.*(pos.first + 1)/_uu) << " ;\n";
            }
          }
        }
      }
    }
    ofs << "      END\n    END GRID\n";
    ofs << "    OBS\n";
    if (!_tobstacles.empty() || !_obstacles.empty()) {
      for (auto temp : {true, false}) {
        for (auto& l : (temp ? _tobstacles : _obstacles)) {
          ofs << "      LAYER " << LAYER_NAMES[l.first] << " ;\n";
          for (auto& r : l.second) {
            ofs << "        RECT " << (1.*r.xmin()/_uu) << ' ' << (1.*r.ymin()/_uu) << ' ' << (1.*r.xmax()/_uu) << ' ' << (1.*r.ymax()/_uu) << " ;\n";
          }
        }
      }
    }
    if (!_ptobstacles.empty() || !_pobstacles.empty()) {
      for (auto temp : {true, false}) {
        for (auto& l : (temp ? _ptobstacles : _pobstacles)) {
          ofs << "      LAYER " << LAYER_NAMES[l.first] << "_p ;\n";
          PRects prects;
          get_rectangles(prects, l.second);
          for (auto& pr : prects) {
            Geom::Rect r(bp::xl(pr), bp::yl(pr), bp::xh(pr), bp::yh(pr));
            ofs << "        RECT " << (1.*r.xmin()/_uu) << ' ' << (1.*r.ymin()/_uu) << ' ' << (1.*r.xmax()/_uu) << ' ' << (1.*r.ymax()/_uu) << " ;\n";
          }
        }
      }
    }
    ofs << "      LAYER BBOX ;\n";
    ofs << "        RECT " << (1.*_bbox.xmin()/_uu) << ' ' << (1.*_bbox.ymin()/_uu) << ' ' << (1.*_bbox.xmax()/_uu) << ' ' << (1.*_bbox.ymax()/_uu) << " ;\n";
    ofs << "    END\n";
    ofs << "END " << name << "\nEND LIBRARY\n";
  }
}

}
