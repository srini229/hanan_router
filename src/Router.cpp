#include "Router.h"

namespace Router {


CostFn::CostFn(const DRC::LayerInfo& lf)
{
  auto& layers = lf.layers();
  _layerHCost.resize(layers.size(), 100000);
  _layerVCost.resize(layers.size(), 100000);
  for (unsigned i = 0; i < layers.size(); ++i) {
    if (layers[i]->isMetal()) {
      auto r = std::max(1, static_cast<int>(layers[i]->meanR()));
      if (static_cast<DRC::MetalLayer*>(layers[i])->isHorizontal()) {
        _layerHCost[i] = r;
      }
      if (static_cast<DRC::MetalLayer*>(layers[i])->isVertical()) {
        _layerVCost[i] = r;
      }
      _topRoutingLayer = static_cast<int>(i);
    }
  }
  _layerPairCost.resize(_layerHCost.size(), std::vector<CostType>(_layerHCost.size(), 100000));
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
  for (unsigned i = 0; i < _layerHCost.size(); ++i) {
    COUT << "layer : " << i << " cost : " << _layerHCost[i] << ' ' << _layerVCost[i] << '\n';
  }
  for (unsigned i = 0; i < _layerHCost.size(); ++i) {
    if (i > 0) {
      COUT << "layerPairCost : " << i << ' ' << i - 1 << ' ' << _layerPairCost[i][i-1] << '\n';
    }
    if (i < _layerHCost.size() - 1) {
      COUT << "layerPairCost : " << i << ' ' << i + 1 << ' ' << _layerPairCost[i][i+1] << '\n';
    }
  }
}

CostType CostFn::deltaCost(const Node& n1, const Node& n2) const
{
  CostType dc = 0;

  if (n1.z() == n2.z()) {
    if (n1.x() == n2.x() && _layerVCost[n1.z()] < 1000) {
      return (_layerVCost[n1.z()] * std::abs(n1.y() - n2.y()));
    }
    if (n1.y() == n2.y() && _layerHCost[n1.z()] < 1000) {
      return (_layerHCost[n1.z()] * std::abs(n1.x() - n2.x()));
    }
  }
  if (n1.x() == n2.x() && n1.y() == n2.y()) {
    if (n2.z() - n1.z() == 1) {
      return _layerPairCost[n1.z()][n2.z()];
    } else if (n1.z() - n2.z() == 1) {
      return _layerPairCost[n2.z()][n1.z()];
    }
  }
  auto minz = std::min(n1.z(), n2.z());
  auto maxz = std::max(n1.z(), n2.z());
  CostType minHCost(20000), minVCost(20000);
  if (_layerHCost[minz] != _layerVCost[minz]) {
    if (minz < _topRoutingLayer) maxz = minz + 1;
    else minz -= 1;
  }
  for (int i = minz; i <= maxz; ++i) {
    minHCost = std::min(minHCost, _layerHCost[i]);
    minVCost = std::min(minVCost, _layerVCost[i]);
  }
  dc += (minHCost * std::abs(n1.x() - n2.x())) + (minVCost * std::abs(n1.y() - n2.y()));
  for (int i = std::min(n1.z(), minz); i < std::max(n1.z(), minz); ++i) {
    dc += _layerPairCost[i][i+1];
  }
  for (int i = minz; i < maxz; ++i) {
    dc += _layerPairCost[i][i+1];
  }
  for (int i = std::min(n2.z(), maxz); i < std::max(n2.z(), maxz); ++i) {
    dc += _layerPairCost[i][i+1];
  }

  return dc;
}


Router::Router(const DRC::LayerInfo& lf) : _cf{lf}, _sol{nullptr}, _minLayer{100}, _maxLayer{0}, _maxRoutingLayer{0}, _name{}, _lf{lf}
{
  auto& layers = lf.layers();
  _widthx.reserve(layers.size());
  _widthy.reserve(layers.size());
  _spacex.reserve(layers.size());
  _spacey.reserve(layers.size());
  _ndrwidthx.reserve(layers.size());
  _ndrwidthy.reserve(layers.size());
  for (unsigned i = 0; i < layers.size(); ++i) {
    if (layers[i]->isMetal()) {
      auto mlayer = static_cast<DRC::MetalLayer*>(layers[i]);
      _widthx.push_back(mlayer->width());
      _widthy.push_back(mlayer->width());
      _spacex.push_back(mlayer->space());
      _spacey.push_back(mlayer->space());
      COUT << "layer : " << i << " width : " << _widthx.back() << " space : " << _spacex.back() << " v : " << isVert(i) << " h : " << isHor(i) << '\n';
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
    }
  }
  for (unsigned i = 0; i < _aboveViaLayer.size(); ++i) {
      COUT << "layer : " << i << " above : " << _aboveViaLayer[i] << " below : " << _belowViaLayer[i] << '\n';
  }
  _minLayer = lf.signalBottomLayer();
  _maxLayer = lf.signalTopLayer();
  _maxRoutingLayer = static_cast<int>(_widthx.size()) - 1;
  _nodes.clear();
  _nodes.resize(_maxLayer + 1);
}

Node* Router::createNode(const int x, const int y, const int z,
    const Node* parent, const int fcost, const int tcost)
{
  auto& nz = _nodes[z];
  auto coord = std::make_pair(x,y);
  auto it = nz.find(coord);
  Node* n = nullptr;
  if (it == nz.end()) {
    n = new Node(x, y, z, fcost, tcost, parent);
    nz[coord] = n;
#if DEBUG
    COUT << "creating new node : " << x << ',' << y << ',' << z << '\n';
#endif
  } else {
    n = it->second;
  }
  return n;
}

Geom::PointWidthSet Router::findValidPoints(const Geom::Rect& r, const int z, const Direction dir) const
{
  auto vert = isVert(z);
  auto hor = isHor(z);
  auto x = r.xcenter(), y = r.ycenter(); 
  Geom::PointWidthSet points;

  if (dir == DOWN || dir == UP) {
    if ((vert && r.isVert()) || (hor && r.isHor())) {
      auto adj = z;
      if (dir == UP) {
        if (z < _maxLayer) adj = z + 1;
      } else {
        if (z > _minLayer) adj = z - 1;
      }

      if (adj != z) {
        auto wx = widthx(adj), wy = widthy(adj);
        if (vert) {
          int space = _spacey[adj] + ((wy % 2 == 0) ? wy/2 : (wy/2 + 1));
          int yn = r.ymax() - ((r.ymax() - y) % space);
          while (yn > r.ymin()) {
            points.insert(std::make_pair(Geom::Point(x,yn), widthy(z)));
            yn -= space;
          }
        } else {
          int space = _spacex[adj] + ((wx % 2 == 0) ? wx/2 : (wx/2 + 1));
          int xn = r.xmax() - ((r.xmax() - x) % space);
          while (xn > r.xmin()) {
            points.insert(std::make_pair(Geom::Point(xn,y), widthx(z)));
            xn -= space;
          }
        }
      }
    }
  } else if (dir == EAST || dir == WEST) {
    if (hor) {
      auto width = widthx(z);
      COUT << "we : " << width << '\n';
      if (width <= r.height()) {
        int y = (r.ymin() + width / 2);
        int space = _spacex[z] + ((width % 2 == 0) ? width/2 : (width/2 + 1));
        for (auto right : {true, false}) {
          int x = (right ? r.xmax() : r.xmin());
          int yn = y;
          while (yn < r.ymax()) {
            points.insert(std::make_pair(Geom::Point(x,yn), width));
            yn += space;
          }
        }
      }
    }
  } else if (dir == NORTH || dir == SOUTH) {
    if (vert) {
      auto width = widthy(z);
      COUT << "wn : " << width << '\n';
      if (width <= r.width()) {
        int x = (r.xmin() + width / 2);
        int space = _spacey[z] + ((width % 2 == 0) ? width/2 : (width/2 + 1));
        for (auto top : {true, false}) {
          int y = (top ? r.ymax() : r.ymin());
          int xn = x;
          while (xn < r.xmax()) {
            points.insert(std::make_pair(Geom::Point(xn,y), width));
            xn += space;
          }
        }
      }
    }
  }

  return points;
}

void Router::addSourceTargetShapes(const Geom::Rect& r, const int z, const bool src)
{
  auto& shapes = (src ? _sourceshapes : _targetshapes);
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

void Router::addSourceTarget(const Geom::Rect& r, const int z, const bool src)
{
  auto& shapes = (src ? _sourceshapes : _targetshapes);
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
  for (auto dir : {UP, DOWN, EAST, NORTH}) {
    auto points = findValidPoints(r, z, dir);
    for (auto& pp : points) {
      auto& p = pp.first;
      auto n = createNode(p.x(), p.y(), z, nullptr, fcost, tcost);
      n->expand(dir, true);
      if (dir == EAST) {
        n->sethwx(pp.second/2);
        n->expand(WEST, true);
      }
      if (dir == NORTH) {
        n->sethwy(pp.second/2);
        n->expand(SOUTH, true);
      }
#if DEBUG
      COUT << r.str() << '\n';
      n->print((src ? "src" : "tgt"));
#endif
      dest.insert(n);
    }
  }
}

void Router::readDataFile(const std::string& ifile)
{
  setName(ifile.substr(0, ifile.find(".sto")));
  COUT << "reading datafile : " << ifile << '\n';
  std::ifstream ifs(ifile);
  std::string tmps;
  int zmax = -1, zmin = 100;
  while (ifs) {
    ifs >> tmps;
    int x, y, z;
    int w, h;
    if (tmps.empty()) continue;
    switch (tmps[0]) {
      case 'S':
        ifs >> x >> y >> z;
        _sources.insert(createNode(x, y, z, nullptr, 0));
        zmax = std::max(zmax, z);
        zmin = std::min(zmin, z);
        break;
      case 'T':
        ifs >> x >> y >> z;
        _targets.insert(createNode(x, y, z, nullptr, -1, 0));
        zmax = std::max(zmax, z);
        zmin = std::min(zmin, z);
        break;
      case 'O':
        ifs >> x >> y >> w >> h >> z;
        _obstacles[z].push_back(Geom::Rect(x, y, x + w, y + h));
        zmax = std::max(zmax, z);
        zmin = std::min(zmin, z);
        break;
      default:
        break;
    };
  }
  //_cf = CostFn(zmax + 1, zmin + 1, zmin);
  if (zmin == zmax) {
    ++zmax;
  }
  _minLayer = zmin;
  _maxLayer = zmax;
  COUT << "min layer : " << _minLayer << " max layer : " << _maxLayer << '\n';
}

void Router::insertToPQ(const Node* n)
{
#if DEBUG
  n->print("adding to pq :");
  if (n->parent()) n->parent()->print("\tparent:");
#endif
  setexpand(const_cast<Node*>(n), n->parent());
  _pq.insert(n);
}

void Router::setexpand(Node* newn, const Node* parent) const
{
  if (parent) {
    newn->clearexpand();
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
    if (newn->z() < parent->z() || newn->z() >= _maxLayer || !isViaValid(newn, true)) {
      newn->expand(UP, false);
    }
    if (newn->z() > parent->z() || newn->z() <= _minLayer || !isViaValid(newn, false)) {
      newn->expand(DOWN, false);
    }
  }
}

void Router::checkAndInsert(Node* newn, const Node* n)
{
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
      snapc = (vert ? (up ? _bbox.ymax() : _bbox.ymin())
          : (up ? _bbox.xmax() : _bbox.xmin()));
    }
  }
  return snapc;
}

void Router::getAdjacentGrid(std::set<int>& s, const Node* n, const bool above, const bool up, const int snapc)
{
  int adjLayer = (above ? (n->z() < _maxLayer ? n->z() + 1 : -1) : (n->z() > _minLayer ? n->z() - 1 : -1));
  if (adjLayer >= 0) {
    auto vert = isVert(n->z());
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

void Router::expandNode(const Node* n1)
{
  auto n = const_cast<Node*>(n1);
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

  const bool horiz = isHor(n->z());
  const bool vert = isVert(n->z());
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
  if (n->expandnorth()) {
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
  if (n->expandsouth()) {
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
  n->clearexpand(true);
}

void Router::insertRange(IntRangeSet& s, const IntRange& r)
{
  IntRange rc = r;
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

void Router::generateHananGrid()
{
  _hanangridh.clear();
  _hanangridh.resize(_maxLayer + 1);
  _hanangridv.clear();
  _hanangridv.resize(_maxLayer + 1);
  std::set<int> xcoords, ycoords;
  for (auto l = _minLayer; l <= _maxLayer; ++l) {
    auto box = _bbox;
    //box.bloat(_bbox.width(), _bbox.height());
    _tobstacles[l].push_back(Geom::Rect(box.xmin() - 10, box.ymin(), box.xmin(), box.ymax()));
    _tobstacles[l].push_back(Geom::Rect(box.xmin(), box.ymin() - 10, box.xmax(), box.ymin()));
    _tobstacles[l].push_back(Geom::Rect(box.xmax(), box.ymin(), box.xmax() + 10, box.ymax()));
    _tobstacles[l].push_back(Geom::Rect(box.xmin(), box.ymax(), box.xmax(), box.ymax() + 10));
  }
  for (auto& l : _tobstacles) {
    if (l.first > _maxLayer) continue;
    for (auto& o : l.second) {
      if (!o.overlaps(_bbox)) continue;
      xcoords.insert(o.xmin());
      xcoords.insert(o.xmax());
      ycoords.insert(o.ymin());
      ycoords.insert(o.ymax());
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
      if ((i == 0 && !isVert(l.first)) || (i == 1 && !isHor(l.first))) continue;
      std::map<int, IntRangeSet> tmpranges;
      for (auto& x : ((i == 0) ? xcoords : ycoords)) {
        if ((i == 0 && x >= _bbox.xmin() && x <= _bbox.xmax()) || 
            (i == 1 && x >= _bbox.ymin() && x <= _bbox.ymax())) {
          tmpranges[x].clear();
        }
      }

      for (auto& v : tmpranges) {
        for (auto& o : l.second) {
          if (i == 0) {
            if (v.first > o.xmin() && v.first < o.xmax()) {
              auto minv{std::max(o.ymin(), _bbox.ymin() - 1)};
              auto maxv{std::min(o.ymax(), _bbox.ymax())};
              insertRange(tmpranges[v.first], std::make_pair(minv, maxv));
            }
          } else {
            if (v.first > o.ymin() && v.first < o.ymax()) {
              auto minv{std::max(o.xmin(), _bbox.xmin() - 1)};
              auto maxv{std::min(o.xmax(), _bbox.xmax())};
              insertRange(tmpranges[v.first], std::make_pair(minv, maxv));
            }
          }
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

Geom::LayerRects Router::findSol()
{
  TIME_M();
#if DEBUG
  COUT << "routing : " << _name << '\n';
#endif
  static std::string debugplot{getenv("HANAN_DEBUG_WIRE") ? getenv("HANAN_DEBUG_WIRE") : ""};
  Geom::LayerRects sol;
  if (!_sources.empty() && !_targets.empty()) {
    _bbox = Geom::Rect();
    for (auto& s : _sources) {
      evalTCost(s);
      insertToPQ(s);
      _bbox.merge(s->x(), s->y(), s->x(), s->y());
#if DEBUG
      COUT << "src : " << s->x() << ' ' << s->y() << '\n';
#endif
    }
    for (auto& t : _targets) {
      _bbox.merge(t->x(), t->y(), t->x(), t->y());
#if DEBUG
      COUT << "tgt : " << t->x() << ' ' << t->y() << '\n';
#endif
    }

    /*for (auto& l : _tobstacles) {
      COUT << "layer before : " << l.first << '\n';
      for (auto& o : l.second) {
        COUT << "o : " << o.str() << '\n';
      }
    }*/
    Geom::MergeLayerRects(_tobstacles, _obstacles);
#if DEBUG
#else
    if (!debugplot.empty() && (debugplot == "1" || debugplot == _name)) 
#endif
      writeSTO();
    /*for (auto& l : _tobstacles) {
      COUT << "layer after : " << l.first << '\n';
      for (auto& o : l.second) {
        COUT << "o : " << o.str() << '\n';
      }
    }*/
    generateHananGrid();

#if DEBUG
    for (unsigned l = 0; l < _hanangridh.size(); ++l) {
      COUT << "layerh : " << l << '\n';
      for (auto& pos : _hanangridh[l]) {
        COUT << "pos : " << pos.first << " : ";
        for (auto& r : pos.second) {
          std::cout << "[" << r.first << ',' << r.second << "] ";
        }
        std::cout << '\n';
      }
    }
    for (unsigned l = 0; l < _hanangridv.size(); ++l) {
      COUT << "layerv : " << l << '\n';
      for (auto& pos : _hanangridv[l]) {
        COUT << "pos : " << pos.first << " : ";
        for (auto& r : pos.second) {
          std::cout << "[" << r.first << ',' << r.second << "] ";
        }
        std::cout << '\n';
      }
    }
#endif

    while (!_pq.empty()) {
      auto t = const_cast<Node*>(*_pq.begin());
      if (_targets.find(t) != _targets.end()) {
        _sol = t;
        COUT << "sol found " << std::endl;
        break;
      }
      _pq.erase(_pq.begin());
      expandNode(t);
      ++_expansions;
      if (_expansions >= _maxExpansions) break;
    }
    if (!_sol) {
      COUT << "sol not found for " << _name << '\n';
    }
    _pq.clear();
    _hanangridv.clear();
    _hanangridh.clear();
    _expansions = 0;
    if (_sol) {
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
            auto hwx = (n->hwx() == 0 ? widthx(n->z())/2 : n->hwx());
            auto hwy = (n->hwy() == 0 ? widthy(n->z())/2 : n->hwy());
            auto extnx1 = hwx;
            auto extnx2 = hwx;
            auto extny1 = hwy;
            auto extny2 = hwy;
            if (n->y() > parent->y()) {
              extnx1 = hwy;
              extnx2 = hwy;
              if (isTarget(n)) extny2 = 0;
              if (isSource(parent)) extny1 = 0;
              auto it = _endextnymax.find(n);
              if (it != _endextnymax.end()) {
                extny2 = it->second;
              }
              it = _endextnymin.find(parent);
              if (it != _endextnymin.end()) {
                extny1 = it->second;
              }
            } else if (n->y() < parent->y()) {
              extnx1 = hwy;
              extnx2 = hwy;
              if (isTarget(n)) extny1 = 0;
              if (isSource(parent)) extny2 = 0;
              auto it = _endextnymax.find(n);
              if (it != _endextnymax.end()) {
                extny2 = it->second;
              }
              it = _endextnymin.find(parent);
              if (it != _endextnymin.end()) {
                extny1 = it->second;
              }
            }
            if (n->x() > parent->x()) {
              extny1 = hwx;
              extny2 = hwx;
              if (isTarget(n)) extnx2 = 0;
              if (isSource(parent)) extnx1 = 0;
              auto it = _endextnxmax.find(n);
              if (it != _endextnxmax.end()) {
                extnx2 = it->second;
              }
              it = _endextnxmin.find(parent);
              if (it != _endextnxmin.end()) {
                extnx1 = it->second;
              }
            } else if (n->x() < parent->x()) {
              extny1 = hwx;
              extny2 = hwx;
              if (isTarget(n)) extnx1 = 0;
              if (isSource(parent)) extnx2 = 0;
              auto it = _endextnxmax.find(parent);
              if (it != _endextnxmax.end()) {
                extnx2 = it->second;
              }
              it = _endextnxmin.find(n);
              if (it!= _endextnxmin.end()) {
                extnx1 = it->second;
              }
            }
            sol[n->z()].push_back(Geom::Rect(n->x(), n->y(), parent->x(), parent->y()).bloatby(extnx1, extny1, extnx2, extny2));
#if DEBUG
            COUT << n->z() << ' ' << sol[n->z()].back().str() << '\n';
#endif
          } else {
            auto adjLayer = (parent->z() < n->z()) ? _belowViaLayer[n->z()] : _aboveViaLayer[n->z()];
            if (adjLayer >= 0) {
              sol[adjLayer].push_back(Geom::Rect(n->x(), n->y(), n->x(), n->y()).bloatby(_widthx[adjLayer]/2, _widthy[adjLayer]/2));
#if DEBUG
              COUT << adjLayer << ' ' << sol[adjLayer].back().str() << '\n';
#endif
            }
          }
        }
        n = parent;
      }
    }
  } else {
    COUT << "source or target empty!\n";
  }
#if DEBUG
#else
  if (!debugplot.empty() && (debugplot == "1" || debugplot == _name))
#endif
  {
    plot();
    printSol();
  }
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


void Router::writeSTO() const
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
}


void Router::addObstacles(const Geom::LayerRects& lr, const bool temp)
{
  for (auto& l : lr) {
    const auto& layer = l.first;
    for (auto& r : l.second) {
      int spacex{0}, spacey{0};
      if (layer < static_cast<int>(_widthx.size())) {
        spacex = _spacex[layer] + ((widthx(layer) % 2 == 0) ? widthx(layer)/2 : (widthx(layer)/2 + 1));
        spacey = _spacey[layer] + ((widthy(layer) % 2 == 0) ? widthy(layer)/2 : (widthy(layer)/2 + 1));
      }
#if DEBUG
      COUT << "layer : " << layer << " obs : " << spacex << ' ' << spacey << ' ' << r.xmin() << ' ' << r.ymin() << ' ' << r.xmax() << ' ' << r.ymax() << '\n';
#endif
      auto obs = r.bloatby(spacex, spacey);
      int x1 = spacex, x2 = spacex, y1 = spacey, y2 = spacey;
      for (auto src : {true, false}) {
        const auto it = src ? _sourceshapes.find(l.first) : _targetshapes.find(l.first);
        const auto itend = src ? _sourceshapes.end() : _targetshapes.end();
        const auto& nodes = src ? _sources : _targets;
        if (it != itend) {
          for (const auto& s : it->second) {
            if (isVert(l.first)) {
#if DEBUG
              COUT << "source/target shape : " << s.str() << '\n';
              COUT << "obs : " << obs.str() << '\n';
              COUT << "r : " << r.str() << '\n';
#endif
              if (obs.ymin() <= s.ymin() && obs.ymax() >= s.ymax() && 
                  obs.xmin() <= s.xmax() && obs.xmax() >= s.xmin()) {
#if DEBUG
                COUT << "overlapping : \n";
#endif
                if (r.ymin() >= s.ymax()) {
                  for (auto& n : nodes) {
                    if (n->z() == l.first && obs.contains(n->x(), n->y())) {
                      _endextnymax[n] = 0;
                    }
                  }
                  y1 = 0;
                } else {
                  for (auto& n : nodes) {
                    if (n->z() == l.first && obs.contains(n->x(), n->y())) {
                      _endextnymin[n] = 0;
                    }
                  }
                  y2 = 0;
                }
              }
            }
            if (isHor(l.first)) {
              if (obs.xmin() <= s.xmin() && obs.xmax() >= s.xmax() &&
                  obs.ymin() <= s.ymax() && obs.ymax() >= s.ymin() ) {
                if (r.xmin() >= s.xmax()) {
                  for (auto& n : nodes) {
                    if (n->z() == l.first && obs.contains(n->x(), n->y())) {
                      _endextnxmax[n] = 0;
                    }
                  }
                  x1 = 0;
                } else {
                  for (auto& n : nodes) {
                    if (n->z() == l.first && obs.contains(n->x(), n->y())) {
                      _endextnxmin[n] = 0;
                    }
                  }
                  x2 = 0;
                }
              }
            }
          }
        }
      }
      obs = r.bloatby(x1, y1, x2, y2);
      if (temp) {
        _tobstacles[layer].push_back(obs);
        COUT << "tobs : " << _tobstacles[layer].back().str() << '\n';
      } else {
        _obstacles[layer].push_back(obs);
        COUT << "obs : " << _obstacles[layer].back().str() << '\n';
      }
    }
  }
}

void Router::updatendr()
{
  _ndrwidthx.clear(); _ndrwidthy.clear();
  _ndrwidthx.resize(_widthx.size(), INT_MAX);
  _ndrwidthy.resize(_widthy.size(), INT_MAX);
  if (_sourceshapes.size() == 1) {
    auto itsrc = _sourceshapes.begin();
    auto ittgt = _targetshapes.find(itsrc->first);

    if (ittgt != _targetshapes.end() 
        && ittgt->second.size() == 1
        && itsrc->second.size() == 1) {
      auto z = itsrc->first;
      if (isVert(z)) {
        _ndrwidthy[z] = std::min(_ndrwidthy[z], itsrc->second.begin()->width());
        _ndrwidthy[z] = std::min(_ndrwidthy[z], ittgt->second.begin()->width());
      }
      if (isHor(z)) {
        _ndrwidthx[z] = std::min(_ndrwidthx[z], itsrc->second.begin()->height());
        _ndrwidthx[z] = std::min(_ndrwidthx[z], ittgt->second.begin()->height());
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
      if (isVert(z)) {
        _ndrwidthy[z] = std::min(_ndrwidthy[z], itsrc->second.begin()->width());
        _ndrwidthy[z] = std::min(_ndrwidthy[z], ittgt->second.begin()->width());
      }
      if (isHor(z)) {
        _ndrwidthx[z] = std::min(_ndrwidthx[z], itsrc->second.begin()->height());
        _ndrwidthx[z] = std::min(_ndrwidthx[z], ittgt->second.begin()->height());
      }
    }
  }
#if DEBUG
  for (unsigned i = 0; i < _ndrwidthx.size(); ++i) {
    if (_ndrwidthx[i] != INT_MAX) {
      COUT << "ndr widthx z : " << i << ' ' << _ndrwidthx[i] << '\n';
    }
    if (_ndrwidthy[i] != INT_MAX) {
      COUT << "ndr widthy z : " << i << ' ' << _ndrwidthy[i] << '\n';
    }
  }
#endif
  /*if (_targetshapes.size() == 1) {
    auto it = _targetshapes.begin();
    if (it->second.size() == 1) {
      auto z = it->first;
      if (isVert(z)) {
        _ndrwidthy[z] = std::min(_ndrwidthy[z], it->second.begin()->width());
      }
      if (isHor(z)) {
        _ndrwidthx[z] = std::min(_ndrwidthx[z], it->second.begin()->height());
      }
    }
  }*/
  for (const auto& l : _sourceshapes) {
    for (auto& r : l.second) {
      addSource(r, l.first);
    }
  }
  for (const auto& l : _targetshapes) {
    for (auto& r : l.second) {
      addTarget(r, l.first);
    }
  }
}

bool Router::isViaValid(const Node* n, const bool up) const
{
  if (up) {
    if (n->z() < _maxLayer) {
      auto aboveLayer = _aboveViaLayer[n->z()];
      if (aboveLayer >= 0) {
        auto it = _tobstacles.find(aboveLayer);
        if (it != _tobstacles.end()) {
          for (auto& o : it->second) {
            if (o.contains(Geom::Point(n->x(), n->y()))) return false;
          }
        }
      } else {
        return false;
      }
    } else {
      return false;
    }
  } else {
    if (n->z() > _minLayer) {
      auto belowLayer = _belowViaLayer[n->z()];
      if (belowLayer >= 0) {
        auto it = _tobstacles.find(belowLayer);
        if (it != _tobstacles.end()) {
          for (auto& o : it->second) {
            if (o.contains(Geom::Point(n->x(), n->y()))) return false;
          }
        }
      } else {
        return false;
      }
    } else {
      return false;
    }
  }
  return true;
}

}
