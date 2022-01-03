#include "Router.h"

namespace Router {


CostFn::CostFn(const DRC::LayerInfo& lf)
{
  auto& layers = lf.layers();
  for (unsigned i = 0; i < layers.size(); ++i) {
    if (layers[i]->type()) {
      auto r = std::max(1, static_cast<int>(layers[i]->meanR()));
      if (static_cast<DRC::MetalLayer*>(layers[i])->isHorizontal()) {
        _layerHCost.push_back(r);
        _layerVCost.push_back(10000 * r);
      } else {
        _layerHCost.push_back(10000 * r);
        _layerVCost.push_back(r);
      }
      _topRoutingLayer = static_cast<int>(i);
    }
  }
  _layerPairCost.resize(_layerHCost.size(), std::vector<CostType>(_layerHCost.size(), 100000));
  for (unsigned i = 0; i < layers.size(); ++i) {
    if (!layers[i]->type()) {
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
    if (n1.x() == n2.x() && _layerVCost[n1.z()] < 10) {
      return (_layerVCost[n1.z()] * std::abs(n1.y() - n2.y()));
    }
    if (n1.y() == n2.y() && _layerHCost[n1.z()] < 10) {
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
  if (minz == maxz) {
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


Router::Router(const DRC::LayerInfo& lf) : _cf{lf}, _sol{nullptr}, _minLayer{100}, _maxLayer{0}, _maxRoutingLayer{0}, _name{}
{
  auto& layers = lf.layers();
  _widthx.reserve(layers.size());
  _widthy.reserve(layers.size());
  _spacex.reserve(layers.size());
  _spacey.reserve(layers.size());
  for (unsigned i = 0; i < layers.size(); ++i) {
    if (layers[i]->type()) {
      auto mlayer = static_cast<DRC::MetalLayer*>(layers[i]);
      _widthx.push_back(mlayer->width());
      _widthy.push_back(mlayer->width());
      _spacex.push_back(mlayer->space());
      _spacey.push_back(mlayer->space());
      COUT << "layer : " << i << " width : " << _widthx.back() << " space : " << _spacex.back() << '\n';
    }
  }
  _aboveViaLayer.resize(_widthx.size(), -1);
  _belowViaLayer.resize(_widthx.size(), -1);
  for (unsigned i = 0; i < layers.size(); ++i) {
    if (!layers[i]->type()) {
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
}

Node* Router::createNode(const int x, const int y, const int z,
    const Node* parent, const int fcost, const int tcost)
{
  auto tpl = std::make_tuple(x, y, z);
  auto it = _nodes.find(tpl);
  Node* n = nullptr;
  if (it == _nodes.end()) {
    n = new Node(x, y, z, fcost, tcost, parent);
    _nodes[tpl] = n;
#if DEBUG
    COUT << "creating new node : " << x << ',' << y << ',' << z << '\n';
#endif
  } else {
    n = it->second;
  }
  return n;
}

Geom::PointSet Router::findValidPoints(const Geom::Rect& r, const int z) const
{
  auto vert = isVert(z);
  auto prev = z, next = z;
  if (z < _maxLayer) {
    prev = z - 1;
  }
  if (z > _minLayer) {
    next = z + 1;
  }

  auto x = r.xcenter(), y = r.ycenter(); 
  Geom::PointSet points;
  points.insert(Geom::Point(x, y));

  if ((vert && r.isVert()) || (!vert && r.isHor())) {
    for (auto pr : {true, false}) {
      if ((pr && prev == z) || (!pr && next == z)) continue;
      auto layer = (pr ? prev : next);
      if (vert) {
        int space = _spacey[layer] + ((_widthy[layer] % 2 == 0) ? _widthy[layer]/2 : (_widthy[layer]/2 + 1));
        int yn = r.ymax() - ((r.ymax() - y) % space);
        while (yn > r.ymin()) {
          points.insert(Geom::Point(x,yn));
          yn -= space;
        }
      } else {
        int space = _spacex[layer] + ((_widthx[layer] % 2 == 0) ? _widthx[layer]/2 : (_widthx[layer]/2 + 1));
        int xn = r.xmax() - ((r.xmax() - x) % space);
        while (xn > r.xmin()) {
          points.insert(Geom::Point(xn,y));
          xn -= space;
        }
      }
    }
  }
  return points;
}

void Router::addSource(const Geom::Rect& r, const int z)
{
  auto points = findValidPoints(r, z);
  for (auto& p : points) {
    _sources.insert(createNode(p.x(), p.y(), z, nullptr, 0));
  }
}

void Router::addTarget(const Geom::Rect& r, const int z)
{
  auto points = findValidPoints(r, z);
  for (auto& p : points) {
    _targets.insert(createNode(p.x(), p.y(), z, nullptr, -1, 0));
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

void Router::insert(const Node* n)
{
#if DEBUG
  n->print("adding to pq :");
  if (n->parent()) n->parent()->print("\tparent:");
#endif
  _pq.insert(n);
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
    insert(newn);
  } else if (newn->parent() != n) {
    auto oldfcost = newn->fcost();
    auto oldparent = newn->parent();
    auto it = _pq.find(newn);
    newn->setParent(n);
    evalFCost(newn);
    if (newn->fcost() > oldfcost) {
      newn->setParent(oldparent);
      newn->setFCost(oldfcost);
    } else if (it != _pq.end()) {
      _pq.erase(it);
      insert(newn);
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
  auto it = _hanangrid.find(n->z());
  if (it != _hanangrid.end()) {
    int pos = n->y(), lkp = n->x();
    if (vert) {
      std::swap(pos, lkp);
    }
    auto itp = it->second.find(pos);
    if (itp != it->second.end()) {
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
    auto ith = _hanangrid.find(adjLayer);
    if (ith != _hanangrid.end()) {
      auto coord = (isVert(n->z()) ? n->y() : n->x());
      for (auto& pos : ith->second) {
        if ((up && pos.first > coord && pos.first < snapc) || (!up && pos.first < coord && pos.first > snapc)) {
          s.insert(pos.first);
        }
      }
    }
  }
}

void Router::expand(const Node* n)
{
  std::bitset<4> expanddir{0xF}; // 0:dn, 1:up, 2:left/down, 3:right/up
#if DEBUG
  n->print("expanding node :");
#endif
  if ((n->parent() && n->parent()->z() < n->z()) || !isViaValid(n, false)) {
    expanddir.set(0, false);
  }
  if ((n->parent() && n->parent()->z() > n->z()) || !isViaValid(n, true)) {
    expanddir.set(1, false);
  }
  const bool vert = isVert(n->z());
  if (vert) {
    if (n->y() < _bbox.ymin() || (n->parent() && n->parent()->z() == n->z() && n->parent()->y() <= n->y())) {
      expanddir.set(2, false);
    }
    if (n->y() > _bbox.ymax() || (n->parent() && n->parent()->z() == n->z() && n->parent()->y() >= n->y())) {
      expanddir.set(3, false);
    }
  } else {
    if (n->x() < _bbox.xmin()|| (n->parent() && n->parent()->z() == n->z() && n->parent()->x() <= n->x())) {
      expanddir.set(2, false);
    }
    if (n->x() > _bbox.xmax()|| (n->parent() && n->parent()->z() == n->z() && n->parent()->x() <= n->x())) {
      expanddir.set(3, false);
    }
  }


  Node* newn{nullptr};
  if (expanddir.test(0)) {
#if DEBUG
    COUT << "expanding via down\n";
#endif
    newn = createNode(n->x(), n->y(), n->z() - 1, n);
    checkAndInsert(newn, n);
  }
  if (expanddir.test(1)) {
#if DEBUG
    COUT << "expanding via up\n";
#endif
    newn = createNode(n->x(), n->y(), n->z() + 1, n);
    checkAndInsert(newn, n);
  }
  std::set<int> gridpos;
  if (expanddir.test(2)) {
#if DEBUG
    COUT << "expanding left/down\n";
#endif
    int snapc = snap(n, vert, false);
#if DEBUG
    COUT << "snapcd : " << snapc << '\n';
#endif
    newn = nullptr;
    if (vert) {
      if (snapc < n->y()) {
        newn = createNode(n->x(), snapc, n->z(), n);
      }
    } else {
      if (snapc < n->x()) {
        newn = createNode(snapc, n->y(), n->z(), n);
      }
    }
    if (newn) checkAndInsert(newn, n);
    getAdjacentGrid(gridpos, n, true, false, snapc);
    getAdjacentGrid(gridpos, n, false, false, snapc);
    for (auto &pos : gridpos) {
#if DEBUG
      COUT << "grid pos : " << pos << '\n';
#endif
      if (vert) {
        if (pos < n->y()) {
          newn = createNode(n->x(), pos, n->z(), n);
        }
      } else {
        if (pos < n->x()) {
          newn = createNode(pos, n->y(), n->z(), n);
        }
      }
      if (newn) checkAndInsert(newn, n);
    }
    gridpos.clear();
  }
  if (expanddir.test(3)) {
#if DEBUG
    COUT << "expanding top/right\n";
#endif
    Node* newn{nullptr};
    int snapc = snap(n, vert, true);
#if DEBUG
    COUT << "snapcu : " << snapc << '\n';
#endif
    newn = nullptr;
    if (vert) {
      if (snapc > n->y()) {
        newn = createNode(n->x(), snapc, n->z(), n);
      }
    } else {
      if (snapc > n->x()) {
        newn = createNode(snapc, n->y(), n->z(), n);
      }
    }
    if (newn) checkAndInsert(newn, n);
    getAdjacentGrid(gridpos, n, true, true, snapc);
    getAdjacentGrid(gridpos, n, false, true, snapc);
    for (auto &pos : gridpos) {
#if DEBUG
      COUT << "grid pos : " << pos << '\n';
#endif
      if (vert) {
        if (pos > n->y()) {
          newn = createNode(n->x(), pos, n->z(), n);
        }
      } else {
        if (pos > n->x()) {
          newn = createNode(pos, n->y(), n->z(), n);
        }
      }
      if (newn) checkAndInsert(newn, n);
    }
  }
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
  _hanangrid.clear();
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
    bool vert = isVert(l.first);
    std::map<int, IntRangeSet> tmpranges;
    for (auto& x : (vert ? xcoords : ycoords)) {
      if ((vert && x >= _bbox.xmin() && x <= _bbox.xmax()) || 
          (!vert && x >= _bbox.ymin() && x <= _bbox.ymax())) {
        tmpranges[x].clear();
      }
    }

    for (auto& v : tmpranges) {
      for (auto& o : l.second) {
        if (vert) {
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
      invertRange(r.second, vert);
    }
    _hanangrid.emplace(l.first, tmpranges);
  }
}

Geom::LayerRects Router::findSol()
{
#if DEBUG
  SaveRestoreStream src(_name + "_route.log");
#endif
  static std::string debugplot{getenv("HANAN_DEBUG_WIRE") ? getenv("HANAN_DEBUG_WIRE") : ""};
  if (!debugplot.empty() && (debugplot == "1" || debugplot == _name)) writeSTO();
  Geom::LayerRects sol;
  if (!_sources.empty() && !_targets.empty()) {
    _bbox = Geom::Rect();
    for (auto& s : _sources) {
      evalTCost(s);
      insert(s);
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
    /*for (auto& l : _tobstacles) {
      COUT << "layer after : " << l.first << '\n';
      for (auto& o : l.second) {
        COUT << "o : " << o.str() << '\n';
      }
    }*/
    generateHananGrid();

#if DEBUG
    for (auto& l : _hanangrid) {
      COUT << "layer : " << l.first << '\n';
      for (auto& pos : l.second) {
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
#if DEBUG
        printSol();
#endif
        break;
      }
      _pq.erase(_pq.begin());
      expand(t);
      ++_expansions;
      if (_expansions >= _maxExpansions) break;
    }
    if (!_sol) {
      COUT << "sol not found for " << _name << '\n';
    }
    _pq.clear();
    _hanangrid.clear();
    _expansions = 0;
    if (_sol) {
      const Node* n = _sol;
      while (n) {
        auto parent = n->parent();
        if (parent) {
          if (parent->z() == n->z()) {
            sol[n->z()].push_back(Geom::Rect(n->x(), n->y(), parent->x(), parent->y()).bloatby(_widthx[n->z()], _widthy[n->z()]));
#if DEBUG
            COUT << n->z() << ' ' << sol[n->z()].back().str() << '\n';
#endif
          } else {
            auto adjLayer = (parent->z() < n->z()) ? _belowViaLayer[n->z()] : _aboveViaLayer[n->z()];
            if (adjLayer >= 0) {
              sol[adjLayer].push_back(Geom::Rect(n->x(), n->y(), n->x(), n->y()).bloatby(_widthx[adjLayer], _widthy[adjLayer]));
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
  if (!debugplot.empty() && (debugplot == "1" || debugplot == _name)) plot();
  return sol;
}

void Router::printSol() const
{
#if DEBUG
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
#endif
}


void Router::plot() const
{
  std::ofstream ofs(_name + "_route.gplt");
  COUT << "plotting route to " << _name << "_route.gplt\n";
  if (ofs.is_open()) {
    COUT << "plotting route to " << _name << "_route.gplt\n";
    ofs << "unset key\n";
    unsigned cnt{1};
    for (auto& l : _tobstacles) {
      const auto& color = LAYER_COLORS[l.first % LAYER_COLORS.size()];
      for (auto& b : l.second) {
        if (b.valid() && b.width() && b.height()) {
          ofs << "set object " << cnt++ << " rect from ";
          ofs << b.xmin() << "," << b.ymin() << " to " << b.xmax() << "," << b.ymax() << " fillstyle transparent solid 0.5 fillcolor \"" << color << "\" behind\n";
        }
      }
    }
    ofs << "plot[:][:] '-' using 1:2 w filledcurves lt -1 lw 2 lc 'red', '-' using 1:2 w filledcurves lt -1 lw 2 lc 'blue', '-' using 1:2 w l lt -1 lw 3 lc 6\n";
    int dx{std::max(2, _bbox.width()/100)};
    int dy{std::max(2, _bbox.height()/100)};
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
        ofs << "Obstacle " << o.xmin() << ' ' << o.xmax() << ' ' << o.width() << ' ' << o.height() << ' ' << l.first << '\n';
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
        spacex = _spacex[layer] + ((_widthx[layer] % 2 == 0) ? _widthx[layer]/2 : (_widthx[layer]/2 + 1));
        spacey = _spacey[layer] + ((_widthy[layer] % 2 == 0) ? _widthy[layer]/2 : (_widthy[layer]/2 + 1));
      }
      COUT << "layer : " << layer << " obs : " << spacex << ' ' << spacey << ' ' << r.xmin() << ' ' << r.ymin() << ' ' << r.xmax() << ' ' << r.ymax() << '\n';
      if (temp) {
        _tobstacles[layer].push_back(r.bloatby(spacex, spacey));
        //COUT << "tobs : " << _tobstacles[layer].back().str() << '\n';
      } else {
        _obstacles[layer].push_back(r.bloatby(spacex, spacey));
        //COUT << "obs : " << _obstacles[layer].back().str() << '\n';
      }
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
