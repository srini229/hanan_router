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
  _width.reserve(layers.size());
  for (unsigned i = 0; i < layers.size(); ++i) {
    if (layers[i]->type()) {
      _width.push_back(static_cast<DRC::MetalLayer*>(layers[i])->width());
      COUT << "layer : " << i << " width : " << _width.back() << '\n';
    }
  }
  _maxRoutingLayer = static_cast<int>(_width.size()) - 1;
}

void Router::readDataFile(const std::string& ifile)
{
  setName(ifile);
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
  _minLayer = zmin;
  _maxLayer = zmax;
  COUT << "min layer : " << _minLayer << " max layer : " << _maxLayer << '\n';
}

void Router::insert(const Node* n)
{
  n->print("adding to pq :");
  if (n->parent()) n->parent()->print("\tparent:");
  _pq.insert(n);
}

void Router::checkAndInsert(Node* newn, const Node* n)
{
  newn->print("newn bef :");
  if (newn->parent()) {
    newn->parent()->print("newn parent bef :");
  }
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
  newn->print("newn aft :");
  if (newn->parent()) {
    newn->parent()->print("newn parent aft :");
  }
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
  n->print("expanding node :");
  if (n->z() <= _minLayer || (n->parent() && n->parent()->z() < n->z())) {
    expanddir.set(0, false);
  }
  if (n->z() >= _maxLayer || (n->parent() && n->parent()->z() > n->z())) {
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
    COUT << "expanding via down\n";
    newn = createNode(n->x(), n->y(), n->z() - 1, n);
    checkAndInsert(newn, n);
  }
  if (expanddir.test(1)) {
    COUT << "expanding via up\n";
    newn = createNode(n->x(), n->y(), n->z() + 1, n);
    checkAndInsert(newn, n);
  }
  std::set<int> gridpos;
  if (expanddir.test(2)) {
    COUT << "expanding left/down\n";
    int snapc = snap(n, vert, false);
    COUT << "snapcd : " << snapc << '\n';
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
      COUT << "grid pos : " << pos << '\n';
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
    COUT << "expanding top/right\n";
    Node* newn{nullptr};
    int snapc = snap(n, vert, true);
    COUT << "snapcu : " << snapc << '\n';
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
      COUT << "grid pos : " << pos << '\n';
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
    sout.insert(std::make_pair(start, r.first));
    start = r.second;
  }
  sout.insert(std::make_pair(start, end));
  s = sout;
}

void Router::generateHananGrid()
{
  std::set<int> xcoords, ycoords;
  for (auto tmp : {false, true}) {
    for (auto& l : (tmp ? _tobstacles : _obstacles)) {
      for (auto& o : l.second) {
        xcoords.insert(o.xmin());
        xcoords.insert(o.xmax());
        ycoords.insert(o.ymin());
        ycoords.insert(o.ymax());
      }
    }
  }
  for (bool src : {true, false}) {
    for (auto& s : (src ? _sources : _targets)) {
      xcoords.insert(s->x());
      xcoords.insert(s->y());
    }
  }
  for (auto tmp : {false, true}) {
    for (auto& l : (tmp ? _tobstacles : _obstacles)) {
      bool vert = isVert(l.first);
      std::map<int, IntRangeSet> tmpranges;
      for (auto& x : (vert ? xcoords : ycoords)) {
        tmpranges[x].clear();
      }

      for (auto& v : tmpranges) {
        for (auto& o : l.second) {
          if (vert) {
            if (v.first > o.xmin() && v.first < o.xmax()) {
              insertRange(tmpranges[v.first], std::make_pair(o.ymin(), o.ymax()));
            }
          } else {
            if (v.first > o.ymin() && v.first < o.ymax()) {
              insertRange(tmpranges[v.first], std::make_pair(o.xmin(), o.xmax()));
            }
          }
        }
      }
      /*for (auto& o : l.second) {
        if (vert) {
        insertRange(tmpranges[o.xmin()], std::make_pair(o.ymin(), o.ymax()));
        insertRange(tmpranges[o.xmax()], std::make_pair(o.ymin(), o.ymax()));
        } else {
        insertRange(tmpranges[o.ymin()], std::make_pair(o.xmin(), o.xmax()));
        insertRange(tmpranges[o.ymax()], std::make_pair(o.xmin(), o.xmax()));
        }
        }*/
      for (auto& r : tmpranges) {
        invertRange(r.second, vert);
      }
      _hanangrid.emplace(l.first, tmpranges);
    }
  }
}

void Router::findSol()
{
  for (auto& s : _sources) {
    evalTCost(s);
    _bbox.merge(s->x(), s->y(), s->x(), s->y());
  }
  for (auto& t : _targets) {
    _bbox.merge(t->x(), t->y(), t->x(), t->y());
  }
  for (auto tmp : {false, true}) {
    for (auto& l : (tmp ? _tobstacles : _obstacles)) {
      for (auto& o : l.second) {
        _bbox.merge(o);
      }
    }
  }

  for (auto& s : _sources) {
    insert(s);
  }

  generateHananGrid();

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

  while (!_pq.empty()) {
    auto t = const_cast<Node*>(*_pq.begin());
    if (_targets.find(t) != _targets.end()) {
      _sol = t;
      COUT << "sol found : " << _sol->x() << ',' << _sol->y() << ',' << _sol->z() << std::endl;
      break;
    }
    _pq.erase(_pq.begin());
    expand(t);
    ++_expansions;
    if (_expansions >= _maxExpansions) break;
  }
}

void Router::printSol() const
{
  for (auto& s : _sources) {
    s->print("source : ");
  }
  for (auto& t : _targets) {
    t->print("targets : ");
  }
  for (auto tmp : {false, true}) {
    for (auto& l : (tmp ? _tobstacles : _obstacles)) {
      for (auto& o : l.second) {
        COUT << "obs : " << o.str() << ' ' << l.first << '\n';
      }
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
  COUT << "plotting route to " << _name << "_route.gplt\n";
  if (ofs.is_open()) {
    COUT << "plotting route to " << _name << "_route.gplt\n";
    ofs << "unset key\n";
    unsigned cnt{1};
    for (auto tmp : {false, true}) {
      for (auto& l : (tmp ? _tobstacles : _obstacles)) {
        const auto& color = LAYER_COLORS[l.first % LAYER_COLORS.size()];
        for (auto& b : l.second) {
          if (b.valid() && b.width() && b.height()) {
            ofs << "set object " << cnt++ << " rect from ";
            ofs << b.xmin() << "," << b.ymin() << " to " << b.xmax() << "," << b.ymax() << " fillstyle transparent solid 0.5 fillcolor \"" << color << "\" behind\n";
          }
        }
      }
    }
    ofs << "plot[:][:] '-' using 1:2 w filledcurves lt -1 lw 2 lc 'red', '-' using 1:2 w filledcurves lt -1 lw 2 lc 'blue', '-' using 1:2 w l lt -1 lw 3 lc 6\n";
    for (auto& s : _sources) {
      Geom::Rect b(s->x() - 2, s->y() - 2, s->x() + 2, s->y() + 2);
      ofs << b.xmin() << " " << b.ymin() << "\n";
      ofs << b.xmax() << " " << b.ymin() << "\n";
      ofs << b.xmax() << " " << b.ymax() << "\n";
      ofs << b.xmin() << " " << b.ymax() << "\n";
      ofs << b.xmin() << " " << b.ymin() << "\n\n";
    }
    ofs << "EOF\n";
    for (auto& t : _targets) {
      Geom::Rect b(t->x() - 2, t->y() - 2, t->x() + 2, t->y() + 2);
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

void Router::addObstacles(const Geom::LayerRects& lr, const bool temp)
{
  auto& obs(temp ? _tobstacles : _obstacles);
  for (auto& l : lr) {
    const auto& layer = l.first;
    for (auto& r : l.second) {
      int hw = (layer < static_cast<int>(_width.size())) ? 
        ((_width[layer] % 2 == 0) ? _width[layer]/2 : (_width[layer]/2 + 1)) : 0;
      obs[layer].push_back(r.bloatby(hw));
    }
  }
}

}
