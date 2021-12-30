#include "Router.h"
#include <bitset>

namespace Router {


CostType CostFn::deltaCost(const Node& n1, const Node& n2) const
{
  CostType dc = 0;

  if (n1.x() == n2.x() && n1.y() == n2.y() && n1.z() == n2.z()) return dc;
  auto minz = std::min(n1.z(), n2.z());
  auto maxz = std::max(n1.z(), n2.z());
  CostType minHCost(20000), minVCost(20000);
  if (minz == maxz) {
    if (minz < _topLayer) maxz = minz + 1;
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


void HananRouterDB::readDataFile(const std::string& ifile)
{
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
  _cf = CostFn(zmax + 1, zmin, zmin + 1);
  _minLayer = zmin;
  _maxLayer = zmax;
  COUT << "min layer : " << _minLayer << " max layer : " << _maxLayer << '\n';
}

void HananRouterDB::insert(const Node* n)
{
  n->print("adding to pq :");
  if (n->parent()) n->parent()->print("\tparent:");
  _pq.insert(n);
}

void HananRouterDB::checkAndInsert(Node* newn, const Node* n)
{
  newn->print("newn bef :");
  if (newn->parent()) {
    newn->parent()->print("newn parent bef :");
  }
  if (newn->cost() < 0) {
    if (newn->parent() != n) newn->setParent(n);
    newn->evalFCost(_cf);
    evalTCost(newn);
    insert(newn);
  } else if (newn->parent() != n) {
    auto oldfcost = newn->fcost();
    auto oldparent = newn->parent();
    auto it = _pq.find(newn);
    newn->setParent(n);
    newn->evalFCost(_cf);
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

int HananRouterDB::snap(const Node* n, const bool vert, const bool up) const
{
  int snapc = (vert ? (up ? (_bbox.ymax()) : (_bbox.ymin()))
      : (up ? (_bbox.xmax()) : (_bbox.xmin())));
  auto it = _hanangrid.find(n->z());
  if (it != _hanangrid.end()) {
    int pos = n->y(), lkp = n->x();
    if (vert) {
      std::swap(pos, lkp);
    }
    auto itp = it->second.find(pos);
    if (itp != it->second.end()) {
      for (const auto& r : itp->second) {
        if (lkp >= r.first && lkp < r.second) {
          snapc = (up ? r.second : r.first);
          break;
        }
      }
    }
  }
  return snapc;
}

void HananRouterDB::expand(const Node* n)
{
  n->print("expanding node :");
  std::bitset<4> expanddir{0xF}; // 0:dn, 1:up, 2:left/down, 3:right/up
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

  COUT << "expanding : " << expanddir.to_string() << '\n';

  Node* newn{nullptr};
  if (expanddir.test(0)) {
    newn = createNode(n->x(), n->y(), n->z() - 1, n);
    checkAndInsert(newn, n);
  }
  if (expanddir.test(1)) {
    newn = createNode(n->x(), n->y(), n->z() + 1, n);
    checkAndInsert(newn, n);
  }
  if (expanddir.test(2)) {
    int snapc = snap(n, vert, false);
    COUT << "snapcd : " << snapc << '\n';
    newn = nullptr;
    if (vert) {
      if (snapc != n->y() && snapc >= _bbox.xmin()) {
        newn = createNode(n->x(), snapc, n->z(), n);
      }
    } else {
      if (snapc != n->x() && snapc >= _bbox.xmax()) {
        newn = createNode(snapc, n->y(), n->z(), n);
      }
    }
    if (newn) checkAndInsert(newn, n);
  }
  if (expanddir.test(3)) {
    Node* newn{nullptr};
    int snapc = snap(n, vert, true);
    COUT << "snapcu : " << snapc << '\n';
    newn = nullptr;
    if (vert) {
      if (snapc != n->y() && snapc >= _bbox.ymin()) {
        newn = createNode(n->x(), snapc, n->z(), n);
      }
    } else {
      if (snapc != n->x() && snapc <= _bbox.ymax()) {
        newn = createNode(snapc, n->y(), n->z(), n);
      }
    }
    if (newn) checkAndInsert(newn, n);
  }
}

void HananRouterDB::insertRange(IntRangeSet& s, const IntRange& r)
{
  auto it = s.begin();
  for (; it != s.end(); ++it) {
    if (it->first > r.second) {
      s.insert(r);
      break;
    } else if (it->first <= r.second && it->second >= r.first) {
      s.erase(it);
      s.insert(std::make_pair(std::min(it->first, r.first), std::max(it->second, r.second)));
      break;
    }
  }
  if (it == s.end()) {
    s.insert(r);
  }
}

void HananRouterDB::invertRange(IntRangeSet& s, const bool vert)
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

void HananRouterDB::generateHananGrid()
{
  for (auto& l : _obstacles) {
    std::map<int, IntRangeSet> tmpranges;
    bool vert = isVert(l.first);
    for (auto& o : l.second) {
      if (vert) {
        insertRange(tmpranges[o.xmin()], std::make_pair(o.ymin(), o.ymax()));
        insertRange(tmpranges[o.xmax()], std::make_pair(o.ymin(), o.ymax()));
      } else {
        insertRange(tmpranges[o.ymin()], std::make_pair(o.xmin(), o.xmax()));
        insertRange(tmpranges[o.ymax()], std::make_pair(o.xmin(), o.xmax()));
      }
    }
    for (auto& r : tmpranges) {
      invertRange(r.second, vert);
    }
    _hanangrid.emplace(l.first, tmpranges);
  }
}

void HananRouterDB::findSol()
{
  for (auto& s : _sources) {
    evalTCost(s);
    _bbox.merge(s->x(), s->y(), s->x(), s->y());
  }
  for (auto& t : _targets) {
    t->evalFCost(_cf);
    t->setTCost(0);
    _bbox.merge(t->x(), t->y(), t->x(), t->y());
  }
  for (auto& l : _obstacles) {
    for (auto& o : l.second) {
      _bbox.merge(o);
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
  }
}

void HananRouterDB::printSol() const
{
  for (auto& s : _sources) {
    s->print("source : ");
  }
  for (auto& t : _targets) {
    t->print("targets : ");
  }
  for (auto& obs : _obstacles) {
    for (auto& o : obs.second) {
      COUT << "obs : " << o.str() << ' ' << obs.first << '\n';
    }
  }
}

void HananRouterDB::plot() const
{
  std::ofstream ofs("route.gplt");
  if (ofs.is_open()) {
    COUT << "plotting route to route.gplt\n";
    ofs << "unset key\n";
    unsigned cnt{1};
    for (auto& l : _obstacles) {
      const auto& color = LAYER_COLORS[l.first % LAYER_COLORS.size()];
      for (auto& b : l.second) {
        if (b.valid() && b.width() && b.height()) {
          ofs << "set object " << cnt++ << " rect from ";
          ofs << b.xmin() << "," << b.ymin() << " to " << b.xmax() << "," << b.ymax() << " fillstyle transparent solid 0.5 fillcolor \"" << color << "\" behind\n";
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

}
