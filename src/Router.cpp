#include "Router.h"

namespace Router {


CostType CostFn::deltaCost(const Node& n1, const Node& n2) const
{
  CostType dc = 0;

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
        _sources.insert(createNode(x, y, z, 0));
        zmax = std::max(zmax, z);
        zmin = std::min(zmin, z);
        break;
      case 'T':
        ifs >> x >> y >> z;
        _targets.insert(createNode(x, y, z, -1, 0));
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
}

void HananRouterDB::expand(const Node* n)
{
  n->print("expanding node :");
}

void HananRouterDB::generateHananGrid()
{
  for (auto& l : _obstacles) {
  }
}

const Node* HananRouterDB::findSol()
{
  for (auto& s : _sources) {
    s->evalFCost(_cf);
    s->setTCost(evalTCost(s));
  }
  for (auto& t : _targets) {
    t->evalFCost(_cf);
    t->setTCost(0);
  }

  for (auto& s : _sources) {
    _pq.insert(s);
  }

  generateHananGrid();

  const Node *sol{nullptr};
  while (!_pq.empty()) {
    auto t = const_cast<Node*>(*_pq.begin());
    if (_targets.find(t) != _targets.end()) {
      sol = t;
      break;
    }
    _pq.erase(_pq.begin());
    expand(t);
  }
  return sol;
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

}
