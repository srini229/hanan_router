#include <iostream>
#include <fstream>
#include <queue>

#include "Geom.h"
#include "Layer.h"
#include "Placement.h"

typedef long CostType;

std::string parseArgs(const int argc, char* const argv[], const std::string& arg, std::string str = "")
{
  for (int i = 0; i < argc; ++i) {
    if (std::string(argv[i]) == arg && i != (argc - 1)) {
      str = argv[i+1];
      break;
    }
  }
  return str;
}

class Node;

class CostFn {
  private:
    int _topLayer;
    std::vector<CostType> _layerHCost, _layerVCost;
    std::vector<std::vector<CostType>> _layerPairCost;
  public:
    CostType deltaCost(const Node& n1, const Node& n2) const;
    CostFn(const int numLayers, const int minHLayer = 0, const int minVLayer = 1) : _topLayer(numLayers - 1), _layerHCost(numLayers, 10000), _layerVCost(numLayers, 10000),
    _layerPairCost(numLayers, std::vector<CostType>(numLayers, 10000))
    {
      for (int i = minHLayer; i < numLayers; i += 2) {
        _layerHCost[i] = 1;
      }
      for (int i = minVLayer; i < numLayers; i += 2) {
        _layerVCost[i] = 1;
      }
      for (int i = 0; i < numLayers; ++i) {
        if (i > 0) _layerPairCost[i][i-1] = 2;
        if (i < numLayers-1) _layerPairCost[i][i+1] = 2;
      }
    }
};

class HananRouterDB;

class Node {
  private:
    friend class HananRouterDB;
    int _x, _y, _z;
    CostType _fcost, _tcost;
    Node const* _parent;
    Node(const int x = 0, const int y = 0, const int z = -1,
        const CostType fcost = -1, const CostType tcost = -1, Node const* parent = nullptr)
      : _x(x), _y(y), _z(z), _fcost(fcost), _tcost(tcost), _parent(parent) {}
  public:
    int x() const { return _x; }
    int y() const { return _y; }
    int z() const { return _z; }

    CostType fcost() const { return _fcost; }
    CostType tcost() const { return _tcost; }
    CostType cost()  const { return _fcost + _tcost;  }
    void setCost(CostType fcost, CostType tcost) { _fcost = fcost; _tcost = tcost; }
    void setTCost(CostType tcost) { _tcost = tcost; }
    void evalFCost(const CostFn& c)
    {
      if (_parent != nullptr) {
        _fcost = _parent->_fcost + c.deltaCost(*_parent, *this);
      }
    }
    void print(const std::string& s) const
    {
      std::cout << s << ' ' << _x << ' ' << _y << ' ' << _z << '\n';
    }
};
typedef std::vector<Node*> NodePtrVec;
typedef std::vector<Node> NodeVec;

struct NodeSetComp {
  bool operator () (const Node* n1, const Node* n2) const
  {
    if (n1->x() == n2->x()) {
      if (n1->y() == n2->y()) {
        return n1->z() < n2->z();
      }
      return n1->y() < n2->y();
    }
    return n1->x() < n2->x();
  }
};

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


struct NodeComp {
  bool operator() (const Node* n1, const Node* n2) const
  {
    if (n1 != nullptr && n2 != nullptr) return n1->cost() < n2->cost();
    if (n1 == nullptr) return true;
    return false;
  }
};

class HananRouterDB {
  private:
    std::priority_queue<Node*, NodePtrVec, NodeComp> _pq;
    std::set<Node*, NodeSetComp> _sources, _targets;
    NodePtrVec _nodes;
    std::vector<std::pair<Geom::Rect, int>> _obstacles;

    Node* createNode(const int x = 0, const int y = 0, const int z = 0,
        const int fcost = -1, const int tcost = -1, const Node* parent = nullptr)
    {
      auto n = new Node(x, y, z, fcost, tcost, parent);
      _nodes.push_back(n);
      return n;
    }
    
  public:
    HananRouterDB() 
    {
      _nodes.reserve(1e4);
      _obstacles.reserve(1e4);
    }
    ~HananRouterDB()
    {
      for (auto& n : _nodes) delete n;
      _nodes.clear();
      while (!_pq.empty()) {
        _pq.pop();
      }
      _sources.clear();
      _targets.clear();
    }
    void readDataFile(const std::string& ifile);

    void printSol() const;
};

void HananRouterDB::readDataFile(const std::string& ifile)
{
  std::ifstream ifs(ifile);
  std::string tmps;
  while (ifs) {
    ifs >> tmps;
    int x, y, z;
    int w, h;
    if (tmps.empty()) continue;
    switch (tmps[0]) {
      case 'S':
        ifs >> x >> y >> z;
        _sources.insert(createNode(x, y, z, 0));
        break;
      case 'T':
        ifs >> x >> y >> z;
        _targets.insert(createNode(x, y, z, -1, 0));
        break;
      case 'O':
        ifs >> x >> y >> w >> h >> z;
        _obstacles.push_back(std::make_pair(Geom::Rect(x, y, x + w, y + h), z));
        break;
      default:
        break;
    };
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
    std::cout << "obs : " << obs.first.str() << ' ' << obs.second << '\n';
  }
}

int main(int argc, char* argv[])
{
  Geom::Point pt(10, 10);

  std::string layerJSONFile = parseArgs(argc, argv, "-d");
  DRC::LayerInfo linfo(layerJSONFile);
  int uu{1000};
  try {
    uu = std::stoi(parseArgs(argc, argv, "-uu"));
  } catch (const std::invalid_argument& ia) {}

  std::string plfile = parseArgs(argc, argv, "-p");
  std::string leffile = parseArgs(argc, argv, "-l");
  Placement::Netlist netlist(plfile, leffile, linfo, uu);
  netlist.print();

  std::string stfile = parseArgs(argc, argv, "-s");
  HananRouterDB hrdb;
  if (!stfile.empty()) hrdb.readDataFile(stfile);
  hrdb.printSol();

  return 0;
}
