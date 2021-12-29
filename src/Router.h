#ifndef ROUTER_H_
#define ROUTER_H_
#include "Util.h"
#include <set>
#include <map>
#include <queue>
#include "Geom.h"

namespace Router {

typedef long CostType;
class Node;

class CostFn {
  private:
    int _topLayer;
    std::vector<CostType> _layerHCost, _layerVCost;
    std::vector<std::vector<CostType>> _layerPairCost;
  public:
    CostType deltaCost(const Node& n1, const Node& n2) const;
    CostFn(const int numLayers = 0, const int minHLayer = 0, const int minVLayer = 1) : _topLayer(numLayers - 1), _layerHCost(numLayers, 10000), _layerVCost(numLayers, 10000),
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
      } else {
        _fcost = 0;
      }
    }
    CostType evalTCost(const Node* t, const CostFn& c) const
    {
      if (t) {
        return c.deltaCost(*this, *t);
      }
      return 0;
    }
    void print(const std::string& s) const
    {
      COUT << s << ' ' << _x << ' ' << _y << ' ' << _z << '\n';
    }
};
typedef std::vector<Node*> NodePtrVec;
typedef std::vector<const Node*> NodeCPtrVec;
typedef std::vector<Node> NodeVec;
struct NodeCostComp {
  bool operator() (const Node* n1, const Node* n2) const
  {
    if (n1 != nullptr && n2 != nullptr) return n1->cost() < n2->cost();
    if (n1 == nullptr) return true;
    return false;
  }
};

struct NodeComp {
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
typedef std::set<Node*, NodeComp> NodeSet;
typedef std::set<const Node*, NodeCostComp> PriorityQueue;
class HananRouterDB {
  private:
    PriorityQueue _pq;
    NodeSet _sources, _targets;
    NodePtrVec _nodes;
    Geom::LayerRects _obstacles, _tobstacles;
    CostFn _cf;
    std::map<int, std::map<int, std::vector<std::pair<int, int>>>> _hgrid;

    Node* createNode(const int x = 0, const int y = 0, const int z = 0,
        const int fcost = -1, const int tcost = -1, const Node* parent = nullptr)
    {
      auto n = new Node(x, y, z, fcost, tcost, parent);
      _nodes.push_back(n);
      return n;
    }

    CostType evalTCost(Node* n) const
    {
      CostType tcost = 1e10;
      for (auto& t : _targets) {
        tcost = std::min(tcost, n->evalTCost(t, _cf));
      }
      return tcost;
    }

    void expand(const Node* n);
    void generateHananGrid();
    
  public:
    HananRouterDB() : _cf{}
    {
      _nodes.reserve(1e4);
    }
    ~HananRouterDB()
    {
      for (auto& n : _nodes) delete n;
      _nodes.clear();
      _pq.clear();
      _sources.clear();
      _targets.clear();
    }
    void readDataFile(const std::string& ifile);

    const Node* findSol();
    void printSol() const;
};

}
#endif
