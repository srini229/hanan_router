#ifndef ROUTER_H_
#define ROUTER_H_
#include "Util.h"
#include <set>
#include <map>
#include <queue>
#include "Geom.h"
#include <bitset>

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
    CostFn(const int numLayers = 0, const int minHLayer = 1, const int minVLayer = 0) : _topLayer(numLayers - 1), _layerHCost(numLayers, 10000), _layerVCost(numLayers, 10000),
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
      for (int i = 0; i < numLayers; ++i) {
        COUT << "layer : " << i << " cost : " << _layerHCost[i] << ' ' << _layerVCost[i] << '\n';
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
    Node const* parent() const { return _parent; }

    CostType fcost() const { return _fcost; }
    CostType tcost() const { return _tcost; }
    CostType cost()  const { return _fcost + _tcost;  }
    void setFCost(CostType fcost) { _fcost = fcost; }
    void setTCost(CostType tcost) { _tcost = tcost; }
    void setParent(const Node* n) { _parent = n; }
    void evalFCost(const CostFn& c)
    {
      if (_parent != nullptr) {
        _fcost = _parent->_fcost + c.deltaCost(*_parent, *this);
      } else {
        _fcost = -1;
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
      COUT << s << ' ' << _x << ' ' << _y << ' ' << _z << ' ' << _fcost << ' ' << _tcost <<  ' ' << cost() << '\n';
    }
};
typedef std::vector<Node*> NodePtrVec;
typedef std::vector<const Node*> NodeCPtrVec;
typedef std::vector<Node> NodeVec;
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
struct NodeCostComp {
  bool operator() (const Node* n1, const Node* n2) const
  {
    if (n1 != nullptr && n2 != nullptr) {
      if (n1->cost() == n2->cost()) {
        return NodeComp()(n1, n2);
      }
      return n1->cost() < n2->cost();
    }
    if (n1 == nullptr) return true;
    return false;
  }
};

typedef std::pair<int, int> IntRange;
struct RangeComp {
  bool operator() (const IntRange& p1, const IntRange& p2) const
  {
    if (p1.first == p2.first) return p1.second < p2.second;
    return p1.first < p2.first;
  }
};
typedef std::set<IntRange, RangeComp> IntRangeSet;
typedef std::set<Node*, NodeComp> NodeSet;
typedef std::multiset<const Node*, NodeCostComp> PriorityQueue;
typedef std::map<std::tuple<int, int, int>, Node*> NodeMap;
class HananRouterDB {
  private:
    PriorityQueue _pq;
    NodeSet _sources, _targets;
    NodeMap _nodes;
    Geom::LayerRects _obstacles, _tobstacles;
    CostFn _cf;
    std::map<int, std::map<int, IntRangeSet>> _hanangrid;
    Geom::Rect _bbox;
    const Node *_sol;

    int _minLayer, _maxLayer;

    Node* createNode(const int x = 0, const int y = 0, const int z = 0,
        const Node* parent = nullptr,
        const int fcost = -1, const int tcost = -1)
    {
      auto tpl = std::make_tuple(x, y, z);
      auto it = _nodes.find(tpl);
      Node* n = nullptr;
      if (it == _nodes.end()) {
        n = new Node(x, y, z, fcost, tcost, parent);
        _nodes[tpl] = n;
        COUT << "creating new node : " << x << ',' << y << ',' << z << '\n';
      } else {
        n = it->second;
      }
      return n;
    }

    void evalTCost(Node* n)
    {
      CostType tcost = 1e10;
      for (auto& t : _targets) {
        tcost = std::min(tcost, n->evalTCost(t, _cf));
      }
      n->setTCost(tcost);
    }

    void insert(const Node* n);

    void invertRange(IntRangeSet& s, const bool vert);
    void insertRange(IntRangeSet& s, const IntRange& r);
    void expand(const Node* n);
    void generateHananGrid();
    bool isVert(const int l) const { return (l % 2) == 0; }
    void checkAndInsert(Node* newn, const Node* n);
    int snap(const Node* n, const bool vert, const bool up) const;
    void insertTarget(std::set<int>& s, const Node* n, const bool up, const int snapc);
    void getAdjacentGrid(std::set<int>& s, const Node* n, const bool above, const bool up, const int snapc);
    
  public:
    HananRouterDB() : _cf{}, _sol{nullptr}, _minLayer{100}, _maxLayer{0}
    {
    }
    ~HananRouterDB()
    {
      for (auto& n : _nodes) delete n.second;
      _nodes.clear();
      _pq.clear();
      _sources.clear();
      _targets.clear();
    }
    void readDataFile(const std::string& ifile);

    void findSol();
    void printSol() const;
    void plot() const;
};

}
#endif
