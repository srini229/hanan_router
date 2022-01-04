#ifndef ROUTER_H_
#define ROUTER_H_
#include <set>
#include <map>
#include <queue>
#include <bitset>
#include <limits>

#include "Util.h"
#include "Geom.h"
#include "Layer.h"

namespace Router {

typedef long CostType;
const auto CostMax = std::numeric_limits<CostType>::max();
class Node;

class CostFn {
  private:
    int _topRoutingLayer;
    std::vector<CostType> _layerHCost, _layerVCost;
    std::vector<std::vector<CostType>> _layerPairCost;
  public:
    CostType deltaCost(const Node& n1, const Node& n2) const;
    CostFn(const DRC::LayerInfo& lf);

    CostFn(const int numLayers = 0, const int minHLayer = 1, const int minVLayer = 0): _topRoutingLayer(numLayers - 1), _layerHCost(numLayers, 10000), _layerVCost(numLayers, 10000),
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
      //for (int i = 0; i < numLayers; ++i) {
      //  COUT << "layer : " << i << " cost : " << _layerHCost[i] << ' ' << _layerVCost[i] << '\n';
      //}
    }
};

class Router;

class Node {
  private:
    friend class Router;
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
typedef std::vector<std::map<std::pair<int, int>, Node*>> NodeMap;
class Router {
  private:
    PriorityQueue _pq;
    NodeSet _sources, _targets;
    NodeMap _nodes;
    Geom::LayerRects _obstacles, _tobstacles;
    CostFn _cf;
    std::vector<std::map<int, IntRangeSet>> _hanangrid;
    Geom::Rect _bbox;
    const Node *_sol;
    std::vector<int> _widthx, _spacex;
    std::vector<int> _widthy, _spacey;
    int _minLayer, _maxLayer, _maxRoutingLayer;
    std::string _name;
    size_t _expansions{0};
    const size_t _maxExpansions{100000};
    std::vector<int> _aboveViaLayer, _belowViaLayer;
    const DRC::LayerInfo& _lf;

    Node* createNode(const int x = 0, const int y = 0, const int z = 0,
        const Node* parent = nullptr, const int fcost = -1, const int tcost = -1);

    void evalFCost(Node* n)
    {
      CostType fcost{0};
      if (n->parent()) {
        fcost = _cf.deltaCost(*n, *(n->parent())) + n->parent()->fcost();
      }
      n->setFCost(fcost);
    }

    void evalTCost(Node* n)
    {
      CostType tcost = CostMax;
      for (auto& t : _targets) {
        tcost = std::min(tcost, _cf.deltaCost(*n, *t));
      }
      n->setTCost(tcost);
    }

    void evalCost(Node* n) { evalFCost(n); evalTCost(n); }

    void insert(const Node* n);

    void invertRange(IntRangeSet& s, const bool vert);
    void insertRange(IntRangeSet& s, const IntRange& r);
    void expand(const Node* n);
    void generateHananGrid();
    bool isVert(const int l) const { return _lf.isVertical(l); }
    bool isHor(const int l) const { return _lf.isHorizontal(l); }
    void checkAndInsert(Node* newn, const Node* n);
    int snap(const Node* n, const bool vert, const bool up) const;
    void getAdjacentGrid(std::set<int>& s, const Node* n, const bool above, const bool up, const int snapc);
    void flushNodes()
    {
      COUT << "flushing nodes\n";
      _pq.clear();
      for (auto& l : _nodes) {
        for (auto& n : l) delete n.second;
      }
      _nodes.clear();
      _expansions = 0;
      _bbox = Geom::Rect();
    }
    Geom::PointSet findValidPoints(const Geom::Rect& r, const int z) const;
    
  public:
    Router(const DRC::LayerInfo& lf);
    ~Router()
    {
      flushNodes();
      _sources.clear();
      _targets.clear();
    }
    void readDataFile(const std::string& ifile);
    const int maxRoutingLayer() const { return _maxRoutingLayer; }
    const int maxLayer() const { return _maxLayer; }
    const int minLayer() const { return _minLayer; }
    void setName(const std::string& n) { _name = n; }

    void clearSourceTargets() {
      _sources.clear();
      _targets.clear();
      _sol = nullptr;
      flushNodes();
    }
    Geom::LayerRects findSol();
    void printSol() const;
    void plot() const;
    void writeSTO() const;
    void clearObstacles(bool temp = false)
    {
      if(temp) _tobstacles.clear();
      else _obstacles.clear();
    }
    void addObstacles(const Geom::LayerRects& lr, const bool temp = false);

    void addSource(const Geom::Rect& r, const int z);
    void addTarget(const Geom::Rect& r, const int z);
    bool isViaValid(const Node* n, const bool up) const;
};

}
#endif
