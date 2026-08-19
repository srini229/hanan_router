#ifndef ROUTER_H_
#define ROUTER_H_
#include <set>
#include <map>
#include <queue>
#include <bitset>
#include <limits>
#include <memory>

#include "Util.h"
#include "Geom.h"
#include "Layer.h"

#include "polygon/polygon.hpp"

namespace bp = boost::polygon;
typedef bp::polygon_90_set_data<int> PolySet;
typedef bp::polygon_90_data<int> PPoly;
typedef bp::polygon_90_with_holes_data<int> PPolyWH;
typedef std::vector<PPoly> PPolys;
typedef std::vector<PPolyWH> PPolyWHs;
typedef bp::rectangle_data<int> PRect;
typedef std::vector<PRect> PRects ;
typedef std::map<int, PolySet> LayerPolySet ;

#define COST_MAX 10000

namespace Router {

typedef double CostType;
const auto CostTypeMax = std::numeric_limits<CostType>::max();
class Node;

class Via {
  private:
    int _l, _u, _c;
    Geom::Point _center;
    Geom::Rect _lb, _ub, _cut, _bbox;
    Geom::Rects _cuts;
  public:
    Via(const int l, const int u, const int c, const Geom::Point& ctr = Geom::Point(0, 0)) : _l{l}, _u{u}, _c{c}, _center{ctr}, _lb{}, _ub{}, _cut{}, _bbox{} {}
    Via(const Via& via, const Geom::Point& p = Geom::Point(0, 0));
    const Geom::Rect& bbox() const { return _bbox; }
    void setLB(const Geom::Rect& r) { _lb = r; _bbox.merge(_lb); }
    void setUB(const Geom::Rect& r) { _ub = r; _bbox.merge(_ub); }
    const Geom::Rects& cuts() const { return _cuts; }
    const Geom::Rect& upad() const { return _ub; }
    const Geom::Rect& lpad() const { return _lb; }
    const int u() const { return _u; }
    const int l() const { return _l; }
    void addCuts(const Geom::Point& o, const int wx, const int wy, const int nrow = 1, const int ncol = 1, const int sx = 0, const int sy = 0);
    std::string str() const;
    void addShapes(Geom::LayerRects& lr) const;
};
typedef std::vector<std::shared_ptr<Via>> Vias;

class CostFn {
  private:
    int _topRoutingLayer;
    std::vector<CostType> _layerHCost, _layerVCost;
    std::vector<CostType> _savedLayerHCost, _savedLayerVCost;
    std::vector<std::vector<CostType>> _layerPairCost;
    std::vector<std::vector<CostType>> _baseLayerPairCost;
    std::vector<CostType> _bendCost;
    static constexpr double BEND_PITCHES = 0.75;
    std::set<int> _preflayers;
    std::map<int, Geom::Rects> _relaxzones;
    std::map<int, std::vector<int>> _relaxuf;   // union-find over overlapping zones
    CostType _minMetalCost{0};
    static int root(const std::vector<int>& uf, int a)
    {
      while (uf[a] != a) a = uf[a];
      return a;
    }
  public:
    CostType deltaCost(const Node& n1, const Node& n2) const;
    CostFn(const DRC::LayerInfo& lf);
    void setRelaxFloor(const CostType c) { if (c > 0 && c < COST_MAX) _minMetalCost = c; }
    void clearRelaxZones() { _relaxzones.clear(); _relaxuf.clear(); }
    void addRelaxZone(const int z, const Geom::Rect& r)
    {
      auto& v = _relaxzones[z];
      auto& uf = _relaxuf[z];
      const int idx = static_cast<int>(v.size());
      v.push_back(r);
      uf.push_back(idx);
      for (int i = 0; i < idx; ++i) {
        if (v[i].overlaps(r)) {
          const int a = root(uf, i), b = root(uf, idx);
          if (a != b) uf[a] = b;
        }
      }
    }
    CostType relaxed(const CostType base, const int z, const int x1, const int y1,
                     const int x2, const int y2) const
    {
      if (base <= _minMetalCost || _relaxzones.empty()) return base;
      auto it = _relaxzones.find(z);
      if (it == _relaxzones.end()) return base;
      const auto& v = it->second;
      const auto& uf = _relaxuf.at(z);
      const Geom::Point p1(x1, y1), p2(x2, y2);
      int g1[8], g2[8];
      int n1 = 0, n2 = 0;
      for (size_t i = 0; i < v.size(); ++i) {
        if (n1 < 8 && v[i].contains(p1, false)) g1[n1++] = root(uf, static_cast<int>(i));
        if (n2 < 8 && v[i].contains(p2, false)) g2[n2++] = root(uf, static_cast<int>(i));
      }
      for (int a = 0; a < n1; ++a)
        for (int b = 0; b < n2; ++b)
          if (g1[a] == g2[b]) return _minMetalCost;
      return base;
    }
    bool isVert(const int l) const { return _layerVCost[l] <= _layerHCost[l]; }
    bool isHor(const int l) const { return _layerHCost[l] <= _layerVCost[l]; }
    int topRoutingLayer() const { return _topRoutingLayer; }

    void addViaPadCost(const int z1, const int z2, const CostType c)
    {
      if (z1 < 0 || z2 < 0 || _baseLayerPairCost.empty()) return;
      if (_baseLayerPairCost[z1][z2] < COST_MAX) _layerPairCost[z1][z2] = _baseLayerPairCost[z1][z2] + c;
      if (_baseLayerPairCost[z2][z1] < COST_MAX) _layerPairCost[z2][z1] = _baseLayerPairCost[z2][z1] + c;
    }
    CostType offCentreEscapeCost(const int dist) const
    {
      return (_minMetalCost * dist) / ESCAPE_OFFCENTRE_DIV;
    }
    static const int ESCAPE_OFFCENTRE_DIV = 4;
    static const int SMALL_PIN_WIDTHS = 10;
    CostType hcost(int i) const { return _layerHCost[i]; }
    CostType vcost(int i) const { return _layerVCost[i]; }
    void updatendr(const std::map<int, DRC::Direction>& ndrdir, const std::set<int>& preflayers);
    void resetdirs() {
      if (!_savedLayerHCost.empty()) _layerHCost = _savedLayerHCost;
      if (!_savedLayerVCost.empty()) _layerVCost = _savedLayerVCost;
      _preflayers.clear();
    }
};

class Router;

enum Direction
{
  DOWN = 0, UP, EAST, WEST, NORTH, SOUTH, MAXDIR
};

class Node {
#if DEBUG
  public:
    static size_t _nodectr;
#endif
  private:
    friend class Router;
    int _x, _y, _z;
    int _hwx, _hwy;
    CostType _fcost, _tcost;
    Node const* _parent;
    const Via *_upVia, *_dnVia;
    std::bitset<MAXDIR> _expanddir;
    Node(const int x = 0, const int y = 0, const int z = -1,
        const CostType fcost = -1, const CostType tcost = -1, Node const* parent = nullptr)
      : _x(x), _y(y), _z(z), _hwx{0}, _hwy{0}, _fcost(fcost), _tcost(tcost),
      _parent(parent), _upVia(nullptr), _dnVia(nullptr)
      {
        _expanddir.reset();
#if DEBUG
        ++_nodectr;
#endif
      }

    ~Node()
    {
      delete _upVia;
      delete _dnVia;
#if DEBUG
      --_nodectr;
#endif
    }
  public:
    bool closed() const { return _expanddir.none(); }
    int x() const { return _x; }
    int y() const { return _y; }
    int z() const { return _z; }
    Node const* parent() const { return _parent; }
    void sethwx(const int hwx) { _hwx = hwx; }
    void sethwy(const int hwy) { _hwy = hwy; }
    int hwx() const { return _hwx; }
    int hwy() const { return _hwy; }

    bool viadown()     const { return _expanddir.test(DOWN); }
    bool viaup()       const { return _expanddir.test(UP); }
    bool expandeast()  const { return _expanddir.test(EAST); }
    bool expandwest()  const { return _expanddir.test(WEST); }
    bool expandnorth() const { return _expanddir.test(NORTH); }
    bool expandsouth() const { return _expanddir.test(SOUTH); }

    void expand(const int dir, const bool val) { if (dir < MAXDIR) _expanddir.set(dir, val); }
    void setexpand() { _expanddir.set(); }
    void resetexpand() { _expanddir.reset(); }

    const Via* upVia() const { return _upVia; }
    const Via* dnVia() const { return _dnVia; }
    void upVia(const Via* v) 
    {
        if (_upVia) {
          delete _upVia;
          _upVia = nullptr;
        }
        _upVia = v;
    }
    void dnVia(const Via* v)
    {
        if (_dnVia) {
          delete _dnVia;
          _dnVia = nullptr;
        }
        _dnVia = v;
    }

    CostType fcost() const { return _fcost; }
    CostType cost()  const { return _fcost + _tcost;  }
    void setFCost(CostType fcost) { _fcost = fcost; }
    void setTCost(CostType tcost) { _tcost = tcost; }
    void setParent(const Node* n) { _parent = n; }
    void print(const std::string& s) const
    {
      COUT << s << "(" << _x << ',' << _y << ',' << _z << ") cost : (" << _fcost << ',' << _tcost <<  ',' << cost() << ") ";
      if (_expanddir.test(NORTH)) COUT << " N";
      if (_expanddir.test(SOUTH)) COUT << " S";
      if (_expanddir.test(EAST))  COUT << " E";
      if (_expanddir.test(WEST))  COUT << " W";
      if (_expanddir.test(UP))    COUT << " VU";
      if (_expanddir.test(DOWN))  COUT << " VD";
      COUT << '\n';
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
        if (n1->fcost() == n2->fcost()) {
          return NodeComp()(n1, n2);
        }
        return n1->fcost() > n2->fcost();
      }
      return n1->cost() < n2->cost();
    }
    if (n1 == nullptr) return true;
    return false;
  }
};

typedef std::pair<int, int> IntPair;
struct IntPairComp {
  bool operator() (const IntPair& p1, const IntPair& p2) const
  {
    if (p1.first == p2.first) return p1.second < p2.second;
    return p1.first < p2.first;
  }
};
typedef std::set<IntPair, IntPairComp> IntRangeSet;
typedef std::set<Node*, NodeComp> NodeSet;
typedef std::multiset<const Node*, NodeCostComp> PriorityQueue;
typedef std::vector<std::map<IntPair, Node*, IntPairComp>> NodeMap;
class Router {
  private:
    PriorityQueue _pq;
    NodeSet _sources, _targets;
    NodeMap _nodes;
#if DEBUG
    std::set<Node*> _nodeset;
#endif
    Geom::LayerRects _obstacles, _tobstacles;
    LayerPolySet _pobstacles, _ptobstacles;
    LayerPolySet _psources, _ptargets;

    CostFn _cf;
    std::vector<std::map<int, IntRangeSet>> _hanangridh, _hanangridv;
    Geom::Rect _bbox, _mbox;
    const Node *_sol;
    // findSol() clears _sol (via clearSourceTargets()) before returning, so a
    // caller can't tell "no path found" apart from "found a trivial path
    // needing zero additional shapes" (source and target already coincide)
    // just by checking whether the returned shape list is empty -- both
    // return an empty list. This captures which one actually happened.
    bool _lastSolFound{false};
    std::vector<int> _widthx, _ndrwidthx, _spacex, _ndrspacex;
    std::vector<int> _drcspacex, _drcspacey;
    std::vector<int> _widthy, _ndrwidthy, _spacey, _ndrspacey;
    int _minLayer, _maxLayer, _maxRoutingLayer;
    Vias _vias;
    std::vector<Vias> _upVias, _dnVias;
    std::string _name;
    size_t _expansions{0};
    const size_t _maxExpansions{100000};
    std::vector<int> _aboveViaLayer, _belowViaLayer;
    const DRC::LayerInfo& _lf;
    std::map<const Node*, int> _endextnxmin, _endextnymin, _endextnxmax, _endextnymax;
    std::map<int, std::set<Geom::Rect>> _sourceshapes, _targetshapes;
    std::string _modname, _netname;
    int _uu;
    Geom::LayerTree _ltree;
    std::set<int> _preflayers;
    bool _usepinwidth{false}, _debugplot{false};
    int _reorderPasses{10};
    int _threads{1};
    int _attemptno{1};
    bool _cornerEscape{false};
    bool _relaxViaEscape{false};
    // Transient per-net state: only true while findSol() is retrying a still-
    // unrouted net's own pin escape via with relaxed spacing (see isViaValid).
    // Reset to false at the top of every findSol() call.
    bool _relaxSrcViaEscape{false};
    bool _relaxTgtViaEscape{false};
    static const int MIN_ESCAPE_SPACE = 5;
    // How far around a pin the on-layer cost is relaxed, in multiples of that
    // layer's spacing. Two spacings covers a hop to a pin one or two tracks away,
    // which is the case where escaping to another layer costs a via pair to cross
    // a gap the via's own pad would have bridged.
    static const int RELAX_PIN_SPACINGS = 2;
    // How close a pin's extent must be to the routing width, as a percentage, for
    // its escapes to be offered on the centre line only. At 100 every pin
    // qualifies, so escapes are always centred rather than pushed to the edges.
    static const int PIN_CENTRE_ESCAPE_PCT = 100;
    static bool centreEscapeOnly(const int extent, const int w)
    {
      return 100 * std::abs(extent - w) <= PIN_CENTRE_ESCAPE_PCT * std::max(extent, w);
    }
    // In the default centre-track pin escape, if a shape's centre track yields
    // this few (or fewer) candidate points, the corner/edge escape points are
    // added too -- so a nearly-blocked pin gets a way out on the very first
    // attempt instead of failing, forcing a (useless) reorder retry, and only
    // then getting corner points from a second, design-wide pass.
    static const size_t MIN_ESCAPE_POINTS = 5;
    // When set, Net::route writes a debug LEF (pins = sources/targets + the three
    // obstacle sets the search faced) for any net it leaves open; used in the
    // final pass so remaining opens can be inspected.
    bool _dumpOpenNets{false};

    // Optional "guide" geometry (the mirrored route of a symmetric partner net).
    // When set, the A* g-cost is biased by each node's distance from the guide so
    // the route is pulled toward it -- making a symmetric pair appear mirrored.
    // Empty / weight 0 -> _hasGuide false -> zero effect on cost (default).
    Geom::LayerRects _guide;
    std::map<int, Geom::Rects> _guideByLayer;
    Geom::Rects _guideAll;
    CostType _guideWeight{0};
    bool _hasGuide{false};

    Node* createNode(const int x = 0, const int y = 0, const int z = 0,
        const Node* parent = nullptr, const int fcost = -1, const int tcost = -1);

    void evalFCost(Node* n)
    {
      CostType fcost{0};
      if (n->parent()) {
        fcost = _cf.deltaCost(*n, *(n->parent())) + n->parent()->fcost();
        auto p = n->parent();
        auto gp = p->parent();
        bool sameLayerBend{false};
        if (gp && ((gp->x() != p->x() && p->x() == n->x()) || (gp->y() != p->y() && p->y() == n->y()))) {
          fcost += 1;
          sameLayerBend = true;
        }
        if (!sameLayerBend) {
          auto ggp = gp ? gp->parent() : nullptr;
          if (ggp && ((ggp->x() != p->x() && p->x() == n->x()) || (ggp->y() != p->y() && p->y() == n->y()))) {
            fcost += 1;
          }
        }
      }
      // Bias toward the symmetry guide: penalise this node's planar distance from
      // the guide. The term depends only on n's position, so it accumulates along
      // the path (via parent->fcost()) without changing parent selection.
      if (_hasGuide) {
        fcost += _guideWeight * guideDeviation(n->x(), n->y(), n->z());
      }
      n->setFCost(fcost);
      /*CostType bends{0};
      const Node* p = n;
      int prev{0};
      int count{0};
      while (p) {
        auto par = p->parent();
        if (par) {
          if (prev == 0) {
            if (p->x() == par->x()) {
              prev = 1;
            } else if (p->y() == par->y()) {
              prev = 2;
            }
          } else {
            if (prev == 1) {
              if (p->x() != par->x()) {
                prev = 2;
                ++bends;
              }
            } else {
              if (p->y() != par->y()) {
                prev = 1;
                ++bends;
              }
            }
          }
        }
        if (++count > 100) break;
        p = par;
      }
      n->setBCost(bends);*/
    }

    void evalTCost(Node* n)
    {
      CostType tcost = CostTypeMax;
      for (auto& t : _targets) {
        tcost = std::min(tcost, _cf.deltaCost(*n, *t));
      }
      n->setTCost(tcost);
    }

    void evalCost(Node* n) { evalFCost(n); evalTCost(n); }

    void insertToPQ(const Node* n);

    void invertRange(IntRangeSet& s, const bool vert);
    void insertRange(IntRangeSet& s, const IntPair& r);
    void expandNode(const Node* n);
    void generateHananGrid();
    void checkAndInsert(Node* newn, const Node* n);
    int snap(const Node* n, const bool vert, const bool up) const;
    void getTargetGrid(std::set<int>& s, const Node* n, const bool vert, const int snapc);
    void getAdjacentGrid(std::set<int>& s, const Node* n, const bool above, const bool up, const int snapc);
    void getCrossGrid(std::set<int>& s, const Node* n, const bool vert, const int snapc);
    void flushNodes()
    {
      _pq.clear();
      for (auto& l : _nodes) {
        for (auto& n : l) {
          delete n.second;
#if DEBUG
          _nodeset.erase(n.second);
#endif
          n.second = nullptr;
        }
        l.clear();
      }
      _nodes.clear();
      _nodes.resize(_maxLayer + 1);
      // these maps are keyed by Node*, which are gone now
      _endextnxmin.clear();
      _endextnymin.clear();
      _endextnxmax.clear();
      _endextnymax.clear();
      COUT << "flushing nodes\n";
#if DEBUG
      COUT << " remaining " << Node::_nodectr << ' ' << _nodeset.size() << "\n";
      for (auto& n : _nodeset) {
        n->print("rem node : ");
      }
#endif
      _expansions = 0;
      _bbox = Geom::Rect();
    }
    Geom::PointWidthSet findValidPoints(const Geom::Rect& r, const int z, const Direction dir) const;

    bool isTarget(const Node* n) const { return _targets.find(const_cast<Node*>(n)) != _targets.end(); }
    bool isSource(const Node* n) const { return _sources.find(const_cast<Node*>(n)) != _sources.end(); }
    // Coordinate-based counterparts of isSource/isTarget, used to recognise a
    // pin's own escape via from the *other* side of the via check: isViaValid
    // is called on the node the search is expanding FROM, which for a via that
    // lands ON a source/target pin (e.g. the final via-down onto a target pin
    // approached from the layer above) is the adjacent layer's node, not the
    // pin node itself. These look up whatever node (if any) already exists at
    // (x,y,z) and test that node instead.
    bool isSourceAt(const int x, const int y, const int z) const {
      auto it = _nodes[z].find(std::make_pair(x, y));
      return it != _nodes[z].end() && isSource(it->second);
    }
    bool isTargetAt(const int x, const int y, const int z) const {
      auto it = _nodes[z].find(std::make_pair(x, y));
      return it != _nodes[z].end() && isTarget(it->second);
    }
    void setexpand(Node* newn, const Node* parent) const;

    void constructVias(const std::map<int, DRC::ViaArray>* ndrvias = nullptr);
    void createSourceTargetNodes();
    void buildSol(Geom::LayerRects& sol);
    int roundup(const int x) const
    {
      if (_precision <= 0) return x;
      auto r = x % _precision;
      return (r == 0) ? x : (x + _precision - r);
    }

    
  public:
    Router(const DRC::LayerInfo& lf);
    static int _precision;
    ~Router()
    {
      flushNodes();
      _sources.clear();
      _targets.clear();
      _vias.clear();
    }
    const int maxLayer() const { return _maxLayer; }
    const int minLayer() const { return _minLayer; }
    bool canViaUp(const int z) const
    { return z >= 0 && z < static_cast<int>(_aboveViaLayer.size()) && _aboveViaLayer[z] >= 0; }
    bool canViaDown(const int z) const
    { return z >= 0 && z < static_cast<int>(_belowViaLayer.size()) && _belowViaLayer[z] >= 0; }
    void setName(const std::string& n) { _name = n; }
    void setusepinwidth(const bool u) { _usepinwidth = u; }

    int widthx(const int z) const { return (_ndrwidthx[z] != INT_MAX ? _ndrwidthx[z] : _widthx[z]); }
    int widthy(const int z) const { return (_ndrwidthy[z] != INT_MAX ? _ndrwidthy[z] : _widthy[z]); }
    int spacex(const int z) const { return (_ndrspacex[z] != INT_MAX ? _ndrspacex[z] : _spacex[z]); }
    int spacey(const int z) const { return (_ndrspacey[z] != INT_MAX ? _ndrspacey[z] : _spacey[z]); }
    // base (non-NDR) width/space, valid before any net's updatendr() runs.
    int baseWidthX(const int z) const { return (z >= 0 && z < static_cast<int>(_widthx.size())) ? _widthx[z] : 0; }
    int baseWidthY(const int z) const { return (z >= 0 && z < static_cast<int>(_widthy.size())) ? _widthy[z] : 0; }
    int drcSpaceX(const int z) const { return (z >= 0 && z < static_cast<int>(_drcspacex.size())) ? _drcspacex[z] : 0; }
    int drcSpaceY(const int z) const { return (z >= 0 && z < static_cast<int>(_drcspacey.size())) ? _drcspacey[z] : 0; }
    int baseSpaceX(const int z) const { return (z >= 0 && z < static_cast<int>(_spacex.size())) ? _spacex[z] : 0; }
    int baseSpaceY(const int z) const { return (z >= 0 && z < static_cast<int>(_spacey.size())) ? _spacey[z] : 0; }

    void clearSourceTargets() {
      _cf.resetdirs();
      _sources.clear();
      _targets.clear();
      _sourceshapes.clear();
      _cf.clearRelaxZones();
      _psources.clear();
      _targetshapes.clear();
      _ptargets.clear();
      _endextnxmin.clear();
      _endextnymin.clear();
      _endextnxmax.clear();
      _endextnymax.clear();
      _sol = nullptr;
      flushNodes();
      _bbox = Geom::Rect();
      _preflayers.clear();
    }
    Geom::LayerRects findSol();
    // Whether the most recent findSol() call actually found a solution --
    // check this instead of the returned shape list's emptiness, which is
    // also empty on a legitimate zero-length (already-coincident) solution.
    bool lastSolutionFound() const { return _lastSolFound; }
    void setMBox(const Geom::Rect& box) { _mbox = box; }
    void printSol() const;
    void plot() const;
    //void writeSTO() const;
    void clearObstacles(bool temp = false)
    {
      // _ltree's RTree2D entries hold a bound reference into the Rects vector
      // they were built from (see generateHananGrid(), which already clears
      // _ltree before _tobstacles for this reason) -- clear it first so no
      // entry is left referencing an already-cleared obstacle vector.
      _ltree.clear();
      if(temp) {
          _tobstacles.clear();
          _ptobstacles.clear();
      }
      else {
          _obstacles.clear();
          _pobstacles.clear();
      }
    }
    void addObstacles(const Geom::LayerRects& lr, const bool temp = false);

    void addSourceTargetShapes(const Geom::Rect& r, const int z, const bool src);
    void addSourceShapes(const Geom::Rect& r, const int z) { addSourceTargetShapes(r, z, true); }
    void addTargetShapes(const Geom::Rect& r, const int z) { addSourceTargetShapes(r, z, false); }
    void addSourceTarget(const Geom::Rect& r, const int z, const bool src);
    const Via* isViaValid(const Node* n, const bool up) const;
    void updatendr(const bool usendr, const std::map<int, int>& ndrwidths,
        const std::map<int, int>& ndrspaces, const std::map<int, DRC::Direction>& ndrdirs,
        const std::set<int>& preflayers, const std::map<int, DRC::ViaArray>& ndrvias);
    void setModName(const std::string& n) { _modname = n; }
    void setNetName(const std::string& n) { _netname = n; }
    void setuu(const int uu) { _uu = uu; }
    void allowDetour() { _bbox.expand(std::max(_bbox.width(), _bbox.height()) * 10); }
    void writeLEF(const std::string& prefix="DEBUG", const Geom::LayerRects* sol = nullptr) const;
    void setEnableDebug(const bool b) { _debugplot = b; }
    bool debug() const { return _debugplot; }
    void setReorderPasses(const int n) { _reorderPasses = n; }
    int reorderPasses() const { return _reorderPasses; }
    void setCornerEscape(const bool b) { _cornerEscape = b; }
    bool cornerEscape() const { return _cornerEscape; }
    void setRelaxViaEscape(const bool b) { _relaxViaEscape = b; }
    bool relaxViaEscape() const { return _relaxViaEscape; }
    void setDumpOpenNets(const bool b) { _dumpOpenNets = b; }
    bool dumpOpenNets() const { return _dumpOpenNets; }
    static const int TRACE_SAMENET_FROM_ATTEMPT = 3;
    void setAttemptNo(const int n) { _attemptno = n; }
    int attemptNo() const { return _attemptno; }
    bool traceSameNetObstacles() const { return _attemptno >= TRACE_SAMENET_FROM_ATTEMPT; }
    void setThreads(const int n) { _threads = n; }
    int threads() const { return _threads; }
    const DRC::LayerInfo& layerInfo() const { return _lf; }

    // Symmetry guide. setGuide installs the mirrored partner route; weight scales
    // the per-node deviation penalty (0 or empty guide disables it). guideDeviation
    // returns the Manhattan distance from (x,y,z) to the nearest guide shape on the
    // same layer (falling back to any layer). baseUnitCost is the cheapest per-DBU
    // wire cost, used by the caller to scale the weight into cost units.
    void setGuide(const Geom::LayerRects& g, const CostType weight);
    void clearGuide() { _guide.clear(); _guideByLayer.clear(); _guideAll.clear(); _guideWeight = 0; _hasGuide = false; }
    bool hasGuide() const { return _hasGuide; }
    CostType guideDeviation(const int x, const int y, const int z) const;
    CostType baseUnitCost() const;
};

}
#endif
