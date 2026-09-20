#ifndef ROUTER_H_
#define ROUTER_H_
#include <set>
#include <unordered_map>
#include <cstdint>
#include <new>
#include <map>
#include <queue>
#include <bitset>
#include <limits>
#include <memory>
#include <array>

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
#define PATTERN_MIDS 12
#define PATTERN_PAIRS 8u

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
    const int c() const { return _c; }
    void addCuts(const Geom::Point& o, const int wx, const int wy, const int nrow = 1, const int ncol = 1, const int sx = 0, const int sy = 0);
    std::string str() const;
    void addShapes(Geom::LayerRects& lr) const;
};
typedef std::vector<std::shared_ptr<Via>> Vias;

struct ViaEscapeAttempt {
  bool accessible{false};
  int lowerLayer{-1}, upperLayer{-1}, viaLayer{-1};
  Geom::Rect lpad, upad;
  Geom::Rects cuts;
  int blockedLayer{-1};
  Geom::Rects blockingObs;
};

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
    // Extent of all of a layer's zones. Both endpoints have to fall inside some
    // zone for the relaxed rate to apply, so one point outside this settles it
    // without touching the zone list.
    std::map<int, Geom::Rect> _relaxbbox;
    // ...and inside the extent, a uniform bucket grid over the zones, so a
    // lookup touches the handful of zones near the point instead of all of them.
    // A net with hundreds of pin rectangles has hundreds of zones, and relaxed()
    // is called for every cost evaluation.
    struct RelaxIndex {
      Geom::Rect bbox;
      int nx{0}, ny{0};
      long long cw{1}, ch{1};
      std::vector<std::vector<int>> cell;
    };
    mutable std::map<int, RelaxIndex> _relaxindex;
    mutable bool _relaxdirty{true};
    void buildRelaxIndex() const
    {
      _relaxindex.clear();
      for (const auto& lz : _relaxzones) {
        const auto& v = lz.second;
        if (v.empty()) continue;
        RelaxIndex ix;
        ix.bbox = _relaxbbox.at(lz.first);
        int dim = 1;
        while (dim * dim < static_cast<int>(v.size()) && dim < 128) ++dim;
        ix.nx = ix.ny = dim;
        ix.cw = std::max<long long>(1, (ix.bbox.width()  + dim - 1) / dim);
        ix.ch = std::max<long long>(1, (ix.bbox.height() + dim - 1) / dim);
        ix.cell.resize(static_cast<size_t>(dim) * dim);
        for (size_t i = 0; i < v.size(); ++i) {
          const int cx0 = cellOfCoord(v[i].xmin() - ix.bbox.xmin(), ix.cw, dim);
          const int cx1 = cellOfCoord(v[i].xmax() - ix.bbox.xmin(), ix.cw, dim);
          const int cy0 = cellOfCoord(v[i].ymin() - ix.bbox.ymin(), ix.ch, dim);
          const int cy1 = cellOfCoord(v[i].ymax() - ix.bbox.ymin(), ix.ch, dim);
          for (int cx = cx0; cx <= cx1; ++cx)
            for (int cy = cy0; cy <= cy1; ++cy)
              ix.cell[static_cast<size_t>(cy) * dim + cx].push_back(static_cast<int>(i));
        }
        _relaxindex.emplace(lz.first, std::move(ix));
      }
      _relaxdirty = false;
    }
    static int cellOfCoord(const long long off, const long long span, const int dim)
    {
      if (off <= 0) return 0;
      const long long c = off / span;
      return static_cast<int>(c >= dim ? dim - 1 : c);
    }
    // zones whose cell the point falls in; empty span means the point is outside
    const std::vector<int>* relaxCandidates(const int z, const Geom::Point& p) const
    {
      auto it = _relaxindex.find(z);
      if (it == _relaxindex.end()) return nullptr;
      const RelaxIndex& ix = it->second;
      if (!ix.bbox.contains(p, false)) return nullptr;
      const int cx = cellOfCoord(p.x() - ix.bbox.xmin(), ix.cw, ix.nx);
      const int cy = cellOfCoord(p.y() - ix.bbox.ymin(), ix.ch, ix.ny);
      return &ix.cell[static_cast<size_t>(cy) * ix.nx + cx];
    }
    CostType _minMetalCost{0};
    static int root(const std::vector<int>& uf, int a)
    {
      while (uf[a] != a) a = uf[a];
      return a;
    }
    // patternLowerBound()'s widened (minz,maxz) search result, keyed by the
    // *un*-widened (minz,maxz) pair it started from -- that result depends
    // only on the layer cost tables, never on which nodes are being
    // compared, so it is the same for every pair patternRoute() ranks with
    // the same starting layers. Computed once per _layerHCost/_layerVCost
    // state (there are at most a handful of distinct (minz,maxz) pairs to
    // begin with -- (topRoutingLayer+1)^2), instead of re-walked by every
    // one of the thousands of pair evaluations a single net's pattern
    // routing can do -- that re-walk, not the O(layers) cost of any single
    // widen, is what showed up as a real wall-clock slowdown on the
    // sky130-benchmarks suite (mfb_biquad +18%, mos_bandgap_fb +105%).
    // Invalidated wherever _layerHCost/_layerVCost themselves change
    // (updatendr(), resetdirs()), never elsewhere.
    mutable std::vector<std::vector<std::pair<int, int>>> _widenedWindowCache;
    mutable bool _widenedWindowDirty{true};
    void buildWidenedWindowCache() const
    {
      const int n = _topRoutingLayer + 1;
      _widenedWindowCache.assign(n, std::vector<std::pair<int, int>>(n, {0, 0}));
      for (int minz0 = 0; minz0 < n; ++minz0) {
        for (int maxz0 = minz0; maxz0 < n; ++maxz0) {
          int minz = minz0, maxz = maxz0;
          if (_layerHCost[minz] != _layerVCost[minz]) {
            CostType best = std::min(_layerHCost[minz], _layerVCost[minz]);
            while (maxz < _topRoutingLayer) {
              const CostType next = std::min(_layerHCost[maxz + 1], _layerVCost[maxz + 1]);
              if (next >= best) break;
              ++maxz;
              best = std::min(best, next);
            }
            while (minz > 0) {
              const CostType next = std::min(_layerHCost[minz - 1], _layerVCost[minz - 1]);
              if (next >= best) break;
              --minz;
              best = std::min(best, next);
            }
          }
          _widenedWindowCache[minz0][maxz0] = {minz, maxz};
        }
      }
      _widenedWindowDirty = false;
    }
  public:
    CostType deltaCost(const Node& n1, const Node& n2) const;
    // Same shape as deltaCost's general (non-adjacent, non-same-layer) case,
    // used only to pre-rank patternRoute()'s candidate (source, target)
    // pairs -- see the definition for why it needs a wider layer window than
    // deltaCost's real per-move cost accounting can safely use everywhere.
    CostType patternLowerBound(const Node& n1, const Node& n2) const;
    void markCostTablesDirty() { _widenedWindowDirty = true; }
    CostFn(const DRC::LayerInfo& lf);
    void setRelaxFloor(const CostType c) { if (c > 0 && c < COST_MAX) _minMetalCost = c; }
    void clearRelaxZones()
    { _relaxzones.clear(); _relaxuf.clear(); _relaxbbox.clear(); _relaxindex.clear(); _relaxdirty = true; }
    void addRelaxZone(const int z, const Geom::Rect& r)
    {
      auto& v = _relaxzones[z];
      auto& uf = _relaxuf[z];
      _relaxbbox[z].merge(r);
      _relaxdirty = true;
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
      if (_relaxdirty) buildRelaxIndex();
      const Geom::Point p1(x1, y1), p2(x2, y2);
      const std::vector<int>* c1 = relaxCandidates(z, p1);
      if (!c1 || c1->empty()) return base;
      const std::vector<int>* c2 = relaxCandidates(z, p2);
      if (!c2 || c2->empty()) return base;
      auto it = _relaxzones.find(z);
      if (it == _relaxzones.end()) return base;
      const auto& v = it->second;
      const auto& uf = _relaxuf.at(z);
      int g1[8], g2[8];
      int n1 = 0, n2 = 0;
      for (const int i : *c1) {
        if (n1 >= 8) break;
        if (v[i].contains(p1, false)) g1[n1++] = root(uf, i);
      }
      if (n1 == 0) return base;
      for (const int i : *c2) {
        if (n2 >= 8) break;
        if (v[i].contains(p2, false)) g2[n2++] = root(uf, i);
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
      _widenedWindowDirty = true;
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
    bool _noVia{false};
    // This node's slot in the priority queue, -1 when not queued. Holding it on
    // the node is what makes re-prioritising O(1) to locate: the queue never has
    // to be searched for the entry.
    int _pqidx{-1};
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
    int pqidx() const { return _pqidx; }
    void setPQIdx(const int i) { _pqidx = i; }
    bool noVia() const { return _noVia; }
    void setNoVia() { _noVia = true; }
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
// Free intervals along one grid line, sorted by start and kept disjoint. Every
// consumer scans it linearly, and a grid build allocates one of these per line
// per layer -- twice per search -- so a contiguous buffer beats a tree here on
// both counts.
typedef std::vector<IntPair> IntRangeSet;
typedef std::set<Node*, NodeComp> NodeSet;
// Binary min-heap whose elements know where they are (Node::_pqidx). A node
// whose cost improves sifts up from its own slot, so the queue is never searched
// and never holds a superseded copy.
typedef std::vector<Node*> PriorityQueue;
// Nodes are looked up by coordinate several times per expansion, so the layer
// maps are hashed rather than ordered -- nothing ever iterates them in order.
inline uint64_t nodeKey(const int x, const int y)
{
  const uint64_t k = (static_cast<uint64_t>(static_cast<uint32_t>(x)) << 32)
                   | static_cast<uint32_t>(y);
  // splitmix64 finaliser: coordinates are multiples of the manufacturing grid,
  // so the low bits are poorly distributed on their own
  uint64_t h = k + 0x9e3779b97f4a7c15ULL;
  h = (h ^ (h >> 30)) * 0xbf58476d1ce4e5b9ULL;
  h = (h ^ (h >> 27)) * 0x94d049bb133111ebULL;
  return h ^ (h >> 31);
}
struct NodeKeyHash {
  size_t operator() (const uint64_t k) const { return static_cast<size_t>(k); }
};
typedef std::vector<std::unordered_map<uint64_t, Node*, NodeKeyHash>> NodeMap;
bool replay(class Router& r, const std::string& leffile, const int uu, const bool detour,
    const std::string& ndrfile, const DRC::LayerInfo& lf);

Geom::Rects intersectPObstacles(const LayerPolySet& pobs, const LayerPolySet& ptobs,
                                const int layer, const Geom::Rect& query);

class Router {
  private:
    PriorityQueue _pq;
    std::vector<int> _gridposbuf;
    NodeSet _sources, _targets;
    NodeMap _nodes;
    // Nodes are created and released in bulk -- thousands per search, all torn
    // down together by flushNodes -- so they come out of chunked storage rather
    // than one malloc each. The chunks themselves are kept and reused across
    // searches; only the objects in them are destroyed.
    static const size_t NODE_CHUNK = 4096;
    std::vector<char*> _nodechunks;
    size_t _nodechunk{0};      // chunk the next node comes from
    size_t _nodeinchunk{NODE_CHUNK};
    Node* allocNode(const int x, const int y, const int z, const CostType fcost,
                    const CostType tcost, const Node* parent)
    {
      if (_nodeinchunk == NODE_CHUNK) {       // current chunk full, or none yet
        if (_nodechunk == _nodechunks.size()) {
          _nodechunks.push_back(static_cast<char*>(::operator new(sizeof(Node) * NODE_CHUNK)));
        }
        _nodeinchunk = 0;
      }
      void* slot = _nodechunks[_nodechunk] + sizeof(Node) * _nodeinchunk;
      if (++_nodeinchunk == NODE_CHUNK) ++_nodechunk;   // next call takes a fresh chunk
      return new (slot) Node(x, y, z, fcost, tcost, parent);
    }
    void resetNodePool() { _nodechunk = 0; _nodeinchunk = NODE_CHUNK; }
    void freeNodePool()
    {
      for (auto* c : _nodechunks) ::operator delete(c);
      _nodechunks.clear();
      resetNodePool();
    }
#if DEBUG
    std::set<Node*> _nodeset;
#endif
    Geom::LayerRects _obstacles, _tobstacles;
    LayerPolySet _pobstacles, _ptobstacles;
    // Obstacles added via addObstacles(..., true, /*noCoverHoles=*/true):
    // merged into _ptobstacles in generateHananGrid() *after* coverHoles()
    // runs, so they never contribute a "hole" for it to see and fill in --
    // for a keepout ring (RSMT corridor wall) whose whole interior is
    // legitimate open routing space, not a small void worth paving over.
    // Kept separate rather than a per-rect flag so real obstacles' own
    // coverHoles behaviour (correct for genuine obstacle geometry) is
    // completely untouched.
    LayerPolySet _ptobstaclesNoHole;
    LayerPolySet _psources, _ptargets;

    CostFn _cf;
    std::vector<std::map<int, IntRangeSet>> _hanangridh, _hanangridv;
    Geom::Rect _bbox, _mbox;
    const Node *_sol;
    bool _lastSolFound{false};
    std::vector<std::array<int, 6>> _exploredEdges;
    std::vector<int> _widthx, _ndrwidthx, _spacex, _ndrspacex;
    std::vector<int> _drcspacex, _drcspacey;
    std::vector<int> _widthy, _ndrwidthy, _spacey, _ndrspacey;
    int _minLayer, _maxLayer, _maxRoutingLayer;
    Vias _vias;
    std::vector<Vias> _upVias, _dnVias;
    std::string _name;
    size_t _expansions{0};
    // Cumulative expansions since the last reset, across every search this
    // Router ran. Placement uses it to size a block's reorder effort.
    size_t _moduleExpansions{0};
    long _sollen{0};
    size_t _maxExpansions{100000};
    std::vector<int> _aboveViaLayer, _belowViaLayer;
    const DRC::LayerInfo& _lf;
    std::map<const Node*, int> _endextnxmin, _endextnymin, _endextnxmax, _endextnymax;
    std::map<int, std::set<Geom::Rect>> _sourceshapes, _targetshapes;
    // Cells already covered by an accepted escape point, keyed by (layer, dir).
    // Per net; cleared with the sources and targets.
    std::map<IntPair, std::set<IntPair>> _escapecells;
    // Searches that already failed on an identical problem, per module. A* is
    // deterministic, so the same obstacles, the same entry points and the same
    // flags must fail again; skipping them cannot change any result.
    std::set<unsigned long long> _failedSearches;
    size_t _memoHits{0};
    std::string _modname, _netname;
    int _uu;
    Geom::LayerTree _ltree;
    std::set<int> _preflayers;
    bool _usepinwidth{false}, _debugplot{false};
    bool _useWidePatternBound{false};
    int _reorderPasses{10};
    long long _reorderBudget{15000000};
    int _threads{1};
    bool _rsmtcorridor{false};
    int _attemptno{1};
    bool _cornerEscape{false};
    bool _relaxViaEscape{false};
    int _escapePitchMul{1};
    bool _pruneEscapes{true};
    bool _viaAlign{false};
    bool _satFirst{false};
    int _gridprune{0};     // collapse grid coordinates closer than this % of pitch
    bool _abutEscape{false};
    mutable size_t _abutSeen{0}, _abutAllowed{0};
    // Raw (un-bloated) obstacle rects touching a pin, cached per pin: the pads of
    // every candidate via at that pin ask the same question.
    mutable std::map<std::pair<int, IntPair>, Geom::Rects> _rawnear;
    const Geom::Rects& rawNear(const int l, const Geom::Rect& pin) const
    {
      const auto key = std::make_pair(l, std::make_pair(pin.xmin(), pin.ymin()));
      auto it = _rawnear.find(key);
      if (it != _rawnear.end()) return it->second;
      return _rawnear.emplace(key,
        intersectPObstacles(_pobstacles, _ptobstacles, l, pin.bloatby(1, 1))).first->second;
    }
    int _hopelessAfter{3};
    int _maxSeedPolys{0};
    bool _seedPolysAlways{false};
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
    static bool inPQ(const Node* n) { return n->pqidx() >= 0; }
    void clearPQ()
    {
      for (auto* n : _pq) n->setPQIdx(-1);
      _pq.clear();
    }
    Node* popPQ();
    void pqSiftUp(int i);
    void pqSiftDown(int i);
    void pqPlace(Node* n, const int i) { _pq[i] = n; n->setPQIdx(i); }
    // A node's cost only ever improves, so re-prioritising is a sift up.
    void pqImproved(Node* n) { if (inPQ(n)) pqSiftUp(n->pqidx()); }

    void invertRange(IntRangeSet& s, const bool vert);
    void insertRange(IntRangeSet& s, const IntPair& r);
    void expandNode(const Node* n);
    void generateHananGrid();
    void checkAndInsert(Node* newn, const Node* n);
    int snap(const Node* n, const bool vert, const bool up) const;
    void getTargetGrid(std::vector<int>& s, const Node* n, const bool vert, const int snapc);
    void getAdjacentGrid(std::vector<int>& s, const Node* n, const bool above, const bool up, const int snapc);
    void getCrossGrid(std::vector<int>& s, const Node* n, const bool vert, const int snapc);
    void flushNodes()
    {
      clearPQ();
      for (auto& l : _nodes) {
        for (auto& n : l) {
          n.second->~Node();
#if DEBUG
          _nodeset.erase(n.second);
#endif
          n.second = nullptr;
        }
        l.clear();
      }
      resetNodePool();   // storage stays, the objects in it are gone
      // The maps above were cleared, which keeps their bucket arrays. Destroying
      // and rebuilding the vector would throw those away and make every wire
      // rehash its way back up from zero buckets -- the cost of that grows with
      // the design, so per-expansion cost degrades as blocks get larger.
      if (_nodes.size() != static_cast<size_t>(_maxLayer) + 1) {
        _nodes.clear();
        _nodes.resize(_maxLayer + 1);
      }
      // these maps are keyed by Node*, which are gone now
      _endextnxmin.clear();
      _endextnymin.clear();
      _endextnxmax.clear();
      _endextnymax.clear();
      if (verboseLog()) COUT << "flushing nodes\n";
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
    Geom::PointWidthSet findBoundaryPoints(const Geom::Rect& r, const int z, const Direction dir,
        const Geom::PointWidthSet& centre) const;

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
      auto it = _nodes[z].find(nodeKey(x, y));
      return it != _nodes[z].end() && isSource(it->second);
    }
    bool isTargetAt(const int x, const int y, const int z) const {
      auto it = _nodes[z].find(nodeKey(x, y));
      return it != _nodes[z].end() && isTarget(it->second);
    }
    const Geom::Rect* pinShapeAt(const int x, const int y, const int z) const
    {
      for (const auto* m : {&_sourceshapes, &_targetshapes}) {
        auto it = m->find(z);
        if (it == m->end()) continue;
        for (const auto& r : it->second) {
          if (x >= r.xmin() && x <= r.xmax() && y >= r.ymin() && y <= r.ymax()) return &r;
        }
      }
      return nullptr;
    }
    // A shape that already touches the pin has no spacing to it left to
    // protect: the pin is up against it whatever the via does. So when the via
    // is landing on a pin, such a shape is judged on overlap alone -- the pad
    // may sit beside it, but must not run into it, which would be a short.
    // Returns true if this obstacle should stop the via.
    // The pin to compare against is the one on the pad's OWN layer, not the one
    // under the node being expanded: a via landing on a pin from the layer above
    // is evaluated from a node that has no pin beneath it, and that is exactly
    // the case this rule has to cover.
    bool padBlocked(const Geom::Rect& shrunk, const Geom::Rect& pad, const int l,
                    const int x, const int y) const
    {
      const Geom::Rect* pin = _abutEscape ? pinShapeAt(x, y, l) : nullptr;
      if (!_abutEscape || !pin) return shrunk.overlaps(pad, true);
      // Ask the un-bloated geometry, not a shrunk-back copy of the bloated rect:
      // splitRects merges and splits as it builds _ltree, so un-bloating does not
      // land back on the drawn shape.
      const Geom::Rects& raw = rawNear(l, *pin);
      bool abutting = false, shorts = false;
      for (const auto& r : raw) {
        if (!r.overlaps(*pin, false)) continue;
        abutting = true;
        if (r.overlaps(pad, true)) { shorts = true; break; }
      }
      if (!abutting) return shrunk.overlaps(pad, true);
      ++_abutSeen;
      if (!shorts) ++_abutAllowed;
      return shorts;
    }
    static long long padPinOverlap(const Geom::Rect& pad, const Geom::Rect& pin)
    {
      const long long w = std::min(pad.xmax(), pin.xmax()) - std::max(pad.xmin(), pin.xmin());
      const long long h = std::min(pad.ymax(), pin.ymax()) - std::max(pad.ymin(), pin.ymin());
      return (w <= 0 || h <= 0) ? 0 : w * h;
    }
    void setexpand(Node* newn, const Node* parent) const;

    void constructVias(const std::map<int, DRC::ViaArray>* ndrvias = nullptr);
    void createSourceTargetNodes();
    // An escape point the search can neither leave nor be reached at. Needs the
    // Hanan grid, so it can only be asked after generateHananGrid().
    bool escapeUsable(const Node* n) const;
    bool pruneDeadEscapes();
    // Fingerprint of everything the search reads: the resolved obstacle set, the
    // entry points, and the flags that steer expansion. Needs the Hanan grid to
    // have been built, since that is what resolves _tobstacles.
    unsigned long long searchKey(const int pass) const;
    // Over-approximate reachability over the Hanan grid: true means "maybe",
    // false means the search provably cannot reach any target. Only the false
    // answer is acted on, so over-connecting is safe and under-connecting is not.
    bool escapesConnected() const;
    static const int REACH_VISIT_LIMIT = 20000;
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
      freeNodePool();
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
    const std::string& name() const { return _name; }
    void setusepinwidth(const bool u) { _usepinwidth = u; }
    // patternRoute()'s pair pre-ranking uses CostFn::patternLowerBound (a
    // wider, more optimistic layer-cost search) instead of deltaCost when
    // this is set -- validated to improve both wirelength and confinement
    // for an isolated net with little competition for the layers it wants
    // (the RSMT-corridor GUI-completion path), but measured as a net
    // *regression* -- worse average wirelength, more full-A* fallback, real
    // slowdowns -- on whole-chip multi-net sky130-benchmarks circuits: many
    // nets chasing the same cheap layer congest it, an effect a single
    // isolated net's test can't show. Module::route() sets this per module
    // from its own net count, not globally, so a small hierarchy benefits
    // without risking the main flow's larger blocks.
    void setWidePatternBound(const bool b) { _useWidePatternBound = b; }
    bool useWidePatternBound() const { return _useWidePatternBound; }

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
      _rawnear.clear();
      _cf.resetdirs();
      _sources.clear();
      _targets.clear();
      _escapecells.clear();
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
    long solLength() const { return _sollen; }
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
          _ptobstaclesNoHole.clear();
      }
      else {
          _obstacles.clear();
          _pobstacles.clear();
      }
    }
    void addObstacles(const Geom::LayerRects& lr, const bool temp = false, const bool noCoverHoles = false);

    void addSourceTargetShapes(const Geom::Rect& r, const int z, const bool src);
    void addSourceShapes(const Geom::Rect& r, const int z) { addSourceTargetShapes(r, z, true); }
    void addTargetShapes(const Geom::Rect& r, const int z) { addSourceTargetShapes(r, z, false); }
    void addSourceTarget(const Geom::Rect& r, const int z, const bool src);
    const Via* isViaValid(const Node* n, const bool up) const;
    // How well a via candidate's pads lie along the wires they join. The pads in
    // this technology are ~3:1, so a pad turned across its wire buys nothing and
    // occupies the neighbouring track instead.
    int viaPadAlign(const Node* n, const bool up, const Via& v) const;

    struct PatWp { int x, y, z; bool via; };
    bool patternRun(const int x, const int y, const int z, const bool vert, const int to) const;
    typedef std::map<std::tuple<int,int,int,bool>, bool> ViaCache;
    bool patternVias(const int x, const int y, int zf, const int zt, std::vector<PatWp>& w, ViaCache& vc) const;
    CostType patternCost(const std::vector<PatWp>& w) const;
    bool patternRoute();
    std::vector<ViaEscapeAttempt> diagnoseViaEscape(const Node* n, const bool up) const;
    void updatendr(const bool usendr, const std::map<int, int>& ndrwidths,
        const std::map<int, int>& ndrspaces, const std::map<int, DRC::Direction>& ndrdirs,
        const std::set<int>& preflayers, const std::map<int, DRC::ViaArray>& ndrvias);
    void setModName(const std::string& n)
    {
      // obstacles differ from block to block, so nothing carries over
      if (n != _modname) { _failedSearches.clear(); _memoHits = 0; }
      _modname = n;
    }
    size_t memoHits() const { return _memoHits; }
    void setNetName(const std::string& n) { _netname = n; }
    void setuu(const int uu) { _uu = uu; }
    void allowDetour() { _bbox.expand(std::max(_bbox.width(), _bbox.height()) * 10); }
    void writeLEF(const std::string& prefix="DEBUG", const Geom::LayerRects* sol = nullptr) const;
    void setEnableDebug(const bool b) { _debugplot = b; }
    bool debug() const { return _debugplot; }
    void setReorderPasses(const int n) { _reorderPasses = n; }
    int reorderPasses() const { return _reorderPasses; }
    // A* node-expansion budget per search stage. Searches that fail burn the whole
    // budget, so the cap trades runtime against nets the search would have closed
    // had it been allowed to keep going.
    void setMaxExpansions(const size_t n) { _maxExpansions = n; }
    size_t maxExpansions() const { return _maxExpansions; }
    void resetModuleExpansions() { _moduleExpansions = 0; }
    void addModuleExpansions(const size_t n) { _moduleExpansions += n; }
    size_t moduleExpansions() const { return _moduleExpansions; }
    // A reorder pass costs about what the block's first routing attempt cost, so
    // a block that is already expensive cannot afford ten of them. Scale the
    // pass count by measured search work rather than wall time, so the result
    // stays reproducible. 0 disables the cap.
    void setReorderBudget(const long long n) { _reorderBudget = n; }
    long long reorderBudget() const { return _reorderBudget; }
    static const int MIN_REORDER_PASSES = 2;
    // Reorder passes in a row that fail to improve on the best before giving up.
    static const int REORDER_STALE_LIMIT = 2;
    int effectiveReorderPasses(const size_t baseExpansions) const
    {
      if (_reorderBudget <= 0 || baseExpansions == 0) return _reorderPasses;
      const long long n = _reorderBudget / static_cast<long long>(baseExpansions);
      if (n >= _reorderPasses) return _reorderPasses;
      return static_cast<int>(std::max<long long>(MIN_REORDER_PASSES, n));
    }
    // Escape-point thinning radius, as a multiple of the layer's wire pitch.
    // 0 disables it and every candidate point becomes a search entry.
    void setEscapePitchMul(const int n) { _escapePitchMul = n; }
    int escapePitchMul() const { return _escapePitchMul; }
    // Whole attempts a net may go through without routing any wire before it is
    // retired from the reorder loop. 0 never retires.
    void setHopelessAfter(const int n) { _hopelessAfter = n < 0 ? 0 : n; }
    int hopelessAfter() const { return _hopelessAfter; }
    void setGridPrune(const int p) { _gridprune = p; }
    int gridPrune() const { return _gridprune; }
    void setAbutEscape(const bool b) { _abutEscape = b; }
    bool abutEscape() const { return _abutEscape; }
    size_t abutSeen() const { return _abutSeen; }
    size_t abutAllowed() const { return _abutAllowed; }
    void setSatFirst(const bool b) { _satFirst = b; }
    bool satFirst() const { return _satFirst; }
    void setViaAlign(const bool b) { _viaAlign = b; }
    bool viaAlign() const { return _viaAlign; }
    void setPruneEscapes(const bool b) { _pruneEscapes = b; }
    bool pruneEscapes() const { return _pruneEscapes; }
    // A pin split into many polygons does not need an escape on every one: the
    // ones nearest the other terminal are the ones a route would use, and the
    // rest just add Hanan lines. 0 seeds them all.
    void setMaxSeedPolys(const int n) { _maxSeedPolys = n; }
    int maxSeedPolys() const { return _maxSeedPolys; }
    void setSeedPolysAlways(const bool b) { _seedPolysAlways = b; }
    bool seedPolysAlways() const { return _seedPolysAlways; }
    // Two escape points closer together than one wire pitch are interchangeable:
    // no wire can use them as distinct tracks, yet each contributes its own pair
    // of Hanan lines. This is the cell size within which the second one is
    // redundant. 0 means keep everything.
    int escapeCellSize(const int z) const
    {
      if (_escapePitchMul <= 0) return 0;
      const int w = std::max(baseWidthX(z), baseWidthY(z));
      const int sp = std::max(baseSpaceX(z), baseSpaceY(z));
      return (w + sp > 0) ? (_escapePitchMul * (w + sp)) : 0;
    }
    void setCornerEscape(const bool b) { _cornerEscape = b; }
    bool cornerEscape() const { return _cornerEscape; }
    void setRelaxViaEscape(const bool b) { _relaxViaEscape = b; }
    bool relaxViaEscape() const { return _relaxViaEscape; }
    void setDumpOpenNets(const bool b) { _dumpOpenNets = b; }
    bool dumpOpenNets() const { return _dumpOpenNets; }
    static const int RSMT_RELAX_AFTER_PASS = 3;
    void setRSMTCorridor(const bool b) { _rsmtcorridor = b; }
    bool rsmtCorridor() const { return _rsmtcorridor; }
    // A net still open by this attempt gets an escape on every one of its pin
    // polygons, not just the nearest ones.
    static const int SEED_ALL_POLYS_FROM_ATTEMPT = 3;
    static const int TRACE_SAMENET_FROM_ATTEMPT = 3;
    static const int BOUNDARY_ESCAPE_FROM_ATTEMPT = 3;
    void setAttemptNo(const int n) { _attemptno = n; }
    int attemptNo() const { return _attemptno; }
    static bool seedAllPolysAt(const int n) { return n >= SEED_ALL_POLYS_FROM_ATTEMPT; }
    bool limitSeedPolys() const
    { return _maxSeedPolys > 0 && (_seedPolysAlways || !seedAllPolysAt(_attemptno)); }
    static bool traceSameNetObstaclesAt(const int n) { return n >= TRACE_SAMENET_FROM_ATTEMPT; }
    bool traceSameNetObstacles() const { return traceSameNetObstaclesAt(_attemptno); }
    static bool boundaryEscapeAt(const int n) { return n >= BOUNDARY_ESCAPE_FROM_ATTEMPT; }
    bool boundaryEscape() const { return boundaryEscapeAt(_attemptno); }
    static int attemptMode(const int n)
    { return (traceSameNetObstaclesAt(n) ? 1 : 0) | (boundaryEscapeAt(n) ? 2 : 0)
           | (seedAllPolysAt(n) ? 4 : 0); }
    static std::string attemptModeName(const int n)
    {
      return std::string(traceSameNetObstaclesAt(n) ? "same-net-tracing=on" : "same-net-tracing=off")
           + (boundaryEscapeAt(n) ? " boundary-escapes=on" : " boundary-escapes=off")
           + (seedAllPolysAt(n) ? " seed-all-polys=on" : " seed-all-polys=off");
    }
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
