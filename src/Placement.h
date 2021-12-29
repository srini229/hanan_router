#ifndef PLACEMENT_H_
#define PLACEMENT_H_
#include <fstream>
#include "Geom.h"
#include "Layer.h"

namespace Placement {


class Module;
class Netlist;
class Instance;
typedef std::vector<Instance*> Instances;
typedef std::map<std::string, Module*> Modules;

class Pin {
  private:
    std::string _name;
    Geom::LayerRects _shapes;
    Geom::Rect _bbox;
  public:
    Pin(const std::string& name = "") : _name{name}, _bbox{} {}
    const std::string& name() const { return _name; }
    const std::map<int, Geom::Rects>& getShapes() const { return _shapes; }
    void addRect(const int layer, const Geom::Rect& r)
    {
      _shapes[layer].push_back(r);
      _bbox.merge(r);
    }
    const Geom::LayerRects& shapes() const { return _shapes; }
    const Geom::Rect& bbox() const { return _bbox; }
    void copyRects(const Geom::LayerRects& lr)
    {
      Geom::MergeLayerRects(_shapes, lr, &_bbox);
    }
};
typedef std::map<std::string, Pin*> Pins;


class Net {
  private:
    std::string _name;
    std::set<const Pin*> _pins;
    Geom::LayerRects _routeshapes;
    Geom::Rect _bbox;
  public:
    Net(const std::string& name) : _name{name}, _bbox{} {}
    const std::set<const Pin*>& pins() const { return _pins; }
    void addPin(const Pin* p) { _pins.insert(p); }
    void print() const;
    const std::string& name() const { return _name; }
    void route(const Geom::LayerRects& l1, const Geom::LayerRects& l2, const Geom::LayerRects& l3);
    const Geom::LayerRects& routeShapes() const { return _routeshapes; }
    const Geom::Rect& bbox() const { return _bbox; }
};
typedef std::map<std::string, Net> Nets;


class Instance {
  friend class Module;
  private:
    std::string _name, _modname;
    Geom::Transform _tr;
    const Module* _m;
    Pins _pins;
    Geom::LayerRects _routeshapes;
    void build();
    Geom::Rect _bbox;
  public:
    Instance(const std::string& iname = "", const std::string& mname = "", const Geom::Transform& tr = Geom::Transform()) :
      _name(iname), _modname(mname), _tr(tr), _m(nullptr), _bbox{} {}
    ~Instance();
    const Module* module() const {return _m; }
    const std::string& name() const { return _name; }
    const std::string& moduleName() const { return _modname; }
    const Geom::Transform& transform() const { return _tr; }
    Geom::Rect transform(const Geom::Rect& r) const { return _tr.transform(r); }
    void setModule(const Module* m);
    void print(const std::string& prefix = "") const;
    const Geom::Rect& bbox() const { return _bbox; }
    const Pins& pins() const { return _pins; }
    const Geom::LayerRects& routeShapes() const { return _routeshapes; }
};


class Module {
  friend class Netlist;
  private:
    std::string _name;
    int _leaf : 1;
    int _routed : 1;
    Nets _nets;
    Pins _pins;
    Instances _instances;
    std::map<const Net*, std::vector<std::pair<Instance*, std::string>>> _tmpnetpins;
    Geom::LayerRects _obstacles;
    Geom::Rect _bbox;

    void build();
  public:
    Module(const std::string& name, const int leaf) : _name(name), _leaf(leaf), _routed{leaf}, _bbox{} {_instances.reserve(64);}
    ~Module();
    Instance* addInstance(const std::string& name, const std::string& mname, const Geom::Transform& tr)
    {
      _instances.emplace_back(new Instance(name, mname, tr)); 
      return _instances.back();
    }
    bool routed() const { return (_routed ? true : false); }
    const std::string& name() const { return _name; }
    const Instances& instances() const { return _instances; }
    Instances& instances() { return _instances; }
    const Geom::LayerRects& obstacles() const { return _obstacles; }
    bool isLeaf() const { return _leaf ? true : false; }
    const Nets& nets() const { return _nets; }
    const Pins& pins() const { return _pins; }

    void setBBox(const Geom::Rect& b) { _bbox = b; }
    void setRouted() { _routed = 1; }
    Pin* addPin(const std::string& name)
    {
      Pin* p{nullptr};
      auto it = _pins.find(name);
      if (_pins.find(name) == _pins.end()) {
        p = new Pin(name);
        _pins[name] = p;
      } else {
        p = it->second;
      }
      return p;
    }
    const Net& addNet(const std::string& name)
    {
      auto it = _nets.find(name);
      if (it == _nets.end()) {
        it = _nets.emplace(name, name).first;
      }
      return it->second;
    }
    Net* net(const std::string& name)
    {
      auto it = _nets.find(name);
      if (it != _nets.end()) return &(it->second);
      return nullptr;
    }
    void addTmpPin(const Net* n, Instance* inst, const std::string& pname)
    {
      if(n) _tmpnetpins[n].push_back(std::make_pair(inst, pname));
    }
    Pin* getPin(const std::string& pn)
    {
      auto it = _pins.find(pn);
      return ((it != _pins.end()) ? it->second : nullptr);
    }
    void addObstacle(const int layer, const Geom::Rect& r)
    {
      _obstacles[layer].push_back(r);
    }

    void print() const;
    void route();
    void plot() const;

    const Geom::Rect& bbox() const { return _bbox; }
};


class Netlist {
  private:
    const int _uu;
    Modules _modules;
    void build();
    void loadLEF(const std::string& leffile, const DRC::LayerInfo& lf);

  public:
    Netlist(const std::string& plfile, const::std::string& leffile, const DRC::LayerInfo& lf, const int uu);
    ~Netlist();
    void print() const;
    void route()
    {
      for (auto& m : _modules) m.second->route();
    }
    void plot() const
    {
      for (auto& m : _modules) m.second->plot();
    }
};


}
#endif
