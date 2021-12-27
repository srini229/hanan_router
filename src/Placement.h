#ifndef PLACEMENT_H_
#define PLACEMENT_H_
#include "nlohmann/json.hpp"
#include "Geom.h"
#include "Layer.h"

namespace Placement {


class Module;
class Netlist;
class Instance;
typedef std::vector<Instance*> Instances;
typedef std::map<std::string, Module*> Modules;

typedef std::map<int, Geom::Rects> LayerRects;
class Pin {
  private:
    std::string _name;
    LayerRects _shapes;
  public:
    Pin(const std::string& name = "") : _name{name} {}
    const std::string& name() const { return _name; }
    const std::map<int, Geom::Rects>& getShapes() const { return _shapes; }
    void addRect(const int layer, const Geom::Rect& r)
    {
      _shapes[layer].push_back(r);
    }
    const LayerRects& shapes() const { return _shapes; }
};
typedef std::map<std::string, Pin*> Pins;


class Net {
  private:
    std::string _name;
    std::set<const Pin*> _pins;
  public:
    Net(const std::string& name) : _name{name} {}
    const std::set<const Pin*>& pins() const { return _pins; }
    void addPin(const Pin* p) { _pins.insert(p); }
    void print() const;
    const std::string& name() const { return _name; }
};
typedef std::map<std::string, Net> Nets;


class Instance {
  friend class Module;
  private:
    std::string _name, _modname;
    Geom::Transform _tr;
    const Module* _m;
    Pins _pins;
    void build();
  public:
    Instance(const std::string& iname = "", const std::string& mname = "", const Geom::Transform& tr = Geom::Transform()) :
      _name(iname), _modname(mname), _tr(tr), _m(nullptr) {}
    ~Instance();
    void addModule(const Module* m) { _m = m; }
    const std::string& name() const { return _name; }
    const std::string& moduleName() const { return _modname; }
    const Geom::Transform& transform() const { return _tr; }
    void setModule(const Module* m) { _m = m; }
    void print(const std::string& prefix = "") const;
};


class Module {
  friend class Netlist;
  private:
    std::string _name;
    int _leaf : 1;
    Nets _nets;
    Pins _pins;
    Instances _instances;
    std::map<const Net*, std::vector<std::pair<Instance*, std::string>>> _tmpnetpins;
    void build();
    LayerRects _obstacles;
  public:
    Module(const std::string& name, const int leaf) : _name(name), _leaf(leaf) {_instances.reserve(64);}
    ~Module();
    Instance* addInstance(const std::string& name, const std::string& mname, const Geom::Transform& tr)
    {
      _instances.emplace_back(new Instance(name, mname, tr)); 
      return _instances.back();
    }
    const std::string& name() const { return _name; }
    const Instances& instances() const { return _instances; }
    Instances& instances() { return _instances; }
    bool isLeaf() const { return _leaf ? true : false; }
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
    const Nets& nets() const { return _nets; }
    const Pins& pins() const { return _pins; }
    void print() const;

    Pin* getPin(const std::string& pn)
    {
      auto it = _pins.find(pn);
      return ((it != _pins.end()) ? it->second : nullptr);
    }
    void addObstacle(const int layer, const Geom::Rect& r) { _obstacles[layer].push_back(r); }
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
};


}
#endif
