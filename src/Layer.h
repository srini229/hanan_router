#ifndef LAYER_H_
#define LAYER_H_

#include <string>
#include <vector>
#include <iostream>

namespace DRC {

using namespace std; 

enum class Direction {
  HORIZONTAL,
  VERTICAL,
  ORTHOGONAL
};

class Layer {
  private:
    const int _gdsNo;
    const std::string _name;
    const float _r[3]; // mean, -3\sigma, 3\sigma
  public:
    Layer(const int gdsNo, const std::string& name, const float mur, const float lr, const float ur)
      : _gdsNo(gdsNo), _name(name), _r{mur, lr, ur} {}
    int gdsNo() const { return _gdsNo; }
    const std::string& name() const { return _name; }
    float meanR() const { return _r[0]; }
    ~Layer()
    {
      std::cout << "layer : " << _name << ' ' << _gdsNo << " {" << _r[0] << ',' << _r[1] << ',' << _r[2] << "}\n";
    }
};

class gdsDatatype {
  public:
    int _draw{0}, _pin{0}, _label{0}, _blockage{0};
};

class MetalLayer : public Layer {
  private:
    int _pitch, _width, _minL, _maxL;
    int _e2e, _offset;
    float _c[3], _cc[3];
    Direction _dir;
  public:
    MetalLayer(const int gdsNo, const std::string& name, const float mur, const float lr, const float ur)
      : Layer(gdsNo, name, mur, lr, ur), _pitch(0), _width(0), _minL(0), _maxL(0), _e2e(0), _offset(0),
    _c{0,0,0}, _cc{0, 0, 0}, _dir(Direction::ORTHOGONAL) {}
    void setPitch(const int p) {_pitch = p;}
};
typedef std::vector<MetalLayer*> MetalLayers;

class ViaLayer : public Layer {
  private:
    int _space[2]; // 0 : x, 1 : y
    int _width[2]; // 0 : x, 1 : y
    std::pair<MetalLayer*, MetalLayer*> _layerPair; // first : lower, second : upper
    int _coverl[2], _coveru[2]; // 0 : low, 1 : high

};
typedef std::vector<ViaLayer*> ViaLayers;

class LayerInfo {
  private:
    MetalLayers _mlayers;
    ViaLayers _vlayers;
    std::vector<ViaLayer*> _vldn, _vlup;
  public:
    LayerInfo(const std::string& lj);
    void print() const;
    ~LayerInfo();
};

}

#endif
