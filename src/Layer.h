#ifndef LAYER_H_

#include<string>

namespace RouterDB {

using namespace std; 

enum class Direction {
  HORIZONTAL,
  VERTICAL,
  ORTHOGONAL
};

class Layer {
  private:
    unsigned _gdsNo;
    std::string _name;
    float _r[3];
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
};

class ViaLayer : public Layer {
  private:
    int _space[2]; // 0 : x, 1 : y
    int _width[2]; // 0 : x, 1 : y
    std::pair<MetalLayer*, MetalLayer*> _layerPair; // first : lower, second : upper
    int _coverl[2], _coveru[2]; // 0 : low, 1 : high

};

}

#endif
