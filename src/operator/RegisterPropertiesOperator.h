#ifndef REGISTER_PROPERTIES_OPERATOR_H
#define REGISTER_PROPERTIES_OPERATOR_H
#include "core.h"
#include <functional>

class RegisterPropertiesOperator {
  public:
    RegisterPropertiesOperator(std::function<void(int)> register_func, std::function<void(MCGAL::Mesh*)> init_func) : register_func_(register_func), init_func_(init_func) {}

    void registerProperties(int meshId);

    void initProperties(MCGAL::Mesh* mesh);

  private:
    std::function<void(int)> register_func_;
    std::function<void(MCGAL::Mesh* mesh)> init_func_;
};

#endif