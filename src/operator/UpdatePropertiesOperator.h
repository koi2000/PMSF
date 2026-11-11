#ifndef UPDATEPROPERTITY_H
#define UPDATEPROPERTITY_H

#include <memory>
#include <vector>
#include "Halfedge.h"
#include "../handler/PropertyHandleWrapper.h"

class UpdatePropertiesOperator {
  public:
    UpdatePropertiesOperator() = default;
    ~UpdatePropertiesOperator() = default;

    virtual void updatePropertity(int meshId, MCGAL::Halfedge* halfedge, const std::vector<std::unique_ptr<PropertyHandleWrapper>>& handles) = 0;
};

#endif