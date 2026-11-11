#ifndef DEFAULT_UPDATE_PROPERTIES_OPERATOR_H
#define DEFAULT_UPDATE_PROPERTIES_OPERATOR_H

#include <memory>
#include <vector>
#include "Halfedge.h"
#include "UpdatePropertiesOperator.h"

class DefaultUpdatePropertiesOperator : public UpdatePropertiesOperator {
  public:
  DefaultUpdatePropertiesOperator() = default;
    ~DefaultUpdatePropertiesOperator() = default;

    void updatePropertity(int meshId, MCGAL::Halfedge* halfedge, const std::vector<std::unique_ptr<PropertyHandleWrapper>>& handles) override;
};

#endif