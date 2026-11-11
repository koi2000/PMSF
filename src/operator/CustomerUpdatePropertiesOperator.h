#ifndef CUSTOMER_UPDATE_PROPERTIES_OPERATOR_H
#define CUSTOMER_UPDATE_PROPERTIES_OPERATOR_H

#include <memory>
#include <vector>
#include "Halfedge.h"
#include "UpdatePropertiesOperator.h"

class CustomerUpdatePropertiesOperator : public UpdatePropertiesOperator {
  public:
    CustomerUpdatePropertiesOperator() = default;
    ~CustomerUpdatePropertiesOperator() = default;

    void updatePropertity(int meshId, MCGAL::Halfedge* halfedge, const std::vector<std::unique_ptr<PropertyHandleWrapper>>& handles) override;
};

#endif