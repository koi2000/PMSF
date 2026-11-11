#include "../common/Logutil.h"
#include "RegisterPropertiesOperator.h"

void RegisterPropertiesOperator::registerProperties(int meshId) {
    if (register_func_) {
        register_func_(meshId);
    } else {
        LOG("No function stored!");
    }
}

void RegisterPropertiesOperator::initProperties(MCGAL::Mesh* mesh) {
    if (init_func_) {
        init_func_(mesh);
    } else {
        LOG("No function stored!");
    }
}
