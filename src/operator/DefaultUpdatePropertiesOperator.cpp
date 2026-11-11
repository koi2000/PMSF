#include "DefaultUpdatePropertiesOperator.h"
#include "MeshUtils.h"
#include <memory>
#include <typeinfo>

template <typename T> static bool tryProcessHalfedgeHandle(const std::unique_ptr<PropertyHandleWrapper>& handle) {
    using HandleType = OpenMesh::HPropHandleT<T>;
    if (auto* wrapper = dynamic_cast<const PropertyHandleWrapperImpl<HandleType>*>(handle.get())) {
        const HandleType& vertexHandle = wrapper->get();
        // 处理顶点句柄
        // processVertexHandle<T>(mesh, halfedge, vertexHandle);
        return true;
    }
    return false;
}

void DefaultUpdatePropertiesOperator::updatePropertity(int meshId, MCGAL::Halfedge* halfedge, const std::vector<std::unique_ptr<PropertyHandleWrapper>>& handles) {
    for (const auto& handle : handles) {
        if (handle->isHalfedgeHandle()) {
            if (tryProcessHalfedgeHandle<bool>(handle)) {
                std::cout << "success1" << std::endl;
            } else {
                std::cout << "fail1" << std::endl;
            }
            if (tryProcessHalfedgeHandle<MCGAL::Point>(handle)) {
                std::cout << "success2" << std::endl;
            } else {
                std::cout << "fail2" << std::endl;
            }

            // handle.get();
        } else if (handle->isHalfedgeHandle()) {
            std::cout << "HalfedgeHandle" << std::endl;
        } else if (handle->isFaceHandle()) {
            std::cout << "FaceHandle" << std::endl;
        }
    }
};
