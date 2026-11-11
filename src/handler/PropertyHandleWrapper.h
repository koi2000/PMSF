#ifndef PROPERTY_HANDLE_WRAPPER_H
#define PROPERTY_HANDLE_WRAPPER_H

#include <memory>
#include <typeindex>
#include "TypeTraitsRegistry.h"

class PropertyHandleWrapper {
  public:
    virtual ~PropertyHandleWrapper() = default;

    virtual std::type_index handleType() const = 0;
    virtual std::type_index entityType() const = 0;
    virtual bool isVertexHandle() const = 0;
    virtual bool isHalfedgeHandle() const = 0;
    virtual bool isFaceHandle() const = 0;
    virtual std::unique_ptr<PropertyHandleWrapper> clone() const = 0;
};

template <typename HandleType> 
class PropertyHandleWrapperImpl : public PropertyHandleWrapper {
    HandleType handle_;

  public:
    explicit PropertyHandleWrapperImpl(const HandleType& handle) : handle_(handle) {}

    std::type_index handleType() const override {
        return typeid(HandleType);
    }

    std::type_index entityType() const override {
        return TypeTraitsRegistry::getTraits<HandleType>().entity_type;
    }

    bool isVertexHandle() const override {
        return TypeTraitsRegistry::getTraits<HandleType>().is_vertex_handle;
    }

    bool isHalfedgeHandle() const override {
        return TypeTraitsRegistry::getTraits<HandleType>().is_halfedge_handle;
    }

    bool isFaceHandle() const override {
        return TypeTraitsRegistry::getTraits<HandleType>().is_face_handle;
    }

    std::unique_ptr<PropertyHandleWrapper> clone() const override {
        return std::make_unique<PropertyHandleWrapperImpl>(handle_);
    }

    const HandleType& get() const {
        return handle_;
    }
};

#endif