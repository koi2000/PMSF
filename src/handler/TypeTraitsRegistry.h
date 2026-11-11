#ifndef TYPE_TRAITS_REGISTRY_H
#define TYPE_TRAITS_REGISTRY_H


#include <stdexcept>
#include <typeindex>
#include <typeinfo>
#include <unordered_map>

class TypeTraitsRegistry {
  public:
    struct Traits {
        bool is_vertex_handle = false;
        bool is_halfedge_handle = false;
        bool is_face_handle = false;
        std::type_index entity_type;

        Traits() : entity_type(typeid(void)) {}
        
        // Traits(bool vertex, bool halfedge, bool face, std::type_index et = typeid(void))
        //     : is_vertex_handle(vertex),
        //       is_halfedge_handle(halfedge),
        //       is_face_handle(face),
        //       entity_type(et) {}
    };

    template <typename HandleType> static void registerTraits(Traits traits) {
        instance().traits_[typeid(HandleType)] = traits;
    }

    template <typename HandleType> static const Traits& getTraits() {
        return getTraits(typeid(HandleType));
    }

    static const Traits& getTraits(const std::type_info& type) {
        auto it = instance().traits_.find(type);
        if (it == instance().traits_.end()) {
            throw std::runtime_error("Type not registered in traits registry: " + std::string(type.name()));
        }
        return it->second;
    }

    template <typename HandleType> static bool isRegistered() {
        return isRegistered(typeid(HandleType));
    }

    static bool isRegistered(const std::type_info& type) {
        return instance().traits_.find(type) != instance().traits_.end();
    }

  private:
    static TypeTraitsRegistry& instance() {
        static TypeTraitsRegistry registry;
        return registry;
    }

    std::unordered_map<std::type_index, Traits> traits_;
};

#endif