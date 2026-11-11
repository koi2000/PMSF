/*
 * @Author: koi
 * @Description: SelectOperator的配置项，包括使用那个propertity作为error项，以及如何阻止一个半边被选择
 */
#ifndef SELECT_OPTIONS_H
#define SELECT_OPTIONS_H

#include "../core/SelectionPolicy.h"
#include "../operator/IsRemovableOperator.h"
#include "../common/HalfedgeErrorEntry.h"
#include "Mesh.h"
#include "selection/ErrorSource.h"
#include <functional>
#include <memory>
#include <vector>

/**
 * 需要传入error accessor，通过传入func的形式来做，而不是指定属性了
 */

class SelectOptions {
  public:
    SelectOptions() = default;
    virtual ~SelectOptions() = default;

    // 获取半边误差访问器
    const std::function<float(MCGAL::Halfedge*, MCGAL::Point&)>& getHalfedgeErrorAccessor() const {
        return halfedgeErrorAccessor_;
    }

    // 获取顶点误差访问器
    const std::function<float(MCGAL::Vertex*)>& getVertexErrorAccessor() const {
        return vertexErrorAccessor_;
    }

    const std::function<bool(HalfedgeErrorEntry&, MCGAL::Halfedge*)>& getEntryCustomizer() const {
        return entryCustomizer_;
    }

    const std::function<void(std::shared_ptr<MCGAL::Mesh>)>& getRegisterFunc() const {
        return register_func_;
    }

    // QEM/边折叠类算法在 eliminate 之后需要刷新被影响顶点的属性（如 Q 矩阵）。
    // 框架在 select->update(entry) 内部回调一次，参数是合并后的"幸存顶点"。
    const std::function<void(MCGAL::Vertex*)>& getOnAfterEliminate() const {
        return onAfterEliminate_;
    }

    // 获取ErrorSource
    ErrorSource getErrorSource() const {
        return errorSource_;
    }

    // 获取所有可移除算子
    const std::vector<std::shared_ptr<IsRemovableOperator>>& getIsRemovableOperators() const {
        return isRemovableOperators_;
    }

    // 设置半边误差访问器
    void setHalfedgeErrorAccessor(std::function<float(MCGAL::Halfedge*, MCGAL::Point&)> accessor) {
        halfedgeErrorAccessor_ = std::move(accessor);
    }

    void setRegisterProperties(std::function<void(std::shared_ptr<MCGAL::Mesh>)> register_func) {
        register_func_ = std::move(register_func);
    }

    // 设置顶点误差访问器
    void setVertexErrorAccessor(std::function<float(MCGAL::Vertex*)> accessor) {
        vertexErrorAccessor_ = std::move(accessor);
    }

    void setEntryCustomizer(std::function<bool(HalfedgeErrorEntry&, MCGAL::Halfedge*)> accessor) {
        entryCustomizer_ = std::move(accessor);
    }

    /// @brief 选择 / 可消除性策略（见 core/SelectionPolicy.h）。
    const pmsf::SelectionPolicy& getSelectionPolicy() const { return selectionPolicy_; }
    void setSelectionPolicy(pmsf::SelectionPolicy policy) { selectionPolicy_ = std::move(policy); }

    void setOnAfterEliminate(std::function<void(MCGAL::Vertex*)> cb) {
        onAfterEliminate_ = std::move(cb);
    }

    // 设置ErrorSource
    void setErrorSource(ErrorSource source) {
        errorSource_ = source;
    }

    // 添加可移除算子
    void addIsRemovableOperator(std::shared_ptr<IsRemovableOperator> op) {
        isRemovableOperators_.push_back(std::move(op));
    }

    class Builder {
      public:
        Builder() = default;

        Builder& withHalfedgeErrorAccessor(std::function<float(MCGAL::Halfedge*, MCGAL::Point&)> accessor) {
            halfedgeErrorAccessor_ = std::move(accessor);
            return *this;
        }

        Builder& withRegisterProperties(std::function<void(std::shared_ptr<MCGAL::Mesh>)> register_func) {
            register_func_ = std::move(register_func);
            return *this;
        }

        Builder& withEntryCustomizer(std::function<bool(HalfedgeErrorEntry&, MCGAL::Halfedge*)> entry_customizer) {
            entryCustomizer_ = std::move(entry_customizer);
            return *this;
        }

        Builder& withOnAfterEliminate(std::function<void(MCGAL::Vertex*)> cb) {
            onAfterEliminate_ = std::move(cb);
            return *this;
        }

        Builder& withVertexErrorAccessor(std::function<float(MCGAL::Vertex*)> accessor) {
            vertexErrorAccessor_ = std::move(accessor);
            return *this;
        }

        Builder& withErrorSource(ErrorSource source) {
            errorSource_ = source;
            return *this;
        }

        Builder& withIsRemovableOperator(std::shared_ptr<IsRemovableOperator> op) {
            isRemovableOperators_.push_back(std::move(op));
            return *this;
        }

        Builder& withIsRemovableOperators(const std::vector<std::shared_ptr<IsRemovableOperator>>& ops) {
            isRemovableOperators_.insert(isRemovableOperators_.end(), ops.begin(), ops.end());
            return *this;
        }

        /// @brief 选择策略。`build()` 时同时写进 SelectOptions 与可消除性算子链。
        Builder& withSelectionPolicy(pmsf::SelectionPolicy policy) {
            selectionPolicy_ = std::move(policy);
            return *this;
        }

        std::shared_ptr<SelectOptions> build() {
            auto options = std::make_shared<SelectOptions>();
            options->setHalfedgeErrorAccessor(std::move(halfedgeErrorAccessor_));
            options->setVertexErrorAccessor(std::move(vertexErrorAccessor_));
            options->setErrorSource(errorSource_);
            options->setRegisterProperties(register_func_);
            options->setEntryCustomizer(entryCustomizer_);
            options->setOnAfterEliminate(onAfterEliminate_);
            options->setSelectionPolicy(selectionPolicy_);
            for (auto& op : isRemovableOperators_) {
                // 算子挂在链上、拿不到 SelectOptions，所以在这里逐个下发。
                if (op) op->setSelectionPolicy(selectionPolicy_);
                options->addIsRemovableOperator(std::move(op));
            }
            return options;
        }

        Builder& reset() {
            halfedgeErrorAccessor_ = nullptr;
            vertexErrorAccessor_ = nullptr;
            entryCustomizer_ = nullptr;
            onAfterEliminate_ = nullptr;
            isRemovableOperators_.clear();
            return *this;
        }

      private:
        ErrorSource errorSource_{};
        std::function<bool(HalfedgeErrorEntry&, MCGAL::Halfedge*)> entryCustomizer_;
        std::function<float(MCGAL::Halfedge*, MCGAL::Point&)> halfedgeErrorAccessor_;
        std::function<float(MCGAL::Vertex*)> vertexErrorAccessor_;
        std::function<void(std::shared_ptr<MCGAL::Mesh>)> register_func_;
        std::function<void(MCGAL::Vertex*)> onAfterEliminate_;
        std::vector<std::shared_ptr<IsRemovableOperator>> isRemovableOperators_;
        pmsf::SelectionPolicy selectionPolicy_;
    };

    static Builder builder() {
        return Builder{};
    }

  private:
    ErrorSource errorSource_{};
    std::function<bool(HalfedgeErrorEntry&, MCGAL::Halfedge*)> entryCustomizer_;
    std::function<float(MCGAL::Halfedge*, MCGAL::Point&)> halfedgeErrorAccessor_;
    std::function<float(MCGAL::Vertex*)> vertexErrorAccessor_;
    std::function<void(std::shared_ptr<MCGAL::Mesh>)> register_func_;
    std::function<void(MCGAL::Vertex*)> onAfterEliminate_;
    std::vector<std::shared_ptr<IsRemovableOperator>> isRemovableOperators_;
    pmsf::SelectionPolicy selectionPolicy_;
};

#endif