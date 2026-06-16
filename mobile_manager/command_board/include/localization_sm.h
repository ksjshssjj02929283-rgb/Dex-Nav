// ----------------------------------------------------------------------------
//
// Copyright (c) 2021--2026 DexForce Technology Co., Ltd.
//
// All Rights Reserved.
//
//    Authors: Li Yao <liyao2@dexforce.top>
// ----------------------------------------------------------------------------

#pragma once

#include <glog/logging.h>

#include <functional>
#include <memory>
#include <mutex>

#include "fsm/localization_state_sm.h"

namespace command_board {

using Callback = std::function<void()>;

class Localization_SM {
public:
    explicit Localization_SM(Callback on_success, Callback on_fail)
        : _fsm(*this), success_cb_(std::move(on_success)), fail_cb_(std::move(on_fail)) {
        _fsm.enterStartState();
    }

    virtual ~Localization_SM() = default;

    void LocalizationFailed() {
        std::lock_guard<std::mutex> lock(fsm_mutex_);
        try {
            _fsm.LocalizationFailed();
        } catch (const std::exception& e) {
            LOG(ERROR) << "LocalizationFailed event handling error." << e.what();
        }
    }
    void LocalizationSuccess() {
        std::lock_guard<std::mutex> lock(fsm_mutex_);
        try {
            _fsm.LocalizationSuccess();
        } catch (const std::exception& e) {
            LOG(ERROR) << "LocalizationSuccess event handling error." << e.what();
        }
    }

    void OnLocalizationFailed() {
        // Handle localization failure
        LOG(INFO) << "Localization failed.";
        if (fail_cb_) fail_cb_();
    }
    void OnLocalizationSuccess() {
        // Handle localization success
        LOG(INFO) << "Localization successful.";
        if (success_cb_) success_cb_();
    }

    localization_stateContext& getFsm() { return _fsm; }

private:
    localization_stateContext _fsm;
    Callback success_cb_;
    Callback fail_cb_;
    std::mutex fsm_mutex_;
};

}  // namespace command_board
