#pragma once

#include "core/Parameters.h"

#include <cstddef>
#include <cstdint>

namespace ruido {

// Release behavior when letting go of the momentary keybed GLO switch (GLOBAL control).
enum class GloReleaseMode : uint8_t {
    Latch = 0,   // Keep multi-voice parameter offsets on GLO release (default)
    SnapBack = 1 // Temporary performance twist: restore pre-GLO voice state on release
};

// Scalable descriptor IDs for the right-panel GLO vertical GLOBAL CONFIG list menu.
enum class ConfigItemId : uint8_t { GloRelease = 0, OutputRouting = 1, Count = 2 };

inline constexpr size_t kNumGlobalConfigItems = static_cast<size_t>(ConfigItemId::Count);

struct GlobalConfigState {
    GloReleaseMode gloRelease = GloReleaseMode::Latch;
};

constexpr const char* globalConfigItemLabel(ConfigItemId id) noexcept {
    switch (id) {
    case ConfigItemId::GloRelease:
        return "GLO RELEASE";
    case ConfigItemId::OutputRouting:
        return "OUTPUT ROUTING";
    case ConfigItemId::Count:
        break;
    }
    return "";
}

constexpr int32_t globalConfigMaxIndex(ConfigItemId id) noexcept {
    switch (id) {
    case ConfigItemId::GloRelease:
        return 1;
    case ConfigItemId::OutputRouting:
        return 1;
    case ConfigItemId::Count:
        break;
    }
    return 0;
}

constexpr int32_t globalConfigGetIndex(ConfigItemId id, const GlobalConfigState& cfg,
                                       const GlobalParams& global) noexcept {
    switch (id) {
    case ConfigItemId::GloRelease:
        return (cfg.gloRelease == GloReleaseMode::SnapBack) ? 1 : 0;
    case ConfigItemId::OutputRouting:
        return global.monoCollapse ? 1 : 0;
    case ConfigItemId::Count:
        break;
    }
    return 0;
}

inline bool globalConfigSetIndex(ConfigItemId id, int32_t idx, GlobalConfigState& cfg,
                                 GlobalParams& global) noexcept {
    switch (id) {
    case ConfigItemId::GloRelease: {
        const auto next = (idx > 0) ? GloReleaseMode::SnapBack : GloReleaseMode::Latch;
        const bool changed = (cfg.gloRelease != next);
        cfg.gloRelease = next;
        return changed;
    }
    case ConfigItemId::OutputRouting: {
        const bool nextMono = (idx > 0);
        const bool changed = (global.monoCollapse != nextMono);
        global.monoCollapse = nextMono;
        return changed;
    }
    case ConfigItemId::Count:
        break;
    }
    return false;
}

constexpr const char* globalConfigValueLabel(ConfigItemId id, const GlobalConfigState& cfg,
                                             const GlobalParams& global) noexcept {
    switch (id) {
    case ConfigItemId::GloRelease:
        return (cfg.gloRelease == GloReleaseMode::SnapBack) ? "SNAP BACK" : "LATCH";
    case ConfigItemId::OutputRouting:
        return global.monoCollapse ? "DUAL MONO" : "STEREO";
    case ConfigItemId::Count:
        break;
    }
    return "";
}

} // namespace ruido
