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

// Action mode for console MUTE, SOLO, and SNAP keys in GLOBAL CONFIG.
enum class KeySwitchMode : uint8_t {
    Momentary = 0, // Active only while key is held
    Latching = 1   // Key press latches state / recalls slot
};

// External MIDI System Real-Time clock sync mode (TRS, USB-C, and BLE-MIDI).
enum class ClockSyncMode : uint8_t {
    Auto = 0,     // Lock automatically to incoming 24-PPQN MIDI clock; 120 BPM fallback
    Internal = 1, // Ignore incoming MIDI clock and run on internal 120 BPM master clock
};

// Scalable descriptor IDs for the right-panel GLO vertical GLOBAL CONFIG list menu.
// (4-voice GLO macro & GLO RELEASE are tabled pending post-playtest roadmap evaluation.)
enum class ConfigItemId : uint8_t {
    OutputRouting = 0,
    MuteMode = 1,
    SoloMode = 2,
    SnapMode = 3,
    ClockSync = 4,
    Count = 5
};

inline constexpr size_t kNumGlobalConfigItems = static_cast<size_t>(ConfigItemId::Count);

struct GlobalConfigState {
    GloReleaseMode gloRelease = GloReleaseMode::Latch;
    KeySwitchMode muteMode = KeySwitchMode::Momentary;
    KeySwitchMode soloMode = KeySwitchMode::Momentary;
    KeySwitchMode snapMode = KeySwitchMode::Latching;
    ClockSyncMode clockSync = ClockSyncMode::Auto;
};

constexpr const char* globalConfigItemLabel(ConfigItemId id) noexcept {
    switch (id) {
    case ConfigItemId::OutputRouting:
        return "OUTPUT ROUTING";
    case ConfigItemId::MuteMode:
        return "MUTE MODE";
    case ConfigItemId::SoloMode:
        return "SOLO MODE";
    case ConfigItemId::SnapMode:
        return "SCENE MODE";
    case ConfigItemId::ClockSync:
        return "MIDI CLOCK";
    case ConfigItemId::Count:
        break;
    }
    return "";
}

constexpr int32_t globalConfigMaxIndex(ConfigItemId id) noexcept {
    switch (id) {
    case ConfigItemId::OutputRouting:
    case ConfigItemId::MuteMode:
    case ConfigItemId::SoloMode:
    case ConfigItemId::SnapMode:
    case ConfigItemId::ClockSync:
        return 1;
    case ConfigItemId::Count:
        break;
    }
    return 0;
}

constexpr int32_t globalConfigGetIndex(ConfigItemId id, const GlobalConfigState& config,
                                       const GlobalParams& global) noexcept {
    switch (id) {
    case ConfigItemId::OutputRouting:
        return global.monoCollapse ? 1 : 0;
    case ConfigItemId::MuteMode:
        return (config.muteMode == KeySwitchMode::Latching) ? 1 : 0;
    case ConfigItemId::SoloMode:
        return (config.soloMode == KeySwitchMode::Latching) ? 1 : 0;
    case ConfigItemId::SnapMode:
        return (config.snapMode == KeySwitchMode::Latching) ? 1 : 0;
    case ConfigItemId::ClockSync:
        return (config.clockSync == ClockSyncMode::Auto) ? 1 : 0;
    case ConfigItemId::Count:
        break;
    }
    return 0;
}

inline bool globalConfigSetIndex(ConfigItemId id, int32_t idx, GlobalConfigState& config,
                                 GlobalParams& global) noexcept {
    switch (id) {
    case ConfigItemId::OutputRouting: {
        const bool nextMono = (idx > 0);
        const bool changed = (global.monoCollapse != nextMono);
        global.monoCollapse = nextMono;
        return changed;
    }
    case ConfigItemId::MuteMode: {
        const KeySwitchMode next = (idx > 0) ? KeySwitchMode::Latching : KeySwitchMode::Momentary;
        const bool changed = (config.muteMode != next);
        config.muteMode = next;
        return changed;
    }
    case ConfigItemId::SoloMode: {
        const KeySwitchMode next = (idx > 0) ? KeySwitchMode::Latching : KeySwitchMode::Momentary;
        const bool changed = (config.soloMode != next);
        config.soloMode = next;
        return changed;
    }
    case ConfigItemId::SnapMode: {
        const KeySwitchMode next = (idx > 0) ? KeySwitchMode::Latching : KeySwitchMode::Momentary;
        const bool changed = (config.snapMode != next);
        config.snapMode = next;
        return changed;
    }
    case ConfigItemId::ClockSync: {
        const ClockSyncMode next = (idx > 0) ? ClockSyncMode::Auto : ClockSyncMode::Internal;
        const bool changed = (config.clockSync != next);
        config.clockSync = next;
        return changed;
    }
    case ConfigItemId::Count:
        break;
    }
    return false;
}

constexpr const char* globalConfigValueLabel(ConfigItemId id, const GlobalConfigState& config,
                                             const GlobalParams& global) noexcept {
    switch (id) {
    case ConfigItemId::OutputRouting:
        return global.monoCollapse ? "MONO" : "STEREO";
    case ConfigItemId::MuteMode:
        return (config.muteMode == KeySwitchMode::Latching) ? "LATCHING" : "MOMENTARY";
    case ConfigItemId::SoloMode:
        return (config.soloMode == KeySwitchMode::Latching) ? "LATCHING" : "MOMENTARY";
    case ConfigItemId::SnapMode:
        return (config.snapMode == KeySwitchMode::Latching) ? "LATCHING" : "MOMENTARY";
    case ConfigItemId::ClockSync:
        return (config.clockSync == ClockSyncMode::Internal) ? "OFF" : "AUTO";
    case ConfigItemId::Count:
        break;
    }
    return "";
}

} // namespace ruido
