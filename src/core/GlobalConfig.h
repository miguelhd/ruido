#pragma once

#include "core/Parameters.h"

#include <array>
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

// IEEE 802.3 CRC-32 helper shared by FM-1 NVS `PatchSnapshot` (`slot_*`) and `DeviceSettingsBlob`
// (`settings_*`) payloads so corrupted flash sectors fall back cleanly without crashing boot.
inline constexpr uint32_t computeCrc32Bytes(const uint8_t* bytes, size_t length) noexcept {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0u; i < length; ++i) {
        crc ^= static_cast<uint32_t>(bytes[i]);
        for (uint8_t b = 0u; b < 8u; ++b) {
            const uint32_t mask = 0u - (crc & 1u);
            crc = (crc >> 1u) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

// CRC-verified 16-byte device configuration payload for `zvenson/dinghy` `settings_*` NVS storage
// (`USR` partition `0xEA000 - 0xFC000`). Pure POD; zero raw SPI flash register access in `src/`.
struct DeviceSettingsBlob {
    uint32_t crc32 = 0u;
    uint8_t version = 1u;
    uint8_t activeSnapshotSlot = 0u;
    uint8_t keyboardModel = 0u;
    uint8_t configFlags = 0u;
    uint16_t masterTrimQ16 = 55705u;
    std::array<uint8_t, 6> reserved{0u, 0u, 0u, 0u, 0u, 0u};
};
static_assert(sizeof(DeviceSettingsBlob) == 16u, "DeviceSettingsBlob must be 16 bytes");
static_assert(__is_trivially_copyable(DeviceSettingsBlob),
              "DeviceSettingsBlob must remain trivially copyable");

inline DeviceSettingsBlob encodeDeviceSettings(const GlobalConfigState& config,
                                               const GlobalParams& global, KeyboardModel model,
                                               uint8_t activeSlot) noexcept {
    DeviceSettingsBlob blob{};
    blob.version = 1u;
    blob.activeSnapshotSlot = static_cast<uint8_t>(activeSlot & 3u);
    const auto rawModel = static_cast<uint8_t>(model);
    blob.keyboardModel = (rawModel < static_cast<uint8_t>(KeyboardModel::Count)) ? rawModel : 0u;
    uint8_t flags = 0u;
    if (global.monoCollapse) {
        flags |= (1u << 0u);
    }
    if (global.pitchPressureMode) {
        flags |= (1u << 1u);
    }
    if (config.muteMode == KeySwitchMode::Latching) {
        flags |= (1u << 2u);
    }
    if (config.soloMode == KeySwitchMode::Latching) {
        flags |= (1u << 3u);
    }
    if (config.snapMode == KeySwitchMode::Latching) {
        flags |= (1u << 4u);
    }
    if (config.clockSync == ClockSyncMode::Internal) {
        flags |= (1u << 5u);
    }
    if (config.gloRelease == GloReleaseMode::SnapBack) {
        flags |= (1u << 6u);
    }
    blob.configFlags = flags;
    blob.masterTrimQ16 =
        static_cast<uint16_t>(clampf(global.masterTrim, 0.0f, 1.0f) * 65535.0f + 0.5f);
    const auto* payload = reinterpret_cast<const uint8_t*>(&blob) + sizeof(uint32_t);
    blob.crc32 = computeCrc32Bytes(payload, sizeof(DeviceSettingsBlob) - sizeof(uint32_t));
    return blob;
}

inline bool verifyDeviceSettingsCrc(const DeviceSettingsBlob& blob) noexcept {
    if (blob.version != 1u) {
        return false;
    }
    const auto* payload = reinterpret_cast<const uint8_t*>(&blob) + sizeof(uint32_t);
    return blob.crc32 == computeCrc32Bytes(payload, sizeof(DeviceSettingsBlob) - sizeof(uint32_t));
}

inline bool decodeDeviceSettings(const DeviceSettingsBlob& blob, GlobalConfigState& outConfig,
                                 GlobalParams& inOutGlobal, KeyboardModel& outModel,
                                 uint8_t& outActiveSlot) noexcept {
    if (!verifyDeviceSettingsCrc(blob)) {
        return false;
    }
    outActiveSlot = static_cast<uint8_t>(blob.activeSnapshotSlot & 3u);
    outModel = (blob.keyboardModel < static_cast<uint8_t>(KeyboardModel::Count))
                   ? static_cast<KeyboardModel>(blob.keyboardModel)
                   : KeyboardModel::LiveMixer;
    inOutGlobal.monoCollapse = (blob.configFlags & (1u << 0u)) != 0u;
    inOutGlobal.pitchPressureMode = (blob.configFlags & (1u << 1u)) != 0u;
    outConfig.muteMode = ((blob.configFlags & (1u << 2u)) != 0u) ? KeySwitchMode::Latching
                                                                 : KeySwitchMode::Momentary;
    outConfig.soloMode = ((blob.configFlags & (1u << 3u)) != 0u) ? KeySwitchMode::Latching
                                                                 : KeySwitchMode::Momentary;
    outConfig.snapMode = ((blob.configFlags & (1u << 4u)) != 0u) ? KeySwitchMode::Latching
                                                                 : KeySwitchMode::Momentary;
    outConfig.clockSync =
        ((blob.configFlags & (1u << 5u)) != 0u) ? ClockSyncMode::Internal : ClockSyncMode::Auto;
    outConfig.gloRelease =
        ((blob.configFlags & (1u << 6u)) != 0u) ? GloReleaseMode::SnapBack : GloReleaseMode::Latch;
    return true;
}

} // namespace ruido
