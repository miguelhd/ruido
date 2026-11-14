#pragma once

#include "core/RuidoController.h"
#include "fm1_led_modulator.h"
#include "fm1_panel_io.h"
#include "ruido_fm1_bridge.h"

#include <cstdint>
#include <cstring>

namespace ruido::fm1 {

inline constexpr uint32_t kNvsSettingsDebounceMs = 1500u;
inline constexpr uint32_t kNvsSettingsBlobIdx = 4u; // 0xF8000 (blobs 0..3 = 0xF4000..0xF7FFF)
inline constexpr uint32_t kSettingsJournalBaseOff = 0x20u; // After 28 B dinghy area_save record
inline constexpr uint32_t kSettingsJournalEntryBytes = 16u;
inline constexpr uint32_t kSettingsJournalMaxSlots =
    (0x1000u - kSettingsJournalBaseOff) /
    kSettingsJournalEntryBytes; // 254 slots (0xF8020..0xF8FF0)
inline constexpr uint32_t kSettingsJournalBootCompactThreshold = 64u;

inline bool isAllFf16(const uint8_t* bytes) noexcept {
    uint8_t acc = 0xFFu;
    for (uint32_t i = 0u; i < kSettingsJournalEntryBytes; ++i) {
        acc = static_cast<uint8_t>(acc & bytes[i]);
    }
    return acc == 0xFFu;
}

// Compare all persisted fields consumed by `decodeDeviceSettings` on FM-1 (where physical PB6
// `MASTER` pot governs `masterTrim` directly).
inline bool settingsEquivalentOnFm1(const DeviceSettingsBlob& a,
                                    const DeviceSettingsBlob& b) noexcept {
    return a.version == b.version && a.activeSnapshotSlot == b.activeSnapshotSlot &&
           a.keyboardModel == b.keyboardModel && a.configFlags == b.configFlags &&
           a.internalBpmTenths == b.internalBpmTenths && a.midiChannel == b.midiChannel &&
           a.pressureTouchPlus1 == b.pressureTouchPlus1;
}

struct Fm1NvsRuntimeState {
    DeviceSettingsBlob lastSavedSettings{};
    uint32_t lastPanelTouchMs = 0u;
    uint16_t nextJournalSlot = static_cast<uint16_t>(kSettingsJournalMaxSlots);
    uint8_t hasSavedSettings = 0u;
    uint8_t pendingSlotMask = 0u;
    uint8_t pendingSettingsSave = 0u;
    uint8_t bootLoadedMask = 0u;
    uint8_t savesOk = 0u;
    uint8_t baseBlobWritten = 0u;

    void reset() noexcept {
        lastSavedSettings = DeviceSettingsBlob{};
        lastPanelTouchMs = 0u;
        nextJournalSlot = static_cast<uint16_t>(kSettingsJournalMaxSlots);
        hasSavedSettings = 0u;
        pendingSlotMask = 0u;
        pendingSettingsSave = 0u;
        bootLoadedMask = 0u;
        savesOk = 0u;
        baseBlobWritten = 0u;
    }

    void clearPendingAfterRestore() noexcept {
        pendingSlotMask = 0u;
        pendingSettingsSave = 0u;
    }

    uint32_t takeBaseBlobWritten() noexcept {
        const uint32_t v = baseBlobWritten;
        baseBlobWritten = 0u;
        return v;
    }
};

inline uint32_t loadBootNvsBlobs(RuidoFm1BlobLoadFn loadFn, RuidoController& ctrl,
                                 Fm1PanelState& pnl, Fm1LedModulator& leds, float latestMasterGain,
                                 Fm1NvsRuntimeState& nvs,
                                 RuidoFm1SettingsTailReadFn tailReadFn = nullptr,
                                 RuidoFm1BlobSaveFn bootCompactSaveFn = nullptr) noexcept {
    static_assert(sizeof(DeviceSettingsBlob) == kSettingsJournalEntryBytes,
                  "Settings journal slot size must match 16-byte DeviceSettingsBlob");
    uint32_t loadedMask = 0u;
    for (uint8_t s = 0u; s < kNumSnapshots; ++s) {
        PatchSnapshot snap{};
        if (loadFn(s, &snap, static_cast<uint32_t>(sizeof(snap))) == 0 &&
            ctrl.loadSnapshotSlot(s, snap)) {
            loadedMask |= (1u << s);
        }
    }

    DeviceSettingsBlob settings{};
    bool haveValidSettings =
        (loadFn(kNvsSettingsBlobIdx, &settings, static_cast<uint32_t>(sizeof(settings))) == 0) &&
        verifyDeviceSettingsCrc(settings);

    uint32_t afterLastNonFfSlot = kSettingsJournalMaxSlots;
    if (tailReadFn != nullptr) {
        afterLastNonFfSlot = 0u;
        bool readFailed = false;
        uint8_t chunk[4u * kSettingsJournalEntryBytes]{};
        uint32_t slotIdx = 0u;
        while (slotIdx < kSettingsJournalMaxSlots) {
            const uint32_t remSlots = kSettingsJournalMaxSlots - slotIdx;
            const uint32_t batchSlots = (remSlots > 4u) ? 4u : remSlots;
            const uint32_t offInSector =
                kSettingsJournalBaseOff + slotIdx * kSettingsJournalEntryBytes;
            if (tailReadFn(offInSector, chunk, batchSlots * kSettingsJournalEntryBytes) != 0) {
                readFailed = true;
                break;
            }
            for (uint32_t b = 0u; b < batchSlots; ++b) {
                const uint8_t* raw = chunk + b * kSettingsJournalEntryBytes;
                if (!isAllFf16(raw)) {
                    afterLastNonFfSlot = slotIdx + b + 1u;
                    DeviceSettingsBlob cand{};
                    std::memcpy(&cand, raw, sizeof(cand));
                    if (verifyDeviceSettingsCrc(cand)) {
                        settings = cand;
                        haveValidSettings = true;
                    }
                }
            }
            slotIdx += batchSlots;
        }
        if (readFailed) {
            afterLastNonFfSlot = kSettingsJournalMaxSlots;
        }
    }

    if (haveValidSettings && ctrl.loadSettings(settings)) {
        loadedMask |= (1u << kNvsSettingsBlobIdx);
        if (bootCompactSaveFn != nullptr &&
            afterLastNonFfSlot >= kSettingsJournalBootCompactThreshold) {
            if (bootCompactSaveFn(kNvsSettingsBlobIdx, &settings,
                                  static_cast<uint32_t>(sizeof(settings))) == 0) {
                afterLastNonFfSlot = 0u;
            }
        }
    }

    pnl.reset(latestMasterGain);
    pnl.syncMasterPot(ctrl, latestMasterGain);
    (void)ctrl.consumeSavedSlotMask();
    (void)ctrl.consumeSettingsDirty();
    leds.updateTargets(ctrl.leds());
    nvs.lastSavedSettings = ctrl.exportSettings();
    nvs.hasSavedSettings = 1u;
    nvs.nextJournalSlot = static_cast<uint16_t>(afterLastNonFfSlot);
    nvs.bootLoadedMask = static_cast<uint8_t>(loadedMask & 0xFFu);
    return loadedMask;
}

inline uint32_t pollAndSaveNvsBlobs(uint32_t nowMs, uint32_t forceFlush, RuidoFm1BlobSaveFn saveFn,
                                    RuidoFm1PreEraseFn preEraseFn, RuidoController& ctrl,
                                    Fm1NvsRuntimeState& nvs,
                                    RuidoFm1SettingsTailWriteFn tailWriteFn = nullptr) noexcept {
    nvs.pendingSlotMask = static_cast<uint8_t>(nvs.pendingSlotMask | ctrl.consumeSavedSlotMask());
    if (ctrl.consumeSettingsDirty()) {
        nvs.pendingSettingsSave = 1u;
    }
    const bool saveSlotsNow = (nvs.pendingSlotMask != 0u);
    const auto& hud = ctrl.hud();
    const bool keysIdle = (hud.heldBlackMask == 0u) && (hud.heldWhiteMask == 0u);
    const bool saveSettingsNow =
        (nvs.pendingSettingsSave != 0u) &&
        (saveSlotsNow || forceFlush != 0u ||
         (keysIdle && (nowMs - nvs.lastPanelTouchMs) >= kNvsSettingsDebounceMs));
    if (!saveSlotsNow && !saveSettingsNow) {
        return 0u;
    }

    DeviceSettingsBlob blob{};
    if (saveSettingsNow) {
        blob = ctrl.exportSettings();
        if (!saveSlotsNow && nvs.hasSavedSettings != 0u &&
            settingsEquivalentOnFm1(blob, nvs.lastSavedSettings)) {
            nvs.pendingSettingsSave = 0u;
            return 0u;
        }
    }

    // Fast live path: append 16-byte CRC-verified `DeviceSettingsBlob` into an erased `0xFF` slot
    // in `0xF8020..0xF8FF0` (~150 us page program, zero sector erase, zero audio/LED interruption).
    if (!saveSlotsNow && forceFlush == 0u && saveSettingsNow && tailWriteFn != nullptr &&
        nvs.nextJournalSlot < kSettingsJournalMaxSlots) {
        const uint32_t offInSector =
            kSettingsJournalBaseOff +
            static_cast<uint32_t>(nvs.nextJournalSlot) * kSettingsJournalEntryBytes;
        if (tailWriteFn(offInSector, &blob, static_cast<uint32_t>(sizeof(blob))) == 0) {
            nvs.lastSavedSettings = blob;
            nvs.hasSavedSettings = 1u;
            ++nvs.nextJournalSlot;
            nvs.pendingSettingsSave = 0u;
            nvs.baseBlobWritten = 0u;
            if (nvs.savesOk < 255u) {
                ++nvs.savesOk;
            }
            return 1u;
        }
    }

    if (preEraseFn != nullptr) {
        preEraseFn();
    }
    uint32_t writes = 0u;
    if (saveSlotsNow) {
        for (uint8_t s = 0u; s < kNumSnapshots; ++s) {
            const uint8_t sBit = static_cast<uint8_t>(1u << s);
            if ((nvs.pendingSlotMask & sBit) == 0u) {
                continue;
            }
            const PatchSnapshot snap = ctrl.snapshots().slotBinary(s);
            if (saveFn(s, &snap, static_cast<uint32_t>(sizeof(snap))) == 0) {
                nvs.pendingSlotMask = static_cast<uint8_t>(nvs.pendingSlotMask & ~sBit);
                nvs.baseBlobWritten = 1u;
                if (nvs.savesOk < 255u) {
                    ++nvs.savesOk;
                }
                ++writes;
            }
        }
    }
    if (saveSettingsNow) {
        if (saveFn(kNvsSettingsBlobIdx, &blob, static_cast<uint32_t>(sizeof(blob))) == 0) {
            nvs.lastSavedSettings = blob;
            nvs.hasSavedSettings = 1u;
            nvs.nextJournalSlot = 0u;
            nvs.pendingSettingsSave = 0u;
            nvs.baseBlobWritten = 1u;
            if (nvs.savesOk < 255u) {
                ++nvs.savesOk;
            }
            ++writes;
        }
    }
    return writes;
}

} // namespace ruido::fm1
