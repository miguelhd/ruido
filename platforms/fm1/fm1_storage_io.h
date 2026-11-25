#pragma once

#include "core/RuidoController.h"
#include "fm1_led_modulator.h"
#include "fm1_panel_io.h"
#include "ruido_fm1_bridge.h"

#include <array>
#include <cstdint>
#include <cstring>

namespace ruido::fm1 {

inline constexpr uint32_t kNvsSettingsDebounceMs = 1500u;
inline constexpr uint32_t kNvsSettingsBlobIdx = 4u; // 0xF8000 (blobs 0..3 = 0xF4000..0xF7FFF)
inline constexpr uint32_t kNvsLivePatchIdx = 5u;    // 0xF9000 (blob 5 = `LIVE` drone scratchpad)

// 16-byte `DeviceSettingsBlob` journal in sector 4 (`0x020..0xFF0`, 254 slots):
inline constexpr uint32_t kSettingsJournalBaseOff = 0x20u; // After 28 B dinghy area_save record
inline constexpr uint32_t kSettingsJournalEntryBytes = 16u;
inline constexpr uint32_t kSettingsJournalMaxSlots =
    (0x1000u - kSettingsJournalBaseOff) / kSettingsJournalEntryBytes;
inline constexpr uint32_t kSettingsJournalBootCompactThreshold = 64u;

// 128-byte `PatchSnapshot` journal in sectors 0..3 (`Scenes 1-4`) & 5 (`LIVE` scratchpad):
// Each `0x80`-aligned slot (`0x100, 0x180, ..., 0xF80`, 30 slots) fits inside a single 256 B NOR
// page (~500 us page program, zero sector erases, zero audio/LED interruption).
inline constexpr uint32_t kPatchJournalBaseOff = 0x100u; // After 140 B dinghy area_save record
inline constexpr uint32_t kPatchJournalEntryBytes = 128u;
inline constexpr uint32_t kPatchJournalMaxSlots =
    (0x1000u - kPatchJournalBaseOff) / kPatchJournalEntryBytes;
inline constexpr uint32_t kPatchJournalBootCompactThreshold = 8u;

inline bool isAllFfBytes(const uint8_t* bytes, uint32_t len) noexcept {
    uint8_t acc = 0xFFu;
    for (uint32_t i = 0u; i < len; ++i) {
        acc = static_cast<uint8_t>(acc & bytes[i]);
    }
    return acc == 0xFFu;
}

inline bool bytesEqual(const void* a, const void* b, uint32_t len) noexcept {
    const auto* pa = static_cast<const uint8_t*>(a);
    const auto* pb = static_cast<const uint8_t*>(b);
    uint8_t diff = 0u;
    for (uint32_t i = 0u; i < len; ++i) {
        diff = static_cast<uint8_t>(diff | (pa[i] ^ pb[i]));
    }
    return diff == 0u;
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
    PatchSnapshot lastSavedLivePatch{};
    DeviceSettingsBlob lastSavedSettings{};
    uint32_t lastPanelTouchMs = 0u;
    uint16_t nextJournalSlot = static_cast<uint16_t>(kSettingsJournalMaxSlots);
    std::array<uint8_t, kNumSnapshots> nextPatchSlot{
        static_cast<uint8_t>(kPatchJournalMaxSlots),
        static_cast<uint8_t>(kPatchJournalMaxSlots),
        static_cast<uint8_t>(kPatchJournalMaxSlots),
        static_cast<uint8_t>(kPatchJournalMaxSlots),
    };
    uint8_t nextLiveSlot = static_cast<uint8_t>(kPatchJournalMaxSlots);
    uint8_t hasSavedSettings = 0u;
    uint8_t hasSavedLivePatch = 0u;
    uint8_t pendingSlotMask = 0u;
    uint8_t pendingSettingsSave = 0u;
    uint8_t pendingLiveSave = 0u;
    uint8_t bootLoadedMask = 0u;
    uint8_t savesOk = 0u;
    uint8_t baseBlobWritten = 0u;

    void reset() noexcept {
        lastSavedLivePatch = makeEmptyPatchSnapshot();
        lastSavedSettings = DeviceSettingsBlob{};
        lastPanelTouchMs = 0u;
        nextJournalSlot = static_cast<uint16_t>(kSettingsJournalMaxSlots);
        nextPatchSlot.fill(static_cast<uint8_t>(kPatchJournalMaxSlots));
        nextLiveSlot = static_cast<uint8_t>(kPatchJournalMaxSlots);
        hasSavedSettings = 0u;
        hasSavedLivePatch = 0u;
        pendingSlotMask = 0u;
        pendingSettingsSave = 0u;
        pendingLiveSave = 0u;
        bootLoadedMask = 0u;
        savesOk = 0u;
        baseBlobWritten = 0u;
    }

    void clearPendingAfterRestore() noexcept {
        pendingSlotMask = 0u;
        pendingSettingsSave = 0u;
        pendingLiveSave = 0u;
    }
};

inline bool scanPatchSectorBoot(uint32_t sectorId, RuidoFm1BlobLoadFn loadFn,
                                RuidoFm1TailReadFn tailReadFn, PatchSnapshot& outSnap,
                                uint32_t& outAfterLastNonFf, bool& outTailReadOk) noexcept {
    bool haveValid = (loadFn(sectorId, &outSnap, static_cast<uint32_t>(sizeof(outSnap))) == 0) &&
                     verifyPatchSnapshotCrc(outSnap);
    if (tailReadFn == nullptr) {
        outAfterLastNonFf = kPatchJournalMaxSlots;
        outTailReadOk = false;
        return haveValid;
    }
    outAfterLastNonFf = 0u;
    outTailReadOk = true;
    uint32_t ffStreak = 0u;
    PatchSnapshot cand{};
    for (uint32_t slotIdx = 0u; slotIdx < kPatchJournalMaxSlots; ++slotIdx) {
        const uint32_t offInSector = kPatchJournalBaseOff + slotIdx * kPatchJournalEntryBytes;
        if (tailReadFn(sectorId, offInSector, &cand, static_cast<uint32_t>(sizeof(cand))) != 0) {
            outAfterLastNonFf = kPatchJournalMaxSlots;
            outTailReadOk = false;
            break;
        }
        if (!isAllFfBytes(reinterpret_cast<const uint8_t*>(&cand),
                          static_cast<uint32_t>(sizeof(cand)))) {
            outAfterLastNonFf = slotIdx + 1u;
            ffStreak = 0u;
            if (verifyPatchSnapshotCrc(cand)) {
                outSnap = cand;
                haveValid = true;
            }
        } else if (++ffStreak >= 2u) {
            // Sequential append invariant: 2 consecutive untouched 0xFF slots guarantees erased
            // tail.
            break;
        }
    }
    return haveValid;
}

inline uint32_t loadBootNvsBlobs(RuidoFm1BlobLoadFn loadFn, RuidoController& ctrl,
                                 Fm1PanelState& pnl, Fm1LedModulator& leds, float latestMasterGain,
                                 Fm1NvsRuntimeState& nvs, RuidoFm1TailReadFn tailReadFn = nullptr,
                                 RuidoFm1BlobSaveFn bootCompactSaveFn = nullptr) noexcept {
    static_assert(sizeof(DeviceSettingsBlob) == kSettingsJournalEntryBytes,
                  "Settings journal slot size must match 16-byte DeviceSettingsBlob");
    static_assert(sizeof(PatchSnapshot) == kPatchJournalEntryBytes,
                  "Patch journal slot size must match 128-byte PatchSnapshot");

    uint32_t loadedMask = 0u;
    for (uint8_t s = 0u; s < kNumSnapshots; ++s) {
        PatchSnapshot snap{};
        uint32_t afterNonFf = kPatchJournalMaxSlots;
        bool tailReadOk = false;
        const bool haveValid =
            scanPatchSectorBoot(s, loadFn, tailReadFn, snap, afterNonFf, tailReadOk);
        if (haveValid && ctrl.loadSnapshotSlot(s, snap)) {
            loadedMask |= (1u << s);
            if (bootCompactSaveFn != nullptr && tailReadOk &&
                afterNonFf >= kPatchJournalBootCompactThreshold) {
                if (bootCompactSaveFn(s, &snap, static_cast<uint32_t>(sizeof(snap))) == 0) {
                    afterNonFf = 0u;
                }
            }
        } else if (!haveValid && tailReadOk && afterNonFf > 0u && bootCompactSaveFn != nullptr) {
            // Foreign non-0xFF tail bytes with no valid Ruido CRC: clean sector once at cold boot.
            const PatchSnapshot initSnap = ctrl.snapshots().slotBinary(s);
            if (bootCompactSaveFn(s, &initSnap, static_cast<uint32_t>(sizeof(initSnap))) == 0) {
                afterNonFf = 0u;
            }
        }
        nvs.nextPatchSlot[s] = static_cast<uint8_t>(afterNonFf);
    }

    DeviceSettingsBlob settings{};
    bool haveValidSettings =
        (loadFn(kNvsSettingsBlobIdx, &settings, static_cast<uint32_t>(sizeof(settings))) == 0) &&
        verifyDeviceSettingsCrc(settings);

    uint32_t afterLastNonFfSlot = kSettingsJournalMaxSlots;
    bool settingsTailReadOk = false;
    if (tailReadFn != nullptr) {
        afterLastNonFfSlot = 0u;
        settingsTailReadOk = true;
        uint8_t chunk[4u * kSettingsJournalEntryBytes]{};
        uint32_t slotIdx = 0u;
        uint32_t ffStreak = 0u;
        while (slotIdx < kSettingsJournalMaxSlots) {
            const uint32_t remSlots = kSettingsJournalMaxSlots - slotIdx;
            const uint32_t batchSlots = (remSlots > 4u) ? 4u : remSlots;
            const uint32_t offInSector =
                kSettingsJournalBaseOff + slotIdx * kSettingsJournalEntryBytes;
            if (tailReadFn(kNvsSettingsBlobIdx, offInSector, chunk,
                           batchSlots * kSettingsJournalEntryBytes) != 0) {
                settingsTailReadOk = false;
                afterLastNonFfSlot = kSettingsJournalMaxSlots;
                break;
            }
            for (uint32_t b = 0u; b < batchSlots; ++b) {
                const uint8_t* raw = chunk + b * kSettingsJournalEntryBytes;
                if (!isAllFfBytes(raw, kSettingsJournalEntryBytes)) {
                    afterLastNonFfSlot = slotIdx + b + 1u;
                    ffStreak = 0u;
                    DeviceSettingsBlob cand{};
                    std::memcpy(&cand, raw, sizeof(cand));
                    if (verifyDeviceSettingsCrc(cand)) {
                        settings = cand;
                        haveValidSettings = true;
                    }
                } else {
                    ++ffStreak;
                }
            }
            slotIdx += batchSlots;
            if (ffStreak >= 4u) {
                break;
            }
        }
    }

    if (haveValidSettings && ctrl.loadSettings(settings)) {
        loadedMask |= (1u << kNvsSettingsBlobIdx);
        if (bootCompactSaveFn != nullptr && settingsTailReadOk &&
            afterLastNonFfSlot >= kSettingsJournalBootCompactThreshold) {
            if (bootCompactSaveFn(kNvsSettingsBlobIdx, &settings,
                                  static_cast<uint32_t>(sizeof(settings))) == 0) {
                afterLastNonFfSlot = 0u;
            }
        }
    } else if (!haveValidSettings && settingsTailReadOk && afterLastNonFfSlot > 0u &&
               bootCompactSaveFn != nullptr) {
        const DeviceSettingsBlob initCfg = ctrl.exportSettings();
        if (bootCompactSaveFn(kNvsSettingsBlobIdx, &initCfg,
                              static_cast<uint32_t>(sizeof(initCfg))) == 0) {
            afterLastNonFfSlot = 0u;
        }
    }

    // Restore `LIVE` scratchpad (blob 5 @ 0xF9000) AFTER `loadSettings` so the active Home Base
    // slot stays anchored on the `50%` Scene LED while unsaved live tweaks restore seamlessly!
    // Compact any non-empty `LIVE` journal at cold boot (before Core 1 / audio / LEDs start) so
    // every session begins with all 30 erase-free journal slots available.
    PatchSnapshot liveSnap = makeEmptyPatchSnapshot();
    uint32_t afterLiveNonFf = kPatchJournalMaxSlots;
    bool liveTailReadOk = false;
    const bool haveValidLive = scanPatchSectorBoot(kNvsLivePatchIdx, loadFn, tailReadFn, liveSnap,
                                                   afterLiveNonFf, liveTailReadOk);
    if (haveValidLive) {
        loadedMask |= (1u << kNvsLivePatchIdx);
        if (isPatchSnapshotOccupied(liveSnap)) {
            (void)ctrl.loadLivePatch(liveSnap);
        }
        if (bootCompactSaveFn != nullptr && liveTailReadOk && afterLiveNonFf >= 1u) {
            if (bootCompactSaveFn(kNvsLivePatchIdx, &liveSnap,
                                  static_cast<uint32_t>(sizeof(liveSnap))) == 0) {
                afterLiveNonFf = 0u;
            }
        }
    } else if (liveTailReadOk && afterLiveNonFf > 0u && bootCompactSaveFn != nullptr) {
        const PatchSnapshot emptySnap = makeEmptyPatchSnapshot();
        if (bootCompactSaveFn(kNvsLivePatchIdx, &emptySnap,
                              static_cast<uint32_t>(sizeof(emptySnap))) == 0) {
            afterLiveNonFf = 0u;
        }
    }

    pnl.reset(latestMasterGain);
    pnl.syncMasterPot(ctrl, latestMasterGain);
    (void)ctrl.consumeSavedSlotMask();
    (void)ctrl.consumeSettingsDirty();
    (void)ctrl.consumeLivePatchDirty();
    leds.updateTargets(ctrl.leds());
    nvs.lastSavedSettings = ctrl.exportSettings();
    nvs.lastSavedLivePatch = haveValidLive ? liveSnap : makeEmptyPatchSnapshot();
    nvs.hasSavedSettings = 1u;
    nvs.hasSavedLivePatch = 1u;
    nvs.nextJournalSlot = static_cast<uint16_t>(afterLastNonFfSlot);
    nvs.nextLiveSlot = static_cast<uint8_t>(afterLiveNonFf);
    nvs.bootLoadedMask = static_cast<uint8_t>(loadedMask & 0xFFu);
    return loadedMask;
}

inline uint32_t pollAndSaveNvsBlobs(uint32_t nowMs, uint32_t forceFlush, RuidoFm1BlobSaveFn saveFn,
                                    RuidoFm1PreEraseFn preEraseFn, RuidoController& ctrl,
                                    Fm1NvsRuntimeState& nvs,
                                    RuidoFm1TailWriteFn tailWriteFn = nullptr,
                                    uint32_t availAudioBlocks = 16u) noexcept {
    nvs.pendingSlotMask = static_cast<uint8_t>(nvs.pendingSlotMask | ctrl.consumeSavedSlotMask());
    if (ctrl.consumeSettingsDirty()) {
        nvs.pendingSettingsSave = 1u;
    }
    if (ctrl.consumeLivePatchDirty() ||
        (nvs.hasSavedLivePatch != 0u &&
         isPatchSnapshotOccupied(nvs.lastSavedLivePatch) != ctrl.snapshots().isDirty())) {
        nvs.pendingLiveSave = 1u;
    }

    const bool saveSlotsNow = (nvs.pendingSlotMask != 0u);
    const auto& hud = ctrl.hud();
    const bool keysIdle = (hud.heldBlackMask == 0u) && (hud.heldWhiteMask == 0u) &&
                          !ctrl.snapshots().isTransitioning();
    const bool audioRingReady = !ctrl.state().global.masterRun || (availAudioBlocks >= 12u);
    const bool idleReady =
        keysIdle && audioRingReady && (nowMs - nvs.lastPanelTouchMs) >= kNvsSettingsDebounceMs;
    const bool saveSettingsNow =
        (nvs.pendingSettingsSave != 0u) && (saveSlotsNow || forceFlush != 0u || idleReady);
    const bool saveLiveNow =
        (nvs.pendingLiveSave != 0u) && (saveSlotsNow || forceFlush != 0u || idleReady);
    if (!saveSlotsNow && !saveSettingsNow && !saveLiveNow) {
        return 0u;
    }

    bool didPreErase = false;
    auto ensurePreErase = [&]() noexcept {
        if (!didPreErase && preEraseFn != nullptr) {
            preEraseFn();
            didPreErase = true;
        }
    };

    // Never erase a 4 KiB SPI NOR sector (~35-50 ms audio/LED stall) for background `CFG`/`LIVE`
    // auto-saves while live drone audio is running; defer compaction until `PLAY/STOP` mute,
    // explicit `SAVE + Scene`, OTA `forceFlush`, or next cold boot.
    const bool canEraseBgSector = (tailWriteFn == nullptr) || saveSlotsNow || (forceFlush != 0u) ||
                                  !ctrl.state().global.masterRun;

    uint32_t writes = 0u;
    if (saveSlotsNow) {
        for (uint8_t s = 0u; s < kNumSnapshots; ++s) {
            const uint8_t sBit = static_cast<uint8_t>(1u << s);
            if ((nvs.pendingSlotMask & sBit) == 0u) {
                continue;
            }
            const PatchSnapshot snap = ctrl.snapshots().slotBinary(s);
            bool appended = false;
            while (tailWriteFn != nullptr && nvs.nextPatchSlot[s] < kPatchJournalMaxSlots) {
                const uint32_t offInSector =
                    kPatchJournalBaseOff +
                    static_cast<uint32_t>(nvs.nextPatchSlot[s]) * kPatchJournalEntryBytes;
                ++nvs.nextPatchSlot[s];
                if (tailWriteFn(s, offInSector, &snap, static_cast<uint32_t>(sizeof(snap))) == 0) {
                    appended = true;
                    break;
                }
            }
            if (appended) {
                nvs.pendingSlotMask = static_cast<uint8_t>(nvs.pendingSlotMask & ~sBit);
                if (nvs.savesOk < 255u) {
                    ++nvs.savesOk;
                }
                ++writes;
                continue;
            }
            ensurePreErase();
            if (saveFn(s, &snap, static_cast<uint32_t>(sizeof(snap))) == 0) {
                nvs.pendingSlotMask = static_cast<uint8_t>(nvs.pendingSlotMask & ~sBit);
                nvs.nextPatchSlot[s] = 0u;
                nvs.baseBlobWritten = 1u;
                if (nvs.savesOk < 255u) {
                    ++nvs.savesOk;
                }
                ++writes;
            }
        }
    }

    if (saveSettingsNow) {
        const DeviceSettingsBlob blob = ctrl.exportSettings();
        if (nvs.hasSavedSettings != 0u && settingsEquivalentOnFm1(blob, nvs.lastSavedSettings)) {
            nvs.pendingSettingsSave = 0u;
        } else {
            bool appended = false;
            while (tailWriteFn != nullptr && nvs.nextJournalSlot < kSettingsJournalMaxSlots) {
                const uint32_t offInSector =
                    kSettingsJournalBaseOff +
                    static_cast<uint32_t>(nvs.nextJournalSlot) * kSettingsJournalEntryBytes;
                ++nvs.nextJournalSlot;
                if (tailWriteFn(kNvsSettingsBlobIdx, offInSector, &blob,
                                static_cast<uint32_t>(sizeof(blob))) == 0) {
                    appended = true;
                    break;
                }
            }
            if (appended) {
                nvs.lastSavedSettings = blob;
                nvs.hasSavedSettings = 1u;
                nvs.pendingSettingsSave = 0u;
                if (nvs.savesOk < 255u) {
                    ++nvs.savesOk;
                }
                ++writes;
                // Spread background `CFG` + `LIVE` auto-saves across separate audio-refill ticks!
                if (!saveSlotsNow && forceFlush == 0u && ctrl.state().global.masterRun) {
                    return writes;
                }
            } else if (canEraseBgSector) {
                ensurePreErase();
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
        }
    }

    if (saveLiveNow) {
        const PatchSnapshot liveBlob = ctrl.exportLivePatch();
        if (nvs.hasSavedLivePatch != 0u && bytesEqual(&liveBlob, &nvs.lastSavedLivePatch,
                                                      static_cast<uint32_t>(sizeof(liveBlob)))) {
            nvs.pendingLiveSave = 0u;
        } else {
            bool appended = false;
            while (tailWriteFn != nullptr && nvs.nextLiveSlot < kPatchJournalMaxSlots) {
                const uint32_t offInSector =
                    kPatchJournalBaseOff +
                    static_cast<uint32_t>(nvs.nextLiveSlot) * kPatchJournalEntryBytes;
                ++nvs.nextLiveSlot;
                if (tailWriteFn(kNvsLivePatchIdx, offInSector, &liveBlob,
                                static_cast<uint32_t>(sizeof(liveBlob))) == 0) {
                    appended = true;
                    break;
                }
            }
            if (appended) {
                nvs.lastSavedLivePatch = liveBlob;
                nvs.hasSavedLivePatch = 1u;
                nvs.pendingLiveSave = 0u;
                if (nvs.savesOk < 255u) {
                    ++nvs.savesOk;
                }
                ++writes;
            } else if (canEraseBgSector) {
                ensurePreErase();
                if (saveFn(kNvsLivePatchIdx, &liveBlob, static_cast<uint32_t>(sizeof(liveBlob))) ==
                    0) {
                    nvs.lastSavedLivePatch = liveBlob;
                    nvs.hasSavedLivePatch = 1u;
                    nvs.nextLiveSlot = 0u;
                    nvs.pendingLiveSave = 0u;
                    nvs.baseBlobWritten = 1u;
                    if (nvs.savesOk < 255u) {
                        ++nvs.savesOk;
                    }
                    ++writes;
                }
            }
        }
    }
    return writes;
}

} // namespace ruido::fm1
