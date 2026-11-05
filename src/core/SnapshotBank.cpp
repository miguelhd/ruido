#include "core/SnapshotBank.h"

#include "core/DspMath.h"

namespace ruido {

namespace {

inline size_t normalizeHeldWeights(const std::array<bool, kNumSnapshots>& held,
                                   const std::array<float, kNumSnapshots>& pressure,
                                   std::array<float, kNumSnapshots>& outWeights,
                                   float& outMaxP) noexcept {
    size_t count = 0u;
    float sumP = 0.0f;
    outMaxP = 0.0f;
    for (size_t s = 0u; s < kNumSnapshots; ++s) {
        if (held[s]) {
            ++count;
            const float p = clampf(pressure[s], 0.02f, 1.0f);
            sumP += p;
            if (p > outMaxP) {
                outMaxP = p;
            }
        }
    }
    const float invSum = (sumP > 1.0e-5f) ? (1.0f / sumP) : 0.0f;
    for (size_t s = 0u; s < kNumSnapshots; ++s) {
        outWeights[s] = held[s] ? (clampf(pressure[s], 0.02f, 1.0f) * invSum) : 0.0f;
    }
    return count;
}

} // namespace

void SnapshotBank::reset() noexcept {
    for (size_t s = 0u; s < kNumSnapshots; ++s) {
        slots_[s] = encodePatchSnapshot(makeFactorySnapshotPreset(s));
    }
    originFrame_ = extractPatchFrame(EngineControlState::makeDefault());
    slotWeights_ = {1.0f, 0.0f, 0.0f, 0.0f};
    originWeights_ = slotWeights_;
    targetWeights_ = slotWeights_;
    prevHeld_ = {false, false, false, false};
    glideProgress_ = 1.0f;
    glideDurationSec_ = 0.50f;
    momentaryBlend_ = 0.0f;
    morphPos_ = 1.0f;
    activeSlot_ = 0u;
    dirty_ = false;
    latchGliding_ = false;
    latchHybridHeld_ = false;
    momentaryActive_ = false;
}

PatchFrame
SnapshotBank::blendSlots(const std::array<float, kNumSnapshots>& normWeights) const noexcept {
    size_t domSlot = 0u;
    float domWeight = -1.0f;
    size_t nonZeroCount = 0u;
    for (size_t s = 0u; s < kNumSnapshots; ++s) {
        if (normWeights[s] > 1.0e-4f) {
            ++nonZeroCount;
            if (normWeights[s] > domWeight) {
                domWeight = normWeights[s];
                domSlot = s;
            }
        }
    }
    if (nonZeroCount <= 1u) {
        return decodePatchSnapshot(slots_[domSlot]);
    }

    std::array<PatchFrame, kNumSnapshots> decoded{};
    for (size_t s = 0u; s < kNumSnapshots; ++s) {
        if (normWeights[s] > 1.0e-4f) {
            decoded[s] = decodePatchSnapshot(slots_[s]);
        }
    }

    PatchFrame out{};
    out.circuitCoupling = 0.0f;
    for (size_t s = 0u; s < kNumSnapshots; ++s) {
        out.circuitCoupling += normWeights[s] * decoded[s].circuitCoupling;
    }

    for (size_t v = 0u; v < kNumOscillators; ++v) {
        float logF = 0.0f, logHp = 0.0f, logLp = 0.0f, logRa = 0.0f, logRb = 0.0f;
        float pw = 0.0f, drift = 0.0f, hpRes = 0.0f, lpRes = 0.0f, amp = 0.0f, pan = 0.0f;
        float shA = 0.0f, dpA = 0.0f, shB = 0.0f, dpB = 0.0f;
        bool allMuted = true;

        for (size_t s = 0u; s < kNumSnapshots; ++s) {
            const float w = normWeights[s];
            if (w <= 1.0e-4f) {
                continue;
            }
            const VoiceParams& vp = decoded[s].voices[v];
            logF += w * fastLog2(clampf(vp.frequencyHz, kAudioFreqMinHz, kAudioFreqMaxHz));
            logHp += w * fastLog2(clampf(vp.hpCutoffHz, kFilterCutoffMinHz, kFilterCutoffMaxHz));
            logLp += w * fastLog2(clampf(vp.lpCutoffHz, kFilterCutoffMinHz, kFilterCutoffMaxHz));
            logRa += w * fastLog2(clampf(vp.lfoRateHz, kLfoFreqMinHz, kLfoFreqMaxHz));
            logRb += w * fastLog2(clampf(vp.lfoBRateHz, kLfoFreqMinHz, kLfoFreqMaxHz));
            pw += w * vp.pulseWidth;
            drift += w * vp.drift;
            hpRes += w * vp.hpResonance;
            lpRes += w * vp.lpResonance;
            amp += w * (vp.muted ? 0.0f : vp.amplitude);
            pan += w * vp.pan;
            shA += w * vp.lfoShape;
            dpA += w * vp.lfoDepth;
            shB += w * vp.lfoBShape;
            dpB += w * vp.lfoBDepth;
            if (!vp.muted) {
                allMuted = false;
            }
        }

        VoiceParams& dst = out.voices[v];
        dst.frequencyHz = clampf(fastExp2(logF), kAudioFreqMinHz, kAudioFreqMaxHz);
        dst.hpCutoffHz = clampf(fastExp2(logHp), kFilterCutoffMinHz, kFilterCutoffMaxHz);
        dst.lpCutoffHz = clampf(fastExp2(logLp), kFilterCutoffMinHz, kFilterCutoffMaxHz);
        dst.lfoRateHz = clampf(fastExp2(logRa), kLfoFreqMinHz, kLfoFreqMaxHz);
        dst.lfoBRateHz = clampf(fastExp2(logRb), kLfoFreqMinHz, kLfoFreqMaxHz);
        dst.pulseWidth = clampf(pw, kPulseWidthMin, kPulseWidthMax);
        dst.drift = clampf(drift, 0.0f, 1.0f);
        dst.hpResonance = clampf(hpRes, 0.0f, 1.0f);
        dst.lpResonance = clampf(lpRes, 0.0f, 1.0f);
        dst.muted = allMuted;
        dst.amplitude = allMuted ? decoded[domSlot].voices[v].amplitude : clampf(amp, 0.0f, 1.0f);
        dst.pan = clampf(pan, -1.0f, 1.0f);
        dst.lfoShape = clampf(shA, 0.0f, 1.0f);
        dst.lfoDepth = clampf(dpA, 0.0f, 1.0f);
        dst.lfoBShape = clampf(shB, 0.0f, 1.0f);
        dst.lfoBDepth = clampf(dpB, 0.0f, 1.0f);
        dst.lfoTarget = decoded[domSlot].voices[v].lfoTarget;
        dst.lfoBTarget = decoded[domSlot].voices[v].lfoBTarget;
    }
    return out;
}

void SnapshotBank::saveSlot(uint8_t slot, EngineControlState& state) noexcept {
    const uint8_t idx = slot & 3u;
    if (momentaryActive_) {
        applyPatchFrame(originFrame_, state);
        momentaryActive_ = false;
        momentaryBlend_ = 0.0f;
    }
    slots_[idx] = encodePatchSnapshot(state);
    activeSlot_ = idx;
    dirty_ = false;
    latchGliding_ = false;
    latchHybridHeld_ = false;
    glideProgress_ = 1.0f;
    morphPos_ = 1.0f;
    slotWeights_ = {0.0f, 0.0f, 0.0f, 0.0f};
    slotWeights_[idx] = 1.0f;
    originWeights_ = slotWeights_;
    targetWeights_ = slotWeights_;
}

bool SnapshotBank::loadSlotBinary(uint8_t slot, const PatchSnapshot& snap,
                                  EngineControlState& state) noexcept {
    if (!verifyPatchSnapshotCrc(snap)) {
        return false;
    }
    const uint8_t idx = slot & 3u;
    slots_[idx] = snap;
    if (idx == activeSlot_ && !dirty_ && !isTransitioning()) {
        originFrame_ = decodePatchSnapshot(slots_[idx]);
        applyPatchFrame(originFrame_, state);
    }
    return true;
}

void SnapshotBank::selectActiveSlotImmediate(uint8_t slot, EngineControlState& state) noexcept {
    const uint8_t idx = slot & 3u;
    activeSlot_ = idx;
    dirty_ = false;
    latchGliding_ = false;
    latchHybridHeld_ = false;
    momentaryActive_ = false;
    glideProgress_ = 1.0f;
    momentaryBlend_ = 0.0f;
    morphPos_ = 1.0f;
    slotWeights_ = {0.0f, 0.0f, 0.0f, 0.0f};
    slotWeights_[idx] = 1.0f;
    originWeights_ = slotWeights_;
    targetWeights_ = slotWeights_;
    originFrame_ = decodePatchSnapshot(slots_[idx]);
    applyPatchFrame(originFrame_, state);
}

void SnapshotBank::recallSlotStepped(uint8_t slot, KnobRate rate,
                                     EngineControlState& state) noexcept {
    const uint8_t idx = slot & 3u;
    if (momentaryActive_) {
        momentaryActive_ = false;
        momentaryBlend_ = 0.0f;
    }
    originFrame_ = extractPatchFrame(state);
    originWeights_ = slotWeights_;
    targetWeights_ = {0.0f, 0.0f, 0.0f, 0.0f};
    targetWeights_[idx] = 1.0f;
    activeSlot_ = idx;
    dirty_ = false;
    latchHybridHeld_ = false;
    glideDurationSec_ = (rate == KnobRate::Fine)     ? 1.80f
                        : (rate == KnobRate::Coarse) ? 0.10f
                                                     : 0.50f;
    glideProgress_ = 0.0f;
    morphPos_ = 0.0f;
    latchGliding_ = true;
}

bool SnapshotBank::onSnapKeysChanged(const std::array<bool, kNumSnapshots>& held,
                                     const std::array<float, kNumSnapshots>& pressure,
                                     KeySwitchMode snapMode, EngineControlState& state) noexcept {
    std::array<float, kNumSnapshots> normW{};
    float maxP = 0.0f;
    const size_t heldCount = normalizeHeldWeights(held, pressure, normW, maxP);
    uint8_t domSlot = activeSlot_;
    float domP = -1.0f;
    bool anyRising = false;

    for (size_t s = 0u; s < kNumSnapshots; ++s) {
        if (held[s] && !prevHeld_[s]) {
            anyRising = true;
        }
        if (held[s]) {
            const float p = clampf(pressure[s], 0.02f, 1.0f);
            if (p > domP || !prevHeld_[s]) {
                domP = p;
                domSlot = static_cast<uint8_t>(s);
            }
        }
    }
    prevHeld_ = held;

    if (snapMode == KeySwitchMode::Momentary) {
        if (heldCount > 0u) {
            if (!momentaryActive_) {
                latchGliding_ = false;
                originFrame_ = extractPatchFrame(state);
                momentaryBlend_ = 0.0f;
                momentaryActive_ = true;
            }
            targetWeights_ = normW;
            activeSlot_ = domSlot;
            const float tgtBlend = clampf(maxP, 0.0f, 1.0f);
            if (tgtBlend > momentaryBlend_) {
                momentaryBlend_ += 0.35f * (tgtBlend - momentaryBlend_);
            }
            morphPos_ = momentaryBlend_;
            for (size_t s = 0u; s < kNumSnapshots; ++s) {
                slotWeights_[s] = normW[s] * momentaryBlend_;
            }
            applyPatchFrame(
                morphPatchFrames(originFrame_, blendSlots(targetWeights_), momentaryBlend_), state);
            return true;
        }
        return false;
    }

    // Latching mode: rising edge or multi-key pressure sculpt
    if (heldCount == 0u) {
        return false;
    }

    if (anyRising || heldCount >= 2u || latchGliding_) {
        momentaryActive_ = false;
        originFrame_ = extractPatchFrame(state);
        originWeights_ = slotWeights_;
        targetWeights_ = normW;
        activeSlot_ = domSlot;
        if (heldCount >= 2u) {
            latchHybridHeld_ = true;
            dirty_ = true;
        } else if (anyRising) {
            latchHybridHeld_ = false;
            dirty_ = false;
        }
        // Pressure accelerates glide: light touch = ~0.55s dreamy glide, 100% firm = ~0.08s snap
        glideDurationSec_ = 0.55f - 0.47f * clampf(maxP, 0.0f, 1.0f);
        if (anyRising) {
            glideProgress_ = 0.0f;
            morphPos_ = 0.0f;
        }
        latchGliding_ = true;
    }
    return false;
}

bool SnapshotBank::step(float dtSec, const std::array<bool, kNumSnapshots>& held,
                        const std::array<float, kNumSnapshots>& pressure, KeySwitchMode snapMode,
                        EngineControlState& state) noexcept {
    const float dt = clampf(dtSec, 0.0f, 0.25f);
    if (dt <= 0.0f) {
        return false;
    }

    if (momentaryActive_) {
        if (snapMode != KeySwitchMode::Momentary) {
            momentaryActive_ = false;
            momentaryBlend_ = 0.0f;
        } else {
            float maxP = 0.0f;
            std::array<float, kNumSnapshots> liveW{};
            const size_t heldCount = normalizeHeldWeights(held, pressure, liveW, maxP);
            if (heldCount > 0u) {
                targetWeights_ = liveW;
                const float alphaAtk = 1.0f - fastExp2(-1.44269504f * dt * (1.0f / 0.032f));
                momentaryBlend_ += alphaAtk * (clampf(maxP, 0.0f, 1.0f) - momentaryBlend_);
            } else {
                const float alphaRel = 1.0f - fastExp2(-1.44269504f * dt * (1.0f / 0.135f));
                momentaryBlend_ *= (1.0f - alphaRel);
                if (momentaryBlend_ <= 0.002f) {
                    momentaryBlend_ = 0.0f;
                    momentaryActive_ = false;
                    morphPos_ = dirty_ ? 0.0f : 1.0f;
                    slotWeights_ = {0.0f, 0.0f, 0.0f, 0.0f};
                    if (!dirty_) {
                        slotWeights_[activeSlot_] = 1.0f;
                    }
                    applyPatchFrame(originFrame_, state);
                    return true;
                }
            }

            morphPos_ = momentaryBlend_;
            for (size_t s = 0u; s < kNumSnapshots; ++s) {
                slotWeights_[s] = targetWeights_[s] * momentaryBlend_;
            }
            applyPatchFrame(
                morphPatchFrames(originFrame_, blendSlots(targetWeights_), momentaryBlend_), state);
            return true;
        }
    }

    if (!latchGliding_) {
        return false;
    }

    if (snapMode == KeySwitchMode::Latching) {
        float maxHeldP = 0.0f;
        size_t heldCount = 0u;
        for (size_t s = 0u; s < kNumSnapshots; ++s) {
            if (held[s]) {
                ++heldCount;
                if (pressure[s] > maxHeldP) {
                    maxHeldP = pressure[s];
                }
            }
        }
        if (heldCount == 0u && latchHybridHeld_) {
            latchHybridHeld_ = false;
            latchGliding_ = false;
            glideProgress_ = 1.0f;
            morphPos_ = 1.0f;
            return false;
        }
        if (heldCount > 0u) {
            const float pDur = 0.55f - 0.47f * clampf(maxHeldP, 0.0f, 1.0f);
            if (pDur < glideDurationSec_) {
                glideDurationSec_ = pDur;
            }
        }
    }

    const PatchFrame targetFrame = blendSlots(targetWeights_);
    const float dur = (glideDurationSec_ > 0.02f) ? glideDurationSec_ : 0.02f;
    glideProgress_ += dt / dur;
    if (glideProgress_ >= 1.0f) {
        glideProgress_ = 1.0f;
        latchGliding_ = false;
        morphPos_ = 1.0f;
        slotWeights_ = targetWeights_;
        applyPatchFrame(targetFrame, state);
        return true;
    }

    const float eased = smoothstep01(glideProgress_);
    morphPos_ = eased;
    for (size_t s = 0u; s < kNumSnapshots; ++s) {
        slotWeights_[s] = originWeights_[s] + eased * (targetWeights_[s] - originWeights_[s]);
    }
    applyPatchFrame(morphPatchFrames(originFrame_, targetFrame, eased), state);
    return true;
}

void SnapshotBank::notifyManualEdit(EngineControlState& state) noexcept {
    if (momentaryActive_) {
        originFrame_ = extractPatchFrame(state);
    }
    latchGliding_ = false;
    latchHybridHeld_ = false;
    glideProgress_ = 1.0f;
    dirty_ = true;
}

bool SnapshotBank::cancelMomentaryIfActive(EngineControlState& state) noexcept {
    prevHeld_ = {false, false, false, false};
    if (!momentaryActive_) {
        return false;
    }
    momentaryActive_ = false;
    momentaryBlend_ = 0.0f;
    morphPos_ = dirty_ ? 0.0f : 1.0f;
    slotWeights_ = {0.0f, 0.0f, 0.0f, 0.0f};
    if (!dirty_) {
        slotWeights_[activeSlot_] = 1.0f;
    }
    applyPatchFrame(originFrame_, state);
    return true;
}

void SnapshotBank::syncHud(HudState& hud) const noexcept {
    hud.snapshotSlot = activeSlot_;
    hud.snapshotWeights = slotWeights_;
    hud.morphPosition = morphPos_;
    hud.snapshotDirty = dirty_;
    hud.snapshotGliding = latchGliding_ || momentaryActive_;
}

} // namespace ruido
