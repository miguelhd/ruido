#include "core/SnapshotBank.h"

#include "core/DspMath.h"

namespace ruido {

namespace {

inline size_t normalizeHeldWeights(const std::array<bool, kNumSnapshots>& held,
                                   const std::array<float, kNumSnapshots>& pressure,
                                   uint8_t occMask, std::array<float, kNumSnapshots>& outWeights,
                                   float& outMaxP) noexcept {
    size_t count = 0u;
    float sumP = 0.0f;
    outMaxP = 0.0f;
    for (size_t s = 0u; s < kNumSnapshots; ++s) {
        if (held[s] && ((occMask & (1u << s)) != 0u)) {
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
        const bool active = held[s] && ((occMask & (1u << s)) != 0u);
        outWeights[s] = active ? (clampf(pressure[s], 0.02f, 1.0f) * invSum) : 0.0f;
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
    knobMorphPos_ = 0.0f;
    activeSlot_ = 0u;
    dirty_ = false;
    latchGliding_ = false;
    latchHybridHeld_ = false;
    momentaryActive_ = false;
    knobMorphActive_ = false;
}

void SnapshotBank::syncKnobMorphAnchor() noexcept {
    knobMorphActive_ = false;
    std::array<uint8_t, kNumSnapshots> occSlots{};
    int32_t occCount = 0;
    for (uint8_t s = 0u; s < kNumSnapshots; ++s) {
        if (isSlotOccupied(s)) {
            occSlots[static_cast<size_t>(occCount++)] = s;
        }
    }
    if (occCount <= 1) {
        knobMorphPos_ = 0.0f;
        return;
    }
    int32_t ord = 0;
    for (int32_t i = 0; i < occCount; ++i) {
        if (occSlots[static_cast<size_t>(i)] == activeSlot_) {
            ord = i;
            break;
        }
        if (occSlots[static_cast<size_t>(i)] < activeSlot_) {
            ord = i;
        }
    }
    knobMorphPos_ = static_cast<float>(ord) / static_cast<float>(occCount - 1);
}

uint8_t SnapshotBank::occupiedMask() const noexcept {
    uint8_t mask = 0u;
    for (size_t s = 0u; s < kNumSnapshots; ++s) {
        if (isSlotOccupied(s)) {
            mask |= static_cast<uint8_t>(1u << s);
        }
    }
    return mask;
}

PatchFrame
SnapshotBank::blendSlots(const std::array<float, kNumSnapshots>& normWeights) const noexcept {
    size_t domSlot = 0u;
    float domWeight = -1.0f;
    size_t nonZeroCount = 0u;
    for (size_t s = 0u; s < kNumSnapshots; ++s) {
        if (isSlotOccupied(s) && normWeights[s] > 1.0e-4f) {
            ++nonZeroCount;
            if (normWeights[s] > domWeight) {
                domWeight = normWeights[s];
                domSlot = s;
            }
        }
    }
    if (nonZeroCount == 0u) {
        return originFrame_;
    }
    if (nonZeroCount == 1u) {
        return decodePatchSnapshot(slots_[domSlot]);
    }

    std::array<PatchFrame, kNumSnapshots> decoded{};
    for (size_t s = 0u; s < kNumSnapshots; ++s) {
        if (isSlotOccupied(s) && normWeights[s] > 1.0e-4f) {
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
            const float w = isSlotOccupied(s) ? normWeights[s] : 0.0f;
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
    syncKnobMorphAnchor();
}

bool SnapshotBank::deleteSlot(uint8_t slot, EngineControlState& state) noexcept {
    const uint8_t idx = slot & 3u;
    if (momentaryActive_) {
        applyPatchFrame(originFrame_, state);
        momentaryActive_ = false;
        momentaryBlend_ = 0.0f;
    }
    if (latchGliding_) {
        originFrame_ = extractPatchFrame(state);
        latchGliding_ = false;
        latchHybridHeld_ = false;
        glideProgress_ = 1.0f;
    }
    const bool wasOccupied = isSlotOccupied(idx);
    slots_[idx] = makeEmptyPatchSnapshot();
    slotWeights_[idx] = 0.0f;
    originWeights_[idx] = 0.0f;
    targetWeights_[idx] = 0.0f;
    if (idx == activeSlot_) {
        slotWeights_ = {0.0f, 0.0f, 0.0f, 0.0f};
        originWeights_ = slotWeights_;
        targetWeights_ = slotWeights_;
        morphPos_ = 0.0f;
        dirty_ = true;
    } else {
        float sumW = 0.0f;
        for (float w : slotWeights_) {
            sumW += w;
        }
        if (sumW > 1.0e-5f) {
            const float invSum = 1.0f / sumW;
            for (float& w : slotWeights_) {
                w *= invSum;
            }
        } else {
            dirty_ = true;
        }
        originWeights_ = slotWeights_;
        targetWeights_ = slotWeights_;
    }
    syncKnobMorphAnchor();
    return wasOccupied;
}

bool SnapshotBank::loadSlotBinary(uint8_t slot, const PatchSnapshot& snap,
                                  EngineControlState& state) noexcept {
    if (!verifyPatchSnapshotCrc(snap)) {
        return false;
    }
    const uint8_t idx = slot & 3u;
    slots_[idx] = snap;
    if (!isTransitioning()) {
        if (idx == activeSlot_ && !dirty_) {
            selectActiveSlotImmediate(activeSlot_, state);
        } else if (!isSlotOccupied(activeSlot_) && isSlotOccupied(idx)) {
            selectActiveSlotImmediate(idx, state);
        } else {
            syncKnobMorphAnchor();
        }
    }
    return true;
}

void SnapshotBank::selectActiveSlotImmediate(uint8_t slot, EngineControlState& state) noexcept {
    uint8_t idx = slot & 3u;
    if (!isSlotOccupied(idx)) {
        const uint8_t mask = occupiedMask();
        if (mask == 0u) {
            activeSlot_ = 0u;
            dirty_ = true;
            latchGliding_ = false;
            latchHybridHeld_ = false;
            momentaryActive_ = false;
            knobMorphActive_ = false;
            glideProgress_ = 1.0f;
            momentaryBlend_ = 0.0f;
            morphPos_ = 0.0f;
            knobMorphPos_ = 0.0f;
            slotWeights_ = {0.0f, 0.0f, 0.0f, 0.0f};
            originWeights_ = slotWeights_;
            targetWeights_ = slotWeights_;
            originFrame_ = extractPatchFrame(EngineControlState::makeDefault());
            applyPatchFrame(originFrame_, state);
            return;
        }
        for (uint8_t s = 0u; s < kNumSnapshots; ++s) {
            if (isSlotOccupied(s)) {
                idx = s;
                break;
            }
        }
    }
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
    syncKnobMorphAnchor();
    applyPatchFrame(originFrame_, state);
}

void SnapshotBank::recallSlotStepped(uint8_t slot, KnobRate rate,
                                     EngineControlState& state) noexcept {
    const uint8_t idx = slot & 3u;
    if (!isSlotOccupied(idx)) {
        return;
    }
    if (momentaryActive_) {
        momentaryActive_ = false;
        momentaryBlend_ = 0.0f;
    }
    originFrame_ = extractPatchFrame(state);
    float keptSum = 0.0f;
    for (size_t s = 0u; s < kNumSnapshots; ++s) {
        const float w = isSlotOccupied(s) ? slotWeights_[s] : 0.0f;
        originWeights_[s] = (w >= 0.035f) ? w : 0.0f;
        keptSum += originWeights_[s];
    }
    if (keptSum > 1.0e-5f) {
        const float invSum = 1.0f / keptSum;
        for (float& w : originWeights_) {
            w *= invSum;
        }
    }
    slotWeights_ = originWeights_;
    targetWeights_ = {0.0f, 0.0f, 0.0f, 0.0f};
    targetWeights_[idx] = 1.0f;
    activeSlot_ = idx;
    syncKnobMorphAnchor();
    dirty_ = false;
    latchHybridHeld_ = false;
    glideDurationSec_ = (rate == KnobRate::Fine)     ? 1.80f
                        : (rate == KnobRate::Coarse) ? 0.10f
                                                     : 0.50f;
    glideProgress_ = 0.0f;
    morphPos_ = 0.0f;
    latchGliding_ = true;
}

bool SnapshotBank::morphByEncoder(int32_t ticks, KnobRate rate,
                                  EngineControlState& state) noexcept {
    if (ticks == 0) {
        return false;
    }
    std::array<uint8_t, kNumSnapshots> occSlots{};
    int32_t occCount = 0;
    for (uint8_t s = 0u; s < kNumSnapshots; ++s) {
        if (isSlotOccupied(s)) {
            occSlots[static_cast<size_t>(occCount++)] = s;
        }
    }
    if (occCount == 0) {
        knobMorphActive_ = false;
        return false;
    }
    if (occCount == 1) {
        const uint8_t onlySlot = occSlots[0];
        if (!dirty_ && !knobMorphActive_ && !momentaryActive_ && !latchGliding_ &&
            activeSlot_ == onlySlot && slotWeights_[onlySlot] >= 0.999f) {
            return false;
        }
        selectActiveSlotImmediate(onlySlot, state);
        return true;
    }

    const float maxU = static_cast<float>(occCount - 1);
    if (momentaryActive_ || latchGliding_) {
        float wSum = 0.0f;
        float wOrdSum = 0.0f;
        for (int32_t i = 0; i < occCount; ++i) {
            const float w = slotWeights_[occSlots[static_cast<size_t>(i)]];
            wSum += w;
            wOrdSum += w * static_cast<float>(i);
        }
        if (wSum > 1.0e-5f) {
            knobMorphPos_ = clampf((wOrdSum / wSum) / maxU, 0.0f, 1.0f);
        }
        momentaryActive_ = false;
        momentaryBlend_ = 0.0f;
        latchGliding_ = false;
        latchHybridHeld_ = false;
        glideProgress_ = 1.0f;
    } else if (!knobMorphActive_) {
        syncKnobMorphAnchor();
    }

    const float stepPerTick = (rate == KnobRate::Fine)     ? (1.0f / 192.0f)
                              : (rate == KnobRate::Coarse) ? (1.0f / 16.0f)
                                                           : (1.0f / 48.0f);
    const float prevU = clampf(knobMorphPos_ * maxU, 0.0f, maxU);
    const float nextU = clampf(prevU + static_cast<float>(ticks) * stepPerTick, 0.0f, maxU);
    if (absf(nextU - prevU) <= 1.0e-6f && !dirty_ && !knobMorphActive_) {
        return false;
    }

    knobMorphPos_ = nextU / maxU;
    const float railDist = (nextU < (maxU - nextU)) ? nextU : (maxU - nextU);
    const bool atRail = (railDist <= 1.0e-4f);
    // Overlapping multi-scene kernel: radius = 1.0 at exact endpoints so 0% and 100% seat cleanly
    // on pure first/last scenes, expanding to 1.55 across the interior so 2-3 scenes overlap.
    const float radius = 1.0f + 0.55f * clampf(railDist * 1.25f, 0.0f, 1.0f);
    const float invRadius = 1.0f / radius;

    slotWeights_ = {0.0f, 0.0f, 0.0f, 0.0f};
    float wSum = 0.0f;
    uint8_t domSlot = occSlots[0];
    float domW = -1.0f;
    for (int32_t i = 0; i < occCount; ++i) {
        const float d = absf(nextU - static_cast<float>(i)) * invRadius;
        const float oneMinusD2 = (d < 1.0f) ? (1.0f - d * d) : 0.0f;
        const float w = oneMinusD2 * oneMinusD2;
        const uint8_t s = occSlots[static_cast<size_t>(i)];
        slotWeights_[s] = w;
        wSum += w;
        if (w > domW) {
            domW = w;
            domSlot = s;
        }
    }
    if (wSum > 1.0e-5f) {
        const float invSum = 1.0f / wSum;
        for (float& w : slotWeights_) {
            w *= invSum;
        }
    } else {
        slotWeights_[domSlot] = 1.0f;
    }

    activeSlot_ = domSlot;
    originWeights_ = slotWeights_;
    targetWeights_ = slotWeights_;
    knobMorphActive_ = !atRail;
    dirty_ = false;
    morphPos_ = atRail ? 1.0f : knobMorphPos_;
    originFrame_ = blendSlots(slotWeights_);
    applyPatchFrame(originFrame_, state);
    return true;
}

bool SnapshotBank::onSnapKeysChanged(const std::array<bool, kNumSnapshots>& held,
                                     const std::array<float, kNumSnapshots>& pressure,
                                     KeySwitchMode snapMode, EngineControlState& state) noexcept {
    const uint8_t occMask = occupiedMask();
    std::array<float, kNumSnapshots> normW{};
    float maxP = 0.0f;
    const size_t heldCount = normalizeHeldWeights(held, pressure, occMask, normW, maxP);
    uint8_t domSlot = activeSlot_;
    float domP = -1.0f;
    bool anyRising = false;

    for (size_t s = 0u; s < kNumSnapshots; ++s) {
        if ((occMask & (1u << s)) == 0u) {
            continue;
        }
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
                knobMorphActive_ = false;
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
        syncKnobMorphAnchor();
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
            const size_t heldCount =
                normalizeHeldWeights(held, pressure, occupiedMask(), liveW, maxP);
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
                    if (!dirty_ && isSlotOccupied(activeSlot_)) {
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
            if (held[s] && isSlotOccupied(s)) {
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
    // Endpoint deadband on visual scene weights keeps intermediate detents brushed past on
    // `PRESETS` locked at 0% (eliminating `0% -> 1% -> 0%` wobble) while audio morphs continuously.
    const float wEased = (eased < 0.015f) ? 0.0f : ((eased > 0.985f) ? 1.0f : eased);
    for (size_t s = 0u; s < kNumSnapshots; ++s) {
        slotWeights_[s] =
            isSlotOccupied(s)
                ? (originWeights_[s] + wEased * (targetWeights_[s] - originWeights_[s]))
                : 0.0f;
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
    knobMorphActive_ = false;
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
    if (!dirty_ && isSlotOccupied(activeSlot_)) {
        slotWeights_[activeSlot_] = 1.0f;
    }
    applyPatchFrame(originFrame_, state);
    return true;
}

void SnapshotBank::syncHud(HudState& hud) const noexcept {
    hud.snapshotSlot = activeSlot_;
    hud.snapshotOccupiedMask = occupiedMask();
    hud.snapshotWeights = slotWeights_;
    hud.morphPosition = morphPos_;
    hud.snapshotDirty = dirty_;
    hud.snapshotGliding = latchGliding_ || momentaryActive_ || knobMorphActive_;
}

} // namespace ruido
