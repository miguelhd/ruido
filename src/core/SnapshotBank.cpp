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

inline float interpolateLoopPhase4(float fromPhase, uint8_t toSlot, float t01,
                                   int8_t arcDir) noexcept {
    const float a = wrapCircularPhase(fromPhase, 4.0f);
    const float b = static_cast<float>(toSlot & 3u);
    float delta = b - a;
    if (absf(delta) > 1.0e-5f) {
        if (arcDir > 0) {
            if (delta < 0.0f) {
                delta += 4.0f;
            }
        } else if (arcDir < 0) {
            if (delta > 0.0f) {
                delta -= 4.0f;
            }
        } else {
            if (delta > 2.0f) {
                delta -= 4.0f;
            } else if (delta < -2.0f) {
                delta += 4.0f;
            }
        }
    }
    return wrapCircularPhase(a + clampf(t01, 0.0f, 1.0f) * delta, 4.0f);
}

inline float computeShortestArcPhase(uint8_t fromSlot, uint8_t toSlot, float t01,
                                     int8_t arcDir) noexcept {
    return interpolateLoopPhase4(static_cast<float>(fromSlot & 3u), toSlot, t01, arcDir);
}

inline uint8_t dominantWeightSlot(const std::array<float, kNumSnapshots>& weights, uint8_t occMask,
                                  uint8_t fallback) noexcept {
    uint8_t best = fallback & 3u;
    float bestW = -1.0f;
    for (uint8_t s = 0u; s < kNumSnapshots; ++s) {
        if (((occMask & (1u << s)) != 0u) && weights[s] > bestW) {
            bestW = weights[s];
            best = s;
        }
    }
    return best;
}

} // namespace

void SnapshotBank::reset() noexcept {
    for (size_t s = 0u; s < kNumSnapshots; ++s) {
        slots_[s] = encodePatchSnapshot(makeFactorySnapshotPreset(s));
    }
    originFrame_ = decodePatchSnapshot(slots_[0]);
    slotWeights_ = {1.0f, 0.0f, 0.0f, 0.0f};
    originWeightsQ16_ = packWeightsQ16(slotWeights_);
    targetWeightsQ16_ = originWeightsQ16_;
    prevHeld_ = {false, false, false, false};
    glideProgress_ = 1.0f;
    glideDurationSec_ = 0.50f;
    momentaryBlend_ = 0.0f;
    morphPos_ = 1.0f;
    loopPhase_ = 0.0f;
    visualLoopPhase_ = 0.0f;
    liveTakeoverBlend_ = 0.0f;
    glideOriginPhaseQ14_ = 0u;
    presetsCoarseAccum_ = 0;
    glideArcDir_ = 0;
    activeSlot_ = 0u;
    momentaryHomeSlot_ = 0u;
    dirty_ = false;
    latchGliding_ = false;
    latchHybridHeld_ = false;
    momentaryActive_ = false;
    knobMorphActive_ = false;
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

PatchSnapshot SnapshotBank::exportLiveSnapshot(const EngineControlState& state) const noexcept {
    return dirty_ ? encodePatchSnapshot(state) : makeEmptyPatchSnapshot();
}

bool SnapshotBank::restoreLiveSnapshot(const PatchSnapshot& snap,
                                       EngineControlState& state) noexcept {
    if (!verifyPatchSnapshotCrc(snap)) {
        return false;
    }
    if (!isPatchSnapshotOccupied(snap)) {
        return true;
    }
    originFrame_ = decodePatchSnapshot(snap);
    applyPatchFrame(originFrame_, state);
    dirty_ = true;
    latchGliding_ = false;
    latchHybridHeld_ = false;
    momentaryActive_ = false;
    knobMorphActive_ = false;
    liveTakeoverBlend_ = 0.0f;
    glideProgress_ = 1.0f;
    momentaryBlend_ = 0.0f;
    morphPos_ = 1.0f;
    slotWeights_ = {0.0f, 0.0f, 0.0f, 0.0f};
    if (isSlotOccupied(activeSlot_)) {
        slotWeights_[activeSlot_] = 1.0f;
    }
    originWeightsQ16_ = packWeightsQ16(slotWeights_);
    targetWeightsQ16_ = originWeightsQ16_;
    syncKnobMorphAnchor();
    return true;
}

void SnapshotBank::saveSlot(uint8_t slot, EngineControlState& state) noexcept {
    const uint8_t idx = slot & 3u;
    if (momentaryActive_) {
        applyPatchFrame(originFrame_, state);
        momentaryActive_ = false;
        momentaryBlend_ = 0.0f;
    }
    slots_[idx] = encodePatchSnapshot(state);
    originFrame_ = decodePatchSnapshot(slots_[idx]);
    activeSlot_ = idx;
    momentaryHomeSlot_ = idx;
    dirty_ = false;
    latchGliding_ = false;
    latchHybridHeld_ = false;
    glideProgress_ = 1.0f;
    morphPos_ = 1.0f;
    slotWeights_ = {0.0f, 0.0f, 0.0f, 0.0f};
    slotWeights_[idx] = 1.0f;
    originWeightsQ16_ = packWeightsQ16(slotWeights_);
    targetWeightsQ16_ = originWeightsQ16_;
    syncKnobMorphAnchor();
}

bool SnapshotBank::deleteSlot(uint8_t slot, EngineControlState& state) noexcept {
    const uint8_t idx = slot & 3u;
    if (momentaryActive_) {
        applyPatchFrame(originFrame_, state);
        momentaryActive_ = false;
        momentaryBlend_ = 0.0f;
    }
    if (latchGliding_ || knobMorphActive_) {
        originFrame_ = extractPatchFrame(state);
        latchGliding_ = false;
        latchHybridHeld_ = false;
        glideProgress_ = 1.0f;
    }
    const bool wasOccupied = isSlotOccupied(idx);
    slots_[idx] = makeEmptyPatchSnapshot();
    slotWeights_[idx] = 0.0f;
    if (idx == activeSlot_) {
        slotWeights_ = {0.0f, 0.0f, 0.0f, 0.0f};
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
    }
    originWeightsQ16_ = packWeightsQ16(slotWeights_);
    targetWeightsQ16_ = originWeightsQ16_;
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
            momentaryHomeSlot_ = 0u;
            dirty_ = true;
            latchGliding_ = false;
            latchHybridHeld_ = false;
            momentaryActive_ = false;
            knobMorphActive_ = false;
            liveTakeoverBlend_ = 0.0f;
            glideProgress_ = 1.0f;
            momentaryBlend_ = 0.0f;
            morphPos_ = 0.0f;
            loopPhase_ = 0.0f;
            visualLoopPhase_ = 0.0f;
            slotWeights_ = {0.0f, 0.0f, 0.0f, 0.0f};
            originWeightsQ16_ = packWeightsQ16(slotWeights_);
            targetWeightsQ16_ = originWeightsQ16_;
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
    momentaryHomeSlot_ = idx;
    dirty_ = false;
    latchGliding_ = false;
    latchHybridHeld_ = false;
    momentaryActive_ = false;
    glideProgress_ = 1.0f;
    momentaryBlend_ = 0.0f;
    morphPos_ = 1.0f;
    slotWeights_ = {0.0f, 0.0f, 0.0f, 0.0f};
    slotWeights_[idx] = 1.0f;
    originWeightsQ16_ = packWeightsQ16(slotWeights_);
    targetWeightsQ16_ = originWeightsQ16_;
    originFrame_ = decodePatchSnapshot(slots_[idx]);
    syncKnobMorphAnchor();
    applyPatchFrame(originFrame_, state);
}

void SnapshotBank::applyMomentaryWeightsAndPhase(
    const std::array<float, kNumSnapshots>& tgtW) noexcept {
    const bool canHomeTrack = !dirty_ && isSlotOccupied(momentaryHomeSlot_);
    visualLoopPhase_ =
        canHomeTrack ? computeShortestArcPhase(momentaryHomeSlot_, activeSlot_, momentaryBlend_, 0)
                     : static_cast<float>(activeSlot_ & 3u);
    morphPos_ = momentaryBlend_;
    for (size_t s = 0u; s < kNumSnapshots; ++s) {
        const float homeW = (canHomeTrack && s == momentaryHomeSlot_) ? 1.0f : 0.0f;
        slotWeights_[s] = homeW + momentaryBlend_ * (tgtW[s] - homeW);
    }
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
                latchHybridHeld_ = false;
                knobMorphActive_ = false;
                liveTakeoverBlend_ = 0.0f;
                momentaryHomeSlot_ = activeSlot_;
                originFrame_ = extractPatchFrame(state);
                momentaryBlend_ = 0.0f;
                momentaryActive_ = true;
            }
            targetWeightsQ16_ = packWeightsQ16(normW);
            activeSlot_ = domSlot;
            const float tgtBlend = clampf(maxP, 0.0f, 1.0f);
            if (tgtBlend > momentaryBlend_) {
                momentaryBlend_ += 0.35f * (tgtBlend - momentaryBlend_);
            }
            applyMomentaryWeightsAndPhase(normW);
            applyPatchFrame(morphPatchFrames(originFrame_, blendSlots(normW), momentaryBlend_),
                            state);
            return true;
        }
        return false;
    }

    // Latching mode: multi-key chord release protection keeps sculpted hybrid frozen as fingers
    // lift
    if (heldCount == 0u) {
        if (latchHybridHeld_) {
            latchHybridHeld_ = false;
            latchGliding_ = false;
            glideProgress_ = 1.0f;
            morphPos_ = 1.0f;
            originFrame_ = extractPatchFrame(state);
        }
        return false;
    }

    if (latchHybridHeld_ && heldCount < 2u && !anyRising) {
        // Staggered release from a 2..4-finger chord: hold frozen hybrid weights until all lift
        return false;
    }

    if (heldCount == 1u && anyRising && !dirty_ && !latchGliding_ && !knobMorphActive_ &&
        domSlot == activeSlot_ && slotWeights_[domSlot] >= 0.999f) {
        // Already sitting cleanly at 100% of this unedited Scene
        morphPos_ = 1.0f;
        visualLoopPhase_ = static_cast<float>(domSlot);
        return false;
    }

    if (anyRising || heldCount >= 2u || latchGliding_) {
        const uint8_t prevSlot = activeSlot_;
        const float prevVisLoop = wrapCircularPhase(visualLoopPhase_, 4.0f);
        uint8_t prevActiveWeights = 0u;
        for (uint8_t s = 0u; s < kNumSnapshots; ++s) {
            if (isSlotOccupied(s) && slotWeights_[s] > 0.035f) {
                ++prevActiveWeights;
            }
        }
        const bool startNewGlide = anyRising || (!latchGliding_ && !latchHybridHeld_);
        const bool isSnapBack = (heldCount == 1u && domSlot == prevSlot && dirty_);
        momentaryActive_ = false;
        liveTakeoverBlend_ = 0.0f;
        if (startNewGlide) {
            originFrame_ = extractPatchFrame(state);
            if (isSnapBack) {
                slotWeights_ = normW;
            }
            originWeightsQ16_ = packWeightsQ16(slotWeights_);
            glideProgress_ = 0.0f;
            morphPos_ = 0.0f;
        }
        targetWeightsQ16_ = packWeightsQ16(normW);
        activeSlot_ = domSlot;
        momentaryHomeSlot_ = domSlot;
        syncKnobMorphAnchor();
        if (heldCount >= 2u) {
            latchHybridHeld_ = true;
            dirty_ = true;
            visualLoopPhase_ = static_cast<float>(domSlot);
        } else if (anyRising) {
            latchHybridHeld_ = false;
            dirty_ = false;
            visualLoopPhase_ = (prevActiveWeights > 0u && isSlotOccupied(prevSlot))
                                   ? prevVisLoop
                                   : static_cast<float>(domSlot & 3u);
            glideOriginPhaseQ14_ =
                static_cast<uint16_t>(clampf(visualLoopPhase_, 0.0f, 3.9999f) * 16384.0f + 0.5f);
        }
        // Pressure accelerates glide: light touch = ~0.55 s melt, 100% firm = ~0.08 s snap-back
        glideDurationSec_ = 0.55f - 0.47f * clampf(maxP, 0.0f, 1.0f);
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

    const uint8_t occMask = occupiedMask();
    if (momentaryActive_) {
        if (snapMode != KeySwitchMode::Momentary) {
            momentaryActive_ = false;
            momentaryBlend_ = 0.0f;
            activeSlot_ = momentaryHomeSlot_;
        } else {
            float maxP = 0.0f;
            std::array<float, kNumSnapshots> liveW{};
            const size_t heldCount = normalizeHeldWeights(held, pressure, occMask, liveW, maxP);
            if (heldCount > 0u) {
                targetWeightsQ16_ = packWeightsQ16(liveW);
                activeSlot_ = dominantWeightSlot(liveW, occMask, activeSlot_);
                const float alphaAtk = 1.0f - fastExp2(-1.44269504f * dt * (1.0f / 0.032f));
                momentaryBlend_ += alphaAtk * (clampf(maxP, 0.0f, 1.0f) - momentaryBlend_);
            } else {
                const float alphaRel = 1.0f - fastExp2(-1.44269504f * dt * (1.0f / 0.135f));
                momentaryBlend_ *= (1.0f - alphaRel);
                if (momentaryBlend_ <= 0.002f) {
                    momentaryBlend_ = 0.0f;
                    momentaryActive_ = false;
                    activeSlot_ = momentaryHomeSlot_;
                    morphPos_ = dirty_ ? 0.0f : 1.0f;
                    slotWeights_ = {0.0f, 0.0f, 0.0f, 0.0f};
                    if (!dirty_ && isSlotOccupied(activeSlot_)) {
                        slotWeights_[activeSlot_] = 1.0f;
                    }
                    visualLoopPhase_ = static_cast<float>(activeSlot_ & 3u);
                    applyPatchFrame(originFrame_, state);
                    return true;
                }
            }

            const auto tgtW = unpackWeightsQ16(targetWeightsQ16_);
            applyMomentaryWeightsAndPhase(tgtW);
            applyPatchFrame(morphPatchFrames(originFrame_, blendSlots(tgtW), momentaryBlend_),
                            state);
            return true;
        }
    }

    if (!latchGliding_ && !latchHybridHeld_) {
        return false;
    }

    if (snapMode == KeySwitchMode::Latching) {
        float maxHeldP = 0.0f;
        std::array<float, kNumSnapshots> liveW{};
        const size_t heldCount = normalizeHeldWeights(held, pressure, occMask, liveW, maxHeldP);

        if (latchHybridHeld_) {
            if (heldCount == 0u) {
                latchHybridHeld_ = false;
                latchGliding_ = false;
                glideProgress_ = 1.0f;
                morphPos_ = 1.0f;
                originFrame_ = extractPatchFrame(state);
                return false;
            }
            if (heldCount < 2u) {
                // Hold sculpted chord weights steady while remaining fingers lift asynchronously
                latchGliding_ = false;
                glideProgress_ = 1.0f;
                morphPos_ = 1.0f;
                originFrame_ = extractPatchFrame(state);
                return false;
            }
            targetWeightsQ16_ = packWeightsQ16(liveW);
            activeSlot_ = dominantWeightSlot(liveW, occMask, activeSlot_);
            momentaryHomeSlot_ = activeSlot_;
            visualLoopPhase_ = static_cast<float>(activeSlot_);
            if (glideProgress_ >= 1.0f) {
                // Continuous live pressure sculpting across 2..4 held Scene keys indefinitely!
                const float alphaSculpt = 1.0f - fastExp2(-1.44269504f * dt * (1.0f / 0.032f));
                float wSum = 0.0f;
                for (size_t s = 0u; s < kNumSnapshots; ++s) {
                    const float nextW =
                        slotWeights_[s] + alphaSculpt * (liveW[s] - slotWeights_[s]);
                    slotWeights_[s] = isSlotOccupied(s) ? clampf(nextW, 0.0f, 1.0f) : 0.0f;
                    wSum += slotWeights_[s];
                }
                if (wSum > 1.0e-5f) {
                    const float invSum = 1.0f / wSum;
                    for (float& w : slotWeights_) {
                        w *= invSum;
                    }
                }
                originWeightsQ16_ = packWeightsQ16(slotWeights_);
                morphPos_ = 1.0f;
                originFrame_ = blendSlots(slotWeights_);
                applyPatchFrame(originFrame_, state);
                return true;
            }
        }

        if (heldCount > 0u) {
            const float pDur = 0.55f - 0.47f * clampf(maxHeldP, 0.0f, 1.0f);
            if (pDur < glideDurationSec_) {
                glideDurationSec_ = pDur;
            }
        }
    }

    if (!latchGliding_) {
        return false;
    }

    const auto origW = unpackWeightsQ16(originWeightsQ16_);
    const auto tgtW = unpackWeightsQ16(targetWeightsQ16_);
    const PatchFrame targetFrame = blendSlots(tgtW);
    const float dur = (glideDurationSec_ > 0.02f) ? glideDurationSec_ : 0.02f;
    glideProgress_ += dt / dur;
    if (glideProgress_ >= 1.0f) {
        glideProgress_ = 1.0f;
        latchGliding_ = latchHybridHeld_;
        glideArcDir_ = 0;
        morphPos_ = 1.0f;
        slotWeights_ = tgtW;
        originWeightsQ16_ = targetWeightsQ16_;
        originFrame_ = targetFrame;
        visualLoopPhase_ = static_cast<float>(activeSlot_ & 3u);
        applyPatchFrame(targetFrame, state);
        return true;
    }

    const float eased = smoothstep01(glideProgress_);
    morphPos_ = eased;
    const float wEased = (eased < 0.015f) ? 0.0f : ((eased > 0.985f) ? 1.0f : eased);
    bool hasOrig = false;
    for (uint8_t s = 0u; s < kNumSnapshots; ++s) {
        if (isSlotOccupied(s) && origW[s] > 1.0e-4f) {
            hasOrig = true;
        }
        slotWeights_[s] = isSlotOccupied(s) ? (origW[s] + wEased * (tgtW[s] - origW[s])) : 0.0f;
    }
    if (!latchHybridHeld_) {
        if (hasOrig) {
            const float origPhase = static_cast<float>(glideOriginPhaseQ14_) * (1.0f / 16384.0f);
            visualLoopPhase_ = interpolateLoopPhase4(origPhase, activeSlot_, eased, glideArcDir_);
        } else {
            visualLoopPhase_ = static_cast<float>(activeSlot_ & 3u);
        }
    }
    applyPatchFrame(morphPatchFrames(originFrame_, targetFrame, eased), state);
    return true;
}

} // namespace ruido
