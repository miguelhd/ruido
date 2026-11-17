#include "core/DspMath.h"
#include "core/SnapshotBank.h"

namespace ruido {

namespace {

inline int32_t gatherOccupiedSlots(const SnapshotBank& bank,
                                   std::array<uint8_t, kNumSnapshots>& outSlots) noexcept {
    int32_t count = 0;
    for (uint8_t s = 0u; s < kNumSnapshots; ++s) {
        if (bank.isSlotOccupied(s)) {
            outSlots[static_cast<size_t>(count++)] = s;
        }
    }
    return count;
}

inline float computeVisualLoopPhase(uint8_t slotA, uint8_t slotB, float t01) noexcept {
    const uint8_t a = slotA & 3u;
    const uint8_t b = slotB & 3u;
    if (a == b) {
        return static_cast<float>(a);
    }
    const uint8_t cwSpan = static_cast<uint8_t>((b + 4u - a) & 3u);
    return wrapCircularPhase(
        static_cast<float>(a) + clampf(t01, 0.0f, 1.0f) * static_cast<float>(cwSpan), 4.0f);
}

} // namespace

void SnapshotBank::syncKnobMorphAnchor() noexcept {
    knobMorphActive_ = false;
    liveTakeoverBlend_ = 0.0f;
    presetsCoarseAccum_ = 0;
    glideArcDir_ = 0;
    visualLoopPhase_ = static_cast<float>(activeSlot_ & 3u);

    std::array<uint8_t, kNumSnapshots> occSlots{};
    const int32_t occCount = gatherOccupiedSlots(*this, occSlots);
    if (occCount <= 1) {
        loopPhase_ = 0.0f;
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
    loopPhase_ = static_cast<float>(ord);
}

void SnapshotBank::recallSlotStepped(uint8_t slot, KnobRate rate,
                                     EngineControlState& state) noexcept {
    const uint8_t idx = slot & 3u;
    if (!isSlotOccupied(idx)) {
        return;
    }
    const uint8_t prevSlot = activeSlot_;
    const float prevVisLoop = wrapCircularPhase(visualLoopPhase_, 4.0f);
    if (momentaryActive_) {
        momentaryActive_ = false;
        momentaryBlend_ = 0.0f;
    }
    originFrame_ = extractPatchFrame(state);
    float keptSum = 0.0f;
    uint8_t prevActiveWeights = 0u;
    std::array<float, kNumSnapshots> origW{};
    for (uint8_t s = 0u; s < kNumSnapshots; ++s) {
        const float w = isSlotOccupied(s) ? slotWeights_[s] : 0.0f;
        origW[s] = (w >= 0.035f) ? w : 0.0f;
        keptSum += origW[s];
        if (origW[s] > 0.0f) {
            ++prevActiveWeights;
        }
    }
    if (keptSum > 1.0e-5f) {
        const float invSum = 1.0f / keptSum;
        for (float& w : origW) {
            w *= invSum;
        }
    }
    slotWeights_ = origW;
    originWeightsQ16_ = packWeightsQ16(origW);
    std::array<float, kNumSnapshots> tgtW{0.0f, 0.0f, 0.0f, 0.0f};
    tgtW[idx] = 1.0f;
    targetWeightsQ16_ = packWeightsQ16(tgtW);
    activeSlot_ = idx;
    momentaryHomeSlot_ = idx;
    syncKnobMorphAnchor();
    visualLoopPhase_ = (prevActiveWeights > 0u && isSlotOccupied(prevSlot))
                           ? prevVisLoop
                           : static_cast<float>(idx & 3u);
    glideOriginPhaseQ14_ =
        static_cast<uint16_t>(clampf(visualLoopPhase_, 0.0f, 3.9999f) * 16384.0f + 0.5f);
    dirty_ = false;
    latchHybridHeld_ = false;
    glideDurationSec_ = (rate == KnobRate::Fine)     ? 1.80f
                        : (rate == KnobRate::Coarse) ? 0.09f
                                                     : 0.45f;
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
    const int32_t occCount = gatherOccupiedSlots(*this, occSlots);
    if (occCount == 0) {
        knobMorphActive_ = false;
        liveTakeoverBlend_ = 0.0f;
        return false;
    }
    if (occCount == 1) {
        const uint8_t onlySlot = occSlots[0];
        if (!dirty_ && !knobMorphActive_ && !momentaryActive_ && !latchGliding_ &&
            activeSlot_ == onlySlot && slotWeights_[onlySlot] >= 0.999f) {
            return false;
        }
        if (rate == KnobRate::Coarse) {
            recallSlotStepped(onlySlot, KnobRate::Coarse, state);
            return true;
        }
        if ((dirty_ || (latchGliding_ && glideProgress_ < 0.999f) ||
             (momentaryActive_ && momentaryBlend_ > 0.001f)) &&
            !knobMorphActive_ && liveTakeoverBlend_ <= 0.0f) {
            originFrame_ = extractPatchFrame(state);
            liveTakeoverBlend_ = 1.0f;
            momentaryActive_ = false;
            momentaryBlend_ = 0.0f;
            latchGliding_ = false;
            latchHybridHeld_ = false;
            glideProgress_ = 1.0f;
            glideArcDir_ = 0;
        }
        const float spanPerTick = (rate == KnobRate::Fine) ? (1.0f / 192.0f) : (1.0f / 48.0f);
        liveTakeoverBlend_ = clampf(
            liveTakeoverBlend_ - absf(static_cast<float>(ticks) * spanPerTick) * 2.0f, 0.0f, 1.0f);
        if (liveTakeoverBlend_ <= 1.0e-4f) {
            selectActiveSlotImmediate(onlySlot, state);
            return true;
        }
        activeSlot_ = onlySlot;
        momentaryHomeSlot_ = onlySlot;
        visualLoopPhase_ = static_cast<float>(onlySlot);
        slotWeights_ = {0.0f, 0.0f, 0.0f, 0.0f};
        slotWeights_[onlySlot] = 1.0f;
        originWeightsQ16_ = packWeightsQ16(slotWeights_);
        targetWeightsQ16_ = originWeightsQ16_;
        knobMorphActive_ = true;
        dirty_ = true;
        const float takeT = smoothstep01(1.0f - liveTakeoverBlend_);
        morphPos_ = takeT;
        applyPatchFrame(
            morphPatchFrames(originFrame_, decodePatchSnapshot(slots_[onlySlot]), takeT), state);
        return true;
    }

    // Symmetric +/-0.044 keyframe plateau (total width 0.088 > 1/12 detent step) guarantees every
    // physical click pass hits 100% pure Scene at nodes 1..4 while keeping smooth between-node
    // S-curves.
    constexpr float kPlateau = 0.044f;
    constexpr float kInvInterior = 1.0f / (1.0f - 2.0f * kPlateau);

    // OCT+ (Coarse): discrete 1-click = 1-scene circular step (1 -> 2 -> 3 -> 4 -> 1)
    if (rate == KnobRate::Coarse) {
        const int32_t total = static_cast<int32_t>(presetsCoarseAccum_) + ticks;
        const int32_t detents = total / 4;
        presetsCoarseAccum_ = static_cast<int8_t>(total - detents * 4);
        if (detents == 0) {
            return false;
        }
        if (!knobMorphActive_) {
            syncKnobMorphAnchor();
        }
        int32_t segIdx = static_cast<int32_t>(loopPhase_);
        if (segIdx < 0) {
            segIdx = 0;
        } else if (segIdx >= occCount) {
            segIdx = occCount - 1;
        }
        const float rawFrac = loopPhase_ - static_cast<float>(segIdx);
        const bool inInterior =
            knobMorphActive_ && (rawFrac > kPlateau) && (rawFrac < (1.0f - kPlateau));
        int32_t nextOrd = segIdx;
        if (inInterior) {
            nextOrd = (detents > 0) ? (segIdx + detents) : (segIdx + detents + 1);
        } else {
            const int32_t baseOrd = (rawFrac >= (1.0f - kPlateau)) ? (segIdx + 1) : segIdx;
            nextOrd = baseOrd + detents;
        }
        nextOrd %= occCount;
        if (nextOrd < 0) {
            nextOrd += occCount;
        }
        liveTakeoverBlend_ = 0.0f;
        recallSlotStepped(occSlots[static_cast<size_t>(nextOrd)], KnobRate::Coarse, state);
        glideArcDir_ = (detents > 0) ? static_cast<int8_t>(1) : static_cast<int8_t>(-1);
        return true;
    }

    presetsCoarseAccum_ = 0;
    if (momentaryActive_ || latchGliding_) {
        const bool needsTakeover = dirty_ || (latchGliding_ && glideProgress_ < 0.999f) ||
                                   (momentaryActive_ && momentaryBlend_ > 0.001f);
        if (needsTakeover) {
            originFrame_ = extractPatchFrame(state);
        }
        int32_t domOrd = 0;
        float domW = -1.0f;
        for (int32_t i = 0; i < occCount; ++i) {
            const float w = slotWeights_[occSlots[static_cast<size_t>(i)]];
            if (w > domW) {
                domW = w;
                domOrd = i;
            }
        }
        const int32_t fwdOrd = (domOrd + 1) % occCount;
        const int32_t bwdOrd = (domOrd + occCount - 1) % occCount;
        const float wDom = slotWeights_[occSlots[static_cast<size_t>(domOrd)]];
        const float wFwd = slotWeights_[occSlots[static_cast<size_t>(fwdOrd)]];
        const float wBwd = slotWeights_[occSlots[static_cast<size_t>(bwdOrd)]];
        if (wFwd >= wBwd && (wDom + wFwd) > 1.0e-5f) {
            loopPhase_ = static_cast<float>(domOrd) + (wFwd / (wDom + wFwd));
        } else if ((wDom + wBwd) > 1.0e-5f) {
            loopPhase_ = static_cast<float>(bwdOrd) + (wDom / (wDom + wBwd));
        } else {
            loopPhase_ = static_cast<float>(domOrd);
        }
        momentaryActive_ = false;
        momentaryBlend_ = 0.0f;
        latchGliding_ = false;
        latchHybridHeld_ = false;
        glideProgress_ = 1.0f;
        glideArcDir_ = 0;
        knobMorphActive_ = true;
        if (needsTakeover) {
            liveTakeoverBlend_ = 1.0f;
        }
    } else if (!knobMorphActive_) {
        const bool wasEdited = dirty_;
        if (wasEdited) {
            originFrame_ = extractPatchFrame(state);
        }
        syncKnobMorphAnchor();
        if (wasEdited) {
            liveTakeoverBlend_ = 1.0f;
        }
    }

    const float spanPerTick = (rate == KnobRate::Fine) ? (1.0f / 192.0f) : (1.0f / 48.0f);
    const float deltaSpan = static_cast<float>(ticks) * spanPerTick;
    loopPhase_ = wrapCircularPhase(loopPhase_ + deltaSpan, static_cast<float>(occCount));
    if (liveTakeoverBlend_ > 0.0f) {
        liveTakeoverBlend_ = clampf(liveTakeoverBlend_ - absf(deltaSpan) * 2.0f, 0.0f, 1.0f);
    }

    int32_t segIdx = static_cast<int32_t>(loopPhase_);
    if (segIdx < 0) {
        segIdx = 0;
    } else if (segIdx >= occCount) {
        segIdx = occCount - 1;
    }
    const float rawFrac = clampf(loopPhase_ - static_cast<float>(segIdx), 0.0f, 1.0f);
    float t = 0.0f;
    if (rawFrac <= kPlateau) {
        t = 0.0f;
    } else if (rawFrac >= (1.0f - kPlateau)) {
        t = 1.0f;
    } else {
        const float u = (rawFrac - kPlateau) * kInvInterior;
        t = 0.35f * u + 0.65f * smoothstep01(u);
    }

    const uint8_t slotA = occSlots[static_cast<size_t>(segIdx)];
    const uint8_t slotB = occSlots[static_cast<size_t>((segIdx + 1) % occCount)];
    slotWeights_ = {0.0f, 0.0f, 0.0f, 0.0f};
    slotWeights_[slotA] += (1.0f - t);
    slotWeights_[slotB] += t;

    const bool atPureNode = (t <= 1.0e-4f || t >= 0.9999f);
    const uint8_t domSlot = (t >= 0.5f) ? slotB : slotA;
    const bool hasTakeover = (liveTakeoverBlend_ > 1.0e-4f);
    activeSlot_ = domSlot;
    momentaryHomeSlot_ = domSlot;
    visualLoopPhase_ = computeVisualLoopPhase(slotA, slotB, t);
    originWeightsQ16_ = packWeightsQ16(slotWeights_);
    targetWeightsQ16_ = originWeightsQ16_;
    knobMorphActive_ = !atPureNode || hasTakeover;
    dirty_ = !atPureNode || hasTakeover;
    const float takeT = smoothstep01(1.0f - liveTakeoverBlend_);
    morphPos_ = atPureNode ? (hasTakeover ? takeT : 1.0f) : t;
    const PatchFrame trackFrame = blendSlots(slotWeights_);
    if (hasTakeover) {
        applyPatchFrame(morphPatchFrames(originFrame_, trackFrame, takeT), state);
    } else {
        originFrame_ = trackFrame;
        applyPatchFrame(originFrame_, state);
    }
    return true;
}

void SnapshotBank::notifyManualEdit(EngineControlState& state) noexcept {
    if (momentaryActive_ || knobMorphActive_) {
        originFrame_ = extractPatchFrame(state);
    }
    latchGliding_ = false;
    latchHybridHeld_ = false;
    knobMorphActive_ = false;
    liveTakeoverBlend_ = 0.0f;
    glideProgress_ = 1.0f;
    glideArcDir_ = 0;
    slotWeights_ = {0.0f, 0.0f, 0.0f, 0.0f};
    if (isSlotOccupied(activeSlot_)) {
        slotWeights_[activeSlot_] = 1.0f;
    }
    originWeightsQ16_ = packWeightsQ16(slotWeights_);
    targetWeightsQ16_ = originWeightsQ16_;
    morphPos_ = 0.0f;
    visualLoopPhase_ = static_cast<float>(activeSlot_ & 3u);
    dirty_ = true;
}

bool SnapshotBank::cancelMomentaryIfActive(EngineControlState& state) noexcept {
    prevHeld_ = {false, false, false, false};
    if (!momentaryActive_) {
        return false;
    }
    momentaryActive_ = false;
    momentaryBlend_ = 0.0f;
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

void SnapshotBank::syncHud(HudState& hud) const noexcept {
    hud.snapshotSlot = activeSlot_;
    hud.snapshotOccupiedMask = occupiedMask();
    hud.snapshotWeights = slotWeights_;
    hud.morphPosition = morphPos_;
    hud.snapshotLoopPhase = visualLoopPhase_;
    hud.snapshotDirty = dirty_;
    hud.snapshotGliding = latchGliding_ || momentaryActive_ || knobMorphActive_;
}

} // namespace ruido
