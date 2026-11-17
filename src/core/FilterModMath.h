#pragma once

#include "core/Config.h"
#include "core/DspMath.h"
#include "core/LfoMath.h"
#include "core/Parameters.h"

namespace ruido {

// Effective filter + VCA parameters produced by White Keys 5-8 continuous pressure, dual LFOs,
// and momentary keybed performance macros (shared 1-to-1 between Core 1 DSP and Core 0 HUD/LEDs).
struct ModulatedFilterParams {
    float hpCutoffHz;
    float hpResonance;
    float lpCutoffHz;
    float lpResonance;
    float effectiveGate;
};

// POD bundle of momentary keybed performance macro depths affecting MS-20 HPF/LPF response.
struct FilterMacroContext {
    float dubLpfDrop = 0.0f;
    float overloadSurge = 0.0f;
    float filterScream = 0.0f;
    float rhythmPluck = 0.0f;
    float pluckEnv01 = 0.0f;
    float masterHpfSweep = 0.0f;

    static constexpr FilterMacroContext fromKeyMod(const KeyboardModState& km,
                                                   float pluckEnv = 0.0f) noexcept {
        return {
            km.dubLpfDrop,  km.overloadSurge, km.filterScream,
            km.rhythmPluck, pluckEnv,         km.masterHpfSweep,
        };
    }
};

// White Keys 1-4 (Live Mixer FILTER 1-4) continuous pressure macro:
//  - Filter/LPG mode (pitchMode = 0): active voices sweep MS-20 vocal formant + Korg35 resonance;
//    muted voices swell through a warm 85 Hz -> base LPF vactrol low-pass gate.
//  - Pitch Bend mode (pitchMode = 1): filter stays at base settings while muted voices swell gate.
RUIDO_ALWAYS_INLINE ModulatedFilterParams
computeVoicePressureMacro(float hpBase, float hpResBase, float lpBase, float lpResBase,
                          float pressure, float activeGate, float pitchMode = 0.0f) noexcept {
    const float p = clampf(pressure, 0.0f, 1.0f);
    const float g = clampf(activeGate, 0.0f, 1.0f);
    if (p <= 0.0f) {
        return {hpBase, hpResBase, lpBase, lpResBase, g};
    }

    const float pm = clampf(pitchMode, 0.0f, 1.0f);
    const float fp = p * (1.0f - pm);

    const float lpgGate = p * (0.65f + 0.35f * p);
    const float effGate = clampf(g + (1.0f - g) * lpgGate, 0.0f, 1.0f);
    if (fp <= 0.0f) {
        return {hpBase, hpResBase, lpBase, lpResBase, effGate};
    }

    // 1. Active Voice: Full-Throw [0..1] MS-20 Vocal Formant + Korg35 HP/LP Resonance
    const float lpOpenness = smoothstep01((fastLog2(lpBase) - 11.287712f) * (1.0f / 3.5f));
    const float lpOctDelta = (4.8f * (1.0f - lpOpenness) - 3.80f * lpOpenness) * fp;
    const float lpAccent =
        clampf(lpBase * fastExp2(lpOctDelta), kFilterCutoffMinHz, kFilterCutoffMaxHz);

    const float hpFloorHz =
        (hpBase < 22.0f) ? (hpBase + (22.0f - hpBase) * smoothstep01(fp * 4.5f)) : hpBase;
    const float hpOctDelta = 4.65f * lpOpenness * fp;
    const float hpAccent =
        clampf(hpFloorHz * fastExp2(hpOctDelta), kFilterCutoffMinHz, kFilterCutoffMaxHz);

    const float resCurve = fp * (0.65f + 0.35f * fp);
    const float hpResAccent = clampf(hpResBase + 0.80f * resCurve, 0.0f, 1.0f);
    const float lpResAccent = clampf(lpResBase + 0.85f * resCurve, 0.0f, 1.0f);

    // 2. Muted Voice: Vactrol Low-Pass Gate (warm 85 Hz sub murmur -> full base LPF harmonics)
    const float lpgCeil = (lpBase > 85.0f) ? lpBase : 85.0f;
    const float lpgSpanOct = fastLog2(lpgCeil * (1.0f / 85.0f));
    const float lpLpg =
        clampf(85.0f * fastExp2(lpgSpanOct * fp * (0.48f + 0.52f * fp)), 85.0f, kFilterCutoffMaxHz);
    const float hpLpg = hpBase;
    const float hpResLpg = hpResBase;
    const float lpResLpg = clampf(lpResBase + 0.55f * fp * fp, 0.0f, 1.0f);

    return {
        hpLpg + g * (hpAccent - hpLpg),
        hpResLpg + g * (hpResAccent - hpResLpg),
        lpLpg + g * (lpAccent - lpLpg),
        lpResLpg + g * (lpResAccent - lpResLpg),
        effGate,
    };
}

// Overlays momentary keybed filter macros (OVERLOAD surge, DUB LPF drop, GLO SCREAM, LPG PLUCK).
// Uses division-free log2 subtraction (`fastLog2(a) - fastLog2(b)`) to eliminate software
// `__divsf3` float divisions on AC791N (`pi32v2`).
RUIDO_ALWAYS_INLINE ModulatedFilterParams
applyFilterPerformanceMacros(ModulatedFilterParams mod, const FilterMacroContext& macros) noexcept {
    const float surge = clampf(macros.overloadSurge, 0.0f, 1.0f);
    if (surge > 0.22f) {
        const float u = (surge - 0.22f) * (1.0f / 0.78f);
        const float driveCeilHz = 28000.0f * fastExp2(-3.05f * u); // 28 kHz -> 3.38 kHz OTA scream
        if (driveCeilHz < mod.lpCutoffHz) {
            mod.lpCutoffHz = clampf(driveCeilHz, kFilterCutoffMinHz, kFilterCutoffMaxHz);
        }
        mod.lpResonance = clampf(mod.lpResonance + 0.54f * u * u, 0.0f, 0.90f);
    }
    const float dubDrop = clampf(macros.dubLpfDrop, 0.0f, 1.0f);
    if (dubDrop > 1.0e-4f) {
        const float dubCeilHz = 28000.0f * fastExp2(-9.8f * dubDrop); // 28 kHz -> 31.4 Hz
        if (dubCeilHz < mod.lpCutoffHz) {
            mod.lpCutoffHz = clampf(dubCeilHz, kFilterCutoffMinHz, kFilterCutoffMaxHz);
        }
        const float dubResBump = 0.76f * dubDrop * (0.55f + 0.45f * dubDrop);
        mod.lpResonance = clampf(mod.lpResonance + dubResBump, 0.0f, 0.92f);
    }
    const float hpfSweep = clampf(macros.masterHpfSweep, 0.0f, 1.0f);
    if (hpfSweep > 1.0e-4f) {
        const float hpFloorHz = 20.0f * fastExp2(7.32f * hpfSweep); // 20 Hz -> 3.2 kHz bass cut
        if (hpFloorHz > mod.hpCutoffHz) {
            mod.hpCutoffHz = clampf(hpFloorHz, kFilterCutoffMinHz, kFilterCutoffMaxHz);
        }
        const float hpResBump = 0.74f * hpfSweep * (0.55f + 0.45f * hpfSweep);
        mod.hpResonance = clampf(mod.hpResonance + hpResBump, 0.0f, 0.90f);
    }
    const float scream = clampf(macros.filterScream, 0.0f, 1.0f);
    if (scream > 1.0e-4f) {
        const float engage = smoothstep01(scream * 5.0f);
        const float screamHpHz = 55.0f * fastExp2(2.90f * scream);     // 55 Hz -> 410 Hz
        const float screamLpHz = 14000.0f * fastExp2(-3.95f * scream); // 14 kHz -> 905 Hz
        if (screamHpHz > mod.hpCutoffHz) {
            const float hpSpanOct = fastLog2(screamHpHz) - fastLog2(mod.hpCutoffHz);
            mod.hpCutoffHz = clampf(mod.hpCutoffHz * fastExp2(engage * hpSpanOct),
                                    kFilterCutoffMinHz, kFilterCutoffMaxHz);
        }
        if (screamLpHz < mod.lpCutoffHz) {
            const float lpSpanOct = fastLog2(screamLpHz) - fastLog2(mod.lpCutoffHz);
            mod.lpCutoffHz = clampf(mod.lpCutoffHz * fastExp2(engage * lpSpanOct),
                                    kFilterCutoffMinHz, kFilterCutoffMaxHz);
        }
        const float screamRes = 0.96f * scream * (0.68f + 0.32f * scream);
        if (screamRes > mod.hpResonance) {
            mod.hpResonance = screamRes;
        }
        if (screamRes > mod.lpResonance) {
            mod.lpResonance = screamRes;
        }
    }
    const float pluck = clampf(macros.rhythmPluck, 0.0f, 1.0f);
    if (pluck > 1.0e-4f) {
        const float env = clampf(macros.pluckEnv01, 0.0f, 1.0f);
        const float engage = smoothstep01(pluck * 4.5f);
        const float floorHz = 250.0f * fastExp2(-1.18f * pluck); // 250 Hz -> 110 Hz woody floor
        const float peakHz = 3200.0f * fastExp2(3.12f * pluck);  // 3.2 kHz -> 27.8 kHz snap
        const float spanOct = fastLog2(peakHz) - fastLog2(floorHz);
        const float pluckHz =
            clampf(floorHz * fastExp2(spanOct * env), kFilterCutoffMinHz, kFilterCutoffMaxHz);
        const float pluckSpanOct = fastLog2(pluckHz) - fastLog2(mod.lpCutoffHz);
        mod.lpCutoffHz = clampf(mod.lpCutoffHz * fastExp2(engage * pluckSpanOct),
                                kFilterCutoffMinHz, kFilterCutoffMaxHz);
        const float pluckRes = engage * 0.72f * pluck * (0.35f + 0.65f * env);
        mod.lpResonance = clampf(mod.lpResonance + pluckRes, 0.0f, 0.90f);
    }
    return mod;
}

RUIDO_ALWAYS_INLINE ModulatedFilterParams
computeLiveVoiceFilter(const VoiceParams& vp, float pressure, float pitchMode, float lfoARaw,
                       float lfoBRaw = 0.0f, const FilterMacroContext& macros = {}) noexcept {
    const float dA = computeLiveLfoADepth(vp, lfoBRaw);
    float hpBase = (vp.lfoTarget == LfoTarget::HpCutoff)
                       ? applyLfoCutoffHz(vp.hpCutoffHz, dA, lfoARaw)
                       : vp.hpCutoffHz;
    if (vp.lfoBTarget == LfoTarget::HpCutoff) {
        hpBase = applyLfoCutoffHz(hpBase, vp.lfoBDepth, lfoBRaw);
    }
    float hpResBase = (vp.lfoTarget == LfoTarget::HpResonance)
                          ? applyLfoResonance(vp.hpResonance, dA, lfoARaw)
                          : vp.hpResonance;
    if (vp.lfoBTarget == LfoTarget::HpResonance) {
        hpResBase = applyLfoResonance(hpResBase, vp.lfoBDepth, lfoBRaw);
    }
    float lpBase = (vp.lfoTarget == LfoTarget::LpCutoff)
                       ? applyLfoCutoffHz(vp.lpCutoffHz, dA, lfoARaw)
                       : vp.lpCutoffHz;
    if (vp.lfoBTarget == LfoTarget::LpCutoff) {
        lpBase = applyLfoCutoffHz(lpBase, vp.lfoBDepth, lfoBRaw);
    }
    float lpResBase = (vp.lfoTarget == LfoTarget::LpResonance)
                          ? applyLfoResonance(vp.lpResonance, dA, lfoARaw)
                          : vp.lpResonance;
    if (vp.lfoBTarget == LfoTarget::LpResonance) {
        lpResBase = applyLfoResonance(lpResBase, vp.lfoBDepth, lfoBRaw);
    }
    return applyFilterPerformanceMacros(
        computeVoicePressureMacro(hpBase, hpResBase, lpBase, lpResBase, pressure,
                                  vp.muted ? 0.0f : 1.0f, pitchMode),
        macros);
}

inline constexpr bool isFilterLfoTarget(LfoTarget t) noexcept {
    return t == LfoTarget::LpCutoff || t == LfoTarget::LpResonance || t == LfoTarget::HpCutoff ||
           t == LfoTarget::HpResonance;
}

inline bool isVoiceFilterActive(const VoiceParams& vp, float pressure, float pitchMode) noexcept {
    if (vp.hpCutoffHz > 2.05f || vp.lpCutoffHz < 27900.0f || vp.hpResonance > 0.002f ||
        vp.lpResonance > 0.002f) {
        return true;
    }
    if ((vp.lfoDepth > 0.002f && isFilterLfoTarget(vp.lfoTarget)) ||
        (vp.lfoBDepth > 0.002f && isFilterLfoTarget(vp.lfoBTarget))) {
        return true;
    }
    const float fp = clampf(pressure, 0.0f, 1.0f) * (1.0f - clampf(pitchMode, 0.0f, 1.0f));
    return fp > 0.002f;
}

} // namespace ruido
