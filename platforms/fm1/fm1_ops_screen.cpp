#include "fm1_ops_screen.h"

extern "C" {

void ruido_ops_render_strip_be(const RuidoOpsState* state, uint16_t y0, uint16_t strip_h,
                               uint16_t* out_strip_be) {
    if (state == nullptr) {
        return;
    }
    ruido::fm1::renderOpsScreenStripImpl(*state, y0, strip_h, out_strip_be, true);
}

#if !defined(RUIDO_LDR_SUBSET)
void ruido_ops_render_strip_le(const RuidoOpsState* state, uint16_t y0, uint16_t strip_h,
                               uint16_t* out_strip_le) {
    if (state == nullptr) {
        return;
    }
    ruido::fm1::renderOpsScreenStripImpl(*state, y0, strip_h, out_strip_le, false);
}
#endif

} // extern "C"
