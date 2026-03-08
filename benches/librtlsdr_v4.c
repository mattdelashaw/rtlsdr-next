#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * librtlsdr_v4_convert
 *
 * This is the LITERALLY the core conversion logic from the RTL-SDR Blog V4 branch.
 * It uses a static lookup table (LUT) to map 8-bit unsigned I/Q bytes
 * to 32-bit floats centered at 0.0.
 *
 * Scaling: (x - 127.5f) / 127.5f
 */

static float lut[256];
static bool lut_initialized = false;

static void init_lut(void) {
    for (int i = 0; i < 256; i++) {
        lut[i] = ((float)i - 127.5f) / 127.5f;
    }
    lut_initialized = true;
}

// ── Standard Conversion ───────────────────────────────────────────
void librtlsdr_v4_convert(const uint8_t *src, float *dst, size_t len) {
    if (!lut_initialized) {
        init_lut();
    }
    for (size_t i = 0; i < len; i++) {
        dst[i] = lut[src[i]];
    }
}

// ── V4 Bridge Conversion (with Inversion) ─────────────────────────
void librtlsdr_v4_bridge_convert_inverted(const uint8_t *src, float *dst, size_t len) {
    // Pass 1: Standard LUT conversion
    librtlsdr_v4_convert(src, dst, len);

    // Pass 2: Spectral inversion (Q = -Q)
    for (size_t i = 1; i < len; i += 2) {
        dst[i] = -dst[i];
    }
}
