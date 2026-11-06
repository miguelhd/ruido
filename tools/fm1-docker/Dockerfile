# syntax=docker/dockerfile:1
# Reproducible linux/amd64 FM-1 firmware build & verification image for Ruido.
# Shares a single debian:bookworm-slim base across both stages to keep disk footprint minimal (~450 MB).
# Bundles:
#   1. joakimeriksson/fm1-emu (MIT, pinned commit, zero external crates) -> /usr/local/bin/fm1-emu
#   2. JieLi pi32v2 Clang 4.0.1 + binutils toolchain -> /opt/jieli/{pi32v2,common,toolchain}
#   3. JieLi AC79NN_SDK_V1.2.1_2023-12-13 boot blobs (Apache-2.0, SHA-256 verified) -> /opt/jieli/ac79-sdk
#   4. zvenson/dinghy FM-1 platform reference (GPL-3.0-only, pinned commit, stripped of non-GPL logo) -> /opt/dinghy

FROM rust:1-slim-bookworm AS emu-builder
ARG FM1_EMU_COMMIT=1c6e80a816f07f350aa8abbbce5f040992b219f3
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates \
        git \
    && rm -rf /var/lib/apt/lists/* \
    && git clone https://github.com/joakimeriksson/fm1-emu.git /src/fm1-emu \
    && cd /src/fm1-emu \
    && git checkout "${FM1_EMU_COMMIT}" \
    && cargo build --release -p fm1-emu \
    && strip /src/fm1-emu/target/release/fm1-emu

FROM debian:bookworm-slim
ARG DINGHY_COMMIT=f77bf03283bbb2c05294b01cdad68a30b410f93d
ARG AC79_SDK_BRANCH=AC79NN_SDK_V1.2.1_2023-12-13

RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates \
        curl \
        git \
        make \
        python3 \
        python3-pil \
        xz-utils \
    && rm -rf /var/lib/apt/lists/*

# 1. Official JieLi Linux pi32v2 toolchain
RUN mkdir -p /opt/jieli \
    && curl -fL --retry 4 https://pkgman.jieliapp.com/s/linux-toolchain | tar -xJ -C /opt/jieli \
    && TC_DIR="$(ls -d /opt/jieli/jieli-linux-toolchains-* | sort | tail -1)" \
    && ln -sfn "${TC_DIR}" /opt/jieli/toolchain \
    && ln -sfn "${TC_DIR}/pi32v2" /opt/jieli/pi32v2 \
    && ln -sfn "${TC_DIR}/common" /opt/jieli/common

# 2. Sparse-fetch ONLY the 3 Apache-2.0 AC79 SDK boot blobs & verify SHA-256s
RUN set -eux; \
    for attempt in 1 2 3 4; do \
        if git clone --depth 1 --filter=blob:none --sparse \
            --branch "${AC79_SDK_BRANCH}" \
            https://gitee.com/Jieli-Tech/fw-AC79_AIoT_SDK.git /tmp/ac79-sdk; then \
            cd /tmp/ac79-sdk && git sparse-checkout set cpu/wl82/tools && break; \
        fi; \
        rm -rf /tmp/ac79-sdk; \
        sleep 10; \
    done; \
    mkdir -p /opt/jieli/ac79-sdk/cpu/wl82/tools/cfg; \
    cp /tmp/ac79-sdk/cpu/wl82/tools/uboot.boot /opt/jieli/ac79-sdk/cpu/wl82/tools/uboot.boot; \
    cp /tmp/ac79-sdk/cpu/wl82/tools/cfg_tool.bin /opt/jieli/ac79-sdk/cpu/wl82/tools/cfg_tool.bin; \
    cp /tmp/ac79-sdk/cpu/wl82/tools/cfg/eq_cfg_hw.bin /opt/jieli/ac79-sdk/cpu/wl82/tools/cfg/eq_cfg_hw.bin; \
    rm -rf /tmp/ac79-sdk; \
    printf '%s  %s\n' \
        '4e3b4c220dc96641cb5a723f41e68ce41d5261ae9434bb33fbd7f2c59976ded4' '/opt/jieli/ac79-sdk/cpu/wl82/tools/uboot.boot' \
        '276579954f076886a6a7694f65dc71c034a63a2c204b76749065c0ac7b010d1b' '/opt/jieli/ac79-sdk/cpu/wl82/tools/cfg_tool.bin' \
        '41167491bffed4651750719c973d2758adeb9021a5670d02d6a53c85ed80ea7d' '/opt/jieli/ac79-sdk/cpu/wl82/tools/cfg/eq_cfg_hw.bin' \
        | sha256sum -c -

# 3. Pinned zvenson/dinghy reference platform (GPL-3.0-only; remove non-GPL upstream signet logo)
RUN git clone https://github.com/zvenson/dinghy.git /opt/dinghy \
    && cd /opt/dinghy \
    && git checkout "${DINGHY_COMMIT}" \
    && rm -rf /opt/dinghy/.git /opt/dinghy/assets/logo.svg /opt/dinghy/assets/logo.png

# 4. Headless AC791N / FM-1 SoC emulator
COPY --from=emu-builder /src/fm1-emu/target/release/fm1-emu /usr/local/bin/fm1-emu

ENV JIELI_TOOLCHAIN=/opt/jieli/toolchain \
    JIELI_DOCKER=0 \
    AC79_SDK=/opt/jieli/ac79-sdk \
    DINGHY_DIR=/opt/dinghy \
    PATH="/opt/jieli/pi32v2/bin:/opt/jieli/common/bin:${PATH}"

WORKDIR /work
