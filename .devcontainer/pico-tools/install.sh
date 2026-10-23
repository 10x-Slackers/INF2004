#!/bin/sh
set -eu

# podman/buildah commits /tmp as 0755 after a Feature's RUN --mount step
# (podman-container-tools/buildah#6503), which stops apt's unprivileged _apt
# user from verifying repository signatures. Restore it for this step; the
# entrypoint restores it again in the running container.
chmod 1777 /tmp

apt-get update
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    picotool usbutils screen picocom gcc-arm-none-eabi binutils-arm-none-eabi libnewlib-arm-none-eabi
rm -rf /var/lib/apt/lists/*

cat > /usr/local/share/pico-tools-entrypoint.sh <<'ENTRYPOINT'
#!/bin/sh
# The image's /tmp is 0755 (see install.sh); fix it before anything needs it.
chmod 1777 /tmp 2>/dev/null || sudo -n chmod 1777 /tmp 2>/dev/null || true
exec "$@"
ENTRYPOINT
chmod 0755 /usr/local/share/pico-tools-entrypoint.sh
echo "Pico tools installed"
