"""Compile actual MMIO-facing C sections against deterministic host registers."""
from pathlib import Path
import os
import shlex
import subprocess
import tempfile

root = Path(__file__).resolve().parents[3]
source = (root / 'app_program/subsystem2/drv_motion.c').read_text()


def section(start, end):
    return source[source.index(start):source.index(end, source.index(start))]


prefix = r'''
#include "drv_motion.h"
#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#define UINT unsigned int
#define DRV_MOTION_GPIO_BANKS 4u
#define DRV_MOTION_PINS_PER_BANK 8u
#define DRV_MOTION_GPIO_STATUS 0x100u
#define DRV_MOTION_GPIO_INTR 0x200u
#define DRV_MOTION_ENCODER_MASK 0x3000cu
#define DRV_MOTION_ENCODER_LOCK 4u
#define GPIO_IN 0x300u
static drv_motion_encoders_t g_counts;
static uint32_t raw[4], levels, scenario, callbacks, gpio_reads;
static bool locked;
static struct { uint32_t pins; } g_callback = {1u << 6u};
static void callback(uint32_t pins)
{
    assert(pins == (1u << 6u)); assert(!locked); callbacks++;
}
static drv_motion_gpio_callback_t g_p_callback_isr = callback;
static void fall_a(void) { levels &= ~(1u << 16u); raw[2] |= 4u; }
static uint32_t in_w(uint32_t address)
{
    if (GPIO_IN == address)
    {
        gpio_reads++;
        if (1u == scenario && 2u == gpio_reads) { fall_a(); scenario = 9u; }
        return levels;
    }
    assert(address >= 0x100u && address <= 0x10cu);
    return raw[(address - 0x100u) / 4u];
}
static void out_w(uint32_t address, uint32_t value)
{
    uint32_t bank = (address - 0x200u) / 4u;
    assert(bank < 4u && 0u == (value & ~0xccccccccu));
    if (2u == bank && 2u == scenario) { fall_a(); scenario = 9u; }
    raw[bank] &= ~value;
}
static uint32_t drv_motion_enter(uint32_t lock)
{
    assert(lock == 4u && !locked); locked = true; return 0u;
}
static void drv_motion_leave(uint32_t lock, uint32_t saved)
{
    assert(lock == 4u && saved == 0u && locked); locked = false;
}
uint32_t drv_motion_time_us(void) { assert(locked); return 12345u; }
'''
suffix = r'''
int main(void)
{
    for (uint32_t test = 0u; test < 3u; test++)
    {
        g_counts = (drv_motion_encoders_t){0};
        g_counts.left = 8388608u; g_counts.right = 8388608u;
        g_counts.left_phase = 3u;
        levels = 1u << 16u; /* Left B fell: 11 -> 10. */
        raw[0] = 8u << 24u; /* Other owner's GP6 rising. */
        raw[2] = 0x40u; raw[1] = raw[3] = 0u;
        callbacks = gpio_reads = 0u; scenario = test;
        if (0u == test) { fall_a(); }
        drv_motion_gpio_isr(13u);
        assert(callbacks == 1u && !locked);
        if (test < 2u)
        {
            assert(g_counts.invalid == 1u && g_counts.left_invalid == 1u);
            assert(g_counts.left == 8388608u && g_counts.left_phase == 0u);
        }
        else
        {
            assert(g_counts.invalid == 0u && g_counts.left_phase == 2u);
            assert(raw[2] == 4u); /* New flag survives earlier W1C mask. */
        }
        scenario = 8u;
        drv_motion_gpio_isr(13u);
        assert(raw[2] == 0u && callbacks == 1u);
        if (2u == test) { assert(g_counts.left == 8388610u); }
    }
    for (uint32_t left = 0u; left < 4u; left++)
    {
        for (uint32_t right = 0u; right < 4u; right++)
        {
            levels = ((left >> 1u) << 16u) | ((left & 1u) << 17u) |
                     ((right >> 1u) << 2u) | ((right & 1u) << 3u);
            initialise();
            assert(g_counts.left_phase == left && g_counts.right_phase == right);
            raw[0] = 0x8800u; raw[2] = 0x88u; scenario = 8u;
            drv_motion_gpio_isr(13u);
            assert(g_counts.left_phase == left && g_counts.right_phase == right);
        }
    }
    drv_motion_encoders_t copy = {0};
    assert(drv_motion_sample(&copy) == 12345u && !locked);
    assert(copy.left == g_counts.left && copy.right == g_counts.right);
    puts("PASS actual ISR: W1C interleavings, shared callback, 16 startup phases, locked sample time");
}
'''
init = section('    levels = in_w(GPIO_IN);', '    definition.intatr')
body = (prefix + section('uint32_t drv_motion_sample(', '/* GPIO edge flags') +
        section('static void drv_motion_gpio_isr', 'static void drv_motion_enable_edges') +
        'static void initialise(void) { uint32_t levels;\n' + init + '}\n' + suffix)


def compile_run(folder, name, path, extra=()):
    output = Path(folder) / name
    subprocess.run(shlex.split(os.environ.get('HOST_CC', 'cc')) +
                   ['-std=c99', '-Wall', '-Wextra', '-Werror', '-pedantic'] +
                   shlex.split(os.environ.get('HOST_CFLAGS', '')) +
                   ['-I' + folder, '-I' + str(root / 'app_program/subsystem2'),
                    str(path)] + list(extra) + ['-o', str(output)], check=True)
    subprocess.run([str(output)], check=True)


with tempfile.TemporaryDirectory(prefix='s2-port-') as folder:
    path = Path(folder) / 'irq.c'
    path.write_text(body)
    math = str(root / 'app_program/subsystem2/ctrl_math.c')
    compile_run(folder, 'irq', path, [math])
    (Path(folder) / 'driver_output.inc').write_text(
        source[source.index('static void drv_motion_output('):])
    compile_run(folder, 'output', root / 'testing/subsystem2/host/test_output.c', [math])
