"""health_f4.h: translate the F4 (dos) firmware's health v16 packet into openpilot 0.11.2's health_t.
Compiled and run on the host with g++; the 0.11.2 layout is mirrored here so the test needs no openpilot tree."""
import os
import shutil
import struct
import subprocess

import pytest

HEADER = os.path.join(os.path.dirname(__file__), '..', 'src', 'pandad', 'health_f4.h')

PROG = r'''
#include <cstdint>
#include <cstdio>
#include <cstring>
#include "health_f4.h"
struct __attribute__((packed)) health_t {   // openpilot 0.11.2 panda/board/health.h
  uint32_t uptime_pkt; uint16_t voltage_pkt; uint16_t current_pkt;
  uint32_t safety_tx_blocked_pkt; uint32_t safety_rx_invalid_pkt; uint32_t tx_buffer_overflow_pkt; uint32_t rx_buffer_overflow_pkt;
  uint32_t faults_pkt; uint16_t flags_pkt; uint8_t car_harness_status_pkt; uint8_t safety_mode_pkt; uint16_t safety_param_pkt;
  uint8_t fault_status_pkt; uint16_t alternative_experience_pkt; uint8_t interrupt_load_pkt; uint8_t fan_power;
  uint16_t spi_error_count_pkt; uint16_t sbu1_voltage_mV; uint16_t sbu2_voltage_mV; uint16_t sound_output_level_pkt; uint8_t temperature_pkt;
};
int main() {
  static_assert(sizeof(health_f4_t) == 58, "v16 packet is 58 bytes");
  static_assert(sizeof(health_t) == 48, "0.11.2 packet is 48 bytes");
  unsigned char buf[58]; size_t n = fread(buf, 1, sizeof(buf), stdin); if (n != 58) return 2;
  health_f4_t v; memcpy(&v, buf, sizeof(v));
  health_t h; memset(&h, 0xAA, sizeof(h));   // poison: every field must be written
  health_from_f4(h, v);
  printf("%u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u\n", h.uptime_pkt, h.voltage_pkt, h.current_pkt,
    h.safety_tx_blocked_pkt, h.safety_rx_invalid_pkt, h.tx_buffer_overflow_pkt, h.rx_buffer_overflow_pkt, h.faults_pkt, h.flags_pkt,
    h.car_harness_status_pkt, h.safety_mode_pkt, h.safety_param_pkt, h.fault_status_pkt, h.alternative_experience_pkt,
    h.interrupt_load_pkt, h.fan_power, h.spi_error_count_pkt, h.sbu1_voltage_mV, h.sbu2_voltage_mV, h.sound_output_level_pkt, h.temperature_pkt);
  return 0;
}
'''

V16 = struct.Struct('<IIIIIIIIBBBBBHBBBHfBBHBHHB')   # firmware health v16 (panda a0848226 board/health.h)


def _run(packet: bytes, tmp_path):
  gxx = shutil.which('g++') or shutil.which('clang++')
  if gxx is None:
    pytest.skip('no C++ compiler')
  src = tmp_path / 'prog.cc'; src.write_text(PROG)
  exe = tmp_path / 'prog'
  subprocess.run([gxx, '-std=c++17', '-Wall', '-Werror', f'-I{os.path.dirname(HEADER)}', str(src), '-o', str(exe)], check=True)
  out = subprocess.run([str(exe)], input=packet, capture_output=True, check=True).stdout
  return [int(x) for x in out.split()]


def test_ignition_harness_and_flags_translate(tmp_path):
  pkt = V16.pack(11840, 15290, 70000, 3, 4, 5, 6, 0x80,            # uptime, voltage, current(clamps to u16 max), counters, faults
                 1, 0, 1, 2, 19, 0x1234, 0,                        # ignition_line, ignition_can, controls_allowed, harness=flipped, safety noOutput, param, fault_status
                 0, 1, 16,                                         # power_save, heartbeat_lost, alternative_experience
                 0.5, 7, 1, 9, 2, 1234, 2345, 1)                   # interrupt_load, fan_power, rx_checks_invalid, spi_err, fan_stall, sbu1, sbu2, som_reset
  (uptime, voltage, current, txb, rxi, txo, rxo, faults, flags, harness, safety, param, fault_status, alt, load, fan, spi, sbu1, sbu2, sound, temp) = _run(pkt, tmp_path)
  assert (uptime, voltage, current) == (11840, 15290, 65535)
  assert (txb, rxi, txo, rxo, faults) == (3, 4, 5, 6, 0x80)
  assert flags == (1 << 0) | (1 << 2) | (1 << 4) | (1 << 5) | (1 << 6)     # ignition line, controls allowed, heartbeat lost, rx checks invalid, som reset
  assert (harness, safety, param, fault_status, alt) == (2, 19, 0x1234, 0, 16)
  assert load == 128 and fan == 7 and spi == 9 and (sbu1, sbu2) == (1234, 2345)
  assert (sound, temp) == (0, 0)


def test_off_car_packet_has_no_ignition(tmp_path):
  pkt = V16.pack(46, 187, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0,  1, 0, 0,  0.02, 0, 0, 0, 0, 3300, 3300, 0)
  out = _run(pkt, tmp_path)
  flags, harness = out[8], out[9]
  assert flags == (1 << 3)      # only power save
  assert harness == 0
