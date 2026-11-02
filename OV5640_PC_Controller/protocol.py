"""OV5640 / PGL50H UART protocol, matched to video_cmd_ctrl.v (2026-10-27).

12-byte affine: A5 5A 10 flags angle_u16_le tx_i16_le ty_i16_le XOR 0D
24-byte group:  A5 5A 12 00 k1_i32_le k2_i32_le p1_i32_le p2_i32_le 0000 XOR 0D
XOR includes packet type and all bytes preceding the checksum, excluding A5 5A.
No readback/ACK is defined in the inspected FPGA RTL.
"""
from __future__ import annotations

from dataclasses import dataclass
import re
import struct
from functools import reduce
from operator import xor

MAGIC = bytes.fromhex("A5 5A")
TAIL = 0x0D
BAUD = 115200


@dataclass(frozen=True)
class Config:
    angle: int = 0
    tx: int = 0
    ty: int = 0
    thumbnail: bool = False
    correction: bool = True
    gamma: bool = True


def _require_range(name: str, value: int, low: int, high: int) -> None:
    if type(value) is not int or not (low <= value <= high):
        raise ValueError(f"{name} 必须是 {low}～{high} 的整数，当前为 {value!r}")


def checksum(payload: bytes) -> int:
    """XOR every payload byte; payload starts with packet type (0x10/0x12)."""
    return reduce(xor, payload, 0)


def build_config_packet(cfg: Config) -> bytes:
    _require_range("旋转角度", cfg.angle, 0, 359)
    _require_range("水平平移", cfg.tx, -960, 960)
    _require_range("垂直平移", cfg.ty, -540, 540)
    # This is an FPGA guard, not merely a UX preference.
    if not cfg.thumbnail and (cfg.angle != 0 or cfg.ty != 0):
        raise ValueError("FPGA 协议约束：缩略图关闭时，角度必须为 0，垂直平移必须为 0；水平平移仍可使用。")
    flags = (int(cfg.thumbnail) << 0) | (int(cfg.correction) << 1) | (int(cfg.gamma) << 2)
    payload = bytes([0x10, flags]) + struct.pack("<Hhh", cfg.angle, cfg.tx, cfg.ty)
    assert len(payload) == 8
    return MAGIC + payload + bytes([checksum(payload), TAIL])


def parse_int32(text: str) -> int:
    """Accept signed decimal, -0xHEX, 0xHEX (32-bit two's complement), or hex containing A-F.

    Unprefixed digit-only strings are interpreted as decimal. For e.g. 0000028F,
    A-F forces hexadecimal interpretation. Use 0x028F for unambiguous hex.
    """
    raw = text.strip().replace("_", "")
    if not raw:
        raise ValueError("参数不能为空")
    if re.fullmatch(r"[+-]?0[xX][0-9a-fA-F]+", raw):
        value = int(raw, 16)
        if raw[0] not in "+-" and value > 0xFFFFFFFF:
            raise ValueError("十六进制参数超过 32 位")
        if raw[0] not in "+-" and value >= 0x80000000:
            value -= 0x100000000
    elif re.fullmatch(r"[+-]?[0-9]+", raw):
        value = int(raw, 10)
    elif re.fullmatch(r"[0-9a-fA-F]+", raw) and re.search(r"[a-fA-F]", raw):
        value = int(raw, 16)
        if value > 0xFFFFFFFF:
            raise ValueError("十六进制参数超过 32 位")
        if value >= 0x80000000:
            value -= 0x100000000
    else:
        raise ValueError(f"无效整数：{text!r}。请输入十进制整数或以 0x 开头的十六进制整数")
    _require_range("Q16.16 整数", value, -(1 << 31), (1 << 31) - 1)
    return value


def coeff_limits(k1: int, k2: int, p1: int, p2: int) -> None:
    """Mirror distortion_coeff_guard.v bound + weighted safety budget."""
    for name, value, lim in (("K1", k1, 2621), ("K2", k2, 2621),
                             ("P1", p1, 262), ("P2", p2, 262)):
        _require_range(name, value, -lim, lim)
    budget = 6 * abs(k1) + 4 * abs(k2) + 24 * abs(p1) + 14 * abs(p2)
    if budget > 15726:
        raise ValueError(f"FPGA 安全预算超限：{budget} > 15726；请降低参数绝对值。")


def build_coeff_packet(k1: int, k2: int, p1: int, p2: int) -> bytes:
    coeff_limits(k1, k2, p1, p2)
    payload = bytes([0x12, 0x00]) + struct.pack("<iiiiH", k1, k2, p1, p2, 0)
    assert len(payload) == 20
    packet = MAGIC + payload + bytes([checksum(payload), TAIL])
    assert len(packet) == 24
    return packet


def hex_bytes(packet: bytes) -> str:
    return packet.hex(" ").upper()


def format_coeff(value: int) -> str:
    return f"{value}  |  0x{value & 0xFFFFFFFF:08X}  |  {value / 65536:+.6f}"
