"""Run with: python -m unittest -v test_protocol"""
import struct
import unittest
from protocol import (Config, build_config_packet, build_coeff_packet,
                      coeff_limits, checksum, parse_int32)


class ProtocolTests(unittest.TestCase):
    def test_affine_known_fpga_example(self):
        # Example from previously tested command: 90 deg, thumbnail+correction+gamma
        packet = build_config_packet(Config(angle=90, thumbnail=True, correction=True, gamma=True))
        self.assertEqual(packet, bytes.fromhex("A5 5A 10 07 5A 00 00 00 00 00 4D 0D"))
        self.assertEqual(len(packet), 12)

    def test_signed_little_endian(self):
        p = build_config_packet(Config(angle=359, tx=-960, ty=-540, thumbnail=True,
                                       correction=False, gamma=True))
        self.assertEqual(len(p), 12)
        self.assertEqual(p[:4], bytes.fromhex("A5 5A 10 05"))
        self.assertEqual(struct.unpack('<Hhh', p[4:10]), (359, -960, -540))
        self.assertEqual(p[-2], checksum(p[2:10]))
        self.assertEqual(p[-1], 0x0D)

    def test_fullscreen_rule(self):
        build_config_packet(Config(tx=100, thumbnail=False))
        for cfg in [Config(angle=15), Config(ty=1), Config(angle=1, ty=1)]:
            with self.assertRaises(ValueError):
                build_config_packet(cfg)

    def test_config_out_of_range(self):
        for cfg in [Config(angle=360, thumbnail=True), Config(tx=-961),
                    Config(ty=541, thumbnail=True)]:
            with self.assertRaises(ValueError):
                build_config_packet(cfg)

    def test_group_packet(self):
        p = build_coeff_packet(655, -1, 100, -100)
        self.assertEqual(len(p), 24)
        self.assertEqual(p[:4], bytes.fromhex('A5 5A 12 00'))
        self.assertEqual(struct.unpack('<iiiiH', p[4:22]), (655, -1, 100, -100, 0))
        self.assertEqual(p[22], checksum(p[2:22]))
        self.assertEqual(p[23], 0x0D)

    def test_coeff_safety(self):
        coeff_limits(655, 0, 0, 0)
        coeff_limits(2621, 0, 0, 0)
        for data in [(2622, 0, 0, 0), (2621, 1, 0, 0), (0, 0, 263, 0),
                     (0, 0, 0, -263), (2621, 0, 262, 0)]:
            with self.assertRaises(ValueError):
                build_coeff_packet(*data)

    def test_int32_inputs(self):
        cases = {'655':655, '0x0000028F':655, '0000028F':655,
                 '0xFFFFFFFF':-1, '-1':-1, '0xFFFFFD71':-655,
                 '-0x1':-1, '0':0, '0x7FFFFFFF':2147483647}
        for s, expected in cases.items():
            self.assertEqual(parse_int32(s), expected, s)
        for s in ['', '3.14', '0x100000000', '2147483648', 'hello']:
            with self.assertRaises(ValueError):
                parse_int32(s)


if __name__ == '__main__':
    unittest.main()
