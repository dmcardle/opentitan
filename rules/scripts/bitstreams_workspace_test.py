# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

import os
import unittest
import unittest.mock

from bitstreams_workspace import BitstreamCache


class TestBitstreamCache(unittest.TestCase):

    def test_get_from_cache(self):
        BITSTREAM_ORIG = 'lowrisc_systems_chip_earlgrey_cw310_0.1.bit.orig'
        BITSTREAM_SPLICE = 'lowrisc_systems_chip_earlgrey_cw310_0.1.bit.splice'

        MOCKED_OS_WALK_RETURN = [
            # os.walk() yields tuples of the form (root, dir, files).
            ('cache/abcd', [], [BITSTREAM_ORIG, BITSTREAM_SPLICE]),
        ]
        os.walk = unittest.mock.MagicMock(name='os.walk',
                                          return_value=MOCKED_OS_WALK_RETURN)

        cache = BitstreamCache('/',
                               '/tmp/cache/opentitan-bitstreams',
                               'latest.txt',
                               offline=True)
        cache.InitRepository = unittest.mock.MagicMock(name='method')

        cached_files = cache.GetFromCache('abcd')

        # This is more of an implementation detail, but it verifies that we hit
        # the the mocked `os.walk` function as expected.
        os.walk.assert_called_once_with('cache/abcd')

        self.assertEqual(
            cached_files, {
                'orig': os.path.join('cache', 'abcd', BITSTREAM_ORIG),
                'splice': os.path.join('cache', 'abcd', BITSTREAM_SPLICE),
            })

        os.walk.assert_called_once_with('cache/abcd')

    def test_write_build_file(self):
        BITSTREAM_ORIG = 'lowrisc_systems_chip_earlgrey_cw310_0.1.bit.orig'
        BITSTREAM_SPLICE = 'lowrisc_systems_chip_earlgrey_cw310_0.1.bit.splice'

        MOCKED_OS_WALK_RETURN = [
            # os.walk() yields tuples of the form (root, dir, files).
            ('cache/abcd', [], [BITSTREAM_ORIG, BITSTREAM_SPLICE]),
        ]
        os.walk = unittest.mock.MagicMock(name='os.walk',
                                          return_value=MOCKED_OS_WALK_RETURN)
        mocked_open = unittest.mock.mock_open()

        BitstreamCache._GetDateTimeStr = unittest.mock.MagicMock(
            name='BitstreamCache._GetDateTimeStr',
            return_value='2022-07-14T15:02:54.463801')

        cache = BitstreamCache('/',
                               '/tmp/cache/opentitan-bitstreams',
                               'latest.txt',
                               offline=True)
        cache.InitRepository = unittest.mock.MagicMock(name='method')

        with unittest.mock.patch('builtins.open', mocked_open):
            cache.WriteBuildFile('BUILD.mock', 'abcd')

        # This is more of an implementation detail, but it verifies that we hit
        # the the mocked `os.walk` function as expected.
        os.walk.assert_called_once_with('cache/abcd')

        mocked_open.assert_has_calls([
            unittest.mock.call('BUILD.mock', 'wt'),
            unittest.mock.call().__enter__(),
            unittest.mock.call().write(
                '''# This file was autogenerated. Do not edit!
# Built at 2022-07-14T15:02:54.463801.
# Configured for bitstream: abcd

package(default_visibility = ["//visibility:public"])

exports_files(glob(["cache/**"]))

filegroup(
    name = "bitstream_test_rom",
    srcs = ["cache/abcd/lowrisc_systems_chip_earlgrey_cw310_0.1.bit.orig"],
)

filegroup(
    name = "bitstream_mask_rom",
    srcs = ["cache/abcd/lowrisc_systems_chip_earlgrey_cw310_0.1.bit.splice"],
)
'''),
            unittest.mock.call().__exit__(None, None, None),
        ])


if __name__ == '__main__':
    unittest.main()
