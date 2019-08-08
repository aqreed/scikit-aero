# coding: utf-8

"""
    Tests ofc the coordinate transformations
"""

from __future__ import division, absolute_import

import numpy as np
from numpy.testing import assert_array_almost_equal
import unittest as ut

from skaero.geometry.coordinates import lla2ecef, ned2ecef


class Test_lla2ecef(ut.TestCase):
    """
        Test function that returns ecef position from lat, long, altitude
    """
    def test_latitude_input_range(self):
        self.assertRaises(ValueError, lla2ecef, 91.0, 0, 0)
        self.assertRaises(ValueError, lla2ecef, -90.001, 0, 0)

    def test_longitude_input_range(self):
        self.assertRaises(ValueError, lla2ecef, 0, -0.1, 0)
        self.assertRaises(ValueError, lla2ecef, 0, 360., 0)

    def test_altitude_input_range(self):
        self.assertRaises(ValueError, lla2ecef, 0, 0, -1.0)

    def test_OX(self):
        a = 6378137  # [m] Earth equatorial axis
        b = 6356752.3142  # [m] Earth polar axis

        # OX-axis
        lat = 0
        lng = 0
        h = 0
        expected_value = np.array([a, 0, 0])
        self.assertTrue(np.allclose(lla2ecef(lat, lng, h),
                                    expected_value))

        lat = 0
        lng = 180
        h = 0
        expected_value = np.array([-a, 0, 0])
        self.assertTrue(np.allclose(lla2ecef(lat, lng, h),
                                    expected_value))

    def test_OY(self):
        a = 6378137  # [m] Earth equatorial axis
        b = 6356752.3142  # [m] Earth polar axis

        # OY-axis
        lat = 0
        lng = 90
        h = 0
        expected_value = np.array([0, a, 0])
        self.assertTrue(np.allclose(lla2ecef(lat, lng, h),
                                    expected_value))

        lat = 0
        lng = 270
        h = 0
        expected_value = np.array([0, -a, 0])
        self.assertTrue(np.allclose(lla2ecef(lat, lng, h),
                                    expected_value))

    def test_OZ(self):
        a = 6378137  # [m] Earth equatorial axis
        b = 6356752.3142  # [m] Earth polar axis

        # OZ-axis
        lat = 90
        lng = 0
        h = 0
        expected_value = np.array([0, 0, b])
        self.assertTrue(np.allclose(lla2ecef(lat, lng, h),
                                    expected_value))

        lat = -90
        lng = 0
        h = 0
        expected_value = np.array([0, 0, -b])
        self.assertTrue(np.allclose(lla2ecef(lat, lng, h),
                                    expected_value))


class Test_ned2ecef(ut.TestCase):
    """
    Test function that transforms ned-basis vectors to ecef-basis
    """
    def test_latitude_input_range(self):
        v_aux = np.array([1, 0, 0])
        self.assertRaises(ValueError, ned2ecef, v_aux, 91.0, 0)
        self.assertRaises(ValueError, ned2ecef, v_aux, -90.001, 0)

    def test_longitude_input_range(self):
        v_aux = np.array([1, 0, 0])
        self.assertRaises(ValueError, ned2ecef, v_aux, 0, -0.1)
        self.assertRaises(ValueError, ned2ecef, v_aux, 0, 360.0)

    def test_1(self):
        lat, lng = 0, 0

        v_ned = np.array([1, 0, 0])
        expected_value = np.array([0, 0, 1])
        self.assertTrue(np.allclose(ned2ecef(v_ned, lat, lng),
                                    expected_value))

        v_ned = np.array([0, 1, 0])
        expected_value = np.array([0, 1, 0])
        self.assertTrue(np.allclose(ned2ecef(v_ned, lat, lng),
                                    expected_value))

        v_ned = np.array([0, 0, 1])
        expected_value = np.array([-1, 0, 0])
        self.assertTrue(np.allclose(ned2ecef(v_ned, lat, lng),
                                    expected_value))

    def test_2(self):
        lat, lng = 0, 90

        v_ned = np.array([1, 0, 0])
        expected_value = np.array([0, 0, 1])
        self.assertTrue(np.allclose(ned2ecef(v_ned, lat, lng),
                                    expected_value))

        v_ned = np.array([0, 1, 0])
        expected_value = np.array([-1, 0, 0])
        self.assertTrue(np.allclose(ned2ecef(v_ned, lat, lng),
                                    expected_value))

        v_ned = np.array([0, 0, 1])
        expected_value = np.array([0, -1, 0])
        self.assertTrue(np.allclose(ned2ecef(v_ned, lat, lng),
                                    expected_value))

    def test_3(self):
        lat, lng = 90, 0

        v_ned = np.array([1, 0, 0])
        expected_value = np.array([-1, 0, 0])
        self.assertTrue(np.allclose(ned2ecef(v_ned, lat, lng),
                                    expected_value))

        v_ned = np.array([0, 1, 0])
        expected_value = np.array([0, 1, 0])
        self.assertTrue(np.allclose(ned2ecef(v_ned, lat, lng),
                                    expected_value))

        v_ned = np.array([0, 0, 1])
        expected_value = np.array([0, 0, -1])
        self.assertTrue(np.allclose(ned2ecef(v_ned, lat, lng),
                                    expected_value))
