# coding: utf-8

"""
    Coordinate transformations used in flight dynamics
"""

import numpy as np
from numpy import sin, cos, tan, deg2rad


def lla2ecef(lat, lng, h):
    """
    Calculates geocentric coordinates (ECEF - Earth Centered, Earth Fixed)
    for a given set of latitude, longitude and altitude inputs, following
    the WGS84 system.

    Parameters
    ----------
    lat : float
        latitude in degrees
    lng : float
        longitude in degrees
    h : float
        geometric altitude above sea level in meters

    Returns
    -------
    array-like
        ECEF coordinates in meters
    """

    if isinstance(lat, float) and (abs(lat) > 90):
            raise ValueError('latitude should be -90 <= latitude <= 90')

    if isinstance(lng, float) and ((lng < 0) or (lng > 359)):
            raise ValueError('longitude should be 0 <= longitude <= 359')

    if isinstance(h, float) and ((h < 0) or (h > 24000)):
            raise ValueError('pressure model is only valid if 0 <= h <= 24000')

    a = 6378137  # [m] Earth equatorial axis
    b = 6356752.3142  # [m] Earth polar axis
    e = 0.081819190842622  # Earth eccentricity

    lat = deg2rad(lat)  # degrees to radians
    lng = deg2rad(lng)  # degrees to radians

    N = a / (1 - (e * sin(lat))**2)**(.5)

    x = (N + h) * cos(lat) * cos(lng)
    y = (N + h) * cos(lat) * sin(lng)
    z = (((b/a)**2) * N + h) * sin(lat)

    return np.array([x, y, z])


def ned2ecef(v_ned, lat, lng):
    """
    Converts vector from local geodetic horizon reference frame (NED - North,
    East, Down) at a given latitude and longitude to geocentric coordinates
    (ECEF - Earth Centered, Earth Fixed).

    Parameters
    ----------
    v_ned: array-like
        vector expressed in NED coordinates
    lat : float
        latitude in degrees
    lng : float
        longitude in degrees

    Returns
    -------
    v_ecef : array-like
        vector expressed in ECEF coordinates
    """
    if isinstance(lat, float) and (abs(lat) > 90):
            raise ValueError('latitude should be -90 <= latitude <= 90')

    if isinstance(lng, float) and ((lng < 0) or (lng > 359)):
            raise ValueError('longitude should be 0 <= longitude <= 359')

    lat = deg2rad(lat)
    lng = deg2rad(lng)

    Lne = np.array([[-sin(lat) * cos(lng), -sin(lat) * sin(lng), cos(lat)],
                    [-sin(lng), cos(lng), 0],
                    [-cos(lat) * cos(lng), -cos(lat) * sin(lng), -sin(lat)]])

    Len = Lne.transpose()
    v_ecef = Len.dot(v_ned)

    return v_ecef
