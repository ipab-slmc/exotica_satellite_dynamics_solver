#!/usr/bin/env python
# coding: utf-8
import roslib
import unittest
from pyexotica.testing import check_dynamics_solver_derivatives

PKG = 'exotica_satellite_dynamics_solver'
roslib.load_manifest(PKG)  # This line is not needed with Catkin.


class TestDynamicsSolver(unittest.TestCase):
    def test_satellite_dynamics_solver(self):
        check_dynamics_solver_derivatives('exotica/SatelliteDynamicsSolver',
                                          u'{exotica_satellite_dynamics_solver}/resources/robots/satellite/satellite.urdf',
                                          u'{exotica_satellite_dynamics_solver}/resources/robots/satellite/satellite.srdf',
                                          u'base_with_arm')

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'TestDynamicsSolver', TestDynamicsSolver)
