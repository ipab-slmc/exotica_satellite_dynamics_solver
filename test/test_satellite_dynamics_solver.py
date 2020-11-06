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
                                          u'{exotica_satellite_dynamics_solver}/resources/robots/satellite/satellite.noarm.urdf',
                                          u'{exotica_satellite_dynamics_solver}/resources/robots/satellite/satellite.srdf',
                                          u'base',
                                          additional_args={'Thrusters': [('exotica/ForceInput', {'ForceDirection': u'0 0 -1', 'LinkName': u'base_to_thruster_bot_0'}),
                                                                         ('exotica/ForceInput', {'ForceDirection': u'0 0 -1', 'LinkName': u'base_to_thruster_top_0'}),
                                                                         ('exotica/ForceInput', {'ForceDirection': u'0 0 -1', 'LinkName': u'base_to_thruster_bot_1'}),
                                                                         ('exotica/ForceInput', {'ForceDirection': u'0 0 -1', 'LinkName': u'base_to_thruster_bot_2'}),
                                                                         ('exotica/ForceInput', {'ForceDirection': u'0 0 -1', 'LinkName': u'base_to_thruster_bot_3'}),
                                                                         ('exotica/ForceInput', {'ForceDirection': u'0 0 -1', 'LinkName': u'base_to_thruster_bot_4'}),
                                                                         ('exotica/ForceInput', {'ForceDirection': u'0 0 -1', 'LinkName': u'base_to_thruster_bot_5'}),
                                                                         ('exotica/ForceInput', {'ForceDirection': u'0 0 -1', 'LinkName': u'base_to_thruster_bot_6'}),
                                                                         ('exotica/ForceInput', {'ForceDirection': u'0 0 -1', 'LinkName': u'base_to_thruster_bot_7'}),
                                                                         ('exotica/ForceInput', {'ForceDirection': u'0 0 -1', 'LinkName': u'base_to_thruster_bot_8'}),
                                                                         ('exotica/ForceInput', {'ForceDirection': u'0 0 -1', 'LinkName': u'base_to_thruster_top_1'}),
                                                                         ('exotica/ForceInput', {'ForceDirection': u'0 0 -1', 'LinkName': u'base_to_thruster_top_2'}),
                                                                         ('exotica/ForceInput', {'ForceDirection': u'0 0 -1', 'LinkName': u'base_to_thruster_top_3'}),
                                                                         ('exotica/ForceInput', {'ForceDirection': u'0 0 -1', 'LinkName': u'base_to_thruster_top_4'}),
                                                                         ('exotica/ForceInput', {'ForceDirection': u'0 0 -1', 'LinkName': u'base_to_thruster_top_5'}),
                                                                         ('exotica/ForceInput', {'ForceDirection': u'0 0 -1', 'LinkName': u'base_to_thruster_top_6'}),
                                                                         ('exotica/ForceInput', {'ForceDirection': u'0 0 -1', 'LinkName': u'base_to_thruster_top_7'}),
                                                                         ('exotica/ForceInput', {'ForceDirection': u'0 0 -1', 'LinkName': u'base_to_thruster_top_8'})]})

    def test_satellite_dynamics_solver_with_arm(self):
        check_dynamics_solver_derivatives('exotica/SatelliteDynamicsSolver',
                                          u'{exotica_satellite_dynamics_solver}/resources/robots/satellite/satellite.urdf',
                                          u'{exotica_satellite_dynamics_solver}/resources/robots/satellite/satellite.srdf',
                                          u'base_with_arm',
                                          additional_args={'Thrusters': [('exotica/ForceInput', {'ForceDirection': u'0 0 -1', 'LinkName': u'base_to_thruster_bot_0'}),
                                                                         ('exotica/ForceInput', {'ForceDirection': u'0 0 -1', 'LinkName': u'base_to_thruster_top_0'}),
                                                                         ('exotica/ForceInput', {'ForceDirection': u'0 0 -1', 'LinkName': u'base_to_thruster_bot_1'}),
                                                                         ('exotica/ForceInput', {'ForceDirection': u'0 0 -1', 'LinkName': u'base_to_thruster_bot_2'}),
                                                                         ('exotica/ForceInput', {'ForceDirection': u'0 0 -1', 'LinkName': u'base_to_thruster_bot_3'}),
                                                                         ('exotica/ForceInput', {'ForceDirection': u'0 0 -1', 'LinkName': u'base_to_thruster_bot_4'}),
                                                                         ('exotica/ForceInput', {'ForceDirection': u'0 0 -1', 'LinkName': u'base_to_thruster_bot_5'}),
                                                                         ('exotica/ForceInput', {'ForceDirection': u'0 0 -1', 'LinkName': u'base_to_thruster_bot_6'}),
                                                                         ('exotica/ForceInput', {'ForceDirection': u'0 0 -1', 'LinkName': u'base_to_thruster_bot_7'}),
                                                                         ('exotica/ForceInput', {'ForceDirection': u'0 0 -1', 'LinkName': u'base_to_thruster_bot_8'}),
                                                                         ('exotica/ForceInput', {'ForceDirection': u'0 0 -1', 'LinkName': u'base_to_thruster_top_1'}),
                                                                         ('exotica/ForceInput', {'ForceDirection': u'0 0 -1', 'LinkName': u'base_to_thruster_top_2'}),
                                                                         ('exotica/ForceInput', {'ForceDirection': u'0 0 -1', 'LinkName': u'base_to_thruster_top_3'}),
                                                                         ('exotica/ForceInput', {'ForceDirection': u'0 0 -1', 'LinkName': u'base_to_thruster_top_4'}),
                                                                         ('exotica/ForceInput', {'ForceDirection': u'0 0 -1', 'LinkName': u'base_to_thruster_top_5'}),
                                                                         ('exotica/ForceInput', {'ForceDirection': u'0 0 -1', 'LinkName': u'base_to_thruster_top_6'}),
                                                                         ('exotica/ForceInput', {'ForceDirection': u'0 0 -1', 'LinkName': u'base_to_thruster_top_7'}),
                                                                         ('exotica/ForceInput', {'ForceDirection': u'0 0 -1', 'LinkName': u'base_to_thruster_top_8'})]})

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'TestDynamicsSolver', TestDynamicsSolver)
