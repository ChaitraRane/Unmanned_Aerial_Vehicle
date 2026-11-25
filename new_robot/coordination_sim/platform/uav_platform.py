"""UAV vehicle side in the SHARED PyBullet world (simulation side only)."""
from __future__ import annotations

import numpy as np
import pybullet as p

from common.geo import GeoFrame
from sim.battery import BatteryModel
from sim.config import DEFAULT_URDF
from sim.flight_controller_sim import SimulatedFlightController
from sim.mixer import QuadMixer
from sim.motor_model import MotorModel
from sim.physics_pybullet import PyBulletVehicle
from sim.sensors import ImuModel, GpsModel, FcEstimator
from sim.sim_core import UavSim
from sim.vehicle_model import VehicleModel


class SharedUavSim(UavSim):
    def __init__(self, cfg, pb, world_info):            # noqa: super().__init__ deliberately not called (it would connect a new client)
        self.cfg = c = cfg
        self.hz = c["sim"]["physics_hz"]
        self.dt = 1.0 / self.hz
        self.ctrl_div = self.hz // c["flight_controller"]["control_hz"]
        self.imu_div = self.hz // c["sensors"]["imu"]["rate_hz"]
        self.gps_div = max(1, round(self.hz / c["sensors"]["gps"]["rate_hz"]))
        self.pb = pb
        self.world_info = world_info
        self.model = VehicleModel(DEFAULT_URDF, c)
        sp = c["spawn"]
        z = -self._lowest_z(self.model) + 0.003 + sp["surface_z_m"] + sp["height_above_ground_m"]
        self.vehicle = PyBulletVehicle(self.pb, c, self.model, pos_enu=(sp["xy_enu_m"][0], sp["xy_enu_m"][1], z), yaw_ned_deg=sp["yaw_ned_deg"])
        self.motor = MotorModel(c)
        self.battery = BatteryModel(c)
        km = c["propulsion"]["torque_coeff_kQ"] / c["propulsion"]["thrust_coeff_kT"]
        self.mixer = QuadMixer(self.model.rotor_pos_frd, self.model.spin, km)
        self.fc = SimulatedFlightController(c, self.mixer, MotorModel(c))
        self.geo = GeoFrame.from_config(c)
        self.imu = ImuModel(c, self.model, self.imu_div * self.dt)
        self.gps = GpsModel(c, self.model, self.geo)
        self.estimator = FcEstimator(c)
        self.step_count = 0
        self.u = np.zeros(4)
        self.state = self.vehicle.read_state()
        self.imu_sample = self.imu.sample(self.state)
        self.gps_sample = self.gps.sample(self.state)
        self.motor_thrusts = np.zeros(4)
        self.estimate = self.estimator.estimate(self.state, self.imu_sample.gyro)
        self.fc.est = self.estimate
        self.fc.home_down = float(self.estimate.pos_ned[2])
        self.new_imu = False
        self.new_gps = False
        self.hooks = []

    def step_vehicle(self):
        """UavSim.step() WITHOUT p.stepSimulation: the shared world is stepped once per tick for both vehicles."""
        c = self.cfg
        st = self.state = self.vehicle.read_state()
        st.t = self.t
        self.new_imu = self.new_gps = False
        if self.step_count % self.imu_div == 0:
            self.imu_sample = self.imu.sample(st)
            self.new_imu = True
        if self.step_count % self.gps_div == 0:
            self.gps_sample = self.gps.sample(st)
            self.new_gps = True
        if self.step_count % self.ctrl_div == 0:
            self.estimate = self.estimator.estimate(st, self.imu_sample.gyro)
            self.u = self.fc.update(self.t, self.ctrl_div * self.dt, self.estimate,
                                    self.battery.voltage_filtered_v, self.battery.low_battery_failsafe())
        thrust, torque = self.motor.step(self.u, self.battery.voltage_v, self.fc.armed, self.dt)
        self.motor_thrusts = thrust
        self.battery.step(self.motor.electrical_power(c["propulsion"]["avionics_power_w"] if self.fc.armed else 0.0), self.dt)
        self.vehicle.apply_rotor_forces(thrust, torque, st)

    def after_world_step(self):
        self.vehicle.t += self.dt
        self.step_count += 1
        for h in self.hooks:
            h(self)

    def close(self):
        pass
