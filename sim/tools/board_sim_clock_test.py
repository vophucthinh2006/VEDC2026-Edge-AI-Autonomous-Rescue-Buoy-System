#!/usr/bin/env python3
"""The simulated F407's IMU timeout counts on Gazebo's clock. Run in WSL (the board imports the gz modules):
python3 sim/tools/board_sim_clock_test.py"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import stm32_board_sim as bs  # noqa: E402

b = bs.Buoy()
assert not b.fresh(100.0)                                  # no pose yet

b.stamp, b.wall = 50.0, 100.0                              # a pose at sim 50 s, heard at wall 100 s
assert b.fresh(100.05) and not b.fresh(100.2)              # no /clock yet: wall clock, as before

b.sim_now = 50.05
assert b.fresh(101.0)                                      # the host stalled 1 s but so did Gazebo: fresh
b.sim_now = 50.2
assert not b.fresh(100.3)                                  # Gazebo ran on without a pose: IMU lost
b.sim_now = 50.0
assert not b.fresh(100.0 + bs.SIM_STALL_S + 0.1)           # no pose for SIM_STALL_S of wall time: gz sim gone
print("board sim clock: ok")
