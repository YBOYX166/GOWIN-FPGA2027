# Camera preview

OV5640 RGB565 -> DVP_Capture -> RGB565 framebuffer -> 800x480 LCD.

Run `python3 sim/chain_sim.py` for the behavioral capture/framebuffer check.
It is a Python model, not RTL simulation, and does not cover SCCB or PLL.
