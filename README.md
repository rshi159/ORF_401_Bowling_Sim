# ORF_401_Bowling_Sim
### Homework Assignment for ORF401 Electronic Commerce and Sports Analytics

bowling_classes.py sets up lanes and bowling_ball and bowling_pin objects. Also functions for calculating collisions and updating object paramters. Bowiling_sim.py simulates a 10 frame game with logic for the 10th frame. returns a list of length 21 (2\*9 + 1\*3) of the number of pins left standing after each bowl.

[Link to Youtube recording](https://youtu.be/KoEgaf3bGPA)

## Depends on these things:
#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
import sys

matplot plots may be a bit janky depending on your os/version. resize the figure or change the code in the plotting function in run_bowling_sim

## To run:

chmod +x run_bowling_sim.py

./run_bowling_sim.py <True/False> to enable plotting

Returns a list of length 21 with the number of pins remaining.

run_bowling_sim_record is to run a single bowl for video purposes
