import numpy as np
import pandas as pd
from scipy.interpolate import interp1d
from noise import pnoise1
from numpy import interp

'''
Generates fake data for sensor readings
Saves as CSV file
'''


NUM_POINTS = 10000
STEP = 0.01
BASE = 0
FILENAME = 'perlin_data.csv'

# Perlin noise example:
# https://github.com/stephensheridan/python-perlin-noise/blob/master/perlinnoise.py

# Generate N perlin noise values from base with time step
# Step: the increment in time value
# Base: start of time value
# OutputRange: required output range for interpolation [MIN, MAX]
# returns list of perlin noise values
def perlin_vals(base=0, N=1000, step=0.01, outputRange=[0,100]):
    # Store some perlin noise values
    pvalues = []
    perlinRange = [-1, 1]

    for x in range(N):
        # Use 1d pnoise1 to generate noise value
        pval = pnoise1(base)
        # Interpolate/Map the perlin noise values to the required
        # range and store
        pvalues.append(interp(pval, perlinRange, outputRange))
        # Incerement base by step to get next noise value
        base = base + step
    global BASE
    BASE = base
    return pvalues


if __name__ == '__main__':
    # Data and their ranges
    counts = {
        'count': (0, NUM_POINTS,1),
        'millis': (5000, NUM_POINTS*100+5000,100),
    }

    sensors = {
        'airspeed': (0,50),
        'altitude': (0,200),
        'pitch': (-90,90),
        'roll': (-90,90),
        'xaccel': (-4,4),
        'rssi': (-200,0),
    }

    df = pd.DataFrame()

    for col_name in counts:
        _min, _max, _step = counts[col_name]
        df[col_name] = list(range(_min,_max,_step))

    for col_name in sensors:
        scale = sensors[col_name]
        _min, _max = scale
        _range = [_min, _max]

        pvalues = perlin_vals(BASE, NUM_POINTS, STEP, _range)
        df[col_name] = pvalues

    print(df)
    df.to_csv(FILENAME,index=False,header=False)