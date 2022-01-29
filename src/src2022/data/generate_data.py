import numpy as np
import pandas as pd
from scipy.interpolate import interp1d
from noise import pnoise1

NUM_POINTS = 100
OCTAVES = 2

if __name__ == '__main__':
    # Data and their ranges
    format = {
        'count': (0,100000),
        'millis': (5000,5000000),
        'airspeed': (0,50),
        'altitude': (0,200),
        'pitch': (-90,90),
        'roll': (-90,90),
        'xaccel': (-4,4),
        'rssi': (-200,0),
    }

    df = pd.DataFrame()

    octaves = 2

    for col_name in format:
        scale = format[col_name]
        print(col_name)

        col_vals = []
        xs = np.linspace(0.1,0.9,NUM_POINTS)
        map = interp1d([0,1], [scale[0], scale[1]])
        for x in xs:
            p = pnoise1(x, OCTAVES)
            col_vals.append(float(map(p)))

        #print(col_vals)
        df[col_name] = col_vals

    print(df)
    df.to_csv('perlin_data.csv',index=False,header=False)