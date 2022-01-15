# -*- coding: utf-8 -*-
"""
Created on Sat Oct  2 16:02:17 2021

@author: jonathan
"""

# Run this app with `python app.py` and
# visit http://127.0.0.1:7000/ in your web browser.

import dash
import dash_core_components as dcc
import dash_html_components as html
from dash.dependencies import Input, Output, State
import plotly.express as px
import pandas as pd
import serial
import time
import threading
import queue

port = 'COM7'   # Port of Ground Station, check name in Arduino IDE > Tools > Port
baud = 115200   # baud rate (e.g. 9600 or 115200)

# Data format (of radio packet)
format = [
    "millis",
    "count",
    "linx",
    "liny",
    "linz",
    "rssi",
]

# index format as key value pairs
index = {}
for i, entry in enumerate(format):
    index[entry] = i

# dataframe to store packets on the server
df = pd.DataFrame(columns=format, dtype='float')


# Thread to constantly read from ground station
def streamReader(queue):
    # Stack overflow solution:
    # https://stackoverflow.com/questions/61166544/readline-in-pyserial-sometimes-captures-incomplete-values-being-streamed-from
    while True:
        # Add each line to the queue
        time.sleep(.001)                    # delay of 1ms
        val = ser.readline()                # read complete line from serial output
        while not '\\n' in str(val):         # check if full data is received.
            # This loop is entered only if serial read value doesn't contain \n
            time.sleep(.001)
            temp = ser.readline()           # check for serial output.
            if temp.decode():               # if temp is not empty.
                val = (val.decode()+temp.decode()).encode()
        val = val.decode().strip()

        # queue module is already thread safe, don't need locks
        queue.put(val)

# queue for passing packets from stream reader to batch writer
queue = queue.Queue()

# Setup stream reader thread
readerThread = threading.Thread(target=streamReader, args=(queue,), name="streamReader")


# Establish a serial connection to ground station
ser = serial.Serial(port=port,
                    baudrate=baud,
                    timeout=0)


# Create a new Dash web app instance
app = dash.Dash(__name__)

# define a line plot
fig = px.line(df, x="count", y="linx", title="X acceleration")

# layout the components of the web page
app.layout = html.Div(children=[
    html.H1(id='header', children='Live Data'),
    html.H2(id='count', children=''),

    html.Div(children='''
        Data streamed from serial port and plotted every second.
    '''),

    # add our line plot to the page
    dcc.Graph(
        id='line-graph',
        figure=fig
    ),
    # this will trigger a function call to update our data every 1 sec
    dcc.Interval(
        id='interval',
        interval=1000,  # milliseconds
        n_intervals=0
    )
])

# Callback function every interval
# Returns updated figure
@app.callback(Output('line-graph', 'figure'),
              Output('count', 'children'),
              Input('interval', 'n_intervals'),
              State('line-graph', 'figure'))
def update(n, figure):
    # populate figure if first update
    if n == 0:
        figure['data'].append({'x': [], 'y': []})

    items = 0
    while not queue.empty():
        items += 1
        line = queue.get()
        data = line.split(',')
        df.loc[len(df)] = data

    print("Read {} items".format(items))
    #print(df)

    # set the figure data to the most current data
    figure['data'][0]['x'] = df['count'].tolist()
    figure['data'][0]['y'] = df['linx'].tolist()

    # return the figure with the updated data
    return figure, str(df['count'].values[-1])

# run the web app
if __name__ == '__main__':
    readerThread.start()
    app.run_server(port="7000", debug=True, use_reloader=False)