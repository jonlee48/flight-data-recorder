# -*- coding: utf-8 -*-
"""
Created on Sat Oct  2 16:02:17 2021

@author: jonathan
"""

# Run this app with `python app.py` and
# visit http://127.0.0.1:7000/ in your web browser.

import dash
from dash import html, dcc
import plotly
import plotly.graph_objects as go
from dash.dependencies import Input, Output, State
import plotly.express as px
from plotly.subplots import make_subplots
import pandas as pd
import serial
import time
import threading
import queue
import dash_daq as daq
import os

OFFLINE = False      # If False, read data from file. If True, read data from serial port
PORT = 'COM9'       # Port of Ground Station, check name in Arduino IDE > Tools > Port
BAUD_RATE = 115200  # baud rate (e.g. 9600 or 115200)
REFRESH_RATE = 1    # how fast graph updates (milliseconds)
BATCH_RATE = 1000   # how long to wait to batch packets (milliseconds)
X_POINTS = 600      # how many points on the graph x axis


# Data format (of radio packet)
format = [
    'millis',
    'airspeed',
    'altitude',
    'pitch',
    'roll',
    'rssi',
]

# index format as key value pairs
index = {}
# dict format with empty list
dict = {}
for i, entry in enumerate(format):
    index[entry] = i
    dict[entry] = []


# dataframe to store packets on the server
df = pd.DataFrame(columns=format, dtype='float')

# Thread to read from csv file
def fileReader(queue):
    f = open("data/perlin_data.csv", "r")
    while True:
        queue.put(f.readline())
        time.sleep(.1)

# Thread to constantly read from serial port (ground station transmitting data)
def streamReader(queue):
    # Establish a serial connection to ground station
    ser = serial.Serial(port=PORT, baudrate=BAUD_RATE, timeout=0)

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
readerThread = threading.Thread(target=streamReader, args=(queue,), name='streamReader')
fileThread = threading.Thread(target=fileReader, args=(queue,), name='fileReader')



# Create a new Dash web app instance
app = dash.Dash(__name__)

# define a line plot
fig1 = px.line(df, x='millis', y='airspeed', title='airspeed (mph)')
fig2 = px.line(df, x='millis', y='altitude', title='altitude (ft)')
fig3 = px.line(df, x='millis', y='pitch', title='pitch (deg)')
#fig1 = make_subplots(specs=[[{'secondary_y': True}]])
#fig1.add_trace(go.Scatter(x=df['millis'], y=df['pitch'], mode='lines', name='pitch (deg)'), secondary_y=False)
#fig1.add_trace(go.Scatter(x=df['millis'], y=df['altitude'], mode='lines', name='altitude (m)'), secondary_y=False)
#fig1.add_trace(go.Scatter(x=df['millis'], y=df['airspeed'], mode='lines', name='airspeed (m/s)'), secondary_y=True)


# layout the components of the web page
app.layout = html.Div(children=[
    html.H2(id='millis', children=''),
    html.H2(id='airspeed', children=''),
    html.H2(id='altitude', children=''),
    html.H2(id='pitch', children=''),
    html.H2(id='roll', children=''),
    html.H2(id='rssi', children=''),
    daq.Gauge(
        id='gauge-airspeed',
        showCurrentValue=True,
        units='ft',
        label='Airspeed',
        value=5,
        min=0,
        max=50
    ),
    daq.Gauge(
        id ='gauge-pitch',
        showCurrentValue=True,
        units = 'degrees',
        label = 'pitch',
        value = 6,
        min=-90,
        max=90,
        color = {"gradient":True,"ranges":{"yellow":[-30,30],"green":[-90,-30],"red":[30,90]}},
    ),
    dcc.Input(id='file-input', type='text', value='Flight00.html', debounce=True),
    html.H3(id='save-status', children=''),
    html.Button('Save Plot', id='save', n_clicks=0),
    # add line plots to the page
    dcc.Graph(id='line1',figure=fig1),
    dcc.Graph(id='line2',figure=fig2),
    dcc.Graph(id='line3',figure=fig3),
    # triggers update to batch-store
    dcc.Interval(
        id='batch-interval',
        interval=BATCH_RATE,  # milliseconds
        n_intervals=0
    ),
])



# Callback function every interval
# Returns updated figure
@app.callback(
              # graphs:
              Output('line1', 'figure'),
              Output('line2', 'figure'),
              Output('line3', 'figure'),
              # readouts:
              Output('millis', 'children'),
              Output('airspeed', 'children'),
              Output('altitude', 'children'),
              Output('pitch', 'children'),
              Output('roll', 'children'),
              Output('rssi', 'children'),
              # gauges:
              Output('gauge-airspeed','value'),
              Output('gauge-pitch', 'value'),
              # input trigger:
              Input('batch-interval', 'n_intervals'),
              # feed in figure state:
              State('line1', 'figure'),
              State('line2', 'figure'),
              State('line3', 'figure'))
def batchUpdate(n, fig1, fig2, fig3):
    batchStart = len(df)
    while not queue.empty():
        line = queue.get()
        data = line.split(',')
        if len(data) == len(format):
            data = [float(d) for d in data]
            data[index['altitude']] = data[index['altitude']] * 3.2808 # m to ft
            data[index['airspeed']] = data[index['airspeed']] * 2.23694 # m/s to mph
            df.loc[len(df)] = data
    batchEnd = len(df)

    # update figure data with latest new data
    latest = df[batchStart:batchEnd]
    # update traces
    if len(fig1['data']) == 0:
        fig1['data'].append({'x': [], 'y': []})
        fig2['data'].append({'x': [], 'y': []})
        fig3['data'].append({'x': [], 'y': []})

    fig1['data'][0]['x'].extend(latest.index.values.tolist())
    fig1['data'][0]['y'].extend(latest['airspeed'])
    fig2['data'][0]['x'].extend(latest.index.values.tolist())
    fig2['data'][0]['y'].extend(latest['altitude'])
    fig3['data'][0]['x'].extend(latest.index.values.tolist())
    fig3['data'][0]['y'].extend(latest['pitch'])

    # update readouts
    millis = 'Millis: {}'.format(df['millis'].values[-1])
    airspeed = 'Airspeed: {:.2f} mph'.format(float(df['airspeed'].values[-1]))
    altitude = 'Altitude: {:.2f} ft'.format(float(df['altitude'].values[-1]))
    pitch = 'Pitch: {:.2f} deg'.format(float(df['pitch'].values[-1]))
    roll = 'Roll: {:.2f} deg'.format(float(df['roll'].values[-1]))
    rssi = 'Rssi: {:.2f}'.format(float(df['rssi'].values[-1]))

    # update gauges
    gauge_val = float(df['airspeed'].values[-1])
    gauge_pitch = float(df['pitch'].values[-1])


    return fig1, fig2, fig3, millis, airspeed, altitude, pitch, roll, rssi, gauge_val, gauge_pitch


# save line graphs
@app.callback(
    Output('save-status', 'children'),
    Input('save', 'n_clicks'),
    State('file-input', 'value'),
    State('line1', 'figure'),
    State('line2', 'figure'),
    State('line3', 'figure'),
)
def update_output(n_clicks, filename, fig1, fig2, fig3):
    if n_clicks == 0:
        # in-place of prevent_initial_call=True
        return dash.no_update

    f1 = go.Figure(fig1)
    f2 = go.Figure(fig2)
    f3 = go.Figure(fig3)

    path = 'saves/'
    files = os.listdir(path)

    if filename in files:
        status = '{} already exists, enter a different name.'.format(filename)
    else:
        prefix, format = filename.split('.')
        f1.write_html(path+prefix + '_airspeed' + '.' + format)
        f2.write_html(path+filename + '_altitude' + '.' + format)
        f3.write_html(path+filename + '_pitch' + '.' + format)

        status = 'Saved plots as {}'.format(filename)

    return status


# run the web app
if __name__ == '__main__':
    if OFFLINE:
        fileThread.start()  # read from file instead of ground station
    else:
        readerThread.start()  # read from ground station

    app.run_server(port='7000', debug=True, use_reloader=False)

