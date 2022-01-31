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

PORT = 'COM9'       # Port of Ground Station, check name in Arduino IDE > Tools > Port
BAUD_RATE = 115200  # baud rate (e.g. 9600 or 115200)
REFRESH_RATE = 10   # how fast graph updates (milliseconds)
BATCH_RATE = 1000   # how long to wait to batch packets (milliseconds)


# Data format (of radio packet)
format = [
    'count',
    'millis',
    'airspeed',
    'altitude',
    'pitch',
    'roll',
    'xaccel',
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
#readerThread = threading.Thread(target=streamReader, args=(queue,), name='streamReader')
fileThread = threading.Thread(target=fileReader, args=(queue,), name='fileReader')


# Establish a serial connection to ground station
'''
ser = serial.Serial(port=PORT,
                    baudrate=BAUD_RATE,
                    timeout=0)
'''

# Create a new Dash web app instance
app = dash.Dash(__name__)

# define a line plot
#fig = go.Figure()
fig = make_subplots(specs=[[{"secondary_y": True}]])
fig.add_trace(go.Scatter(x=df['count'], y=df['pitch'], mode='lines', name='pitch (deg)'), secondary_y=False)
fig.add_trace(go.Scatter(x=df['count'], y=df['altitude'], mode='lines', name='altitude (m)'), secondary_y=False)
fig.add_trace(go.Scatter(x=df['count'], y=df['airspeed'], mode='lines', name='airspeed (m/s)'), secondary_y=True)
#fig = px.line(df, x='count', y='pitch', title='Live Data')


# layout the components of the web page
app.layout = html.Div(children=[
    html.H2(id='count', children=''),
    html.H2(id='millis', children=''),
    html.H2(id='airspeed', children=''),
    html.H2(id='altitude', children=''),
    html.H2(id='pitch', children=''),
    html.H2(id='roll', children=''),
    html.H2(id='xaccel', children=''),
    html.H2(id='rssi', children=''),
    daq.Gauge(
        id='gauge-1',
        label="Default",
        value=0,
        min=0,
        max=50
    ),
    html.Button('Save Plot', id='save', n_clicks=0),
    # add our line plot to the page
    dcc.Graph(
        id='line-graph',
        figure=fig
    ),
    dcc.Store(
        id='batch-store',
        data={},
        storage_type='memory'
    ),
    dcc.Store(
        id='render-queue',
        data=dict,
        storage_type='memory'
    ),
    # triggers update to batch-store
    dcc.Interval(
        id='batch-interval',
        interval=BATCH_RATE,  # milliseconds
        n_intervals=0
    ),
    # triggers update to line-graph
    dcc.Interval(
        id='render-interval',
        interval=REFRESH_RATE,  # milliseconds
        n_intervals=0
    )
])

@app.callback(
    Output('save', 'children'),
    Input('save', 'n_clicks'),
    State('line-graph', 'figure')
)
def update_output(n_clicks, fig):
    path = "./flight__.html"
    figure = go.Figure(fig)
    figure.write_html(path)


    #print("saved figure as {}".format(path))
    button_text = "Saved plots: {}".format(n_clicks)
    return button_text


# Callback function every interval
# Returns updated figure
@app.callback(Output('batch-store', 'data'),
              Output('count', 'children'),
              Output('millis', 'children'),
              Output('airspeed', 'children'),
              Output('altitude', 'children'),
              Output('pitch', 'children'),
              Output('roll', 'children'),
              Output('xaccel', 'children'),
              Output('rssi', 'children'),
              Output('gauge-1','value'),
              Input('batch-interval', 'n_intervals'))
def batchUpdate(n):
    batchStart = len(df)
    while not queue.empty():
        line = queue.get()
        data = line.split(',')
        #print(data)
        #print(len(data))
        df.loc[len(df)] = data

    batchEnd = len(df)

    # prep to send *all* columns
    batch = df[batchStart:batchEnd]
    dict = batch.to_dict('list')

    #print('Batch updating {} items'.format(batchEnd-batchStart))

    count = 'Count: {}'.format(df['count'].values[-1])
    millis = 'Millis: {}'.format(df['millis'].values[-1])
    airspeed = 'Airspeed: {} m/s'.format(df['airspeed'].values[-1])
    altitude = 'Altitude: {} m'.format(df['altitude'].values[-1])
    pitch = 'Pitch: {} deg'.format(df['pitch'].values[-1])
    roll = 'Roll: {} deg'.format(df['roll'].values[-1])
    xaccel = 'Xaccel: {} m/s^2'.format(df['xaccel'].values[-1])
    rssi = 'Rssi: {}'.format(df['rssi'].values[-1])

    gauge_val = float(df['airspeed'].values[-1])
    print(gauge_val)
    return dict, count, millis, airspeed, altitude, pitch, roll, xaccel, rssi, gauge_val


# appends batch-store to render-queue whenever batch-store is updated
app.clientside_callback(
    # Javascript function executed in client browser
    '''
    function(batchStore, renderQueue) {
        // append batchStore to renderQueue
        keys = Object.keys(batchStore);
        
        for (let i = 0; i < batchStore[keys[0]].length; i++) {
            if (keys[i] in renderQueue) {
                renderQueue[keys[i]] = renderQueue[keys[i]].concat(batchStore[keys[i]]);
            }
            else {
                renderQueue[keys[i]] = []; 
            }
        }

        return renderQueue; 
    } 
    ''',
    Output('render-queue', 'data'),
    Input('batch-store', 'data'),
    State('render-queue', 'data')
)

# takes first item in render_queue and appends it to the line-graph
# triggered by render-interval
app.clientside_callback(
    # Javascript function executed in client browser
    '''
    function(n, renderQueue, figure) {
        console.log(renderQueue['count'].length);
        // deep copy figure
        newFig = JSON.parse(JSON.stringify(figure));
        //console.log(newFig);
        // populate figure if first update
        if (newFig['data'].length == 0) {
            newFig['data'].push({'x': [], 'y': []})
        }
        // add datapoint to figure
        // TODO: parameterize hardcoded count and pitch 
        if (renderQueue['pitch'].length > 0) {
            let x = Number(renderQueue['count'].shift());
            let pitch = Number(renderQueue['pitch'].shift());
            let altitude = Number(renderQueue['altitude'].shift());
            let airspeed = Number(renderQueue['airspeed'].shift());
            newFig['data'][0]['x'].push(x);
            newFig['data'][0]['y'].push(pitch);
            newFig['data'][1]['x'].push(x);
            newFig['data'][1]['y'].push(altitude);
            newFig['data'][2]['x'].push(x);
            newFig['data'][2]['y'].push(airspeed);
        }
        //console.log(figure['data'][0]); 
        return newFig; 
    } 
    ''',
    Output('line-graph', 'figure'),
    Input('render-interval', 'n_intervals'),
    State('render-queue', 'data'),
    State('line-graph', 'figure')
)


# run the web app
if __name__ == '__main__':
    #readerThread.start()
    fileThread.start()
    app.run_server(port='7000', debug=True, use_reloader=False)