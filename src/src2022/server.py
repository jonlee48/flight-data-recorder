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
    'millis',
    'count',
    'linx',
    'liny',
    'linz',
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
readerThread = threading.Thread(target=streamReader, args=(queue,), name='streamReader')


# Establish a serial connection to ground station
ser = serial.Serial(port=port,
                    baudrate=baud,
                    timeout=0)


# Create a new Dash web app instance
app = dash.Dash(__name__)

# define a line plot
fig = px.line(df, x='count', y='linx', title='X acceleration')

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
        interval=1000,  # milliseconds
        n_intervals=0
    ),
    # triggers update to line-graph
    dcc.Interval(
        id='render-interval',
        interval=90,  # milliseconds
        n_intervals=0
    )
])

# Callback function every interval
# Returns updated figure
@app.callback(Output('batch-store', 'data'),
              Output('count', 'children'),
              Input('batch-interval', 'n_intervals'))
def batchUpdate(n):
    batchStart = len(df)
    while not queue.empty():
        line = queue.get()
        data = line.split(',')
        df.loc[len(df)] = data

    batchEnd = len(df)

    # prep to send *all* columns
    batch = df[batchStart:batchEnd]
    dict = batch.to_dict('list')

    print('Batch updating {} items'.format(batchEnd-batchStart))
    return dict, 'Count: {}'.format(df['count'].values[-1])


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
        // TODO: parameterize hardcoded count and linx
        if (renderQueue['linx'].length > 0) {
            let x = Number(renderQueue['count'].shift());
            let y = Number(renderQueue['linx'].shift());
            newFig['data'][0]['x'].push(x);
            newFig['data'][0]['y'].push(y);
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
    readerThread.start()
    app.run_server(port='7000', debug=True, use_reloader=False)