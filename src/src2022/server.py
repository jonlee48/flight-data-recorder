# -*- coding: utf-8 -*-
"""
Created on Sat Oct  2 16:02:17 2021

@author: jonathan
"""

# Run this app with `python app.py` and
# visit http://127.0.0.1:9000/ in your web browser.

import dash
#from dash import dcc
#from dash import html
import dash_core_components as dcc
import dash_html_components as html
import plotly.express as px
import pandas as pd
import serial
import time
from dash.dependencies import Input, Output, State

# Create a new Dash web app instance
app = dash.Dash(__name__)

format = [
    "millis",
    "count",
    "linx",
    "liny",
    "linz",
    "rssi",
]

# store format as key value pairs
index = {}
for i, entry in enumerate(format):
    index[entry] = i

# create an empty dataframe with columns
df = pd.DataFrame(columns=format,
                  dtype='float')

# Establish a serial connection
ser = serial.Serial(port='COM7',
                    baudrate=115200,
                    timeout=0)

# Add a row of fake data so we have something to plot
df.loc[len(df)] = [0,0,0,0,0,0] #TODO: remove this if possible

# create a line plot
fig = px.line(df, x="count", y="linx", title="X acceleration")

# This layout defines the components of the web page
app.layout = html.Div(children=[
    html.H1(id='header',children='Live Data'),
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
        interval=100, # in millis
    )
])

# Stack overflow solution:
# https://stackoverflow.com/questions/61166544/readline-in-pyserial-sometimes-captures-incomplete-values-being-streamed-from
def checkPort():
    time.sleep(.001)                    # delay of 1ms
    val = ser.readline()                # read complete line from serial output
    while not '\\n' in str(val):         # check if full data is received.
        # This loop is entered only if serial read value doesn't contain \n
        time.sleep(.001)
        temp = ser.readline()           # check for serial output.
        if not not temp.decode():       # if temp is not empty.
            val = (val.decode()+temp.decode()).encode()
            # required to decode, sum, then encode because
            # long values might require multiple passes
    val = val.decode()                  # decoding from bytes
    val = val.strip()                   # stripping leading and trailing spaces.
    return val

#@app.callback(Output('line-graph', 'figure'),
#              Input('interval', 'n_intervals'),
#              State('line-graph', 'figure'))
def test(n, figure):
    print("printing")
    print("printing12")
    return figure


# this function is called every interval
# it takes it n (the nth time it's called) and the current figure
# returns the updated figure
@app.callback(Output('line-graph', 'figure'),
              Output('count', 'children'),
              Input('interval', 'n_intervals'),
              State('line-graph', 'figure'))
def update(n, figure):
    line = checkPort()
    data = line.split(',')

    print(data[index['count']])


    df.loc[len(df)] = data

    # set the figure data to the most current data
    figure['data'][0]['x'] = df['count'].tolist()
    figure['data'][0]['y'] = df['linx'].tolist()

    # return the figure with the updated data
    return figure, data[index['count']]

# run the web app
if __name__ == '__main__':
    app.run_server(port="7000",debug=True, use_reloader=False)