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
from dash.dependencies import Input, Output, State

# Create a new Dash web app instance
app = dash.Dash(__name__)

# create an empty dataframe with 3 columns
df = pd.DataFrame(columns=["count","data"],dtype='float')

# Establish a serial connection, port 3 with a 9600 bits/second speed
#ser = serial.Serial('/dev/cu.usbmodem14301',9600)
#ser = serial.Serial('COM7 (Adafruit Feather M0)',115200)
ser = serial.Serial('COM7',115200)

# Add a row of fake data so we have something to plot
df.loc[len(df)] = [0,0]

# create a polar plot
#fig = px.scatter_polar(df, r="cm", theta="degrees",range_theta=[0,180], start_angle=0, direction="counterclockwise",range_r=[0,100])
fig = px.line(df, x="count", y="data", title="X acceleration")
fig.show()

count = 0

# This layout defines the components of the web page
app.layout = html.Div(children=[
    html.H1(id='header',children='Live Data'),

    html.Div(children='''
        Data streamed from serial port and plotteed every second.
    '''),

    # add our polar plot to the page
    dcc.Graph(
        id='polar-graph',
        figure=fig
    ),
    # this will trigger a function call to update our data every 1 sec
    dcc.Interval(
        id='interval',
        interval=500, # in millis
        n_intervals=0,
        
    )
])

'''
@app.callback(Output('header', 'children'),
              Input('interval', 'n_intervals'))
def test(n):
    print("testing")
    return str(n)

'''
# this function is called every second
# it takes it n (the nth time it's called) and the current figure
# returns the updated figure
@app.callback(Output('polar-graph', 'figure'),
              Input('interval', 'n_intervals'),
              State('polar-graph', 'figure'))
def update(n, figure):
    global count
    count += 1
    line = ser.readline().decode('UTF-8')


    if (count < 20):
        print("count{} data {}".format(count,line.strip()))
        return figure
    
    line = line.strip()
    print(line)
    sep = line.split(',')
    data = line.strip().split(',')[0]
    #print(data)
    df.loc[len(df)] = [count, data]
    #random = line.strip().split(',')[13]
    #print(random)
    print(df)
    '''
    #print("updating")
    # read any data in the serial buffer
    while(line := ser.readline()):
        line = line.decode('UTF-8')
        print(line)
        line = line.strip()
        # we use a try, except block to catch any errors when processing the data
        # in the event that the data is corrupted instead of crashing,
        # we will clear the serial buffer, print an exception and go on as normal
        try:
            sep = line.split(',')
            data = line.strip().split(',')[0]
            #print(data)
            df.loc[len(df)] = [count, data]
            #random = line.strip().split(',')[13]
            #print(random)
            #print(df)
        except Exception as e:
            ser.flush()
            print(e)
            break
    '''
    # set the figure data to the most current data
    figure['data'][0]['x'] = df['count'].tolist()
    figure['data'][0]['y'] = df['data'].tolist()
    

    # return the figure with the updated data
    return figure

# run the web app
if __name__ == '__main__':
    app.run_server(port="9000",debug=True, use_reloader=False)