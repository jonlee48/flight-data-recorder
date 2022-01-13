# -*- coding: utf-8 -*-
"""
Created on Sat Oct  2 14:48:08 2021

@author: jonathan
"""

# Run this app with `python app.py` and
# visit http://127.0.0.1:8050/ in your web browser.

import dash
import dash_core_components as dcc
import dash_html_components as html
import plotly.express as px
import pandas as pd
import serial
from dash.dependencies import Input, Output, State

app = dash.Dash(__name__)

data = pd.DataFrame(columns = ['cm', 'degrees'])
input_file_path = 'data.txt'
#data = pd.read_csv(input_file_path, sep=",", header=None)
#data.columns = ['cm', 'degrees']


#ser = serial.Serial('COM3',9600) # change to serial port Arduino is connected to

fig = px.line_polar(data, r="cm", theta="degrees", line_close=True,
                       range_theta=[0,180], start_angle=180)

app.layout = html.Div(children=[
    html.H1(children='Polar Graph'),
    dcc.Graph(
        id='polar-graph',
        figure=fig
    ),
    
    dcc.Interval(
            id='interval-component',
            interval=1000, # in milliseconds
            n_intervals=0
    )
])
             
@app.callback(Output('polar-graph', 'figure'),
              Input('interval-component', 'n_intervals'),
              State('polar-graph', 'figure'))
def update_graph(n, figure):
    global data
    #line = ser.readline().decode('UTF-8') # Arduino encodes UTF-8 in serial data
    line = ""
    print(line)
    cm, deg = line.strip().split(",")
    data.loc[len(data)] = [cm,deg]
    figure['data']['r'] = data['cm']
    figure['data']['theta'] = data['degrees']
    return figure
    '''
    global data
    line = ser.readline().decode('UTF-8') # Arduino encodes UTF-8 in serial data
    print(line)
    cm, deg = line.strip().split(",")
    data.loc[len(data)] = [cm,deg]
    '''

if __name__ == '__main__':
    app.run_server(debug=True, use_reloader=False)