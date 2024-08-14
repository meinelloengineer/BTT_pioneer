# Importing Libraries 
import serial 
import time 
import plotly.graph_objects as go
import dash
from dash.dependencies import Output, Input
import dash_core_components as dcc 
import dash_html_components as html 
fig = go.Figure(go.Scattermapbox())


def extra_lat_lon(data):
    data = bytes.decode(data)
    print(data)
    parts = data.split(';')
    latlist = []
    lonlist = []
    lines = data.strip().split('\n')
    for line in lines:
        parts = line.split(';')
        lat = float(parts[0].split('=')[1])
        lon = float(parts[1].split('=')[1])
        latlist.append(lat)
        lonlist.append(lon)
    return latlist, lonlist

    
#############################
############################# Start from here
#############################

arduino = serial.Serial(port='COM5', baudrate=115200, timeout=.1) 
#arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=.1) 
time.sleep(0.5)
arduino.flush()
time.sleep(0.5)
lat = []
lon = []
index = 0
mapbox_access_token = 'pk.eeeeeeeeeeeeeeejj.xhxhxhxhxh.lul.69.xD.nonono'
app = dash.Dash()
app.layout = html.Div( 
    [ 
        dcc.Graph(id = 'live-graph', animate = True), 
        dcc.Interval( 
            id = 'graph-update', 
            interval = 1000, 
            n_intervals = 0
        ), 
    ] 
) 

@app.callback( 
    Output('live-graph', 'figure'), 
    [ Input('graph-update', 'n_intervals') ] 
) 

def omg_wtf(n_intervals):
    while True:
        data = arduino.readall()
        try:
            latitude, longitude = extra_lat_lon(data)
        except:
            latitude = None
            longitude = None
            time.sleep(1)
        print(latitude, longitude)
        if 2>1:
            print("Latitude:", latitude)
            print("Longitude:", longitude)
            lat.append(latitude)
            lon.append(longitude)
            fig.add_trace(go.Scattermapbox(
                lat=lat[-1],
                lon=lon[-1],
                mode='markers',
                marker=go.scattermapbox.Marker(
                    size=4,
                    color='red',
                ),
                showlegend=False
            ))
            fig.update_layout(
                autosize=True,
                hovermode='closest',
                mapbox=dict(
                    accesstoken=mapbox_access_token,
                    bearing=0,
                    center=dict(
                        lat=lat[-1][-1],
                        lon=lon[-1][-1]
                    ),
                    pitch=0,
                    zoom=16
                ),
            )
            return fig
        else:
            f = 1

# Define layout
app.layout = html.Div([
    dcc.Interval(
        id='graph-update',
        interval=1000,  # in milliseconds
        n_intervals=0
    ),
    dcc.Graph(id='live-graph')
])

if __name__ == '__main__': 
    app.run_server(host="0.0.0.0", port="8050")