# -*- coding: utf-8 -*-
"""
Created on Fri Jul 21 16:39:52 2023

@author: Tomas Ibaceta Guerra
@description:
    This program takes the logged results of current measurement experiments
    and plots them on a standard matplotlib plot.
    
    Experiment Tools:
        *XDM1041 Digital Multimeter
        *DMMEasyControl Software with NI-VISA Drivers (as per DMM Software Guide)
    
    How to use:
        *Make sure that the folder with PowerProfilePlotter.py also includes
        "CurrentMeasurementLog.csv", the logged output to which this
        program is connected. NOTE: DMMEasyControl outputs as ".xls" but
        it sends an error message because it's actually a csv in disguise.
        
    Aditional Notes:
        This is an ad-hoc tool and as such is not supposed to be compatibility
        proof. On this context and stage of development, it would not make sense
        to -for example- Docker this.
"""

import pandas as pd
import plotly.express as px #to get the graph

import plotly.io as io #to use a visual renderer (can't use the spyder one.)

#-------------------------

#Configuration
io.renderers.default='browser'

#constants
A_to_mA = 1000

#------------  Extract the data -------------------

df = pd.read_csv('CurrentMeasurementLog.csv', sep = '\t')
df_extracted = df[["Date/Time", "DCI(A)"]]

#--------------- Process the results ---------------

# convert the current data from A to mA
df_extracted.rename(columns={'DCI(A)': 'DCI(mA)'}, inplace=True)
df_extracted['DCI(mA)'] = df_extracted['DCI(mA)'] * A_to_mA

# convert the time to standard format for further processing

# Two digit milisecond times have a space between them sometimes, so we need
# to remove those spaces before turning them into datetimes (so they can be
# manipulated more easily in the plotter)
df_extracted.rename(columns={'Date/Time': 'Time'}, inplace=True)

#turn into a datetime object
df_extracted['Time'] = df_extracted['Time'].str.replace(' ', '')
df_extracted['Time'] = pd.to_datetime(df_extracted['Time'], format='%Y/%m/%d->%H:%M:%S.%f')

#Turn the absolute time into a relative one, in ms.
df_extracted['Time'] = df_extracted['Time'] - df_extracted['Time'].iloc[0]
df_extracted['Time'] = (df_extracted['Time'].dt.total_seconds())

#it's not completely sorted all the time, remove weird jumps in the line graph:
df_extracted = df_extracted.sort_values(by='Time')

#------------- Plot the results ----------------

# Create the interactive scatter plot
fig = px.line(df_extracted, x='Time', y='DCI(mA)', hover_data=['Time', 'DCI(mA)'], markers=True)

#Other plot style options
fig.update_yaxes(nticks=20)
fig.update_xaxes(nticks=20)

fig.update_xaxes(title_text='Time [s]')
fig.update_yaxes(title_text='Current [mA]')

fig.update_layout(title_text='nRF9160 current[mA] vs time[s] during GNSS operation.')

# Show the plot
fig.show()