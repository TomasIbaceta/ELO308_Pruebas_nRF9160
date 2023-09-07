"""
@brief example to take the points generated by a gps and plotting them on a map
@author Tomas Ibaceta

@notes

*Mostly ai generated to get folium basic syntax running, then 
modified to make it easier to copy and paste the taken data.
*example data taken from copying and pasting the output of "04.PruebaFuncional"
once the gps gets a fix and starts sending data through terminal.
"""

import folium
import webbrowser
import os

#--------------- Functions -------------------

"""
@brief takes a list of positions on the format [ (lat_1, lon_1), (lat_2, lon_2), etc  ]
and returns (average_lat, average_lon)
"""
def average_position_list(position_list):
    lat_list = [lat for (lat, lon) in position_list]
    lon_list = [lon for (lat, lon) in position_list]
    average_lat = sum(lat_list)/len(lat_list)
    average_lon = sum(lon_list)/len(lon_list)
    return (average_lat, average_lon)

#--------------- Program -------------------

#paste here the results of the console.
position_list = [
  (-33.443339, -70.566368), 
  (-33.443360, -70.566419), 
  (-33.443332, -70.566439) 
]

avg_lat, avg_lon = average_position_list(position_list)

# Create a map centered around the average latitude and longitude
m = folium.Map(location=[avg_lat, avg_lon], zoom_start=5)

# Add markers for each point
for index,point in enumerate(position_list):
    folium.Marker(point, tooltip = f"Point {index}").add_to(m)
    
# Draw lines between the points
folium.PolyLine(position_list, color="blue", weight=2.5, opacity=1).add_to(m)

# Save the map to an HTML file
m.save("map.html")

# After saving the map to an HTML file, open it:
file_path = os.path.abspath("map.html")
webbrowser.open('file://' + file_path)