## Setup Guide
Link to see setup is here: https://swri-robotics.github.io/mapviz/

To just install the needed dependencies, run the following commands:
```bash
sudo apt-get install ros-iron-mapviz \
                       ros-iron-mapviz-plugins \
                       ros-iron-tile-map \
                       ros-iron-multires-image
```

### Running:
- Make sure the code is built and sourced. 
- run `ros2 launch maxwell_mapviz mapviz.launch.py`

* You may need to get an api key to use the mapviz plugin. You can get one from here: https://developers.google.com/maps/documentation/javascript/get-api-key 


