# Vision Modul

**Szenario:** Search-and-Rescue Robotics

Dieses ROS Modul dient dazu, mittels [OpenCV](https://opencv.org/) in einem Gebäude Menschen (blaue Quadrate) und Feuer (rote Dreiecke) zu erkennen und deren Koordinaten mit anderen Modulen zu teilen.

## Abhängigkeiten

- `ros-noetic-desktop-full`
- `turtlebot3_msgs`

Die erforderlichen Simulationsdateien sind als Git-Submodul eingerichtet.

Eventuell müssen noch `ros-noetic-amcl` und `ros-noetic-move-base` installiert werden, um die Musterlösungen auszuführen.


## Bauen & Ausführen

1. `catkin_make`
2. `source devel/setup.bash`
3. `rosrun vision vision_node`

### Simulation Starten

Ähnlich wie oben muss mit `catkin` der Code kompiliert werden und Umgebunbgseinstellungen mit `source` festgelegt werden.

Dann beispielsweise mit `roslaunch hwp_turtlebot3_gazebo hwp_rescue_scenario.launch` das Szenario `hwp_rescue_scenario` starten.
