# Vision Modul

[![ROS Version](https://img.shields.io/badge/ROS-Noetic-yellowgreen?logo=ros)](https://wiki.ros.org/noetic)
[![OpenCV](https://img.shields.io/badge/OpenCV-4.2-blue?logo=opencv)](https://opencv.org/opencv-4-2-0/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04%20LTS-orange?logo=ubuntu)](https://releases.ubuntu.com/20.04/)

**Szenario:** Search-and-Rescue Robotics

Dieses ROS Modul dient dazu, mittels [OpenCV](https://opencv.org/) in einem Gebäude Menschen (blaue Quadrate) und Feuer (rote Dreiecke) zu erkennen und deren Koordinaten mit anderen Modulen zu teilen.

Ermittelte Symbole werden gefiltert (um möglicherweise Duplikate zu vermeiden) und dann als `geometry_msgs::PointStamped` auf `/blue_square_pos` und `/red_triangle_pos` veröffentlicht.

## Abhängigkeiten

- `ros-noetic-desktop-full`
- `turtlebot3_msgs`

Die erforderlichen Simulationsdateien sind als Git-Submodul eingerichtet.

Eventuell müssen noch `ros-noetic-navigation` installiert werden, um die Musterlösungen auszuführen.


## Bauen & Ausführen

1. `catkin_make`
2. `source devel/setup.bash`
3. `rosrun vision vision_node`

### Simulation Starten

Ähnlich wie oben muss mit `catkin` der Code kompiliert werden und Umgebunbgseinstellungen mit `source` festgelegt werden.

Dann beispielsweise mit `roslaunch hwp_turtlebot3_gazebo hwp_rescue_scenario.launch` das Szenario `hwp_rescue_scenario` starten.
