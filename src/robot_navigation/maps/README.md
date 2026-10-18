# Maps

Put the field map here:

* `field.pgm` — occupancy grid produced by Cartographer or slam_toolbox
* `field.yaml` — map metadata (resolution, origin, thresholds)

`navigation.launch.py` loads `field.yaml` by default.

The field is 3020 x 2000 mm. At 0.05 m/pixel the map image is about
61 x 41 pixels plus a margin for the walls; a 4 x 3 m map at 0.05 m/px
(80 x 60 pixels) is a comfortable size.
