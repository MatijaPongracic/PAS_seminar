define astro_laser ranger
(
  sensor(
    pose [ 0.038 0 0 0 ]
    size [ 0.07 0.07 0.05 ]
    range [ 0.0 12.0 ]
    fov 360.0
    samples 360
    color_rgba [ 0 0 1 0.15 ]
  )
  model(
    pose [ 0.0 0.0 0.08 0.0 ]
    size [ 0.07 0.07 0.05 ]
    color "blue"
  )
)

define astro_camera camera
(
  range [ 0.3 3.0 ]
  resolution [ 160 90 ]
  fov [ 87 58 ]
  pantilt [ 0 0 ]
  alwayson 1
  size [ 0.025 0.09 0.025 ]
  color "gray"
)

define astro_robot position
(
  drive "diff"
  localization "odom"

  size [ 0.333 0.337 0.085 ]
  origin [ 0 0 0 0 ]
  gui_nose 1
  color "DimGray"

  astro_laser( pose [ 0 0 0 0 ] )
)

