# buggy.inc
# model for outdoor buggy
# Author: Stefan Laible

include "hokuyoUTM.inc"
include "hokuyoURG.inc"

# buggy
define buggy position
(
  # actual size
  size [0.46 0.41 0.275]

  # The buggy's centre of rotation is offset from its centre of area
  #  origin [0 0 0 0]

  # the shape of the buggy
 
  # base plate
  block
  (
    points 4
    point[0] [0 0]
    point[1] [0.45 0]
    point[2] [0.45 0.35]
    point[3] [0 0.35]
    z [0.18 0.205]
    color "dark gray"
  )

  # computer
  block
  (
    points 4
    point[0] [0.065 0.085]
    point[1] [0.285 0.085]
    point[2] [0.285 0.265]
    point[3] [0.065 0.265]
    z [0.205 0.275]
    color "gray"
  )

  # right back wheel
  block
  (
    points 4
    point[0] [-0.005 -0.03]
    point[1] [0.125 -0.03]
    point[2] [0.125 0.045]
    point[3] [-0.005 0.045]
    z [0 0.13]
    color "gray20"
  )

  # right front wheel
  block
  (
    points 4
    point[0] [0.325 -0.03]
    point[1] [0.455 -0.03]
    point[2] [0.455 0.045]
    point[3] [0.325 0.045]
    z [0 0.13]
    color "gray20"
  )

  # left front wheel
  block
  (
    points 4
    point[0] [0.325 0.305]
    point[1] [0.455 0.305]
    point[2] [0.455 0.38]
    point[3] [0.325 0.38]
    z [0 0.13]
    color "gray20"
  )

  # left back wheel
  block
  (
    points 4
    point[0] [-0.005 0.305]
    point[1] [0.125 0.305]
    point[2] [0.125 0.38]
    point[3] [-0.005 00.38]
    z [0 0.13]
    color "gray20"
  )
	
  # car steering model
  drive "diff"
  wheelbase 0.34
  # maximum steering angle in degrees
  max_steer_deg 30.0
	
  # sensors attached to bigbob
  
  hokuyoUTM_laser
  (
    pose [0.255 0 -0.155 0]
  )

#  hokuyoURG_laser
#  (
#    pose [-0.29 0 -0.2 180]
#  )
	
  obstacle_return 1 # can hit things
  laser_return 1 # reflects laser beams
  ranger_return 1 # reflects sonar beams
  blob_return 1	# seen by blobfinders  
)

