# Hokuyo URG-04LX-UG01
# Author: Stefan Laible
#
# Specifications
#
# Power source         USB power via VBUS
# Current consumption  0.5A nominal (rush current 0.8A)
# Detection range      60 to 4095 (mm) 
# Laser wavelength     785nm, Class 1
# Scan angle           240°
# Scan time            100msec/scan (10.0Hz)
# Resolution           1mm
# Angular Resolution   0.36°
# Interface            USB 2.0
# Weight               5.64 oz (160 g) 
# Size (wxdxh)         50 x 50 x 70 mm
# Guaranteed Accuracy (min to 1m)  ±30mm
# Guaranteed Accuracy (1m to max)  3% of detected distance

define hokuyoURG_laser ranger
#define hokuyoURG_laser laser
(
  sensor
  (
    range_min 0.06
    range_max 4.095
    fov 240.0
    samples 660
  )

  size [0.05 0.05 0.07]	
  color "gray20"
)

