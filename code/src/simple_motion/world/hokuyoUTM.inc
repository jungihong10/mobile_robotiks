# Hokuyo UTM-30LX
# Author: Stefan Laible
#
# Specifications
#
# Power source         12V +/-10%
# Current consumption  0.7A (Rush current 1.0A)
# Detection range      0.1 to approximately 60m (<30m guaranteed)
# Laser wavelength     870nm, Class 1
# Scan angle           270°
# Scan time            25msec/scan (40.0Hz)
# Angular Resolution   0.25°
# Interface            USB 2.0
# Weight               8.2 oz (233 g)
# Size (wxdxh)         60 x 60 x 85 mm

define hokuyoUTM_laser ranger
#define hokuyoUTM_laser laser
(
  sensor
  (
    range_min 0.1
    range_max 30.0
    fov 270.0
    samples 1024
  )

  size [0.06 0.06 0.085]	
  color "orange"
)

