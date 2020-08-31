# src/paravane/core/constants.janet
# Physical, geodetic, and oceanographic reference constants for subsea autonomy.

(def wgs84-a 6378137.0)
(def wgs84-f (/ 1.0 298.257223563))
(def wgs84-b (* wgs84-a (- 1.0 wgs84-f)))
(def wgs84-e2 (- (* 2.0 wgs84-f) (* wgs84-f wgs84-f)))
(def wgs84-e-prime2 (/ (- (* wgs84-a wgs84-a) (* wgs84-b wgs84-b)) (* wgs84-b wgs84-b)))

# Earth angular rotation rate in radians per second (WGS-84)
(def earth-omega 7.2921151467e-5)

# Standard gravitational acceleration at sea level (m/s^2)
(def standard-gravity 9.80665)

# Somigliana normal gravity formula constants on the reference ellipsoid
(def somigliana-ge 9.7803253359)
(def somigliana-k 0.00193185265241)

# Seawater and fluid properties
(def default-seawater-density 1025.0) # kg / m^3 at standard temperature and salinity
(def default-freshwater-density 1000.0) # kg / m^3
(def standard-atmospheric-pressure 101325.0) # Pa (1 atm = 1.01325 bar)
(def pascals-per-decibar 10000.0) # 1 dbar = 10^4 Pa
(def pascals-per-bar 100000.0) # 1 bar = 10^5 Pa

# Underwater acoustics reference standards
(def default-sound-speed 1500.0) # m/s nominal sound speed in seawater
(def acoustic-p-ref 1.0e-6) # Reference pressure in underwater acoustics: 1 micro-Pascal

# Mathematical constants
(def pi math/pi)
(def two-pi (* 2.0 math/pi))
(def half-pi (/ math/pi 2.0))
(def rad-to-deg (/ 180.0 math/pi))
(def deg-to-rad (/ math/pi 180.0))

(defn normal-gravity [lat-rad]
  "Calculate theoretical normal gravity on the WGS-84 ellipsoid at geodetic latitude."
  (def s2 (math/pow (math/sin lat-rad) 2))
  (def num (* somigliana-ge (+ 1.0 (* somigliana-k s2))))
  (def den (math/sqrt (- 1.0 (* wgs84-e2 s2))))
  (/ num den))

(defn gravity-at-depth [lat-rad depth-m]
  "Calculate local gravitational acceleration adjusting for latitude and subsea depth."
  (def gamma (normal-gravity lat-rad))
  # Free-air correction under water column: positive depth increases distance toward Earth center
  # Taylor expansion approximation for depth down to 10,000 meters
  (+ gamma (* 2.226e-6 depth-m)))
