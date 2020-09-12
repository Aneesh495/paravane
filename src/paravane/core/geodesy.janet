# src/paravane/core/geodesy.janet
# WGS-84 ellipsoidal geodesy, ECEF, Local Tangent Plane (NED), and UTM conversions.

(import ./constants :as c)
(import ./math :as m)

# -----------------------------------------------------------------------------
# Curvature Radii on WGS-84 Ellipsoid
# -----------------------------------------------------------------------------

(defn prime-vertical-radius [lat-rad]
  "Radius of curvature in the prime vertical N(phi) = a / sqrt(1 - e^2 * sin^2(phi))."
  (def sin-phi (math/sin lat-rad))
  (/ c/wgs84-a (math/sqrt (- 1.0 (* c/wgs84-e2 sin-phi sin-phi)))))

(defn meridian-radius [lat-rad]
  "Radius of curvature along meridian M(phi) = a * (1 - e^2) / (1 - e^2 * sin^2(phi))^(3/2)."
  (def sin-phi (math/sin lat-rad))
  (def den (- 1.0 (* c/wgs84-e2 sin-phi sin-phi)))
  (/ (* c/wgs84-a (- 1.0 c/wgs84-e2)) (* den (math/sqrt den))))

# -----------------------------------------------------------------------------
# Geodetic to/from Earth-Centered Earth-Fixed (ECEF)
# -----------------------------------------------------------------------------

(defn geodetic-to-ecef [lat-rad lon-rad height-m]
  "Convert WGS-84 geodetic latitude, longitude, and ellipsoidal height to ECEF coordinates."
  (def n (prime-vertical-radius lat-rad))
  (def cos-phi (math/cos lat-rad))
  (def sin-phi (math/sin lat-rad))
  (def cos-lam (math/cos lon-rad))
  (def sin-lam (math/sin lon-rad))

  (def x (* (+ n height-m) cos-phi cos-lam))
  (def y (* (+ n height-m) cos-phi sin-lam))
  (def z (* (+ (* n (- 1.0 c/wgs84-e2)) height-m) sin-phi))
  [x y z])

(defn ecef-to-geodetic [ecef-pos]
  "Convert ECEF coordinates [X, Y, Z] to WGS-84 geodetic [lat-rad, lon-rad, height-m] via Bowring closed-form algorithm."
  (def x (in ecef-pos 0))
  (def y (in ecef-pos 1))
  (def z (in ecef-pos 2))

  (def p (math/sqrt (+ (* x x) (* y y))))
  (if (< p 1e-6)
    # Point at or near Earth pole
    (let [lat (if (>= z 0.0) c/half-pi (- c/half-pi))
          h (- (math/abs z) c/wgs84-b)]
      [lat 0.0 h])
    (let [theta (math/atan2 (* z c/wgs84-a) (* p c/wgs84-b))
          s-th (math/sin theta)
          c-th (math/cos theta)
          s3-th (* s-th s-th s-th)
          c3-th (* c-th c-th c-th)
          num (+ z (* c/wgs84-e-prime2 c/wgs84-b s3-th))
          den (- p (* c/wgs84-e2 c/wgs84-a c3-th))
          lat (math/atan2 num den)
          lon (math/atan2 y x)
          n (prime-vertical-radius lat)
          h (- (/ p (math/cos lat)) n)]
      [lat lon h])))

# -----------------------------------------------------------------------------
# Local Tangent Plane: North-East-Down (NED)
# -----------------------------------------------------------------------------

(defn ecef-to-ned-rotation-matrix [lat0-rad lon0-rad]
  "Compute 3x3 rotation matrix from ECEF frame to local North-East-Down (NED) frame at reference origin."
  (def sin-phi (math/sin lat0-rad))
  (def cos-phi (math/cos lat0-rad))
  (def sin-lam (math/sin lon0-rad))
  (def cos-lam (math/cos lon0-rad))

  [(- (* sin-phi cos-lam)) (- (* sin-phi sin-lam)) cos-phi
   (- sin-lam)             cos-lam                 0.0
   (- (* cos-phi cos-lam)) (- (* cos-phi sin-lam)) (- sin-phi)])

(defn geodetic-to-ned [lat-rad lon-rad height-m ref-origin]
  "Compute relative position [north, east, down] in meters given reference origin [lat0, lon0, h0]."
  (def pos-ecef (geodetic-to-ecef lat-rad lon-rad height-m))
  (def ref-ecef (geodetic-to-ecef (in ref-origin 0) (in ref-origin 1) (in ref-origin 2)))
  (def delta-ecef (m/vec3-sub pos-ecef ref-ecef))
  (def r (ecef-to-ned-rotation-matrix (in ref-origin 0) (in ref-origin 1)))
  (m/mat3-mul-vec r delta-ecef))

(defn ned-to-geodetic [ned-pos ref-origin]
  "Convert local NED position [north, east, down] in meters relative to reference origin back to geodetic."
  (def r (ecef-to-ned-rotation-matrix (in ref-origin 0) (in ref-origin 1)))
  (def r-inv (m/mat3-transpose r)) # Orthogonal rotation matrix transpose is inverse
  (def delta-ecef (m/mat3-mul-vec r-inv ned-pos))
  (def ref-ecef (geodetic-to-ecef (in ref-origin 0) (in ref-origin 1) (in ref-origin 2)))
  (def pos-ecef (m/vec3-add ref-ecef delta-ecef))
  (ecef-to-geodetic pos-ecef))

# -----------------------------------------------------------------------------
# Great Circle Navigation
# -----------------------------------------------------------------------------

(defn haversine-distance [lat1-rad lon1-rad lat2-rad lon2-rad]
  "Great-circle distance between two points on spherical Earth model with mean radius 6371008.8m."
  (def dlat (* 0.5 (- lat2-rad lat1-rad)))
  (def dlon (* 0.5 (- lon2-rad lon1-rad)))
  (def a (+ (* (math/sin dlat) (math/sin dlat))
            (* (math/cos lat1-rad) (math/cos lat2-rad)
               (math/sin dlon) (math/sin dlon))))
  (def c-val (* 2.0 (math/atan2 (math/sqrt a) (math/sqrt (max 0.0 (- 1.0 a))))))
  (* 6371008.8 c-val))

(defn initial-bearing [lat1-rad lon1-rad lat2-rad lon2-rad]
  "Initial navigation azimuth bearing from point 1 to point 2 in radians clockwise from true North."
  (def dlon (- lon2-rad lon1-rad))
  (def y (* (math/sin dlon) (math/cos lat2-rad)))
  (def x (- (* (math/cos lat1-rad) (math/sin lat2-rad))
            (* (math/sin lat1-rad) (math/cos lat2-rad) (math/cos dlon))))
  (def brg (math/atan2 y x))
  (if (< brg 0.0) (+ brg c/two-pi) brg))

# -----------------------------------------------------------------------------
# Universal Transverse Mercator (UTM) Conformal Projection
# -----------------------------------------------------------------------------

(defn utm-zone [lon-deg]
  "Determine 6-degree UTM longitudinal zone number (1 to 60)."
  (math/floor (+ (/ (+ lon-deg 180.0) 6.0) 1.0)))

(defn central-meridian-rad [zone]
  "Central meridian for UTM zone in radians."
  (* (- (* (- zone 1) 6.0) 180.0 3.0) c/deg-to-rad))

(defn geodetic-to-utm [lat-rad lon-rad]
  "Project WGS-84 geodetic coordinates to UTM [easting, northing, zone, is-north]."
  (def lon-deg (* lon-rad c/rad-to-deg))
  (def zone (utm-zone lon-deg))
  (def lon0-rad (central-meridian-rad zone))
  (def is-north (>= lat-rad 0.0))

  (def k0 0.9996) # UTM central meridian scale factor
  (def false-easting 500000.0)
  (def false-northing (if is-north 0.0 10000000.0))

  (def a c/wgs84-a)
  (def e2 c/wgs84-e2)
  (def ep2 c/wgs84-e-prime2)

  (def sin-phi (math/sin lat-rad))
  (def cos-phi (math/cos lat-rad))
  (def tan-phi (math/tan lat-rad))

  (def n (/ a (math/sqrt (- 1.0 (* e2 sin-phi sin-phi)))))
  (def t (* tan-phi tan-phi))
  (def c-term (* ep2 cos-phi cos-phi))
  (def a-term (* cos-phi (- lon-rad lon0-rad)))

  # Meridian distance M along ellipsoid
  (def m-dist (* a (- (+ (* (- 1.0 (/ e2 4.0) (/ (* 3.0 e2 e2) 64.0) (/ (* 5.0 e2 e2 e2) 256.0)) lat-rad)
                         (* (+ (/ (* 3.0 e2) 8.0) (/ (* 3.0 e2 e2) 32.0) (/ (* 45.0 e2 e2 e2) 1024.0)) (math/sin (* 4.0 lat-rad)))
                         (* (+ (/ (* 35.0 e2 e2 e2) 3072.0)) (math/sin (* 6.0 lat-rad))))
                      (* (+ (/ (* 3.0 e2) 8.0) (/ (* 3.0 e2 e2) 32.0) (/ (* 45.0 e2 e2 e2) 1024.0)) (math/sin (* 2.0 lat-rad))))))

  (def a2 (* a-term a-term))
  (def a3 (* a2 a-term))
  (def a4 (* a2 a2))
  (def a5 (* a4 a-term))
  (def a6 (* a3 a3))

  (def easting (+ false-easting (* k0 n (+ a-term
                                           (* (- 1.0 t c-term) (/ a3 6.0))
                                           (* (+ 5.0 (* -18.0 t) (* t t) (* 72.0 c-term) (* -58.0 ep2)) (/ a5 120.0))))))

  (def northing (+ false-northing (* k0 (+ m-dist
                                           (* n tan-phi (+ (/ a2 2.0)
                                                           (* (+ 5.0 (- t) (* 9.0 c-term) (* 4.0 c-term c-term)) (/ a4 24.0))
                                                           (* (+ 61.0 (* -58.0 t) (* t t) (* 600.0 c-term) (* -330.0 ep2)) (/ a6 720.0))))))))

  [easting northing zone is-north])
