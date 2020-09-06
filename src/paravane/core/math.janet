# src/paravane/core/math.janet
# High-precision 3D vector, matrix, quaternion, and numerical algebra for subsea robotics.

(import ./constants :as c)

# -----------------------------------------------------------------------------
# 3D Vector Operations
# -----------------------------------------------------------------------------

(defn vec3 [x y z]
  [x y z])

(defn vec3-x [v] (in v 0))
(defn vec3-y [v] (in v 1))
(defn vec3-z [v] (in v 2))

(def vec3-zero [0.0 0.0 0.0])
(def vec3-unit-x [1.0 0.0 0.0])
(def vec3-unit-y [0.0 1.0 0.0])
(def vec3-unit-z [0.0 0.0 1.0])

(defn vec3-add [a b]
  [(+ (in a 0) (in b 0))
   (+ (in a 1) (in b 1))
   (+ (in a 2) (in b 2))])

(defn vec3-sub [a b]
  [(- (in a 0) (in b 0))
   (- (in a 1) (in b 1))
   (- (in a 2) (in b 2))])

(defn vec3-scale [v s]
  [(* (in v 0) s)
   (* (in v 1) s)
   (* (in v 2) s)])

(defn vec3-neg [v]
  [(- (in v 0))
   (- (in v 1))
   (- (in v 2))])

(defn vec3-dot [a b]
  (+ (* (in a 0) (in b 0))
     (* (in a 1) (in b 1))
     (* (in a 2) (in b 2))))

(defn vec3-cross [a b]
  [(- (* (in a 1) (in b 2)) (* (in a 2) (in b 1)))
   (- (* (in a 2) (in b 0)) (* (in a 0) (in b 2)))
   (- (* (in a 0) (in b 1)) (* (in a 1) (in b 0)))])

(defn vec3-norm2 [v]
  (vec3-dot v v))

(defn vec3-norm [v]
  (math/sqrt (vec3-norm2 v)))

(defn vec3-normalize [v]
  (def n (vec3-norm v))
  (if (> n 1e-15)
    (vec3-scale v (/ 1.0 n))
    vec3-zero))

(defn vec3-dist [a b]
  (vec3-norm (vec3-sub a b)))

(defn vec3-lerp [a b t]
  (vec3-add (vec3-scale a (- 1.0 t)) (vec3-scale b t)))

# -----------------------------------------------------------------------------
# 4D Vector Operations
# -----------------------------------------------------------------------------

(defn vec4 [x y z w]
  [x y z w])

(defn vec4-dot [a b]
  (+ (* (in a 0) (in b 0))
     (* (in a 1) (in b 1))
     (* (in a 2) (in b 2))
     (* (in a 3) (in b 3))))

(defn vec4-norm [v]
  (math/sqrt (vec4-dot v v)))

(defn vec4-normalize [v]
  (def n (vec4-norm v))
  (if (> n 1e-15)
    (let [inv (/ 1.0 n)]
      [(* (in v 0) inv) (* (in v 1) inv) (* (in v 2) inv) (* (in v 3) inv)])
    [0.0 0.0 0.0 0.0]))

# -----------------------------------------------------------------------------
# 3x3 Matrix Operations (Row-Major: [m00 m01 m02 m10 m11 m12 m20 m21 m22])
# -----------------------------------------------------------------------------

(defn mat3 [m00 m01 m02
            m10 m11 m12
            m20 m21 m22]
  [m00 m01 m02
   m10 m11 m12
   m20 m21 m22])

(def mat3-identity
  [1.0 0.0 0.0
   0.0 1.0 0.0
   0.0 0.0 1.0])

(def mat3-zero
  [0.0 0.0 0.0
   0.0 0.0 0.0
   0.0 0.0 0.0])

(defn mat3-diag [d0 d1 d2]
  [ d0  0.0 0.0
   0.0  d1  0.0
   0.0 0.0  d2 ])

(defn mat3-transpose [m]
  [(in m 0) (in m 3) (in m 6)
   (in m 1) (in m 4) (in m 7)
   (in m 2) (in m 5) (in m 8)])

(defn mat3-trace [m]
  (+ (in m 0) (in m 4) (in m 8)))

(defn mat3-add [a b]
  [(+ (in a 0) (in b 0)) (+ (in a 1) (in b 1)) (+ (in a 2) (in b 2))
   (+ (in a 3) (in b 3)) (+ (in a 4) (in b 4)) (+ (in a 5) (in b 5))
   (+ (in a 6) (in b 6)) (+ (in a 7) (in b 7)) (+ (in a 8) (in b 8))])

(defn mat3-sub [a b]
  [(- (in a 0) (in b 0)) (- (in a 1) (in b 1)) (- (in a 2) (in b 2))
   (- (in a 3) (in b 3)) (- (in a 4) (in b 4)) (- (in a 5) (in b 5))
   (- (in a 6) (in b 6)) (- (in a 7) (in b 7)) (- (in a 8) (in b 8))])

(defn mat3-scale [m s]
  [(* (in m 0) s) (* (in m 1) s) (* (in m 2) s)
   (* (in m 3) s) (* (in m 4) s) (* (in m 5) s)
   (* (in m 6) s) (* (in m 7) s) (* (in m 8) s)])

(defn mat3-mul-vec [m v]
  [(+ (* (in m 0) (in v 0)) (* (in m 1) (in v 1)) (* (in m 2) (in v 2)))
   (+ (* (in m 3) (in v 0)) (* (in m 4) (in v 1)) (* (in m 5) (in v 2)))
   (+ (* (in m 6) (in v 0)) (* (in m 7) (in v 1)) (* (in m 8) (in v 2)))])

(defn mat3-mul-mat [a b]
  (def a00 (in a 0)) (def a01 (in a 1)) (def a02 (in a 2))
  (def a10 (in a 3)) (def a11 (in a 4)) (def a12 (in a 5))
  (def a20 (in a 6)) (def a21 (in a 7)) (def a22 (in a 8))

  (def b00 (in b 0)) (def b01 (in b 1)) (def b02 (in b 2))
  (def b10 (in b 3)) (def b11 (in b 4)) (def b12 (in b 5))
  (def b20 (in b 6)) (def b21 (in b 7)) (def b22 (in b 8))

  [(+ (* a00 b00) (* a01 b10) (* a02 b20))
   (+ (* a00 b01) (* a01 b11) (* a02 b21))
   (+ (* a00 b02) (* a01 b12) (* a02 b22))

   (+ (* a10 b00) (* a11 b10) (* a12 b20))
   (+ (* a10 b01) (* a11 b11) (* a12 b21))
   (+ (* a10 b02) (* a11 b12) (* a12 b22))

   (+ (* a20 b00) (* a21 b10) (* a22 b20))
   (+ (* a20 b01) (* a21 b11) (* a22 b21))
   (+ (* a20 b02) (* a21 b12) (* a22 b22))])

(defn mat3-det [m]
  (def m00 (in m 0)) (def m01 (in m 1)) (def m02 (in m 2))
  (def m10 (in m 3)) (def m11 (in m 4)) (def m12 (in m 5))
  (def m20 (in m 6)) (def m21 (in m 7)) (def m22 (in m 8))
  (+ (* m00 (- (* m11 m22) (* m12 m21)))
     (* m01 (- (* m12 m20) (* m10 m22)))
     (* m02 (- (* m10 m21) (* m11 m20)))))

(defn mat3-inv [m]
  (def det (mat3-det m))
  (if (< (math/abs det) 1e-15)
    nil
    (let [inv-det (/ 1.0 det)
          m00 (in m 0) m01 (in m 1) m02 (in m 2)
          m10 (in m 3) m11 (in m 4) m12 (in m 5)
          m20 (in m 6) m21 (in m 7) m22 (in m 8)]
      [(* (- (* m11 m22) (* m12 m21)) inv-det)
       (* (- (* m02 m21) (* m01 m22)) inv-det)
       (* (- (* m01 m12) (* m02 m11)) inv-det)

       (* (- (* m12 m20) (* m10 m22)) inv-det)
       (* (- (* m00 m22) (* m02 m20)) inv-det)
       (* (- (* m02 m10) (* m00 m12)) inv-det)

       (* (- (* m10 m21) (* m11 m20)) inv-det)
       (* (- (* m01 m20) (* m00 m21)) inv-det)
       (* (- (* m00 m11) (* m01 m10)) inv-det)])))

(defn mat3-skew [v]
  "Construct the 3x3 skew-symmetric cross-product matrix [v]_x such that [v]_x * u = v x u."
  (def x (in v 0))
  (def y (in v 1))
  (def z (in v 2))
  [ 0.0  (- z)   y
    z     0.0  (- x)
  (- y)   x     0.0 ])

(defn mat3-from-euler [phi theta psi]
  "Construct rotation matrix R_nb from roll (phi), pitch (theta), and yaw (psi) in Z-Y-X nautical sequence."
  (def cphi (math/cos phi)) (def sphi (math/sin phi))
  (def cth  (math/cos theta)) (def sth  (math/sin theta))
  (def cpsi (math/cos psi)) (def spsi (math/sin psi))

  [(* cpsi cth)
   (- (* cpsi sth sphi) (* spsi cphi))
   (+ (* cpsi sth cphi) (* spsi sphi))

   (* spsi cth)
   (+ (* spsi sth sphi) (* cpsi cphi))
   (- (* spsi sth cphi) (* cpsi sphi))

   (- sth)
   (* cth sphi)
   (* cth cphi)])

(defn mat3-to-euler [m]
  "Extract roll, pitch, and yaw angles from rotation matrix R_nb in radians."
  (def r20 (in m 6))
  (def r21 (in m 7))
  (def r22 (in m 8))
  (def r10 (in m 3))
  (def r00 (in m 0))

  (var theta (- (math/asin (max -1.0 (min 1.0 r20)))))
  (var phi 0.0)
  (var psi 0.0)

  (if (< (math/abs (+ (math/abs r20) -1.0)) 1e-6)
    # Gimbal lock condition: pitch at +/- 90 degrees
    (do
      (set phi 0.0)
      (set psi (math/atan2 (- (in m 1)) (in m 4))))
    (do
      (set phi (math/atan2 r21 r22))
      (set psi (math/atan2 r10 r00))))

  [phi theta psi])

# -----------------------------------------------------------------------------
# Unit Quaternion Algebra (Convention: [w, x, y, z] where w is the scalar component)
# -----------------------------------------------------------------------------

(defn quat [w x y z]
  [w x y z])

(def quat-identity [1.0 0.0 0.0 0.0])

(defn quat-w [q] (in q 0))
(defn quat-x [q] (in q 1))
(defn quat-y [q] (in q 2))
(defn quat-z [q] (in q 3))

(defn quat-norm [q]
  (math/sqrt (+ (* (in q 0) (in q 0))
                (* (in q 1) (in q 1))
                (* (in q 2) (in q 2))
                (* (in q 3) (in q 3)))))

(defn quat-normalize [q]
  (def n (quat-norm q))
  (if (> n 1e-15)
    (let [inv (/ 1.0 n)]
      [(* (in q 0) inv) (* (in q 1) inv) (* (in q 2) inv) (* (in q 3) inv)])
    quat-identity))

(defn quat-conjugate [q]
  [(in q 0) (- (in q 1)) (- (in q 2)) (- (in q 3))])

(defn quat-mul [p q]
  "Compute the Hamilton product of two quaternions."
  (def pw (in p 0)) (def px (in p 1)) (def py (in p 2)) (def pz (in p 3))
  (def qw (in q 0)) (def qx (in q 1)) (def qy (in q 2)) (def qz (in q 3))

  [(- (* pw qw) (* px qx) (* py qy) (* pz qz))
   (+ (* pw qx) (* px qw) (* py qz) (- (* pz qy)))
   (+ (* pw qy) (- (* px qz)) (* py qw) (* pz qx))
   (+ (* pw qz) (* px qy) (- (* py qx)) (* pz qw))])

(defn quat-rot-vec [q v]
  "Rotate a 3D vector v by unit quaternion q (representing coordinate frame rotation)."
  (def qw (in q 0)) (def qx (in q 1)) (def qy (in q 2)) (def qz (in q 3))
  (def qvec [qx qy qz])
  # Formula: v' = v + 2 * cross(qvec, cross(qvec, v) + qw * v)
  (def t (vec3-scale (vec3-cross qvec v) 2.0))
  (vec3-add v (vec3-add (vec3-scale t qw) (vec3-cross qvec t))))

(defn quat-to-mat3 [q]
  "Convert unit quaternion [w, x, y, z] to 3x3 rotation matrix."
  (def w (in q 0)) (def x (in q 1)) (def y (in q 2)) (def z (in q 3))
  (def x2 (* x x)) (def y2 (* y y)) (def z2 (* z z))
  (def xy (* x y)) (def xz (* x z)) (def yz (* y z))
  (def wx (* w x)) (def wy (* w y)) (def wz (* w z))

  [(- 1.0 (* 2.0 (+ y2 z2)))
   (* 2.0 (- xy wz))
   (* 2.0 (+ xz wy))

   (* 2.0 (+ xy wz))
   (- 1.0 (* 2.0 (+ x2 z2)))
   (* 2.0 (- yz wx))

   (* 2.0 (- xz wy))
   (* 2.0 (+ yz wx))
   (- 1.0 (* 2.0 (+ x2 y2)))])

(defn quat-from-mat3 [m]
  "Convert 3x3 rotation matrix to unit quaternion using Shepperd algorithm."
  (def tr (mat3-trace m))
  (if (> tr 0.0)
    (let [s (math/sqrt (+ tr 1.0))
          w (* s 0.5)
          inv-4s (/ 0.5 s)]
      (quat-normalize
        [w
         (* (- (in m 7) (in m 5)) inv-4s)
         (* (- (in m 2) (in m 6)) inv-4s)
         (* (- (in m 3) (in m 1)) inv-4s)]))
    (if (and (> (in m 0) (in m 4)) (> (in m 0) (in m 8)))
      (let [s (math/sqrt (+ 1.0 (- (in m 0) (in m 4) (in m 8))))
            x (* s 0.5)
            inv-4s (/ 0.5 s)]
        (quat-normalize
          [(* (- (in m 7) (in m 5)) inv-4s)
           x
           (* (+ (in m 1) (in m 3)) inv-4s)
           (* (+ (in m 2) (in m 6)) inv-4s)]))
      (if (> (in m 4) (in m 8))
        (let [s (math/sqrt (+ 1.0 (- (in m 4) (in m 0) (in m 8))))
              y (* s 0.5)
              inv-4s (/ 0.5 s)]
          (quat-normalize
            [(* (- (in m 2) (in m 6)) inv-4s)
             (* (+ (in m 1) (in m 3)) inv-4s)
             y
             (* (+ (in m 5) (in m 7)) inv-4s)]))
        (let [s (math/sqrt (+ 1.0 (- (in m 8) (in m 0) (in m 4))))
              z (* s 0.5)
              inv-4s (/ 0.5 s)]
          (quat-normalize
            [(* (- (in m 3) (in m 1)) inv-4s)
             (* (+ (in m 2) (in m 6)) inv-4s)
             (* (+ (in m 5) (in m 7)) inv-4s)
             z]))))))

(defn quat-from-euler [phi theta psi]
  "Convert Euler angles (roll, pitch, yaw) in radians to unit quaternion."
  (def hp (* phi 0.5))
  (def ht (* theta 0.5))
  (def hy (* psi 0.5))

  (def c1 (math/cos hp)) (def s1 (math/sin hp))
  (def c2 (math/cos ht)) (def s2 (math/sin ht))
  (def c3 (math/cos hy)) (def s3 (math/sin hy))

  (quat-normalize
    [(+ (* c1 c2 c3) (* s1 s2 s3))
     (- (* s1 c2 c3) (* c1 s2 s3))
     (+ (* c1 s2 c3) (* s1 c2 s3))
     (- (* c1 c2 s3) (* s1 s2 c3))]))

(defn quat-to-euler [q]
  "Convert unit quaternion [w, x, y, z] to roll, pitch, yaw angles in radians."
  (def w (in q 0)) (def x (in q 1)) (def y (in q 2)) (def z (in q 3))

  # Roll (x-axis rotation)
  (def sinr-cosp (* 2.0 (+ (* w x) (* y z))))
  (def cosr-cosp (- 1.0 (* 2.0 (+ (* x x) (* y y)))))
  (def roll (math/atan2 sinr-cosp cosr-cosp))

  # Pitch (y-axis rotation)
  (def sinp (* 2.0 (- (* w y) (* z x))))
  (var pitch 0.0)
  (if (>= (math/abs sinp) 1.0)
    (set pitch (if (> sinp 0.0) c/half-pi (- c/half-pi)))
    (set pitch (math/asin sinp)))

  # Yaw (z-axis rotation)
  (def siny-cosp (* 2.0 (+ (* w z) (* x y))))
  (def cosy-cosp (- 1.0 (* 2.0 (+ (* y y) (* z z)))))
  (def yaw (math/atan2 siny-cosp cosy-cosp))

  [roll pitch yaw])

(defn quat-integrate [q omega dt]
  "Integrate unit quaternion q over time step dt given 3D angular velocity vector omega."
  (def half-dt (* 0.5 dt))
  (def wx (in omega 0)) (def wy (in omega 1)) (def wz (in omega 2))
  (def dq [1.0 (* wx half-dt) (* wy half-dt) (* wz half-dt)])
  (quat-normalize (quat-mul q dq)))

(defn quat-slerp [q0 q1 t]
  "Spherical linear interpolation between two unit quaternions."
  (var cos-half-theta (+ (* (in q0 0) (in q1 0))
                         (* (in q0 1) (in q1 1))
                         (* (in q0 2) (in q1 2))
                         (* (in q0 3) (in q1 3))))

  (var q1-adj q1)
  (when (< cos-half-theta 0.0)
    (set q1-adj [(- (in q1 0)) (- (in q1 1)) (- (in q1 2)) (- (in q1 3))])
    (set cos-half-theta (- cos-half-theta)))

  (if (> (math/abs cos-half-theta) 0.9995)
    # Very close: use normalized linear interpolation (nlerp)
    (quat-normalize
      [(+ (* (in q0 0) (- 1.0 t)) (* (in q1-adj 0) t))
       (+ (* (in q0 1) (- 1.0 t)) (* (in q1-adj 1) t))
       (+ (* (in q0 2) (- 1.0 t)) (* (in q1-adj 2) t))
       (+ (* (in q0 3) (- 1.0 t)) (* (in q1-adj 3) t))])
    (let [half-theta (math/acos cos-half-theta)
          sin-half-theta (math/sin half-theta)
          ratio-a (/ (math/sin (* (- 1.0 t) half-theta)) sin-half-theta)
          ratio-b (/ (math/sin (* t half-theta)) sin-half-theta)]
      [(+ (* (in q0 0) ratio-a) (* (in q1-adj 0) ratio-b))
       (+ (* (in q0 1) ratio-a) (* (in q1-adj 1) ratio-b))
       (+ (* (in q0 2) ratio-a) (* (in q1-adj 2) ratio-b))
       (+ (* (in q0 3) ratio-a) (* (in q1-adj 3) ratio-b))])))
