# src/paravane/navigation/ekf.janet
# 15-state Error-State Extended Kalman Filter (ES-EKF) for subsea AUV navigation.

(import ../core/constants :as c)
(import ../core/math :as m)
(import ../core/stats :as s)
(import ./strapdown :as ins)

# -----------------------------------------------------------------------------
# 15x15 Matrix Utility Routines
# -----------------------------------------------------------------------------

(defn make-mat15-zero []
  (def arr (array/new 225))
  (for i 0 225 (array/push arr 0.0))
  arr)

(defn make-mat15-identity []
  (def arr (make-mat15-zero))
  (for i 0 15
    (put arr (+ (* i 15) i) 1.0))
  arr)

(defn make-mat15-diag [diags]
  (def arr (make-mat15-zero))
  (for i 0 15
    (put arr (+ (* i 15) i) (in diags i)))
  arr)

(defn mat15-get [m r c]
  (in m (+ (* r 15) c)))

(defn mat15-set [m r c val]
  (put m (+ (* r 15) c) val))

(defn mat15-transpose [m]
  (def res (make-mat15-zero))
  (for r 0 15
    (for c 0 15
      (mat15-set res c r (mat15-get m r c))))
  res)

(defn mat15-add [a b]
  (def res (make-mat15-zero))
  (for i 0 225
    (put res i (+ (in a i) (in b i))))
  res)

(defn mat15-mul [a b]
  (def res (make-mat15-zero))
  (for r 0 15
    (for c 0 15
      (var sum 0.0)
      (for k 0 15
        (+= sum (* (mat15-get a r k) (mat15-get b k c))))
      (mat15-set res r c sum)))
  res)

# -----------------------------------------------------------------------------
# ES-EKF Filter State
# -----------------------------------------------------------------------------

(defn make-es-ekf [&opt init-pos init-vel init-quat init-p-diag]
  "Construct 15-state Error-State Extended Kalman Filter for subsea vehicle."
  (default init-pos [0.0 0.0 0.0])
  (default init-vel [0.0 0.0 0.0])
  (default init-quat m/quat-identity)
  (default init-p-diag [1.0 1.0 0.25      # Pos var: 1m horiz, 0.5m vert
                        0.04 0.04 0.04    # Vel var: 0.2 m/s
                        0.001 0.001 0.005 # Att var: ~1.8 deg horiz, ~4 deg yaw
                        0.01 0.01 0.01    # Accel bias var: 0.1 m/s^2
                        0.0001 0.0001 0.0001]) # Gyro bias var: 0.01 rad/s

  {:ins (ins/make-ins-state init-pos init-vel init-quat)
   :cov (make-mat15-diag init-p-diag)
   :time 0.0})

# -----------------------------------------------------------------------------
# State Propagation Step (Inertial Mechanization + Error Covariance Growth)
# -----------------------------------------------------------------------------

(defn ekf-predict [filter raw-accel raw-omega dt &opt q-noise-diag]
  "Propagate nominal state and error covariance P forward by time step dt."
  (default q-noise-diag [1e-4 1e-4 1e-4       # Pos process noise
                         1e-3 1e-3 1e-3       # Vel process noise (accel noise)
                         1e-6 1e-6 1e-6       # Att process noise (gyro noise)
                         1e-7 1e-7 1e-7       # Accel bias random walk
                         1e-9 1e-9 1e-9])     # Gyro bias random walk

  (def cur-ins (get filter :ins))
  (def next-ins (ins/ins-mechanize-step cur-ins raw-accel raw-omega dt))

  # Build 15x15 State Transition Matrix Phi = I + F * dt
  (def phi (make-mat15-identity))
  (def r-nb (m/quat-to-mat3 (get cur-ins :quat)))
  (def r-bn (m/mat3-transpose r-nb))

  # 1. d(pos)/d(vel) = I * dt
  (for i 0 3 (mat15-set phi i (+ 3 i) dt))

  # 2. d(vel)/d(theta) = - [f_ned]_x * dt
  (def f-body (m/vec3-sub raw-accel (get cur-ins :accel-bias)))
  (def f-ned (m/mat3-mul-vec r-nb f-body))
  (def skew-f (m/mat3-skew f-ned))
  (for r 0 3
    (for c 0 3
      (mat15-set phi (+ 3 r) (+ 6 c) (* (- (in skew-f (+ (* r 3) c))) dt))))

  # 3. d(vel)/d(accel_bias) = - R_nb * dt
  (for r 0 3
    (for c 0 3
      (mat15-set phi (+ 3 r) (+ 9 c) (* (- (in r-nb (+ (* r 3) c))) dt))))

  # 4. d(theta)/d(gyro_bias) = - R_nb * dt
  (for r 0 3
    (for c 0 3
      (mat15-set phi (+ 6 r) (+ 12 c) (* (- (in r-nb (+ (* r 3) c))) dt))))

  # Covariance propagation: P_next = Phi * P * Phi^T + Q * dt
  (def p (get filter :cov))
  (def phi-t (mat15-transpose phi))
  (def p-phi-t (mat15-mul p phi-t))
  (def next-p (mat15-mul phi p-phi-t))

  # Add process noise Q * dt
  (for i 0 15
    (def cur-val (mat15-get next-p i i))
    (mat15-set next-p i i (+ cur-val (* (in q-noise-diag i) dt))))

  {:ins next-ins
   :cov next-p
   :time (+ (get filter :time) dt)})

# -----------------------------------------------------------------------------
# DVL Body Velocity Measurement Update
# -----------------------------------------------------------------------------

(defn ekf-update-dvl [filter dvl-vel-body r-dvl-mat3 &opt chi2-gate]
  "Fuse 3D DVL body-frame velocity reading into ES-EKF state with chi-square gating."
  (default chi2-gate 11.34) # 3-DOF 99% gating threshold

  (def cur-ins (get filter :ins))
  (def r-nb (m/quat-to-mat3 (get cur-ins :quat)))
  (def r-bn (m/mat3-transpose r-nb))

  # Predicted DVL velocity in body frame: v_body = R_bn * v_ned
  (def pred-v-body (m/mat3-mul-vec r-bn (get cur-ins :vel)))
  (def residual (m/vec3-sub dvl-vel-body pred-v-body))

  # DVL measurement matrix H: maps velocity states (indices 3,4,5) to body frame
  # z_dvl = R_bn * v_ned -> H_vel = R_bn (3x3)
  (def p (get filter :cov))

  # Extract 3x3 velocity sub-covariance P_vv from P (rows 3..5, cols 3..5)
  (def p-vv [ (mat15-get p 3 3) (mat15-get p 3 4) (mat15-get p 3 5)
              (mat15-get p 4 3) (mat15-get p 4 4) (mat15-get p 4 5)
              (mat15-get p 5 3) (mat15-get p 5 4) (mat15-get p 5 5) ])

  # S = R_bn * P_vv * R_bn^T + R_dvl
  (def temp (m/mat3-mul-mat r-bn p-vv))
  (def s-mat (m/mat3-add (m/mat3-mul-mat temp r-nb) r-dvl-mat3))
  (def inv-s (m/mat3-inv s-mat))

  (if (nil? inv-s)
    filter # Singular innovation covariance
    (do
      # Chi-square gating test
      (def temp-r (m/mat3-mul-vec inv-s residual))
      (def d2 (m/vec3-dot residual temp-r))
      (if (> d2 chi2-gate)
        # Outlier rejected
        filter
        (do
          # Compute Kalman Gain K_15x3 = P * H^T * inv(S)
          # H has non-zero block at cols 3..5 equal to R_bn
          # H^T has non-zero block at rows 3..5 equal to R_nb
          # Therefore: K_i = P_{i, 3..5} * R_nb * inv(S)
          (def k-mat (array/new 45)) # 15 rows x 3 cols
          (for i 0 45 (array/push k-mat 0.0))

          (def r-nb-inv-s (m/mat3-mul-mat r-nb inv-s))

          (for r 0 15
            (def p-row [(mat15-get p r 3) (mat15-get p r 4) (mat15-get p r 5)])
            (def k-row (m/mat3-mul-vec (m/mat3-transpose r-nb-inv-s) p-row))
            (put k-mat (+ (* r 3) 0) (in k-row 0))
            (put k-mat (+ (* r 3) 1) (in k-row 1))
            (put k-mat (+ (* r 3) 2) (in k-row 2)))

          # Error state correction: delta_x = K * residual
          (def delta-x (array/new 15))
          (for r 0 15
            (def k0 (in k-mat (+ (* r 3) 0)))
            (def k1 (in k-mat (+ (* r 3) 1)))
            (def k2 (in k-mat (+ (* r 3) 2)))
            (array/push delta-x (+ (* k0 (in residual 0)) (* k1 (in residual 1)) (* k2 (in residual 2)))))

          # Inject error state into nominal INS state:
          (def cur-pos (get cur-ins :pos))
          (def cur-vel (get cur-ins :vel))
          (def cur-q (get cur-ins :quat))
          (def cur-ba (get cur-ins :accel-bias))
          (def cur-bg (get cur-ins :gyro-bias))

          (def next-pos [(+ (in cur-pos 0) (in delta-x 0))
                         (+ (in cur-pos 1) (in delta-x 1))
                         (+ (in cur-pos 2) (in delta-x 2))])

          (def next-vel [(+ (in cur-vel 0) (in delta-x 3))
                         (+ (in cur-vel 1) (in delta-x 4))
                         (+ (in cur-vel 2) (in delta-x 5))])

          # Attitude injection: delta_theta into quaternion
          (def d-th [(in delta-x 6) (in delta-x 7) (in delta-x 8)])
          (def dq-rot (m/quat-from-euler (in d-th 0) (in d-th 1) (in d-th 2)))
          (def next-q (m/quat-normalize (m/quat-mul cur-q dq-rot)))

          (def next-ba [(+ (in cur-ba 0) (in delta-x 9))
                        (+ (in cur-ba 1) (in delta-x 10))
                        (+ (in cur-ba 2) (in delta-x 11))])

          (def next-bg [(+ (in cur-bg 0) (in delta-x 12))
                        (+ (in cur-bg 1) (in delta-x 13))
                        (+ (in cur-bg 2) (in delta-x 14))])

          # Covariance update: P_new = (I - K*H) * P
          # KH_15x15 = K_15x3 * R_bn_3x3 (placed at cols 3..5)
          (def next-p (make-mat15-zero))
          (for r 0 15
            (for c 0 15
              (def old-val (mat15-get p r c))
              # Compute (K * H * P)_{r, c}
              (def kr0 (in k-mat (+ (* r 3) 0)))
              (def kr1 (in k-mat (+ (* r 3) 1)))
              (def kr2 (in k-mat (+ (* r 3) 2)))
              # (H * P)_{cols 0..2} = R_bn * P_{3..5, c}
              (def p-col [(mat15-get p 3 c) (mat15-get p 4 c) (mat15-get p 5 c)])
              (def hp (m/mat3-mul-vec r-bn p-col))
              (def reduction (+ (* kr0 (in hp 0)) (* kr1 (in hp 1)) (* kr2 (in hp 2))))
              (mat15-set next-p r c (- old-val reduction))))

          {:ins (ins/make-ins-state next-pos next-vel next-q next-ba next-bg)
           :cov next-p
           :time (get filter :time)})))))

# -----------------------------------------------------------------------------
# Absolute Acoustic Position Fix Update (USBL / LBL)
# -----------------------------------------------------------------------------

(defn ekf-update-acoustic-pos [filter pos-meas-ned r-pos-mat3]
  "Fuse absolute 3D position fix (USBL or LBL) into ES-EKF state."
  (def cur-ins (get filter :ins))
  (def cur-pos (get cur-ins :pos))
  (def residual (m/vec3-sub pos-meas-ned cur-pos))

  (def p (get filter :cov))
  # H extracts position (indices 0, 1, 2)
  (def p-pp [ (mat15-get p 0 0) (mat15-get p 0 1) (mat15-get p 0 2)
              (mat15-get p 1 0) (mat15-get p 1 1) (mat15-get p 1 2)
              (mat15-get p 2 0) (mat15-get p 2 1) (mat15-get p 2 2) ])

  (def s-mat (m/mat3-add p-pp r-pos-mat3))
  (def inv-s (m/mat3-inv s-mat))

  (if (nil? inv-s)
    filter
    (do
      # Kalman gain: K = P * H^T * inv(S) -> K_i = P_{i, 0..2} * inv(S)
      (def k-mat (array/new 45))
      (for i 0 45 (array/push k-mat 0.0))

      (for r 0 15
        (def p-row [(mat15-get p r 0) (mat15-get p r 1) (mat15-get p r 2)])
        (def k-row (m/mat3-mul-vec (m/mat3-transpose inv-s) p-row))
        (put k-mat (+ (* r 3) 0) (in k-row 0))
        (put k-mat (+ (* r 3) 1) (in k-row 1))
        (put k-mat (+ (* r 3) 2) (in k-row 2)))

      (def delta-x (array/new 15))
      (for r 0 15
        (def k0 (in k-mat (+ (* r 3) 0)))
        (def k1 (in k-mat (+ (* r 3) 1)))
        (def k2 (in k-mat (+ (* r 3) 2)))
        (array/push delta-x (+ (* k0 (in residual 0)) (* k1 (in residual 1)) (* k2 (in residual 2)))))

      (def next-pos [(+ (in cur-pos 0) (in delta-x 0))
                     (+ (in cur-pos 1) (in delta-x 1))
                     (+ (in cur-pos 2) (in delta-x 2))])

      (def cur-vel (get cur-ins :vel))
      (def next-vel [(+ (in cur-vel 0) (in delta-x 3))
                     (+ (in cur-vel 1) (in delta-x 4))
                     (+ (in cur-vel 2) (in delta-x 5))])

      (def next-p (make-mat15-zero))
      (for r 0 15
        (for c 0 15
          (def old-val (mat15-get p r c))
          (def kr0 (in k-mat (+ (* r 3) 0)))
          (def kr1 (in k-mat (+ (* r 3) 1)))
          (def kr2 (in k-mat (+ (* r 3) 2)))
          (def p-col [(mat15-get p 0 c) (mat15-get p 1 c) (mat15-get p 2 c)])
          (def reduction (+ (* kr0 (in p-col 0)) (* kr1 (in p-col 1)) (* kr2 (in p-col 2))))
          (mat15-set next-p r c (- old-val reduction))))

      {:ins (ins/make-ins-state next-pos next-vel (get cur-ins :quat) (get cur-ins :accel-bias) (get cur-ins :gyro-bias))
       :cov next-p
       :time (get filter :time)})))

# -----------------------------------------------------------------------------
# Scalar Depth Measurement Update
# -----------------------------------------------------------------------------

(defn ekf-update-depth [filter measured-depth &opt depth-var]
  "Scalar Kalman update for high-accuracy piezoresistive pressure depth transducer."
  (default depth-var 0.0025) # (0.05m)^2 = 0.0025 m^2

  (def cur-ins (get filter :ins))
  (def cur-pos (get cur-ins :pos))
  (def residual (- measured-depth (in cur-pos 2))) # z_meas - p_down

  (def p (get filter :cov))
  # H = [0, 0, 1, 0 ... 0] (scalar observation of state index 2)
  (def s-val (+ (mat15-get p 2 2) depth-var))
  (if (<= s-val 1e-9)
    filter
    (let [inv-s (/ 1.0 s-val)
          # K_i = P_{i, 2} / S
          next-pos [(+ (in cur-pos 0) (* (mat15-get p 0 2) inv-s residual))
                    (+ (in cur-pos 1) (* (mat15-get p 1 2) inv-s residual))
                    (+ (in cur-pos 2) (* (mat15-get p 2 2) inv-s residual))]
          next-p (make-mat15-zero)]
      (for r 0 15
        (for c 0 15
          (def old-val (mat15-get p r c))
          (def reduction (* (mat15-get p r 2) inv-s (mat15-get p 2 c)))
          (mat15-set next-p r c (- old-val reduction))))

      {:ins (ins/make-ins-state next-pos (get cur-ins :vel) (get cur-ins :quat) (get cur-ins :accel-bias) (get cur-ins :gyro-bias))
       :cov next-p
       :time (get filter :time)})))
