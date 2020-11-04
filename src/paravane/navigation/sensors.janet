# src/paravane/navigation/sensors.janet
# Subsea navigation sensor models: tactical IMU, DVL, depth transducer, and magnetometer.

(import ../core/constants :as c)
(import ../core/math :as m)
(import ../core/stats :as s)

# -----------------------------------------------------------------------------
# Sensor Noise and Bias Configuration
# -----------------------------------------------------------------------------

(defn make-imu-config [&opt accel-noise gyro-noise accel-rw gyro-rw]
  "Construct tactical grade IMU error characteristics."
  (default accel-noise 0.005) # 5 mg / sqrt(Hz) velocity random walk
  (default gyro-noise 0.0005) # 0.03 deg / sqrt(hr) angular random walk (rad/s)
  (default accel-rw 1.0e-5)   # Accelerometer bias instability (m/s^2 / s)
  (default gyro-rw 1.0e-7)    # Gyroscope bias instability (rad/s / s)
  {:accel-noise accel-noise
   :gyro-noise gyro-noise
   :accel-rw accel-rw
   :gyro-rw gyro-rw})

(defn make-dvl-config [&opt vel-noise freq-hz beam-angle-deg sound-speed-cal]
  "Construct Doppler Velocity Log (DVL) error model."
  (default vel-noise 0.005)      # 5 mm/s velocity measurement precision
  (default freq-hz 10.0)         # 10 Hz ping rate
  (default beam-angle-deg 30.0)  # 30 deg Janus beam angle from vertical
  (default sound-speed-cal 1500.0) # Factory calibrated sound speed
  {:vel-noise vel-noise
   :freq freq-hz
   :beam-angle (* beam-angle-deg c/deg-to-rad)
   :sound-speed sound-speed-cal})

(defn make-depth-config [&opt noise-stddev tare-pressure-dbar]
  "Construct piezoresistive depth transducer error model."
  (default noise-stddev 0.03) # 3 cm depth measurement noise
  (default tare-pressure-dbar 10.1325) # Atmospheric surface pressure tare
  {:noise noise-stddev
   :tare tare-pressure-dbar})

# -----------------------------------------------------------------------------
# Sensor Measurement Generation with Realistic Physical Noise
# -----------------------------------------------------------------------------

(defn sample-imu-reading [true-accel true-omega accel-bias gyro-bias imu-cfg dt]
  "Simulate noisy IMU reading given true specific force, angular velocity, and sensor biases."
  (def na (s/rand-gaussian-vec3 [0.0 0.0 0.0] [(get imu-cfg :accel-noise) (get imu-cfg :accel-noise) (get imu-cfg :accel-noise)]))
  (def ng (s/rand-gaussian-vec3 [0.0 0.0 0.0] [(get imu-cfg :gyro-noise) (get imu-cfg :gyro-noise) (get imu-cfg :gyro-noise)]))

  (def meas-accel (m/vec3-add true-accel (m/vec3-add accel-bias na)))
  (def meas-omega (m/vec3-add true-omega (m/vec3-add gyro-bias ng)))

  # Bias random walk evolution: bias_next = bias + N(0, rw^2 * dt)
  (def dt-sqrt (math/sqrt dt))
  (def d-ba (s/rand-gaussian-vec3 [0.0 0.0 0.0] (m/vec3-scale [1.0 1.0 1.0] (* (get imu-cfg :accel-rw) dt-sqrt))))
  (def d-bg (s/rand-gaussian-vec3 [0.0 0.0 0.0] (m/vec3-scale [1.0 1.0 1.0] (* (get imu-cfg :gyro-rw) dt-sqrt))))

  {:accel meas-accel
   :omega meas-omega
   :next-accel-bias (m/vec3-add accel-bias d-ba)
   :next-gyro-bias (m/vec3-add gyro-bias d-bg)})

(defn sample-depth-reading [true-depth depth-cfg]
  "Simulate noisy depth pressure measurement in meters."
  (def n (s/rand-gaussian 0.0 (get depth-cfg :noise)))
  (max 0.0 (+ true-depth n)))
