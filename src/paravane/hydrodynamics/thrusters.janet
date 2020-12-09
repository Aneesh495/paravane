# src/paravane/hydrodynamics/thrusters.janet
# Thruster Allocation Matrix (TAM) and pseudo-inverse propulsion distribution for survey AUVs.

(import ../core/constants :as c)

# -----------------------------------------------------------------------------
# Thruster System Representation
# -----------------------------------------------------------------------------

(defn make-thruster-system [&opt x-bow-vert x-stern-vert x-bow-horiz max-thrust-n]
  "Construct 4-thruster allocation geometry for survey AUV."
  (default x-bow-vert 0.60)     # Bow vertical thruster at +0.60m forward of origin
  (default x-stern-vert -0.60)  # Stern vertical thruster at -0.60m aft of origin
  (default x-bow-horiz 0.55)    # Bow horizontal thruster at +0.55m forward of origin
  (default max-thrust-n 35.0)   # 35 N maximum bollard thrust per thruster
  {:x-bv x-bow-vert
   :x-sv x-stern-vert
   :x-bh x-bow-horiz
   :max-thrust max-thrust-n
   :min-thrust (- max-thrust-n)})

# -----------------------------------------------------------------------------
# Forward Allocation: Thruster Forces to 6-DOF Generalized Wrench
# -----------------------------------------------------------------------------

(defn thruster-forces-to-wrench [tam u-thrusters]
  "Map 4 individual thruster forces [u_main, u_bow_vert, u_stern_vert, u_bow_horiz] to 6-DOF wrench."
  (def u-main (in u-thrusters 0))
  (def u-bv   (in u-thrusters 1))
  (def u-sv   (in u-thrusters 2))
  (def u-bh   (in u-thrusters 3))

  (def x-bv (get tam :x-bv))
  (def x-sv (get tam :x-sv))
  (def x-bh (get tam :x-bh))

  # Generalized wrench tau = [X (surge), Y (sway), Z (heave), K (roll), M (pitch), N (yaw)]
  (def surge u-main)
  (def sway u-bh)
  (def heave (+ u-bv u-sv))
  (def roll 0.0)
  (def pitch (+ (* (- x-bv) u-bv) (* (- x-sv) u-sv))) # Downward force on bow pitches vehicle down
  (def yaw (* x-bh u-bh)) # Starboard force on bow yaws vehicle clockwise (positive N)

  [surge sway heave roll pitch yaw])

# -----------------------------------------------------------------------------
# Inverse Allocation: Desired Wrench to Thruster Commands (Pseudo-Inverse)
# -----------------------------------------------------------------------------

(defn allocate-thrust [tam desired-wrench]
  "Solve for optimal individual thruster forces given desired generalized control wrench."
  (def x-des (in desired-wrench 0)) # Desired surge force (N)
  (def y-des (in desired-wrench 1)) # Desired sway force (N)
  (def z-des (in desired-wrench 2)) # Desired heave force (N)
  (def m-des (in desired-wrench 4)) # Desired pitch moment (N*m)
  (def n-des (in desired-wrench 5)) # Desired yaw moment (N*m)

  (def x-bv (get tam :x-bv))
  (def x-sv (get tam :x-sv))
  (def x-bh (get tam :x-bh))
  (def max-f (get tam :max-thrust))
  (def min-f (get tam :min-thrust))

  # 1. Main thruster takes full surge demand
  (def u-main-raw x-des)

  # 2. Heave and Pitch are resolved by the two vertical thrusters:
  #    u_bv + u_sv = Z_des
  #    -x_bv * u_bv - x_sv * u_sv = M_des
  # Determinant = -x_sv - (-x_bv) = x_bv - x_sv
  (def det (- x-bv x-sv))
  (def u-bv-raw (/ (- (* (- x-sv) z-des) m-des) det))
  (def u-sv-raw (/ (+ (* x-bv z-des) m-des) det))

  # 3. Sway and Yaw sharing: bow horizontal thruster handles yaw primarily
  (def u-bh-raw (if (> (math/abs x-bh) 1e-3)
                  (/ n-des x-bh)
                  y-des))

  # 4. Saturate thruster limits
  (def u-main (max min-f (min max-f u-main-raw)))
  (def u-bv   (max min-f (min max-f u-bv-raw)))
  (def u-sv   (max min-f (min max-f u-sv-raw)))
  (def u-bh   (max min-f (min max-f u-bh-raw)))

  {:thrusts [u-main u-bv u-sv u-bh]
   :saturated (or (not= u-main u-main-raw)
                  (not= u-bv u-bv-raw)
                  (not= u-sv u-sv-raw)
                  (not= u-bh u-bh-raw))})
