# src/paravane/hydrodynamics/vehicle_model.janet
# Physical, hydrodynamic, and geometric parameters for Remus/Hugin class survey AUV.

(import ../core/constants :as c)

(defn make-default-survey-auv []
  "Construct baseline parameters for 1.6m survey-class torpedo AUV."
  (def mass 38.5) # kg
  (def length-m 1.60)
  (def diameter-m 0.19)

  # Displaced volume for 40g net positive safety buoyancy in standard seawater
  (def volume (/ (+ mass 0.040) c/default-seawater-density))
  (def weight (* mass c/standard-gravity))
  (def buoyancy (* volume c/default-seawater-density c/standard-gravity))

  # Center of gravity located 2.0 cm below origin for metacentric passive righting
  (def rg [0.0 0.0 0.020])
  (def rb [0.0 0.0 0.000])

  # Rigid-body moments of inertia (cylinder approximation)
  (def r-cyl (/ diameter-m 2.0))
  (def ixx (* 0.5 mass r-cyl r-cyl))
  (def iyy (* (/ 1.0 12.0) mass (+ (* 3.0 r-cyl r-cyl) (* length-m length-m))))
  (def izz iyy)

  # Added mass tensor (prolate spheroid analytical approximation)
  (def x-u-dot (* -0.10 mass))
  (def y-v-dot (* -0.90 mass))
  (def z-w-dot (* -0.90 mass))
  (def k-p-dot (* -0.10 ixx))
  (def m-q-dot (* -0.70 iyy))
  (def n-r-dot (* -0.70 izz))

  # Total virtual inertia vector: M_total = M_RB - M_A
  (def m-total-diag [(+ mass (- x-u-dot))
                     (+ mass (- y-v-dot))
                     (+ mass (- z-w-dot))
                     (+ ixx (- k-p-dot))
                     (+ iyy (- m-q-dot))
                     (+ izz (- n-r-dot))])

  # Hydrodynamic damping parameters
  (def d-linear [4.0 15.0 15.0 0.5 5.0 5.0])
  (def d-quad [18.0 65.0 65.0 1.5 25.0 25.0])

  {:mass mass
   :length length-m
   :diameter diameter-m
   :volume volume
   :weight weight
   :buoyancy buoyancy
   :rg rg
   :rb rb
   :ixx ixx
   :iyy iyy
   :izz izz
   :m-total-diag m-total-diag
   :d-linear d-linear
   :d-quad d-quad})
