# src/paravane/init.janet
# Master module export for Paravane autonomous underwater vehicle stack.

(import ./core :as core :export true)
(import ./ocean :as ocean :export true)
(import ./acoustics :as acoustics :export true)
(import ./navigation :as navigation :export true)
(import ./hydrodynamics :as hydrodynamics :export true)
(import ./guidance :as guidance :export true)
(import ./slam :as slam :export true)
(import ./comms :as comms :export true)
(import ./telemetry :as telemetry :export true)
(import ./executive :as executive :export true)

(def version "0.1.0")

(defn banner []
  (string
    "===========================================================\n"
    "    PARAVANE: SUBSEA AUTONOMOUS VEHICLE ROBOTICS ENGINE    \n"
    "    Acoustic Navigation, Sound Velocity & Survey SLAM     \n"
    "===========================================================\n"))
