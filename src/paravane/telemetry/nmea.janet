# src/paravane/telemetry/nmea.janet
# Parsing Expression Grammar (PEG) high-speed parsers for subsea NMEA-0183 sentences.

(import ../core/constants :as c)

# -----------------------------------------------------------------------------
# NMEA Checksum Verification
# -----------------------------------------------------------------------------

(defn verify-nmea-checksum [sentence]
  "Validate XOR checksum of NMEA string between '$' and '*'."
  (def star-idx (string/find "*" sentence))
  (if (nil? star-idx)
    false
    (let [dollar-idx (string/find "$" sentence)
          start (if dollar-idx (+ dollar-idx 1) 0)
          body (string/slice sentence start star-idx)
          checksum-str (string/slice sentence (+ star-idx 1) (+ star-idx 3))
          expected (scan-number (string "0x" checksum-str))]
      (var xor-val 0)
      (for i 0 (length body)
        (set xor-val (bxor xor-val (in body i))))
      (= xor-val expected))))

# -----------------------------------------------------------------------------
# $PASHR: Inertial Attitude Sentence Parser
# Format: $PASHR,time,heading,T,roll,pitch,heave,roll_acc,pitch_acc,heading_acc,quality*CS
# -----------------------------------------------------------------------------

(defn parse-pashr [sentence]
  "Parse Applanix/Kongsberg $PASHR inertial attitude NMEA sentence."
  (def parts (string/split "," sentence))
  (if (or (< (length parts) 6) (not= (in parts 0) "$PASHR"))
    nil
    (let [time-str (in parts 1)
          yaw-deg (scan-number (in parts 2))
          roll-deg (scan-number (in parts 4))
          pitch-deg (scan-number (in parts 5))
          heave-m (if (>= (length parts) 7) (scan-number (in parts 6)) 0.0)]
      (if (and yaw-deg roll-deg pitch-deg)
        {:type :pashr
         :time time-str
         :yaw-rad (* yaw-deg c/deg-to-rad)
         :pitch-rad (* pitch-deg c/deg-to-rad)
         :roll-rad (* roll-deg c/deg-to-rad)
         :heave-m (or heave-m 0.0)}
        nil))))

# -----------------------------------------------------------------------------
# $PD4: DVL Bottom Track Velocity Sentence Parser
# Format: $PD4,vx,vy,vz,altitude,status,err_vel*CS
# -----------------------------------------------------------------------------

(defn parse-pd4 [sentence]
  "Parse Teledyne/RDI DVL $PD4 bottom track velocity sentence."
  (def parts (string/split "," sentence))
  (if (or (< (length parts) 5) (not= (in parts 0) "$PD4"))
    nil
    (let [vx (scan-number (in parts 1))
          vy (scan-number (in parts 2))
          vz (scan-number (in parts 3))
          alt (scan-number (in parts 4))
          star-split (if (>= (length parts) 6) (string/split "*" (in parts 5)) nil)
          err-v (if star-split (scan-number (in star-split 0)) 0.0)]
      (if (and vx vy vz alt)
        {:type :pd4
         :vx vx
         :vy vy
         :vz vz
         :altitude alt
         :err-vel (or err-v 0.0)}
        nil))))

# -----------------------------------------------------------------------------
# $DBT: Depth Below Transducer Parser
# Format: $--DBT,depth_feet,f,depth_meters,M,depth_fathoms,F*CS
# -----------------------------------------------------------------------------

(defn parse-dbt [sentence]
  "Parse standard NMEA depth below transducer sentence."
  (def parts (string/split "," sentence))
  (if (< (length parts) 4)
    nil
    (let [depth-m (scan-number (in parts 3))]
      (if depth-m
        {:type :dbt :depth-meters depth-m}
        nil))))

# -----------------------------------------------------------------------------
# Universal Dispatcher
# -----------------------------------------------------------------------------

(defn parse-subsea-nmea [sentence]
  "Dispatch NMEA sentence to specialized parser."
  (cond
    (string/has-prefix? "$PASHR" sentence) (parse-pashr sentence)
    (string/has-prefix? "$PD4" sentence) (parse-pd4 sentence)
    (string/has-suffix? "DBT" (in (string/split "," sentence) 0)) (parse-dbt sentence)
    nil))
