# test/test_comms_telemetry.janet
# Comprehensive unit tests for JANUS acoustic comms, NMEA parsers, and GeoJSON telemetry.

(import ./helpers :as h)
(import ../src/paravane/core/constants :as c)
(import ../src/paravane/comms/janus :as janus)
(import ../src/paravane/telemetry/nmea :as nmea)
(import ../src/paravane/telemetry/geojson :as geojson)

(h/reset-test-stats)

# -----------------------------------------------------------------------------
# JANUS Acoustic Comms Tests
# -----------------------------------------------------------------------------

# Test Case 1: CRC-16-CCITT verification
(def test-buf (buffer "123456789"))
# Standard CRC-16-CCITT for ASCII "123456789" with init 0xFFFF is 0x29B1
(def crc-val (janus/crc16-ccitt test-buf))
(h/assert-equal 0x29B1 crc-val "CRC-16-CCITT benchmark matches standard value 0x29B1")

# Test Case 2: JANUS baseline 80-bit frame round trip
(def sender 42)
(def receiver 101)
(def app-class 3)
(def mobility 2) # Subsea vehicle
(def payload (buffer/from-bytes 0xAA 0xBB 0xCC 0xDD))

(def encoded-frame (janus/encode-janus-packet sender receiver app-class mobility payload))
(h/assert-equal 10 (length encoded-frame) "JANUS encoded frame is exactly 10 bytes (80 bits)")

(def decoded (janus/decode-janus-packet encoded-frame))
(h/assert-true (get decoded :valid) "JANUS decoded frame is valid")
(h/assert-equal sender (get decoded :sender-id) "Decoded sender ID matches 42")
(h/assert-equal receiver (get decoded :receiver-id) "Decoded receiver ID matches 101")
(h/assert-equal app-class (get decoded :app-class) "Decoded application class matches 3")
(h/assert-equal mobility (get decoded :mobility) "Decoded mobility type matches 2")

# Test Case 3: Corrupted bit CRC detection
(def corrupted (buffer/slice encoded-frame))
(put corrupted 4 (bxor (in corrupted 4) 0x01)) # Flip single bit in payload
(def corrupt-res (janus/decode-janus-packet corrupted))
(h/assert-false (get corrupt-res :valid) "Corrupted frame rejected by CRC-16 check")

# Test Case 4: TDMA slot calculation for 3000m range at 1500 m/s
(def slot-dur (janus/compute-acoustic-slot-duration 3000.0 80.0 80.0 1500.0))
# 1.0s tx + 2.0s prop delay + 0.5s guard = 3.5s
(h/assert-near 3.50 slot-dur 1e-6 "Acoustic TDMA slot duration is exactly 3.50 seconds")

# -----------------------------------------------------------------------------
# NMEA-0183 Telemetry Parser Tests
# -----------------------------------------------------------------------------

# Test Case 5: NMEA checksum verification
# $GPGGA checksum test
(def valid-nmea "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47")
(h/assert-true (nmea/verify-nmea-checksum valid-nmea) "Valid NMEA sentence checksum passes")

(def bad-nmea "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*99")
(h/assert-false (nmea/verify-nmea-checksum bad-nmea) "Corrupted NMEA checksum fails")

# Test Case 6: $PASHR attitude parsing
(def pashr-line "$PASHR,123456.78,135.5,T,1.25,-3.40,0.12,0.05,0.05,0.10,1*1A")
(def pashr-parsed (nmea/parse-pashr pashr-line))
(h/assert-true (not (nil? pashr-parsed)) "PASHR sentence successfully parsed")
(h/assert-near (* 135.5 c/deg-to-rad) (get pashr-parsed :yaw-rad) 1e-4 "Parsed yaw angle matches 135.5 deg")
(h/assert-near (* 1.25 c/deg-to-rad) (get pashr-parsed :roll-rad) 1e-4 "Parsed roll angle matches 1.25 deg")
(h/assert-near (* -3.40 c/deg-to-rad) (get pashr-parsed :pitch-rad) 1e-4 "Parsed pitch angle matches -3.40 deg")

# Test Case 7: $PD4 DVL bottom track parsing
(def pd4-line "$PD4,1.48,-0.02,0.05,15.2,1,0.01*2B")
(def pd4-parsed (nmea/parse-pd4 pd4-line))
(h/assert-true (not (nil? pd4-parsed)) "PD4 sentence successfully parsed")
(h/assert-near 1.48 (get pd4-parsed :vx) 1e-3 "Parsed forward velocity is 1.48 m/s")
(h/assert-near 15.2 (get pd4-parsed :altitude) 1e-2 "Parsed bottom altitude is 15.2m")

# Test Case 8: $DBT depth below transducer parsing
(def dbt-line "$SDDBT,49.2,f,15.0,M,8.2,F*39")
(def dbt-parsed (nmea/parse-dbt dbt-line))
(h/assert-near 15.0 (get dbt-parsed :depth-meters) 1e-3 "Parsed depth is 15.0 meters")

# -----------------------------------------------------------------------------
# GeoJSON Exporter Tests
# -----------------------------------------------------------------------------

(def ref-geo [(* 36.8 c/deg-to-rad) (* -122.0 c/deg-to-rad) 0.0])
(def test-track @[[0.0 0.0 50.0] [100.0 50.0 52.0] [200.0 100.0 55.0]])
(def geojson-str (geojson/export-trajectory-geojson test-track ref-geo))

(h/assert-true (string/has-prefix? "{\n  \"type\": \"FeatureCollection\"" geojson-str)
               "GeoJSON export starts with FeatureCollection header")
(h/assert-true (string/find "LineString" geojson-str) "GeoJSON export contains LineString geometry")

(h/summarize-tests "JANUS Acoustic Comms, NMEA Parsing, and GeoJSON Telemetry")
