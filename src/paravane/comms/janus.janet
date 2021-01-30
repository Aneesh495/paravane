# src/paravane/comms/janus.janet
# JANUS standard (NATO STANAG 4748 / IEEE 1841) underwater acoustic communication packet codec.

(import ../core/constants :as c)

# -----------------------------------------------------------------------------
# CRC-16-CCITT (Polynomial: 0x1021, Initial: 0xFFFF)
# -----------------------------------------------------------------------------

(defn crc16-ccitt [buffer-bytes]
  "Compute 16-bit CRC-CCITT checksum across byte buffer."
  (var crc 0xFFFF)
  (each b buffer-bytes
    (def shifted (band (bxor (brshift crc 8) b) 0xFF))
    (var temp shifted)
    (set temp (bxor temp (brshift temp 4)))
    (set crc (band (bxor (blshift crc 8)
                         (blshift temp 12)
                         (blshift temp 5)
                         temp)
                   0xFFFF)))
  crc)

# -----------------------------------------------------------------------------
# JANUS 80-Bit Baseline Packet Encoder & Decoder
# -----------------------------------------------------------------------------
# Format:
#   Version: 3 bits (standard = 1)
#   App Class: 5 bits
#   Mobility: 2 bits (0=fixed, 1=surface, 2=subsea)
#   Sender ID: 8 bits
#   Receiver ID: 8 bits (0xFF = broadcast)
#   Payload: 32 bits (4 bytes)
#   CRC-16: 16 bits
# Total: 10 bytes = 80 bits
# -----------------------------------------------------------------------------

(defn encode-janus-packet [sender-id receiver-id app-class mobility-type payload-bytes]
  "Encode metadata and payload into standard 10-byte JANUS acoustic baseline frame."
  (def frame (buffer/new 10))

  # Byte 0: Version (3 bits) | App Class (5 bits)
  (def b0 (bor (blshift (band 1 0x07) 5) (band app-class 0x1F)))
  (buffer/push-byte frame b0)

  # Byte 1: Mobility (2 bits) | Reserved (6 bits)
  (def b1 (blshift (band mobility-type 0x03) 6))
  (buffer/push-byte frame b1)

  # Byte 2: Sender ID (8 bits)
  (buffer/push-byte frame (band sender-id 0xFF))

  # Byte 3: Receiver ID (8 bits)
  (buffer/push-byte frame (band receiver-id 0xFF))

  # Bytes 4..7: Payload bytes
  (for i 0 4
    (def pb (if (< i (length payload-bytes)) (in payload-bytes i) 0))
    (buffer/push-byte frame pb))

  # Compute CRC-16 over first 8 bytes
  (def crc (crc16-ccitt frame))
  # Bytes 8..9: CRC-16 high and low
  (buffer/push-byte frame (band (brshift crc 8) 0xFF))
  (buffer/push-byte frame (band crc 0xFF))

  frame)

(defn decode-janus-packet [frame-bytes]
  "Parse and validate standard 10-byte JANUS acoustic frame."
  (if (< (length frame-bytes) 10)
    {:valid false :error "Frame too short"}
    (let [# Verify CRC
          payload-slice (buffer/slice frame-bytes 0 8)
          expected-crc (crc16-ccitt payload-slice)
          received-crc (bor (blshift (in frame-bytes 8) 8) (in frame-bytes 9))]
      (if (not= expected-crc received-crc)
        {:valid false :error "CRC mismatch" :expected expected-crc :received received-crc}
        (let [b0 (in frame-bytes 0)
              b1 (in frame-bytes 1)
              version (band (brshift b0 5) 0x07)
              app-class (band b0 0x1F)
              mobility (band (brshift b1 6) 0x03)
              sender-id (in frame-bytes 2)
              receiver-id (in frame-bytes 3)
              payload (buffer/slice frame-bytes 4 8)]
          {:valid true
           :version version
           :app-class app-class
           :mobility mobility
           :sender-id sender-id
           :receiver-id receiver-id
           :payload payload
           :crc received-crc})))))

# -----------------------------------------------------------------------------
# Acoustic TDMA Media Access Control (MAC) with Propagation Delay Guards
# -----------------------------------------------------------------------------

(defn compute-acoustic-slot-duration [max-range-m payload-bits data-rate-bps sound-speed]
  "Compute TDMA slot duration including transmission time and acoustic propagation guard delay."
  (def tx-duration (/ payload-bits data-rate-bps)) # e.g. 80 bits / 80 bps = 1.0s
  (def prop-delay (/ max-range-m sound-speed))     # e.g. 3000m / 1500 m/s = 2.0s
  (def guard-band 0.5)                             # 0.5s hardware turnaround guard
  (+ tx-duration prop-delay guard-band))
