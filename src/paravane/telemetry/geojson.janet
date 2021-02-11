# src/paravane/telemetry/geojson.janet
# RFC 7946 GeoJSON subsea trajectory, survey trackline, and bathymetry exporter.

(import ../core/constants :as c)
(import ../core/geodesy :as g)

(defn format-float [num &opt precision]
  (default precision 6)
  (string/format (string "%." precision "f") num))

(defn export-trajectory-geojson [trajectory-ned ref-origin-geodetic]
  "Export AUV 3D NED trajectory track to RFC 7946 GeoJSON format string."
  (def coords @[])

  (each pt trajectory-ned
    (def geo (g/ned-to-geodetic pt ref-origin-geodetic))
    (def lat-deg (* (in geo 0) c/rad-to-deg))
    (def lon-deg (* (in geo 1) c/rad-to-deg))
    (def depth-m (in pt 2))
    # GeoJSON coordinates order: [longitude, latitude, elevation/depth]
    (array/push coords (string "[" (format-float lon-deg 7) ", "
                                (format-float lat-deg 7) ", "
                                (format-float (- depth-m) 2) "]")))

  (def coords-str (string/join coords ",\n        "))

  (string "{\n"
          "  \"type\": \"FeatureCollection\",\n"
          "  \"features\": [\n"
          "    {\n"
          "      \"type\": \"Feature\",\n"
          "      \"properties\": {\n"
          "        \"name\": \"AUV Subsea Trajectory\",\n"
          "        \"vehicle\": \"Paravane-Survey-1\"\n"
          "      },\n"
          "      \"geometry\": {\n"
          "        \"type\": \"LineString\",\n"
          "        \"coordinates\": [\n"
          "        " coords-str "\n"
          "        ]\n"
          "      }\n"
          "    }\n"
          "  ]\n"
          "}"))

(defn export-waypoints-geojson [waypoints ref-origin-geodetic]
  "Export mission survey waypoints to RFC 7946 GeoJSON FeatureCollection."
  (def features @[])

  (for i 0 (length waypoints)
    (def wp (in waypoints i))
    (def pos (get wp :pos))
    (def geo (g/ned-to-geodetic pos ref-origin-geodetic))
    (def lat-deg (* (in geo 0) c/rad-to-deg))
    (def lon-deg (* (in geo 1) c/rad-to-deg))
    (def depth-m (in pos 2))

    (def feat-str
      (string "    {\n"
              "      \"type\": \"Feature\",\n"
              "      \"properties\": {\n"
              "        \"index\": " i ",\n"
              "        \"speed_mps\": " (get wp :speed) ",\n"
              "        \"depth_m\": " depth-m "\n"
              "      },\n"
              "      \"geometry\": {\n"
              "        \"type\": \"Point\",\n"
              "        \"coordinates\": [" (format-float lon-deg 7) ", " (format-float lat-deg 7) ", " (format-float (- depth-m) 2) "]\n"
              "      }\n"
              "    }"))
    (array/push features feat-str))

  (def features-str (string/join features ",\n"))

  (string "{\n"
          "  \"type\": \"FeatureCollection\",\n"
          "  \"features\": [\n"
          features-str "\n"
          "  ]\n"
          "}"))
