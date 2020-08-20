(declare-project
  :name "paravane"
  :description "Autonomous Underwater Vehicle (AUV) Mission Executive, Acoustic Navigation, Sound Velocity Ray Tracing, and Bathymetric Survey Engine in Janet"
  :author "Aneesh Parthasarathy"
  :dependencies [])

(declare-source
  :prefix "paravane"
  :source ["src/paravane"])

(declare-executable
  :name "paravane"
  :entry "src/paravane/main.janet"
  :install false)
