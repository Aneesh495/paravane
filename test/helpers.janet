# test/helpers.janet
# Comprehensive unit testing framework for Paravane subsea systems.

(var *total-assertions* 0)
(var *passed-assertions* 0)
(var *failed-assertions* 0)
(var *failures* @[])

(defn reset-test-stats []
  (set *total-assertions* 0)
  (set *passed-assertions* 0)
  (set *failed-assertions* 0)
  (set *failures* @[]))

(defn record-pass []
  (++ *total-assertions*)
  (++ *passed-assertions*))

(defn record-fail [msg file line]
  (++ *total-assertions*)
  (++ *failed-assertions*))

(defmacro assert-true [condition &opt msg]
  (def default-msg (string "Expected truthy value but got: " (string/format "%q" condition)))
  (def final-msg (or msg default-msg))
  ~(if ,condition
     (,record-pass)
     (do
       (++ *total-assertions*)
       (++ *failed-assertions*)
       (array/push *failures* {:message ,final-msg :loc "assert-true"}))))

(defmacro assert-false [condition &opt msg]
  (def default-msg (string "Expected falsey value but got: " (string/format "%q" condition)))
  (def final-msg (or msg default-msg))
  ~(if (not ,condition)
     (,record-pass)
     (do
       (++ *total-assertions*)
       (++ *failed-assertions*)
       (array/push *failures* {:message ,final-msg :loc "assert-false"}))))

(defn assert-equal [expected actual &opt msg]
  (++ *total-assertions*)
  (if (= expected actual)
    (++ *passed-assertions*)
    (let [m (or msg (string "Expected " (string/format "%q" expected) " but got " (string/format "%q" actual)))]
      (++ *failed-assertions*)
      (array/push *failures* {:message m :loc "assert-equal"}))))

(defn assert-near [expected actual &opt eps msg]
  (default eps 1e-6)
  (++ *total-assertions*)
  (def diff (math/abs (- expected actual)))
  (if (<= diff eps)
    (++ *passed-assertions*)
    (let [m (or msg (string "Expected ~" expected " within " eps " but got " actual " (diff=" diff ")"))]
      (++ *failed-assertions*)
      (array/push *failures* {:message m :loc "assert-near"}))))

(defn assert-vec-near [expected actual &opt eps msg]
  (default eps 1e-6)
  (++ *total-assertions*)
  (if (not= (length expected) (length actual))
    (do
      (++ *failed-assertions*)
      (array/push *failures* {:message (string "Vector length mismatch: " (length expected) " vs " (length actual)) :loc "assert-vec-near"}))
    (do
      (var all-ok true)
      (for i 0 (length expected)
        (def diff (math/abs (- (in expected i) (in actual i))))
        (when (> diff eps)
          (set all-ok false)
          (array/push *failures* {:message (string "Vector element " i " mismatch: expected " (in expected i) " got " (in actual i)) :loc "assert-vec-near"})))
      (if all-ok
        (++ *passed-assertions*)
        (++ *failed-assertions*)))))

(defn summarize-tests [suite-name]
  (print (string "\n=== Test Suite: " suite-name " ==="))
  (print (string "Assertions: " *total-assertions* " | Passed: " *passed-assertions* " | Failed: " *failed-assertions*))
  (if (zero? *failed-assertions*)
    (do
      (print "[PASS] All assertions passed successfully.")
      true)
    (do
      (print "[FAIL] " *failed-assertions* " assertion(s) failed:")
      (each f *failures*
        (print (string "   * " (get f :message))))
      false)))
