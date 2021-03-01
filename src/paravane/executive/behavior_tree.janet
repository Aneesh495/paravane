# src/paravane/executive/behavior_tree.janet
# Hierarchical Behavior Tree execution engine for autonomous subsea decision making.

# -----------------------------------------------------------------------------
# Behavior Tree Node Constructors
# -----------------------------------------------------------------------------

(defn bt-action [name action-fn]
  "Leaf node executing an imperative action function taking (blackboard, dt)."
  {:type :action
   :name name
   :fn action-fn})

(defn bt-condition [name cond-fn]
  "Leaf node evaluating a boolean condition function taking (blackboard)."
  {:type :condition
   :name name
   :fn (fn [bb dt]
         (if (cond-fn bb) :success :failure))})

(defn bt-sequence [name children]
  "Composite node executing children in sequential order. Fails if any child fails."
  {:type :sequence
   :name name
   :children children})

(defn bt-selector [name children]
  "Composite node executing children as fallbacks. Succeeds if any child succeeds."
  {:type :selector
   :name name
   :children children})

(defn bt-inverter [name child]
  "Decorator node inverting success to failure and failure to success."
  {:type :inverter
   :name name
   :child child})

(defn bt-retry [name child max-retries]
  "Decorator node retrying child on failure up to max-retries times."
  @{:type :retry
    :name name
    :child child
    :max-retries max-retries
    :retries-left max-retries})

# -----------------------------------------------------------------------------
# Behavior Tree Tick Engine
# -----------------------------------------------------------------------------

(defn bt-tick [node blackboard dt]
  "Evaluate behavior tree node against current vehicle blackboard state."
  (def node-type (get node :type))

  (case node-type
    :action
    ((get node :fn) blackboard dt)

    :condition
    ((get node :fn) blackboard dt)

    :sequence
    (do
      (var result :success)
      (each child (get node :children)
        (def status (bt-tick child blackboard dt))
        (when (not= status :success)
          (set result status)
          (break)))
      result)

    :selector
    (do
      (var result :failure)
      (each child (get node :children)
        (def status (bt-tick child blackboard dt))
        (when (not= status :failure)
          (set result status)
          (break)))
      result)

    :inverter
    (let [status (bt-tick (get node :child) blackboard dt)]
      (case status
        :success :failure
        :failure :success
        :running))

    :retry
    (let [status (bt-tick (get node :child) blackboard dt)]
      (if (and (= status :failure) (> (get node :retries-left) 0))
        (do
          (put node :retries-left (- (get node :retries-left) 1))
          :running)
        (do
          (put node :retries-left (get node :max-retries))
          status)))

    # Fallback
    :failure))
