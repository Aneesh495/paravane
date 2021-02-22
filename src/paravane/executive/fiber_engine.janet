# src/paravane/executive/fiber_engine.janet
# Cooperative fiber task scheduler, event loop, and inter-subsystem channels for subsea autonomy.

(import ../core/constants :as c)

# -----------------------------------------------------------------------------
# Fiber Task Representation
# -----------------------------------------------------------------------------

(defn make-task [task-name task-func priority]
  @{:name task-name
    :fn task-func
    :fiber (fiber/new task-func)
    :priority priority
    :status :ready
    :wake-time 0.0
    :run-count 0})

# -----------------------------------------------------------------------------
# Cooperative Scheduler
# -----------------------------------------------------------------------------

(defn make-scheduler []
  @{:tasks @[]
    :current-time 0.0
    :running false})

(defn schedule-task! [sched task]
  (array/push (get sched :tasks) task)
  sched)

(defn run-scheduler-tick! [sched dt]
  "Execute one discrete scheduling cycle across all cooperative task fibers."
  (def cur-t (+ (get sched :current-time) dt))
  (put sched :current-time cur-t)

  (def tasks (get sched :tasks))
  (def active-tasks @[])

  (each task tasks
    (def f (get task :fiber))
    (def wake-t (get task :wake-time))
    (def status (fiber/status f))

    (when (and (not= status :dead) (>= cur-t wake-t))
      (def yield-val (resume f dt))
      (put task :run-count (+ (get task :run-count) 1))

      (cond
        # Task yields with sleep request: [:sleep seconds]
        (and (tuple? yield-val) (= (in yield-val 0) :sleep))
        (put task :wake-time (+ cur-t (in yield-val 1)))

        # Task yields with idle
        (= yield-val :idle)
        (put task :wake-time cur-t)

        # Default continue
        nil))

    (when (not= (fiber/status f) :dead)
      (array/push active-tasks task)))

  (put sched :tasks active-tasks)
  (length active-tasks))
