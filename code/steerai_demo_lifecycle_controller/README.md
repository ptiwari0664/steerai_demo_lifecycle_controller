# steerai_demo_lifecycle_controller

A **ROS 2 LifecycleNode–based demo controller** for `turtlesim` that demonstrates:

- ✅ **Full LifecycleNode support** (`configure`, `activate`, `deactivate`, `cleanup`, `shutdown`)
- ✅ **Internal operational state machine** (modes: `IDLE`, `CIRCLE`, `ERROR`)
- ✅ **Separation of concerns** — Core algorithm logic is ROS-agnostic and unit-tested
- ✅ **Dynamic parameters** with validation and descriptor metadata
- ✅ **All major ROS 2 communication patterns**:
  - Lifecycle Publisher (`/turtle1/cmd_vel`)
  - Subscriber (`/turtle1/pose`)
  - Services (`/set_mode`, `/toggle_circle`, `/go_home`, `/health`)
  - Client (`/turtle1/teleport_absolute`)
  - Action Server (`/demo_action` implementing `example_interfaces/action/Fibonacci`)
  - Timers

---

## System Architecture and Process Flow

#### 1) Class Diagram (Core + Adapter) 
```mermaid
classDiagram
  direction LR

  class Mode {
    <<enum>>
    IDLE
    CIRCLE
  }

  class Event {
    <<enum>>
    SET_IDLE
    SET_CIRCLE
    ERROR
    CLEAR_ERROR
  }

  class Status {
    +Mode mode
    +bool has_error
    +str? reason
    +float updated_at
  }

  class TransitionRec {
    +float t
    +Mode frm
    +Event evt
    +Mode to
    +str? reason
    +bool has_error
  }

  class StateMachine {
    -Status _status
    -deque~TransitionRec~ _hist
    +status : Status
    +dispatch(evt: Event, reason?: str) : Status
    +history() : list~TransitionRec~
    +last_transition() : TransitionRec?
    +clear_history() : void
    -_apply(status: Status, evt: Event, reason?: str) : Status
  }

  class Command {
    +float lin_x
    +float ang_z
  }

  class Behavior {
    <<module>>
    +compute_command(status: Status, circle_enabled: bool, lin: float, ang: float) : Command
  }

  class TurtleLifecycleController {
    <<LifecycleNode>>
    -StateMachine _sm
    -bool _is_active
    -bool _circle_enabled
    -Publisher~Twist~ _pub
    -Subscription~Pose~ _pose_sub
    -Timer _timer
    -Services: /toggle_circle, /set_mode, /go_home, /health
    -ActionServer~Fibonacci~ _action
    +on_configure() : TransitionCallbackReturn
    +on_activate() : TransitionCallbackReturn
    +on_deactivate() : TransitionCallbackReturn
    +on_cleanup() : TransitionCallbackReturn
    +on_shutdown() : TransitionCallbackReturn
    +_on_timer() : void
    +_on_param_update(...) : SetParametersResult
  }

  StateMachine <.. Behavior : uses
  TurtleLifecycleController --> StateMachine : owns
  TurtleLifecycleController ..> Behavior : calls
```

#### 2) Lifecycle Sequence (Configure → Activate → Run → Deactivate)
```mermaid
sequenceDiagram
  autonumber
  participant L as launcher
  participant C as TurtleLifecycleController
  participant D as ROS2 DDS
  participant T as turtlesim_node

  L->>T: start executable
  L->>C: start lifecycle node UNCONFIGURED

  Note over C: ros2 lifecycle set ... configure
  C->>C: on_configure()\ncreate lifecycle pub sub srv action\ncreate timer paused
  C->>D: register endpoints
  C-->>L: INACTIVE

  Note over C: ros2 lifecycle set ... activate
  C->>C: on_activate()\nset active flag\npublisher.on_activate()\ntimer.reset()
  C-->>L: ACTIVE

  loop every timer tick
    C->>C: read parameters and status\ncompute command
    C->>D: publish Twist on /turtle1/cmd_vel
    D->>T: deliver Twist
  end

  Note over C: ros2 lifecycle set ... deactivate
  C->>C: on_deactivate()\nclear active flag\ntimer.cancel()\npublisher.on_deactivate()
  C-->>L: INACTIVE
```

#### 3) Operational Mode & Events (Internal State Machine)
```mermaid
stateDiagram-v2
    [*] --> IDLE
    state "ERROR FREEZE" as ERR <<choice>>

    IDLE --> CIRCLE: Event.SET_CIRCLE
    CIRCLE --> IDLE: Event.SET_IDLE

    IDLE --> ERR: Event.ERROR(reason)
    CIRCLE --> ERR: Event.ERROR(reason)

    ERR --> IDLE: Event.CLEAR_ERROR [previous mode was IDLE]
    ERR --> CIRCLE: Event.CLEAR_ERROR [previous mode was CIRCLE]

    note right of ERR
      While in error:
      • Mode-change events ignored
      • Behavior publishes safe (zero) commands
    end note
```

#### 4) Service / Action Interactions (when ACTIVE)
```mermaid
sequenceDiagram
  autonumber
  participant OP as Operator
  participant C as TurtleLifecycleController
  participant SM as StateMachine
  participant B as Behavior

  rect rgb(230,255,230)
  Note over C: ACTIVE gate (C._is_active == true)
  end

  OP->>C: /set_mode(SetBool{data:true})
  C->>SM: dispatch(Event.SET_CIRCLE)
  SM-->>C: Status{mode=CIRCLE}
  C-->>OP: success, "Mode=CIRCLE"

  OP->>C: /toggle_circle(SetBool{data:true})
  C->>C: _circle_enabled = true
  C-->>OP: success, "circle ON"

  C->>B: compute_command(Status, true, lin, ang)
  B-->>C: Command{lin_x, ang_z}
  C->>turtlesim: /cmd_vel(Twist)
```