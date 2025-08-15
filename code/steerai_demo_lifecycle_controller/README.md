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

#### 1) Component / Package Structure
```mermaid
flowchart TB
  subgraph Pkg[steerai_demo_lifecycle_controller (ament_python)]
    subgraph Core[core (ROS-agnostic)]
      SM[state_machine.py\nMode/Event/Status + history]
      BHV[behavior.py\ncompute_command()]
    end

    Node[node.py\nLifecycleNode adapter]
    Launch[launch/demo.launch.py]
    Tests[tests/\nunit + launch_testing]
  end

  Turtlesim[[turtlesim_node]]
  ROSDDS[(ROS 2 DDS)]
  X11[X Server]

  Node -- pub/sub/srv/action/timer --> ROSDDS
  Turtlesim -- /pose ↔ /cmd_vel --> ROSDDS
  Node -- converts --> Core
  Turtlesim -. GUI .-> X11
```

#### 2) Class Diagram (Core + Adapter) 
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

#### 3) Lifecycle Sequence (Configure → Activate → Run → Deactivate)
```mermaid
sequenceDiagram
  autonumber
  participant L as launch/demo.launch.py
  participant C as TurtleLifecycleController
  participant DDS as ROS 2 DDS
  participant T as turtlesim_node

  L->>T: start executable
  L->>C: start LifecycleNode (UNCONFIGURED)

  Note over C: ros2 lifecycle set ... configure
  C->>C: on_configure()<br/>create lifecycle publisher, sub, services, action, timer (paused)
  C->>DDS: register pub/sub/services/action
  C-->>L: INACTIVE

  Note over C: ros2 lifecycle set ... activate
  C->>C: on_activate()<br/>_is_active = True, publisher.on_activate(), timer.reset()
  C-->>L: ACTIVE

  loop Every timer tick
    C->>C: read parameters (lin, ang); status := _sm.status
    C->>C: cmd = compute_command(status, _circle_enabled, lin, ang)
    C->>DDS: publish Twist(/turtle1/cmd_vel)
    DDS->>T: deliver Twist
  end

  Note over C: ros2 lifecycle set ... deactivate
  C->>C: on_deactivate()<br/>_is_active=False, timer.cancel(), publisher.on_deactivate()
  C-->>L: INACTIVE
```

#### 4) Operational Mode & Events (Internal State Machine)
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

#### 5) Service / Action Interactions (when ACTIVE)
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