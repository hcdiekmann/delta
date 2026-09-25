# Delta robot simulation

This is a browser-based 3D simulation of a delta robot that can move its end effector to a target position in 3D space. It runs as a small Node.js/Express server that hosts the frontend (built with p5.js/WebGL). The server can also connect to a Beckhoff PLC over ADS to mirror a real robot.

![Demo](/docs/pictures/main.png)

#### Features

- Two delta robots with different arm geometries, using inverse and forward kinematics
- Manual control of the three motor angles with sliders
- A demo movement loop, a conveyor, and a product box
- Visualisation of each robot's kinematic reach
- Replaying a trajectory from a CSV file
- Connecting to a Beckhoff PLC to read the robot's current position over the ADS protocol

## Requirements

- [Node.js](https://nodejs.org/) 20 or newer (tested with Node 22)
- A modern browser with WebGL support
- *Optional:* a Beckhoff TwinCAT PLC and a local ADS router (see [PLC connection](#plc-connection))

## Setup

```bash
git clone <repo-url> delta
cd delta
npm install
npm start
```

Then open <http://localhost:3000/>.

If port 3000 is taken, pick another one:

```bash
PORT=3001 npm start
```

For development, `npm run dev` restarts the server automatically when `server.js` changes. Frontend files in `localhost/` are served statically, so reloading the browser is enough to pick up changes there.

## Usage

| Control | What it does |
| --- | --- |
| **Mouse wheel** | Zoom |
| **Middle mouse drag** | Orbit the camera |
| **Left mouse drag** | Pan the camera |
| **toggle movement loop** | Start/stop the demo motion of the blue robot |
| **motor 1–3 angle** + **update motor angles** | Move the blue robot to the chosen motor angles |
| **add product** | Drop a product onto the conveyor |
| **conveyor speed / product's conveyor position** | Adjust the conveyor |
| **Choose File** + **Read CSV** | Replay a trajectory from a CSV file |
| **Singular / Repeat** | Replay the CSV once or in a loop |
| **Kinematic Reach** | Show or hide the robots' reachable workspace |
| **use PLC values** | Drive the green robot with live PLC values (needs a PLC connection) |

### CSV trajectory format

See `localhost/DeltaRobot.csv` and `localhost/DeltaRobot - Demo.csv` for examples:

```csv
interval,pos0,pos1,pos2,hash
1000,-0.92,-0,0,b6e472987eb05265db453748cb32ba5d1f178411a2561030aa09e4d60e5102753ffee20e043b0096bbbdabc6d3a38ee1
1000,-0.87,-0.359,-0.45
```

- `interval` is the move duration in ms.
- `pos0`–`pos2` are the motor angles in radians.
- The `hash` in the first row identifies the target robot and must match `deltaRobotHash` in `localhost/js/objects/deltaRobot.js`.

## PLC connection

The server uses [ads-client](https://github.com/jisotalo/ads-client) to talk to a TwinCAT PLC. It requires an ADS router: TwinCAT itself on Windows, or a standalone router such as [AdsRouterConsole](https://github.com/Beckhoff/TF6000_ADS_DOTNET_V5_Samples) on Linux/macOS.

1. Enter the PLC's **AmsNetID** (e.g. `192.168.168.18.1.1`) and **AdsPort** (e.g. `851`) in the top right and click **Connect**.
2. Click **use PLC values** to have the green robot follow the PLC variables `MAIN.POS0`, `MAIN.POS1` and `MAIN.POS2`.

Server events and ADS errors are logged to `serverLog.db` (NeDB, one JSON document per line).

### Server API

| Method | Route | Description |
| --- | --- | --- |
| `POST` | `/connect` | `{ "AmsNetID": "...", "AdsPort": 851 }`: connect to the PLC |
| `GET` | `/disconnect` | Disconnect from the PLC |
| `GET` | `/connected` | `200` if connected, else `500` |
| `GET` | `/state` | PLC runtime state |
| `GET` | `/pos0` … `/pos2` | Current value of `MAIN.POS0` … `MAIN.POS2` |
| `GET` | `/config` | Installation settings XML (`config files/installationsettings.xml`) |

## Project structure

```
server.js                 Express server + ADS bridge
config files/             Installation / product settings (XML)
docs/                     Design document and screenshots
localhost/                Static frontend
  index.HTML              Entry page
  Libraries/              Vendored JS libraries (p5.js, glm-js, math.js, PapaParse, ...)
  js/superClasses/        Base classes (DisplayObject, Robot, Actuator, Conveyor, Case)
  js/objects/             DeltaRobot, Servo, StepperMotor, Product, Origin
  js/communication/       Fetch wrappers for the server API
  js/main.js              Scene setup and UI handlers
  js/display.js           Render loop and camera
```

The design document with the kinematics derivation is in [`docs/DesignDocument.pdf`](docs/DesignDocument.pdf).
