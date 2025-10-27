import React, { useState, useRef } from 'react';
import { Gamepad2, Radio, Activity, Power, AlertCircle, Zap, Settings } from 'lucide-react';

export default function CarlROS2Dashboard() {
  // ---- state ----
  const [connected, setConnected] = useState(false);
  const [connectionError, setConnectionError] = useState('');
  const [controllerState, setControllerState] = useState({
    axes: Array(6).fill(0),
    buttons: Array(16).fill(0)
  });
  const [whegFeedback, setWhegFeedback] = useState({
    motor_id: [],
    position_degrees: [],
    velocity_rpm: [],
    load_percentage: [],
    error_status: []
  });
  const [cmdVel, setCmdVel] = useState({
    linear: { x: 0, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: 0 }
  });
  const [gaitSelection, setGaitSelection] = useState({
    gait_number: 1,
    body_number: 1,
    wheg_number: 1
  });
  const [speedMode, setSpeedMode] = useState(2.0);
  const [jointCmd, setJointCmd] = useState({
    front_up: 0,
    front_down: 0,
    back_up: 0,
    back_down: 0
  });

  const rosRef = useRef(null);
  const [raspberryPiIP, setRaspberryPiIP] = useState('10.13.121.89');
  const [showSettings, setShowSettings] = useState(false);

  // ---- helpers ----
  const safeNum = (v, fallback = 0) => {
    // Accept numbers and numeric strings; fall back to `fallback`.
    if (typeof v === 'number' && isFinite(v)) return v;
    const n = Number(v);
    return isFinite(n) ? n : fallback;
  };

  const fmt = (v, decimals = 1) => {
    const n = safeNum(v, null);
    return n === null ? '--' : n.toFixed(decimals);
  };

  // ---- connect / websocket ----
  const connectToROS = () => {
    if (connected) {
      if (rosRef.current) {
        rosRef.current.close();
        rosRef.current = null;
      }
      setConnected(false);
      setConnectionError('');
      return;
    }

    try {
      const ws = new WebSocket(`ws://${raspberryPiIP}:9090`);

      ws.onopen = () => {
        console.log('Connected to ROS bridge');
        setConnected(true);
        setConnectionError('');

        // NOTE: rosbridge type format is usually 'pkg_name/msg/MsgType'
        const subscribe = (topic, type) => ws.send(JSON.stringify({ op: 'subscribe', topic, type }));

        subscribe('/controller_state', 'std_msgs/msg/String');
        subscribe('/wheg_feedback', 'custom_msgs/msg/WhegFeedback');
        subscribe('/cmd_vel', 'geometry_msgs/msg/Twist');
        subscribe('/gait_selection', 'custom_msgs/msg/GaitCommand');
        subscribe('/speed_mode', 'std_msgs/msg/Float32');
        subscribe('/joint_cmd', 'custom_msgs/msg/Joint');
      };

      ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          // debug
          // console.log('Received message:', data.topic, data);

          if (data.topic === '/controller_state') {
            // some nodes wrap the string inside data.msg.data
            try {
              const controllerData = typeof data.msg.data === 'string' ? JSON.parse(data.msg.data) : data.msg;
              setControllerState(controllerData);
            } catch (e) {
              console.warn('Failed to parse controller_state payload', e, data.msg);
            }

          } else if (data.topic === '/wheg_feedback') {
            // raw msg may contain numeric strings depending on rosbridge serialization
            const msg = data.msg || {};

            // Defensive checks
            const motor_id = Array.isArray(msg.motor_id) ? msg.motor_id.map((v) => safeNum(v, null)).filter(v => v !== null) : [];
            const position_degrees = Array.isArray(msg.position_degrees) ? msg.position_degrees.map(v => safeNum(v, 0)) : [];
            const velocity_rpm = Array.isArray(msg.velocity_rpm) ? msg.velocity_rpm.map(v => safeNum(v, 0)) : [];
            const load_percentage = Array.isArray(msg.load_percentage) ? msg.load_percentage.map(v => safeNum(v, 0)) : [];
            const error_status = Array.isArray(msg.error_status) ? msg.error_status.map(v => parseInt(v, 10) || 0) : [];

            // Ensure arrays have same length as motor_id
            const length = motor_id.length || Math.max(position_degrees.length, velocity_rpm.length, load_percentage.length, error_status.length);

            const normalized = {
              motor_id: motor_id.length ? motor_id : Array.from({ length }, (_, i) => i + 1),
              position_degrees: position_degrees.length ? position_degrees.slice(0, length) : Array(length).fill(0),
              velocity_rpm: velocity_rpm.length ? velocity_rpm.slice(0, length) : Array(length).fill(0),
              load_percentage: load_percentage.length ? load_percentage.slice(0, length) : Array(length).fill(0),
              error_status: error_status.length ? error_status.slice(0, length) : Array(length).fill(0)
            };

            // quick type sanity log (only when dev)
            // console.log('Processed wheg feedback:', normalized);
            setWhegFeedback(normalized);

          } else if (data.topic === '/cmd_vel') {
            // geometry_msgs/Twist is usually nested in data.msg
            const m = data.msg || { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } };
            setCmdVel({
              linear: { x: safeNum(m.linear?.x, 0), y: safeNum(m.linear?.y, 0), z: safeNum(m.linear?.z, 0) },
              angular: { x: safeNum(m.angular?.x, 0), y: safeNum(m.angular?.y, 0), z: safeNum(m.angular?.z, 0) }
            });

          } else if (data.topic === '/gait_selection') {
            const m = data.msg || {};
            setGaitSelection({
              gait_number: safeNum(m.gait_number, 1),
              body_number: safeNum(m.body_number, 1),
              wheg_number: safeNum(m.wheg_number, 1)
            });

          } else if (data.topic === '/speed_mode') {
            setSpeedMode(safeNum(data.msg?.data, 2.0));

          } else if (data.topic === '/joint_cmd') {
            const m = data.msg || {};
            setJointCmd({
              front_up: safeNum(m.front_up, 0),
              front_down: safeNum(m.front_down, 0),
              back_up: safeNum(m.back_up, 0),
              back_down: safeNum(m.back_down, 0)
            });
          }
        } catch (e) {
          console.error('Error parsing ROS message:', e, event.data);
        }
      };

      ws.onerror = (error) => {
        console.error('WebSocket error:', error);
        setConnectionError('Failed to connect to ROS bridge at ws://' + raspberryPiIP + ':9090');
        setConnected(false);
      };

      ws.onclose = () => {
        console.log('Disconnected from ROS bridge');
        setConnected(false);
        rosRef.current = null;
      };

      rosRef.current = ws;

    } catch (error) {
      setConnectionError('Error connecting to ROS: ' + error.message);
      setConnected(false);
    }
  };

  // ---- UI helpers ----
  const buttonNames = [
    'Cross', 'Circle', 'Square', 'Triangle', 'Share', 'PS', 'Options',
    'L-Joy', 'R-Joy', 'L1', 'R1', 'Up', 'Down', 'Left', 'Right', 'Touchpad'
  ];

  const gaitNames = ['Gait 1', 'Gait 2', 'Gait 3', 'Gait 4', 'Gait 5'];

  const motorNames = {
    1: 'Front Left',
    2: 'Middle Left', 
    3: 'Back Left',
    4: 'Front Right',
    5: 'Middle Right',
    6: 'Back Right'
  };

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-900 via-purple-900 to-slate-900 text-white p-6">
      <div className="max-w-7xl mx-auto">
        {/* Header */}
        <div className="flex items-center justify-between mb-8">
          <div className="flex items-center gap-3">
            <Activity className="w-8 h-8 text-purple-400" />
            <h1 className="text-3xl font-bold">CARL Robot Dashboard</h1>
          </div>
          <div className="flex items-center gap-4">
            <button
              onClick={() => setShowSettings(!showSettings)}
              className="px-4 py-2 rounded-lg bg-slate-700 hover:bg-slate-600 transition-all"
            >
              <Settings className="w-5 h-5" />
            </button>
            <button
              onClick={connectToROS}
              className={`flex items-center gap-2 px-6 py-3 rounded-lg font-semibold transition-all ${
                connected
                  ? 'bg-red-600 hover:bg-red-700'
                  : 'bg-green-600 hover:bg-green-700'
              }`}
            >
              <Radio className="w-5 h-5" />
              {connected ? 'Disconnect' : 'Connect to ROS'}
            </button>
          </div>
        </div>

        {/* Settings Panel */}
        {showSettings && (
          <div className="bg-slate-800/50 backdrop-blur-lg rounded-xl p-6 mb-6 border border-slate-700">
            <h3 className="text-lg font-semibold mb-4">Connection Settings</h3>
            <div className="flex items-center gap-4">
              <label className="text-sm text-gray-300">Raspberry Pi IP:</label>
              <input
                type="text"
                value={raspberryPiIP}
                onChange={(e) => setRaspberryPiIP(e.target.value)}
                className="bg-slate-700 text-white px-4 py-2 rounded-lg border border-slate-600 focus:border-purple-500 focus:outline-none"
                placeholder="10.13.121.179"
              />
              <span className="text-sm text-gray-400">Port: 9090</span>
            </div>
          </div>
        )}

        {/* Connection Status */}
        {!connected && connectionError && (
          <div className="bg-red-600/20 border border-red-600/50 rounded-lg p-4 mb-6 flex items-center gap-3">
            <AlertCircle className="w-5 h-5 text-red-400" />
            <p className="text-red-100">{connectionError}</p>
          </div>
        )}

        {!connected && !connectionError && (
          <div className="bg-yellow-600/20 border border-yellow-600/50 rounded-lg p-4 mb-6 flex items-center gap-3">
            <AlertCircle className="w-5 h-5 text-yellow-400" />
            <p className="text-yellow-100">
              Not connected to ROS bridge. Make sure rosbridge is running on your Raspberry Pi.
            </p>
          </div>
        )}

        {connected && (
          <div className="bg-green-600/20 border border-green-600/50 rounded-lg p-4 mb-6 flex items-center gap-3">
            <Radio className="w-5 h-5 text-green-400 animate-pulse" />
            <p className="text-green-100">Connected to ROS at ws://{raspberryPiIP}:9090</p>
          </div>
        )}

        {/* Gait and Control Info Bar */}
        <div className="grid grid-cols-4 gap-4 mb-6">
          <div className="bg-purple-900/30 backdrop-blur-lg rounded-xl p-4 border border-purple-500/30">
            <div className="text-sm text-gray-400 mb-1">Current Gait</div>
            <div className="text-2xl font-bold text-purple-400">
              {gaitNames[gaitSelection.gait_number - 1] || 'N/A'}
            </div>
          </div>
          <div className="bg-blue-900/30 backdrop-blur-lg rounded-xl p-4 border border-blue-500/30">
            <div className="text-sm text-gray-400 mb-1">Body Number</div>
            <div className="text-2xl font-bold text-blue-400">
              {whegFeedback.motor_id.length ? gaitSelection.body_number : gaitSelection.body_number}
            </div>
          </div>
          <div className="bg-cyan-900/30 backdrop-blur-lg rounded-xl p-4 border border-cyan-500/30">
            <div className="text-sm text-gray-400 mb-1">Wheg Number</div>
            <div className="text-2xl font-bold text-cyan-400">
              {gaitSelection.wheg_number}
            </div>
          </div>
          <div className="bg-green-900/30 backdrop-blur-lg rounded-xl p-4 border border-green-500/30">
            <div className="text-sm text-gray-400 mb-1">Speed Mode</div>
            <div className="text-2xl font-bold text-green-400">
              {fmt(speedMode, 1)}x
            </div>
          </div>
        </div>

        <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
          {/* Wheg Motor Status */}
          <div className="bg-white/10 backdrop-blur-lg rounded-xl p-6 border border-white/20">
            <div className="flex items-center gap-2 mb-4">
              <Power className="w-6 h-6 text-cyan-400" />
              <h2 className="text-xl font-semibold">Wheg Motors</h2>
            </div>

            <div className="space-y-4">
              {whegFeedback.motor_id && whegFeedback.motor_id.length > 0 ? (
                whegFeedback.motor_id.map((motorId, index) => {
                  const position = whegFeedback.position_degrees?.[index] ?? 0;
                  const velocity = whegFeedback.velocity_rpm?.[index] ?? 0;
                  const load = whegFeedback.load_percentage?.[index] ?? 0;
                  const error = whegFeedback.error_status?.[index] ?? 0;

                  return (
                    <div key={`motor-${motorId}-${index}`} className="bg-slate-800/50 rounded-lg p-3">
                      <div className="flex justify-between items-center mb-2">
                        <h3 className="text-sm font-semibold text-purple-300">
                          {motorNames[motorId] || `Motor ${motorId}`}
                        </h3>
                        {error !== 0 && (
                          <div className="flex items-center gap-1 text-red-400 text-xs">
                            <AlertCircle className="w-3 h-3" />
                            <span>Error: {error}</span>
                          </div>
                        )}
                      </div>

                      <div className="grid grid-cols-3 gap-3 text-xs">
                        <div>
                          <div className="text-gray-400">Position</div>
                          <div className="font-mono text-green-300">{fmt(position, 1)}°</div>
                        </div>
                        <div>
                          <div className="text-gray-400">Velocity</div>
                          <div className="font-mono text-cyan-300">{fmt(velocity, 1)} RPM</div>
                        </div>
                        <div>
                          <div className="text-gray-400">Load</div>
                          <div className="font-mono text-orange-300">{fmt(load, 1)}%</div>
                        </div>
                      </div>

                      <div className="mt-2 grid grid-cols-2 gap-2">
                        <div>
                          <div className="w-full bg-slate-700/50 rounded-full h-1.5">
                            <div
                              className="bg-cyan-500 h-1.5 rounded-full transition-all"
                              style={{ 
                                width: `${Math.min(100, Math.abs(safeNum(velocity)) / 2)}%` 
                              }}
                            />
                          </div>
                        </div>
                        <div>
                          <div className="w-full bg-slate-700/50 rounded-full h-1.5">
                            <div
                              className="bg-orange-500 h-1.5 rounded-full transition-all"
                              style={{ 
                                width: `${Math.min(100, Math.abs(safeNum(load)))}%` 
                              }}
                            />
                          </div>
                        </div>
                      </div>
                    </div>
                  );
                })
              ) : (
                <div className="text-center text-gray-400 py-8">No motor data. Waiting for /wheg_feedback topic...</div>
              )}
            </div>
          </div>

          {/* PS4 Controller Status */}
          <div className="bg-white/10 backdrop-blur-lg rounded-xl p-6 border border-white/20 flex flex-col items-center">
            <div className="flex items-center gap-2 mb-4">
              <Gamepad2 className="w-6 h-6 text-purple-400" />
              <h2 className="text-xl font-semibold">PS4 Controller</h2>
            </div>

            {/* Centered and scaled SVG */}
            <div className="w-full max-w-xs">
              <svg
                viewBox="0 0 441 383"
                width="100%"
                height="auto"
                className="mx-auto block"
                fill="none"
                xmlns="http://www.w3.org/2000/svg"
              >
                <g id="PS4Controller">
                  {/* Controller outlines */}
                  <path id="LOutline" d="M220.5 294.5C220.5 294.5 195 294.5 150 294.5C105 294.5 81.5 378.5 49.5 378.5C17.5 378.5 4 363.9 4 317.5C4 271.1 43.5 165.5 55 137.5C66.5 109.5 95.5 92.0001 128 92.0001C154 92.0001 200.5 92.0001 220.5 92.0001" stroke="hsl(210,50%,85%)" stroke-width="3" stroke-opacity="1"></path>
                  <path id="ROutline" d="M220 294.5C220 294.5 245.5 294.5 290.5 294.5C335.5 294.5 359 378.5 391 378.5C423 378.5 436.5 363.9 436.5 317.5C436.5 271.1 397 165.5 385.5 137.5C374 109.5 345 92.0001 312.5 92.0001C286.5 92.0001 240 92.0001 220 92.0001" stroke="hsl(210,50%,85%)" stroke-width="3" stroke-opacity="1"></path>
                  <circle id="LStickOutline" cx="113" cy="160" r="37.5" stroke="hsl(210,50%,85%)" stroke-opacity="1" stroke-width="3"></circle>
                  <circle id="RStickOutline" cx="278" cy="238" r="37.5" stroke="hsl(210,50%,85%)" stroke-opacity="1" stroke-width="3"></circle>
                  <circle id="DOutline" cx="166" cy="238" r="37.5" stroke="hsl(210,50%,85%)" stroke-opacity="1" stroke-width="3"></circle>
                  <circle id="LeftStick" cx="165.9529411792755" cy="237.9529411792755" r="28" fill="rgba(0,0,0,0)" stroke="rgba(0,0,0,1)" stroke-width="3"></circle>
                  <circle id="LStickDot" cx="165.9529411792755" cy="237.9529411792755" r="20" fill="rgba(0,0,0,0)" stroke="rgba(255,255,255,0)" stroke-width="3"></circle>
                  <circle id="BOutline" cx="329" cy="160" r="37.5" stroke="hsl(210,50%,85%)" stroke-opacity="1" stroke-width="3"></circle>
                  <circle id="RightStick" cx="277.6705882549286" cy="237.9529411792755" r="28" fill="rgba(0,0,0,0)" stroke="rgba(0,0,0,1)" stroke-width="3"></circle>
                  <circle id="RStickDot" cx="277.6705882549286" cy="237.9529411792755" r="20" fill="rgba(0,0,0,0)" stroke="rgba(255,255,255,0)" stroke-width="3"></circle>
                  <g transform="translate(-53, -78)">
                    <g id="DUp">
                      <mask id="path-8-inside-1" fill="white">
                        <path d="M177.669 222.335C180.793 219.21 180.816 213.997 176.868 212.014C176.327 211.743 175.776 211.491 175.215 211.258C172.182 210.002 168.931 209.355 165.648 209.355C162.365 209.355 159.114 210.002 156.081 211.258C155.521 211.491 154.969 211.743 154.429 212.014C150.48 213.997 150.503 219.21 153.627 222.335L159.991 228.698C163.116 231.823 168.181 231.823 171.305 228.698L177.669 222.335Z"></path>
                      </mask>
                      <path 
                        d="M177.669 222.335C180.793 219.21 180.816 213.997 176.868 212.014C176.327 211.743 175.776 211.491 175.215 211.258C172.182 210.002 168.931 209.355 165.648 209.355C162.365 209.355 159.114 210.002 156.081 211.258C155.521 211.491 154.969 211.743 154.429 212.014C150.48 213.997 150.503 219.21 153.627 222.335L159.991 228.698C163.116 231.823 168.181 231.823 171.305 228.698L177.669 222.335Z" 
                        fill={controllerState.buttons.up ? "black" : "transparent"} 
                        stroke="black" 
                        stroke-width="6"
                        mask="url(#path-8-inside-1)">
                      </path>
                    </g>
                    <g id="DRight">
                      <mask id="path-9-inside-2" fill="white">
                          <path d="M181.447 249.669C184.571 252.793 189.785 252.816 191.768 248.868C192.039 248.327 192.291 247.776 192.523 247.215C193.78 244.182 194.426 240.931 194.426 237.648C194.426 234.365 193.78 231.114 192.523 228.081C192.291 227.521 192.039 226.969 191.768 226.429C189.785 222.48 184.571 222.503 181.447 225.627L175.083 231.991C171.959 235.116 171.959 240.181 175.083 243.305L181.447 249.669Z"></path>
                      </mask>
                      <path 
                        d="M181.447 249.669C184.571 252.793 189.785 252.816 191.768 248.868C192.039 248.327 192.291 247.776 192.523 247.215C193.78 244.182 194.426 240.931 194.426 237.648C194.426 234.365 193.78 231.114 192.523 228.081C192.291 227.521 192.039 226.969 191.768 226.429C189.785 222.48 184.571 222.503 181.447 225.627L175.083 231.991C171.959 235.116 171.959 240.181 175.083 243.305L181.447 249.669Z" 
                        fill={controllerState.buttons.right ? "black" : "transparent"} 
                        stroke="black" 
                        stroke-width="6" 
                        mask="url(#path-9-inside-2)">
                      </path>
                    </g>
                    <g id="DDown">
                      <mask id="path-10-inside-3" fill="white">
                        <path d="M154.113 253.447C150.989 256.571 150.966 261.785 154.914 263.767C155.455 264.039 156.006 264.291 156.566 264.523C159.6 265.78 162.85 266.426 166.134 266.426C169.417 266.426 172.667 265.78 175.701 264.523C176.261 264.291 176.812 264.039 177.353 263.767C181.301 261.785 181.279 256.571 178.154 253.447L171.79 247.083C168.666 243.959 163.601 243.959 160.477 247.083L154.113 253.447Z"></path>
                      </mask>
                      <path 
                        d="M154.113 253.447C150.989 256.571 150.966 261.785 154.914 263.767C155.455 264.039 156.006 264.291 156.566 264.523C159.6 265.78 162.85 266.426 166.134 266.426C169.417 266.426 172.667 265.78 175.701 264.523C176.261 264.291 176.812 264.039 177.353 263.767C181.301 261.785 181.279 256.571 178.154 253.447L171.79 247.083C168.666 243.959 163.601 243.959 160.477 247.083L154.113 253.447Z" 
                        fill={controllerState.buttons.down ? "black" : "transparent"} 
                        stroke="black" 
                        stroke-width="6" 
                        mask="url(#path-10-inside-3)"></path>
                    </g>
                    <g id="DLeft">
                      <mask id="path-11-inside-4" fill="white">
                        <path d="M150.335 226.113C147.21 222.989 141.997 222.966 140.014 226.914C139.743 227.455 139.491 228.006 139.258 228.566C138.002 231.6 137.355 234.85 137.355 238.134C137.355 241.417 138.002 244.667 139.258 247.701C139.491 248.261 139.743 248.812 140.014 249.353C141.997 253.301 147.21 253.279 150.335 250.154L156.698 243.79C159.823 240.666 159.823 235.601 156.698 232.477L150.335 226.113Z"></path>
                      </mask>
                      <path 
                        d="M150.335 226.113C147.21 222.989 141.997 222.966 140.014 226.914C139.743 227.455 139.491 228.006 139.258 228.566C138.002 231.6 137.355 234.85 137.355 238.134C137.355 241.417 138.002 244.667 139.258 247.701C139.491 248.261 139.743 248.812 140.014 249.353C141.997 253.301 147.21 253.279 150.335 250.154L156.698 243.79C159.823 240.666 159.823 235.601 156.698 232.477L150.335 226.113Z" 
                        fill={controllerState.buttons.left ? "black" : "transparent"} 
                        stroke="black" 
                        stroke-width="6"
                        mask="url(#path-11-inside-4)">
                      </path>
                    </g>
                  </g>
                  <g id="BTop">
                    <mask id="path-13-inside-5" fill="white">
                      <path d="M340.669 144.335C343.793 141.21 343.816 135.997 339.868 134.014C339.327 133.743 338.776 133.491 338.215 133.258C335.182 132.002 331.931 131.355 328.648 131.355C325.365 131.355 322.114 132.002 319.081 133.258C318.521 133.491 317.969 133.743 317.429 134.014C313.48 135.997 313.503 141.21 316.627 144.335L322.991 150.698C326.116 153.823 331.181 153.823 334.305 150.698L340.669 144.335Z"></path>
                    </mask>
                    <path 
                      d="M340.669 144.335C343.793 141.21 343.816 135.997 339.868 134.014C339.327 133.743 338.776 133.491 338.215 133.258C335.182 132.002 331.931 131.355 328.648 131.355C325.365 131.355 322.114 132.002 319.081 133.258C318.521 133.491 317.969 133.743 317.429 134.014C313.48 135.997 313.503 141.21 316.627 144.335L322.991 150.698C326.116 153.823 331.181 153.823 334.305 150.698L340.669 144.335Z" 
                      fill={controllerState.buttons.triangle ? "black" : "transparent"}  
                      stroke="black" 
                      stroke-width="6" 
                      mask="url(#path-13-inside-5)">
                    </path>
                  </g>
                  <g id="BRight">
                    <mask id="path-14-inside-6" fill="white">
                      <path d="M344.447 171.669C347.571 174.793 352.785 174.816 354.768 170.868C355.039 170.327 355.291 169.776 355.523 169.215C356.78 166.182 357.426 162.931 357.426 159.648C357.426 156.365 356.78 153.114 355.523 150.081C355.291 149.521 355.039 148.969 354.768 148.429C352.785 144.48 347.571 144.503 344.447 147.627L338.083 153.991C334.959 157.116 334.959 162.181 338.083 165.305L344.447 171.669Z"></path>
                    </mask>
                    <path 
                      d="M344.447 171.669C347.571 174.793 352.785 174.816 354.768 170.868C355.039 170.327 355.291 169.776 355.523 169.215C356.78 166.182 357.426 162.931 357.426 159.648C357.426 156.365 356.78 153.114 355.523 150.081C355.291 149.521 355.039 148.969 354.768 148.429C352.785 144.48 347.571 144.503 344.447 147.627L338.083 153.991C334.959 157.116 334.959 162.181 338.083 165.305L344.447 171.669Z" 
                      fill={controllerState.buttons.circle ? "black" : "transparent"}  
                      stroke="black" 
                      stroke-width="6" 
                      mask="url(#path-14-inside-6)">
                    </path>
                  </g>
                  <g id="BBottom">
                    <mask id="path-15-inside-7" fill="white">
                      <path d="M317.113 175.447C313.989 178.571 313.966 183.785 317.914 185.767C318.455 186.039 319.006 186.291 319.566 186.523C322.6 187.78 325.85 188.426 329.134 188.426C332.417 188.426 335.667 187.78 338.701 186.523C339.261 186.291 339.812 186.039 340.353 185.767C344.301 183.785 344.279 178.571 341.154 175.447L334.79 169.083C331.666 165.959 326.601 165.959 323.477 169.083L317.113 175.447Z"></path>
                    </mask>
                    <path 
                      d="M317.113 175.447C313.989 178.571 313.966 183.785 317.914 185.767C318.455 186.039 319.006 186.291 319.566 186.523C322.6 187.78 325.85 188.426 329.134 188.426C332.417 188.426 335.667 187.78 338.701 186.523C339.261 186.291 339.812 186.039 340.353 185.767C344.301 183.785 344.279 178.571 341.154 175.447L334.79 169.083C331.666 165.959 326.601 165.959 323.477 169.083L317.113 175.447Z" 
                      fill={controllerState.buttons.cross ? "black" : "transparent"}  
                      stroke="black" 
                      stroke-width="6" 
                      mask="url(#path-15-inside-7)">
                    </path>
                  </g>
                  <g id="BLeft">
                    <mask id="path-16-inside-8" fill="white">
                      <path d="M313.335 148.113C310.21 144.989 304.997 144.966 303.014 148.914C302.743 149.455 302.491 150.006 302.258 150.566C301.002 153.6 300.355 156.851 300.355 160.134C300.355 163.417 301.002 166.668 302.258 169.701C302.491 170.261 302.743 170.812 303.014 171.353C304.997 175.301 310.21 175.279 313.335 172.154L319.698 165.79C322.823 162.666 322.823 157.601 319.698 154.477L313.335 148.113Z"></path>
                    </mask>
                    <path 
                      d="M313.335 148.113C310.21 144.989 304.997 144.966 303.014 148.914C302.743 149.455 302.491 150.006 302.258 150.566C301.002 153.6 300.355 156.851 300.355 160.134C300.355 163.417 301.002 166.668 302.258 169.701C302.491 170.261 302.743 170.812 303.014 171.353C304.997 175.301 310.21 175.279 313.335 172.154L319.698 165.79C322.823 162.666 322.823 157.601 319.698 154.477L313.335 148.113Z" 
                      fill={controllerState.buttons.square ? "black" : "transparent"} 
                      stroke="black" 
                      stroke-width="6" 
                      mask="url(#path-16-inside-8)">
                    </path>
                  </g>

                  <g id="LMeta">
                    <circle 
                      cx="160" cy="119" r="10" 
                      fill={controllerState.buttons.share ? "black" : "transparent"}
                      stroke="black" 
                      stroke-width="3">
                    </circle>
                  </g>

                  <rect id ="TouchPad"
                    x="175.5"
                    y="129"
                    width="94"
                    height="51"
                    rx="6.5"
                    fill={controllerState.buttons.l1 ? "black" : "transparent"}
                    stroke="black"
                    strokeWidth="3"
                  />
                  
                  <g id="RMeta">
                    <circle 
                      cx="284" cy="119" r="10" 
                      fill={controllerState.buttons.options ? "black" : "transparent"}
                      stroke="black" 
                      strokeWidth="3">
                    </circle>
                  </g>

                  {/* Shoulder buttons */}
                  <rect id ="L1"
                    x="111.5"
                    y="61.5"
                    width="41"
                    height="13"
                    rx="6.5"
                    fill={controllerState.buttons.l1 ? "black" : "transparent"}
                    stroke="black"
                    strokeWidth="3"
                  />
                  <rect id ="R1"
                    x="289.5"
                    y="61.5"
                    width="41"
                    height="13"
                    rx="6.5"
                    fill={controllerState.buttons.r1 ? "black" : "transparent"}
                    stroke="black"
                    strokeWidth="3"
                  />

                  {/* Triggers */}
                  <path id ="L2"
                    d="M152.5 37C152.5 41.1421 149.142 44.5 145 44.5H132C127.858 44.5 124.5 41.1421 124.5 37V16.5C124.5 8.76801 130.768 2.5 138.5 2.5C146.232 2.5 152.5 8.76801 152.5 16.5V37Z"
                    fill={controllerState.buttons.l2 ? "black" : "transparent"}
                    stroke="black"
                    strokeWidth="3"
                  />
                  <path id ="R2"
                    d="M317.5 37C317.5 41.1421 314.142 44.5 310 44.5H297C292.858 44.5 289.5 41.1421 289.5 37V16.5C289.5 8.76801 295.768 2.5 303.5 2.5C311.232 2.5 317.5 8.76801 317.5 16.5V37Z" 
                    fill={controllerState.buttons.r2 ? "black" : "transparent"}
                    stroke="black"
                    strokeWidth="3"
                  />

                  {/* Handle lines */}
                  <line 
                    x1="30" y1="210" x2="130" y2="300" 
                    stroke-width="3" 
                    stroke="hsl(210,50%,85%)" 
                    opacity="0.3">
                  </line>
                  <line 
                    x1="411" y1="210" x2="311" y2="300" 
                    stroke-width="3" 
                    stroke="hsl(210,50%,85%)" 
                    opacity="0.3"></line>
                </g>
              </svg>
            </div>

            {/* Triggers */}
            <div className="mb-4">
              <h3 className="text-xs font-medium text-gray-300 mb-2">Triggers</h3>
              <div className="space-y-1">
                <div className="flex items-center gap-2">
                  <span className="text-xs w-8">L2</span>
                  <div className="flex-1 bg-slate-700/50 rounded-full h-2">
                    <div
                      className="bg-orange-500 h-2 rounded-full transition-all"
                      style={{ width: `${((safeNum(controllerState.axes[4], -1) + 1) / 2) * 100}%` }}
                    />
                  </div>
                  <span className="text-xs font-mono w-12 text-right">{fmt(controllerState.axes[4], 2)}</span>
                </div>
                <div className="flex items-center gap-2">
                  <span className="text-xs w-8">R2</span>
                  <div className="flex-1 bg-slate-700/50 rounded-full h-2">
                    <div
                      className="bg-orange-500 h-2 rounded-full transition-all"
                      style={{ width: `${((safeNum(controllerState.axes[5], -1) + 1) / 2) * 100}%` }}
                    />
                  </div>
                  <span className="text-xs font-mono w-12 text-right">{fmt(controllerState.axes[5], 2)}</span>
                </div>
              </div>
            </div>
          </div>

          {/* Joint Commands */}
          <div className="bg-white/10 backdrop-blur-lg rounded-xl p-6 border border-white/20">
            <div className="flex items-center gap-2 mb-4">
              <Settings className="w-6 h-6 text-yellow-400" />
              <h2 className="text-xl font-semibold">Joint Commands</h2>
            </div>

            <div className="grid grid-cols-2 gap-4">
              <div className="bg-slate-800/50 rounded-lg p-3">
                <h3 className="text-sm font-semibold text-purple-300 mb-2">Front Joint</h3>
                <div className="space-y-2">
                  <div className={`flex items-center justify-between text-xs ${jointCmd.front_up ? 'text-green-400' : 'text-gray-500'}`}>
                    <span>Up</span>
                    <span className="font-mono">{jointCmd.front_up}</span>
                  </div>
                  <div className={`flex items-center justify-between text-xs ${jointCmd.front_down ? 'text-green-400' : 'text-gray-500'}`}>
                    <span>Down</span>
                    <span className="font-mono">{jointCmd.front_down}</span>
                  </div>
                </div>
              </div>

              <div className="bg-slate-800/50 rounded-lg p-3">
                <h3 className="text-sm font-semibold text-purple-300 mb-2">Back Joint</h3>
                <div className="space-y-2">
                  <div className={`flex items-center justify-between text-xs ${jointCmd.back_up ? 'text-green-400' : 'text-gray-500'}`}>
                    <span>Up</span>
                    <span className="font-mono">{jointCmd.back_up}</span>
                  </div>
                  <div className={`flex items-center justify-between text-xs ${jointCmd.back_down ? 'text-green-400' : 'text-gray-500'}`}>
                    <span>Down</span>
                    <span className="font-mono">{jointCmd.back_down}</span>
                  </div>
                </div>
              </div>
            </div>
          </div>

          {/* Velocity Command Display */}
          <div className="bg-white/10 backdrop-blur-lg rounded-xl p-6 border border-white/20">
            <div className="flex items-center gap-2 mb-4">
              <Zap className="w-6 h-6 text-yellow-400" />
              <h2 className="text-xl font-semibold">Velocity (cmd_vel)</h2>
            </div>

            <div className="grid grid-cols-2 gap-4">
              <div>
                <div className="text-sm text-gray-400 mb-2">Linear X</div>
                <div className="text-3xl font-bold font-mono text-cyan-400">{fmt(cmdVel.linear.x, 2)}</div>
                <div className="mt-2 w-full bg-slate-700/50 rounded-full h-3">
                  <div
                    className="bg-cyan-500 h-3 rounded-full transition-all"
                    style={{ 
                      width: `${Math.abs(safeNum(cmdVel.linear.x)) * 100}%`,
                      marginLeft: safeNum(cmdVel.linear.x) < 0 ? `${100 - Math.abs(safeNum(cmdVel.linear.x)) * 100}%` : '0'
                    }}
                  />
                </div>
              </div>

              <div>
                <div className="text-sm text-gray-400 mb-2">Angular Z</div>
                <div className="text-3xl font-bold font-mono text-purple-400">{fmt(cmdVel.angular.z, 2)}</div>
                <div className="mt-2 w-full bg-slate-700/50 rounded-full h-3">
                  <div
                    className="bg-purple-500 h-3 rounded-full transition-all"
                    style={{ 
                      width: `${Math.abs(safeNum(cmdVel.angular.z)) * 100}%`,
                      marginLeft: safeNum(cmdVel.angular.z) < 0 ? `${100 - Math.abs(safeNum(cmdVel.angular.z)) * 100}%` : '0'
                    }}
                  />
                </div>
              </div>
            </div>
          </div>
        </div>

        {/* Setup Instructions */}
        <div className="mt-6 bg-purple-900/30 backdrop-blur-lg rounded-xl p-6 border border-purple-500/30">
          <h3 className="text-lg font-semibold mb-3 text-purple-300">Quick Tips</h3>
          <div className="text-sm text-red-300">
            <p>• Open browser console (F12) to see debug messages</p>
            <p>• Check that rosbridge is running: <code className="bg-slate-700 px-2 py-1 rounded">ros2 run rosbridge_server rosbridge_websocket</code></p>
            <p>• Verify topics: <code className="bg-slate-700 px-2 py-1 rounded">ros2 topic list</code></p>
          </div>
        </div>
      </div>
    </div>
  );
}
