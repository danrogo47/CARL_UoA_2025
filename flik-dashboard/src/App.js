import React, { useState, useEffect, useRef } from 'react';
import { Radio, Activity, Power, AlertCircle, Zap, Settings, Gauge } from 'lucide-react';
import whegImg from './assets/wheg.png';

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
    error_status: [],
    left_reverse: 0,
    right_reverse: 0
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
  const [speedMode, setSpeedMode] = useState(1.00);
  const [jointCmd, setJointCmd] = useState({
    front_up: 0,
    front_down: 0,
    back_up: 0,
    back_down: 0
  });
  
  // Rotation state for wheels
  const [wheelRotations, setWheelRotations] = useState({});

  const rosRef = useRef(null);
  const [raspberryPiIP, setRaspberryPiIP] = useState('172.20.10.2');
  const [showSettings, setShowSettings] = useState(false);
  const rotationRef = useRef({});

  // ---- helpers ----
  const safeNum = (v, fallback = 0) => {
    if (typeof v === 'number' && isFinite(v)) return v;
    const n = Number(v);
    return isFinite(n) ? n : fallback;
  };

  const fmt = (v, decimals = 1) => {
    const n = safeNum(v, null);
    return n === null ? '--' : n.toFixed(decimals);
  };

  // ---- rotation effect ----
  useEffect(() => {
    const animationFrameId = requestAnimationFrame(() => {
      setWheelRotations({ ...rotationRef.current });
    });

    const interval = setInterval(() => {
      // Update rotations based on velocities
      whegFeedback.motor_id?.forEach((motorId, idx) => {
        const velocity = whegFeedback.velocity_rpm?.[idx] ?? 0;
        let direction = -1;
        // Reverse direction for right side wheels (motors 4, 5, 6) if right_reverse is set
        if ([4, 5, 6].includes(motorId) && whegFeedback.right_reverse) {
          direction = -direction;
        }
        // Reverse direction for left side wheels (motors 1, 2, 3) if left_reverse is set
        else if ([1, 2, 3].includes(motorId) && whegFeedback.left_reverse) {
          direction = -direction;
        }
        const speed = Math.abs(velocity) * 0.15; // rotation speed multiplier
        
        if (!rotationRef.current[idx]) {
          rotationRef.current[idx] = 0;
        }
        rotationRef.current[idx] += speed * direction;
        rotationRef.current[idx] %= 360; // Keep within 0-360
      });

      setWheelRotations({ ...rotationRef.current });
    }, 16); // ~60fps

    return () => {
      clearInterval(interval);
      cancelAnimationFrame(animationFrameId);
    };
  }, [whegFeedback]);

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

          if (data.topic === '/controller_state') {
            try {
              const controllerData = typeof data.msg.data === 'string' ? JSON.parse(data.msg.data) : data.msg;
              setControllerState(controllerData);
            } catch (e) {
              console.warn('Failed to parse controller_state payload', e, data.msg);
            }

          } else if (data.topic === '/wheg_feedback') {
            const msg = data.msg || {};

            const motor_id = Array.isArray(msg.motor_id) ? msg.motor_id.map((v) => safeNum(v, null)).filter(v => v !== null) : [];
            const position_degrees = Array.isArray(msg.position_degrees) ? msg.position_degrees.map(v => safeNum(v, 0)) : [];
            const velocity_rpm = Array.isArray(msg.velocity_rpm) ? msg.velocity_rpm.map(v => safeNum(v, 0)) : [];
            const load_percentage = Array.isArray(msg.load_percentage) ? msg.load_percentage.map(v => safeNum(v, 0)) : [];
            const error_status = Array.isArray(msg.error_status) ? msg.error_status.map(v => parseInt(v, 10) || 0) : [];
            const left_reverse = safeNum(msg.left_reverse, 0);
            const right_reverse = safeNum(msg.right_reverse, 0);

            const length = motor_id.length || Math.max(position_degrees.length, velocity_rpm.length, load_percentage.length, error_status.length);

            const normalized = {
              motor_id: motor_id.length ? motor_id : Array.from({ length }, (_, i) => i + 1),
              position_degrees: position_degrees.length ? position_degrees.slice(0, length) : Array(length).fill(0),
              velocity_rpm: velocity_rpm.length ? velocity_rpm.slice(0, length) : Array(length).fill(0),
              load_percentage: load_percentage.length ? load_percentage.slice(0, length) : Array(length).fill(0),
              error_status: error_status.length ? error_status.slice(0, length) : Array(length).fill(0),
              left_reverse,
              right_reverse
            };

            setWhegFeedback(normalized);

          } else if (data.topic === '/cmd_vel') {
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

  const motorNames = { 1: 'BL', 2: 'ML', 3: 'FL', 4: 'BR', 5: 'MR', 6: 'FR' };

  const WhegVisualization = ({ indices, isTop = true }) => {
    return (
      <div className="flex justify-center gap-12">
        {indices.map((idx) => {
          const motorId = whegFeedback.motor_id?.[idx];
          const rotation = wheelRotations[idx] ?? 0;

          // Determine body based on motor ID
          let bodyNum = 0;
          if ([1, 4].includes(motorId)) bodyNum = 1; // front
          else if ([2, 5].includes(motorId)) bodyNum = 2; // middle
          else if ([3, 6].includes(motorId)) bodyNum = 3; // back

          // Gait highlight logic
          const isGait2 = gaitSelection.gait_number === 2;
          const isGait3 = gaitSelection.gait_number === 3;

          let shouldHighlight = false;
          if (isGait2) shouldHighlight = gaitSelection.body_number === bodyNum;
          else if (isGait3) shouldHighlight = gaitSelection.wheg_number === motorId;

          return (
            <div key={`wheg-${isTop ? 'top' : 'bottom'}-${idx}`} className="relative w-24 h-24">
              <div
                className={`w-24 h-24 rounded-full transition-all ${
                  shouldHighlight
                    ? 'ring-4 ring-yellow-400 shadow-lg shadow-yellow-400/50'
                    : ''
                }`}
              >
                <img
                  src={whegImg}
                  alt={`Wheg`}
                  className="w-24 h-24 opacity-90"
                  style={{
                    transform: `rotate(${rotation}deg)`,
                  }}
                />
              </div>
              <div
                className={`absolute -bottom-4 w-full text-center text-xs font-mono font-semibold ${
                  shouldHighlight ? 'text-yellow-300' : 'text-blue-300'
                }`}
              >
                {motorNames[motorId] || idx + 1}
              </div>
            </div>
          );
        })}
      </div>
    );
  };

  return (
    <div className="min-h-screen bg-black text-blue-100 p-3">
      <div className="max-w-[1600px] mx-auto">
        {/* Compact Header */}
        <div className="flex items-center justify-between mb-3 bg-blue-950/30 rounded-lg px-4 py-2 border border-blue-900">
          <div className="flex items-center gap-2">
            <Activity className="w-5 h-5 text-blue-400" />
            <h1 className="text-xl font-bold">FLIK Dashboard</h1>
          </div>
          <div className="flex items-center gap-2">
            {connected && (
              <div className="flex items-center gap-2 text-xs text-green-400">
                <Radio className="w-3 h-3 animate-pulse" />
                Connected
              </div>
            )}
            <button onClick={() => setShowSettings(!showSettings)} className="p-1.5 rounded bg-blue-900 hover:bg-blue-800">
              <Settings className="w-4 h-4" />
            </button>
            <button onClick={connectToROS} className={`px-3 py-1.5 rounded text-sm font-semibold ${connected ? 'bg-red-600 hover:bg-red-700' : 'bg-green-600 hover:bg-green-700'}`}>
              {connected ? 'Disconnect' : 'Connect'}
            </button>
          </div>
        </div>

        {showSettings && (
          <div className="bg-blue-950/30 rounded-lg p-3 mb-3 border border-blue-900">
            <div className="flex items-center gap-3 text-sm">
              <label>Raspberry Pi IP:</label>
              <input type="text" value={raspberryPiIP} onChange={(e) => setRaspberryPiIP(e.target.value)}
                className="bg-black text-blue-100 px-3 py-1 rounded border border-blue-800 focus:border-blue-500 focus:outline-none" />
              <span className="text-blue-400">Port: 9090</span>
            </div>
          </div>
        )}

        {connectionError && (
          <div className="bg-red-900/20 border border-red-700 rounded-lg p-2 mb-3 flex items-center gap-2 text-sm">
            <AlertCircle className="w-4 h-4 text-red-400" />
            <p>{connectionError}</p>
          </div>
        )}

        {/* Status Bar */}
        <div className="grid grid-cols-2 gap-2 mb-3">
          <div className="bg-blue-950/30 rounded-lg p-2 border border-blue-900">
            <div className="text-xs text-blue-400">Gait</div>
            <div className="text-lg font-bold text-blue-300">G{gaitSelection.gait_number}</div>
          </div>
          <div className="bg-blue-950/30 rounded-lg p-2 border border-blue-900">
            <div className="text-xs text-blue-400">Speed</div>
            <div className="text-lg font-bold text-blue-300">{fmt(speedMode, 2)}x</div>
          </div>
        </div>

        <div className="grid grid-cols-3 gap-3">
          {/* Motor Data Table */}
          <div className="col-span-2 bg-blue-950/30 rounded-lg p-3 border border-blue-900">
            <div className="flex items-center gap-2 mb-2">
              <Power className="w-4 h-4 text-blue-400" />
              <h2 className="text-sm font-semibold">Motor Feedback</h2>
            </div>
            {whegFeedback.motor_id && whegFeedback.motor_id.length > 0 ? (
              <div className="overflow-x-auto">
                <table className="w-full text-xs">
                  <thead>
                    <tr className="border-b border-blue-900 text-blue-400">
                      <th className="text-left py-1 px-2">ID</th>
                      <th className="text-right py-1 px-2">Vel (RPM)</th>
                      <th className="text-right py-1 px-2">Load (%)</th>
                    </tr>
                  </thead>
                  <tbody>
                    {whegFeedback.motor_id.map((motorId, idx) => {
                      const vel = whegFeedback.velocity_rpm?.[idx] ?? 0;
                      const load = whegFeedback.load_percentage?.[idx] ?? 0;
                      return (
                        <tr key={`motor-${motorId}-${idx}`} className="border-b border-blue-900/50 hover:bg-blue-900/20">
                          <td className="py-1.5 px-2 font-mono font-semibold text-blue-300">{motorNames[motorId] || motorId}</td>
                          <td className="text-right py-1.5 px-2">
                            <div className="flex items-center justify-end gap-2">
                              <div className="w-20 bg-black rounded-full h-1.5">
                                <div className="bg-cyan-500 h-1.5 rounded-full transition-all" style={{ width: `${Math.min(100, (Math.abs(vel) / 80) * 100)}%` }} />
                              </div>
                              <span className="font-mono w-12">{fmt(vel, 1)}</span>
                            </div>
                          </td>
                          <td className="text-right py-1.5 px-2">
                            <div className="flex items-center justify-end gap-2">
                              <div className="w-20 bg-black rounded-full h-1.5">
                                <div className="bg-orange-500 h-1.5 rounded-full transition-all" style={{ width: `${Math.min(100, Math.abs(load))}%` }} />
                              </div>
                              <span className="font-mono w-12">{fmt(load, 1)}</span>
                            </div>
                          </td>
                        </tr>
                      );
                    })}
                  </tbody>
                </table>
              </div>
            ) : (
              <div className="text-center text-blue-500 py-4 text-xs">No motor data</div>
            )}

            {/* Robot Visualisation */}
            <div className="mt-4 pt-4 border-t border-blue-900">
              <h3 className="text-sm font-semibold mb-3 text-blue-400">Wheg Positions</h3>
              <div className="flex flex-col items-center gap-8">
                <WhegVisualization indices={[0, 1, 2]} isTop={true} />
                <WhegVisualization indices={[3, 4, 5]} isTop={false} />
              </div>
            </div>
          </div>

          {/* Controller & Commands */}
          <div className="space-y-3">
            {/* Joint Commands */}
            <div className="bg-blue-950/30 rounded-lg p-3 border border-blue-900">
              <div className="flex items-center gap-2 mb-2">
                <Gauge className="w-4 h-4 text-blue-400" />
                <h2 className="text-sm font-semibold">Joint Cmd</h2>
              </div>
              <div className="grid grid-cols-2 gap-2 text-xs">
                <div>
                  <div className="text-blue-400 mb-1">Front</div>
                  <div className={`flex justify-between ${jointCmd.front_up ? 'text-green-400' : 'text-blue-700'}`}>
                    <span>↑</span><span className="font-mono">{jointCmd.front_up}</span>
                  </div>
                  <div className={`flex justify-between ${jointCmd.front_down ? 'text-green-400' : 'text-blue-700'}`}>
                    <span>↓</span><span className="font-mono">{jointCmd.front_down}</span>
                  </div>
                </div>
                <div>
                  <div className="text-blue-400 mb-1">Back</div>
                  <div className={`flex justify-between ${jointCmd.back_up ? 'text-green-400' : 'text-blue-700'}`}>
                    <span>↑</span><span className="font-mono">{jointCmd.back_up}</span>
                  </div>
                  <div className={`flex justify-between ${jointCmd.back_down ? 'text-green-400' : 'text-blue-700'}`}>
                    <span>↓</span><span className="font-mono">{jointCmd.back_down}</span>
                  </div>
                </div>
              </div>
            </div>

            {/* Velocity */}
            <div className="bg-blue-950/30 rounded-lg p-3 border border-blue-900">
              <div className="flex items-center gap-2 mb-2">
                <Zap className="w-4 h-4 text-blue-400" />
                <h2 className="text-sm font-semibold">Velocity</h2>
              </div>
              <div className="space-y-2 text-xs">
                <div>
                  <div className="flex justify-between mb-1">
                    <span className="text-blue-400">Linear X</span>
                    <span className="font-mono">{fmt(cmdVel.linear.x, 2)}</span>
                  </div>
                  <div className="w-full bg-black rounded-full h-1.5">
                    <div className="bg-blue-500 h-1.5 rounded-full transition-all" style={{ width: `${Math.abs(safeNum(cmdVel.linear.x)) * 100}%` }} />
                  </div>
                </div>
                <div>
                  <div className="flex justify-between mb-1">
                    <span className="text-blue-400">Angular Z</span>
                    <span className="font-mono">{fmt(cmdVel.angular.z, 2)}</span>
                  </div>
                  <div className="w-full bg-black rounded-full h-1.5">
                    <div className="bg-cyan-500 h-1.5 rounded-full transition-all" style={{ width: `${Math.abs(safeNum(cmdVel.angular.z)) * 100}%` }} />
                  </div>
                </div>
              </div>
            </div>

            {/* PS4 Controller */}
            <div className="bg-blue-950/30 rounded-lg p-3 border border-blue-900">
              <h2 className="text-sm font-semibold mb-2">Controller</h2>
              <div className="w-full max-w-[280px] mx-auto">
                <svg viewBox="0 0 441 383" width="100%" height="auto" fill="none">
                  <g>
                    {/* Controller outlines */}
                    <path d="M220.5 294.5C220.5 294.5 195 294.5 150 294.5C105 294.5 81.5 378.5 49.5 378.5C17.5 378.5 4 363.9 4 317.5C4 271.1 43.5 165.5 55 137.5C66.5 109.5 95.5 92.0001 128 92.0001C154 92.0001 200.5 92.0001 220.5 92.0001" stroke="#60a5fa" strokeWidth="2" />
                    <path d="M220 294.5C220 294.5 245.5 294.5 290.5 294.5C335.5 294.5 359 378.5 391 378.5C423 378.5 436.5 363.9 436.5 317.5C436.5 271.1 397 165.5 385.5 137.5C374 109.5 345 92.0001 312.5 92.0001C286.5 92.0001 240 92.0001 220 92.0001" stroke="#60a5fa" strokeWidth="2" />
                    <circle cx="113" cy="160" r="37.5" stroke="#60a5fa" strokeWidth="2" />
                    <circle cx="278" cy="238" r="37.5" stroke="#60a5fa" strokeWidth="2" />
                    <circle cx="166" cy="238" r="37.5" stroke="#60a5fa" strokeWidth="2" />
                    <circle cx="329" cy="160" r="37.5" stroke="#60a5fa" strokeWidth="2" />
                    
                    {/* D-pad */}
                    <g transform="translate(93, 140)">
                      <rect x="15" y="0" width="10" height="10" rx="2" fill={controllerState.buttons[11] ? "#f97316" : "#1e40af"} stroke="#60a5fa" strokeWidth="1" className="transition-all" />
                      <rect x="15" y="30" width="10" height="10" rx="2" fill={controllerState.buttons[12] ? "#f97316" : "#1e40af"} stroke="#60a5fa" strokeWidth="1" className="transition-all" />
                      <rect x="0" y="15" width="10" height="10" rx="2" fill={controllerState.buttons[13] ? "#f97316" : "#1e40af"} stroke="#60a5fa" strokeWidth="1" className="transition-all" />
                      <rect x="30" y="15" width="10" height="10" rx="2" fill={controllerState.buttons[14] ? "#f97316" : "#1e40af"} stroke="#60a5fa" strokeWidth="1" className="transition-all" />
                    </g>
                    
                    {/* Left Stick */}
                    <circle 
                      cx={166 + safeNum(controllerState.axes[0], 0) * 15} 
                      cy={238 + safeNum(controllerState.axes[1], 0) * 15} 
                      r="20" 
                      fill={controllerState.buttons[7] ? "#f97316" : "#1e40af"} 
                      stroke="#60a5fa" 
                      strokeWidth="2"
                      className="transition-all"
                    />
                    
                    {/* Right Stick */}
                    <circle 
                      cx={278 + safeNum(controllerState.axes[2], 0) * 15} 
                      cy={238 + safeNum(controllerState.axes[3], 0) * 15} 
                      r="20" 
                      fill={controllerState.buttons[8] ? "#f97316" : "#1e40af"} 
                      stroke="#60a5fa" 
                      strokeWidth="2"
                      className="transition-all"
                    />
                    
                    {/* Action buttons */}
                    <g transform="translate(309, 140)">
                      <circle cx="20" cy="0" r="9" fill={controllerState.buttons[3] ? "#f97316" : "#1e40af"} stroke="#60a5fa" strokeWidth="1.5" className="transition-all" />
                      <circle cx="40" cy="20" r="9" fill={controllerState.buttons[1] ? "#f97316" : "#1e40af"} stroke="#60a5fa" strokeWidth="1.5" className="transition-all" />
                      <circle cx="20" cy="40" r="9" fill={controllerState.buttons[0] ? "#f97316" : "#1e40af"} stroke="#60a5fa" strokeWidth="1.5" className="transition-all" />
                      <circle cx="0" cy="20" r="9" fill={controllerState.buttons[2] ? "#f97316" : "#1e40af"} stroke="#60a5fa" strokeWidth="1.5" className="transition-all" />
                    </g>
                    
                    {/* Share/Options/PS */}
                    <circle cx="160" cy="119" r="6" fill={controllerState.buttons[4] ? "#f97316" : "#1e40af"} stroke="#60a5fa" strokeWidth="1.5" className="transition-all" />
                    <circle cx="284" cy="119" r="6" fill={controllerState.buttons[6] ? "#f97316" : "#1e40af"} stroke="#60a5fa" strokeWidth="1.5" className="transition-all" />
                    
                    {/* Touchpad */}
                    <rect x="175" y="129" width="94" height="51" rx="6" fill={controllerState.buttons[15] ? "#f97316" : "#1e40af"} stroke="#60a5fa" strokeWidth="2" className="transition-all" />
                    
                    {/* L1/R1 */}
                    <rect x="111" y="61" width="41" height="13" rx="6" fill={controllerState.buttons[9] ? "#f97316" : "#1e40af"} stroke="#60a5fa" strokeWidth="2" className="transition-all" />
                    <rect x="289" y="61" width="41" height="13" rx="6" fill={controllerState.buttons[10] ? "#f97316" : "#1e40af"} stroke="#60a5fa" strokeWidth="2" className="transition-all" />
                    
                    {/* L2/R2 */}
                    <path d="M152.5 37C152.5 41.1421 149.142 44.5 145 44.5H132C127.858 44.5 124.5 41.1421 124.5 37V16.5C124.5 8.76801 130.768 2.5 138.5 2.5C146.232 2.5 152.5 8.76801 152.5 16.5V37Z" 
                      fill={((safeNum(controllerState.axes[4], -1) + 1) / 2) > 0.5 ? "#f97316" : "#1e40af"} 
                      stroke="#60a5fa" 
                      strokeWidth="2"
                      className="transition-all" />
                    <path d="M317.5 37C317.5 41.1421 314.142 44.5 310 44.5H297C292.858 44.5 289.5 41.1421 289.5 37V16.5C289.5 8.76801 295.768 2.5 303.5 2.5C311.232 2.5 317.5 8.76801 317.5 16.5V37Z" 
                      fill={((safeNum(controllerState.axes[5], -1) + 1) / 2) > 0.5 ? "#f97316" : "#1e40af"} 
                      stroke="#60a5fa" 
                      strokeWidth="2"
                      className="transition-all" />
                  </g>
                </svg>
              </div>
              <div className="mt-2 grid grid-cols-2 gap-2 text-xs">
                <div>
                  <div className="text-blue-400 mb-1">L2: {fmt(controllerState.axes[4], 2)}</div>
                  <div className="w-full bg-black rounded-full h-1.5">
                    <div className="bg-orange-500 h-1.5 rounded-full transition-all" style={{ width: `${((safeNum(controllerState.axes[4], -1) + 1) / 2) * 100}%` }} />
                  </div>
                </div>
                <div>
                  <div className="text-blue-400 mb-1">R2: {fmt(controllerState.axes[5], 2)}</div>
                  <div className="w-full bg-black rounded-full h-1.5">
                    <div className="bg-orange-500 h-1.5 rounded-full transition-all" style={{ width: `${((safeNum(controllerState.axes[5], -1) + 1) / 2) * 100}%` }} />
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}