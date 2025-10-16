import React, { useState, useRef } from 'react';
import { Gamepad2, Radio, Activity, Power, AlertCircle, Zap } from 'lucide-react';

export default function CarlROS2Dashboard() {
  const [connected, setConnected] = useState(false);
  const [connectionError, setConnectionError] = useState('');
  const [controllerState, setControllerState] = useState({
    axes: Array(6).fill(0),
    buttons: Array(16).fill(0)
  });
  const [whegFeedback, setWhegFeedback] = useState({
    front_left: 0,
    middle_left: 0,
    back_left: 0,
    front_right: 0,
    middle_right: 0,
    back_right: 0
  });
  const [cmdVel, setCmdVel] = useState({
    linear: { x: 0, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: 0 }
  });
  const [gaitSelection, setGaitSelection] = useState(0);
  const [speedMode, setSpeedMode] = useState(0.25);
  
  const rosRef = useRef(null);
  const [raspberryPiIP, setRaspberryPiIP] = useState('10.13.121.179');
  const [showSettings, setShowSettings] = useState(false);

  const connectToROS = () => {
    if (connected) {
      // Disconnect
      if (rosRef.current) {
        rosRef.current.close();
        rosRef.current = null;
      }
      setConnected(false);
      setConnectionError('');
      return;
    }

    // Connect using native WebSocket
    try {
      const ws = new WebSocket(`ws://${raspberryPiIP}:9090`);
      
      ws.onopen = () => {
        console.log('Connected to ROS bridge');
        setConnected(true);
        setConnectionError('');
        
        // Subscribe to controller_state topic
        ws.send(JSON.stringify({
          op: 'subscribe',
          topic: '/controller_state',
          type: 'std_msgs/String'
        }));
        
        // Subscribe to wheg_feedback topic
        ws.send(JSON.stringify({
          op: 'subscribe',
          topic: '/wheg_feedback',
          type: 'custom_msgs/WhegFeedback'
        }));
        
        // Subscribe to cmd_vel topic
        ws.send(JSON.stringify({
          op: 'subscribe',
          topic: '/cmd_vel',
          type: 'geometry_msgs/Twist'
        }));
        
        // Subscribe to gait_selection topic
        ws.send(JSON.stringify({
          op: 'subscribe',
          topic: '/gait_selection',
          type: 'std_msgs/Int16'
        }));
        
        // Subscribe to speed_mode topic
        ws.send(JSON.stringify({
          op: 'subscribe',
          topic: '/speed_mode',
          type: 'std_msgs/Float32'
        }));
      };
      
      ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          
          if (data.topic === '/controller_state') {
            const controllerData = JSON.parse(data.msg.data);
            setControllerState(controllerData);
          } else if (data.topic === '/wheg_feedback') {
            setWhegFeedback(data.msg);
          } else if (data.topic === '/cmd_vel') {
            setCmdVel(data.msg);
          } else if (data.topic === '/gait_selection') {
            setGaitSelection(data.msg.data);
          } else if (data.topic === '/speed_mode') {
            setSpeedMode(data.msg.data);
          }
        } catch (e) {
          console.error('Error parsing ROS message:', e);
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

  const buttonNames = [
    'Cross', 'Circle', 'Square', 'Triangle', 'Share', 'PS', 'Options',
    'L-Joy', 'R-Joy', 'L1', 'R1', 'Up', 'Down', 'Left', 'Right', 'Touchpad'
  ];

  const gaitNames = ['Gait 1', 'Gait 2', 'Gait 3', 'Gait 4'];

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-900 via-purple-900 to-slate-900 text-white p-6">
      <div className="max-w-7xl mx-auto">
        {/* Header */}
        <div className="flex items-center justify-between mb-8">
          <div className="flex items-center gap-3">
            <Activity className="w-8 h-8 text-purple-400" />
            <h1 className="text-3xl font-bold">FLIK Robot Dashboard</h1>
          </div>
          <div className="flex items-center gap-4">
            <button
              onClick={() => setShowSettings(!showSettings)}
              className="px-4 py-2 rounded-lg bg-slate-700 hover:bg-slate-600 transition-all"
            >
              Settings
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

        <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
          {/* Wheg Motor Status */}
          <div className="bg-white/10 backdrop-blur-lg rounded-xl p-6 border border-white/20">
            <div className="flex items-center gap-2 mb-4">
              <Power className="w-6 h-6 text-cyan-400" />
              <h2 className="text-xl font-semibold">Wheg Motors</h2>
            </div>

            <div className="space-y-4">
              {Object.entries(whegFeedback).map(([key, value]) => (
                <div key={key}>
                  <div className="flex justify-between text-sm mb-1">
                    <span className="text-gray-300 capitalize">
                      {key.replace('_', ' ')}
                    </span>
                    <span className="font-mono text-cyan-300">
                      {value.toFixed(1)}%
                    </span>
                  </div>
                  <div className="w-full bg-slate-700/50 rounded-full h-2">
                    <div
                      className="bg-cyan-500 h-2 rounded-full transition-all"
                      style={{ width: `${Math.min(100, Math.abs(value))}%` }}
                    />
                  </div>
                </div>
              ))}
            </div>

            {/* Gait and Speed Info */}
            <div className="mt-6 pt-6 border-t border-white/10">
              <div className="grid grid-cols-2 gap-4">
                <div>
                  <div className="text-sm text-gray-400 mb-1">Current Gait</div>
                  <div className="text-2xl font-bold text-purple-400">
                    {gaitNames[gaitSelection] || 'N/A'}
                  </div>
                </div>
                <div>
                  <div className="text-sm text-gray-400 mb-1">Speed Mode</div>
                  <div className="text-2xl font-bold text-green-400">
                    {(speedMode * 100).toFixed(0)}%
                  </div>
                </div>
              </div>
            </div>
          </div>

          {/* PS4 Controller Status */}
          <div className="bg-white/10 backdrop-blur-lg rounded-xl p-6 border border-white/20">
            <div className="flex items-center gap-2 mb-4">
              <Gamepad2 className="w-6 h-6 text-purple-400" />
              <h2 className="text-xl font-semibold">PS4 Controller</h2>
            </div>

            {/* Joysticks */}
            <div className="grid grid-cols-2 gap-6 mb-6">
              <div>
                <h3 className="text-sm font-medium text-gray-300 mb-3">Left Stick</h3>
                <div className="relative w-32 h-32 mx-auto bg-slate-700/50 rounded-full border-2 border-purple-500/30">
                  <div
                    className="absolute w-6 h-6 bg-purple-500 rounded-full shadow-lg transition-all"
                    style={{
                      left: `${(controllerState.axes[0] + 1) * 50}%`,
                      top: `${(controllerState.axes[1] + 1) * 50}%`,
                      transform: 'translate(-50%, -50%)',
                    }}
                  />
                  <div className="absolute inset-0 flex items-center justify-center">
                    <div className="w-1 h-1 bg-white/30 rounded-full" />
                  </div>
                </div>
                <div className="mt-2 text-xs text-center font-mono text-gray-400">
                  X: {controllerState.axes[0].toFixed(2)} Y: {controllerState.axes[1].toFixed(2)}
                </div>
              </div>

              <div>
                <h3 className="text-sm font-medium text-gray-300 mb-3">Right Stick</h3>
                <div className="relative w-32 h-32 mx-auto bg-slate-700/50 rounded-full border-2 border-cyan-500/30">
                  <div
                    className="absolute w-6 h-6 bg-cyan-500 rounded-full shadow-lg transition-all"
                    style={{
                      left: `${(controllerState.axes[2] + 1) * 50}%`,
                      top: `${(controllerState.axes[3] + 1) * 50}%`,
                      transform: 'translate(-50%, -50%)',
                    }}
                  />
                  <div className="absolute inset-0 flex items-center justify-center">
                    <div className="w-1 h-1 bg-white/30 rounded-full" />
                  </div>
                </div>
                <div className="mt-2 text-xs text-center font-mono text-gray-400">
                  X: {controllerState.axes[2].toFixed(2)} Y: {controllerState.axes[3].toFixed(2)}
                </div>
              </div>
            </div>

            {/* Triggers */}
            <div className="mb-6">
              <h3 className="text-sm font-medium text-gray-300 mb-3">Triggers</h3>
              <div className="space-y-2">
                <div>
                  <div className="flex justify-between text-xs mb-1">
                    <span>L2</span>
                    <span className="font-mono">{controllerState.axes[4].toFixed(2)}</span>
                  </div>
                  <div className="w-full bg-slate-700/50 rounded-full h-2">
                    <div
                      className="bg-orange-500 h-2 rounded-full transition-all"
                      style={{ width: `${((controllerState.axes[4] + 1) / 2) * 100}%` }}
                    />
                  </div>
                </div>
                <div>
                  <div className="flex justify-between text-xs mb-1">
                    <span>R2</span>
                    <span className="font-mono">{controllerState.axes[5].toFixed(2)}</span>
                  </div>
                  <div className="w-full bg-slate-700/50 rounded-full h-2">
                    <div
                      className="bg-orange-500 h-2 rounded-full transition-all"
                      style={{ width: `${((controllerState.axes[5] + 1) / 2) * 100}%` }}
                    />
                  </div>
                </div>
              </div>
            </div>

            {/* Buttons */}
            <div>
              <h3 className="text-sm font-medium text-gray-300 mb-3">Active Buttons</h3>
              <div className="flex flex-wrap gap-2">
                {controllerState.buttons.map((pressed, idx) => (
                  pressed === 1 && (
                    <span
                      key={idx}
                      className="px-3 py-1 bg-purple-600 rounded-full text-xs font-medium"
                    >
                      {buttonNames[idx]}
                    </span>
                  )
                ))}
                {controllerState.buttons.every(b => b === 0) && (
                  <span className="text-sm text-gray-500">No buttons pressed</span>
                )}
              </div>
            </div>
          </div>

          {/* Velocity Command Display */}
          <div className="bg-white/10 backdrop-blur-lg rounded-xl p-6 border border-white/20 lg:col-span-2">
            <div className="flex items-center gap-2 mb-4">
              <Zap className="w-6 h-6 text-yellow-400" />
              <h2 className="text-xl font-semibold">Velocity Command (cmd_vel)</h2>
            </div>
            
            <div className="grid grid-cols-2 md:grid-cols-3 gap-6">
              <div>
                <div className="text-sm text-gray-400 mb-2">Linear X</div>
                <div className="text-3xl font-bold font-mono text-cyan-400">
                  {cmdVel.linear.x.toFixed(2)}
                </div>
                <div className="mt-2 w-full bg-slate-700/50 rounded-full h-3">
                  <div
                    className="bg-cyan-500 h-3 rounded-full transition-all"
                    style={{ 
                      width: `${Math.abs(cmdVel.linear.x) * 50}%`,
                      marginLeft: cmdVel.linear.x < 0 ? `${50 - Math.abs(cmdVel.linear.x) * 50}%` : '50%'
                    }}
                  />
                </div>
              </div>
              
              <div>
                <div className="text-sm text-gray-400 mb-2">Linear Y</div>
                <div className="text-3xl font-bold font-mono text-green-400">
                  {cmdVel.linear.y.toFixed(2)}
                </div>
                <div className="mt-2 w-full bg-slate-700/50 rounded-full h-3">
                  <div
                    className="bg-green-500 h-3 rounded-full transition-all"
                    style={{ width: `${Math.abs(cmdVel.linear.y) * 50}%` }}
                  />
                </div>
              </div>
              
              <div>
                <div className="text-sm text-gray-400 mb-2">Angular Z</div>
                <div className="text-3xl font-bold font-mono text-purple-400">
                  {cmdVel.angular.z.toFixed(2)}
                </div>
                <div className="mt-2 w-full bg-slate-700/50 rounded-full h-3">
                  <div
                    className="bg-purple-500 h-3 rounded-full transition-all"
                    style={{ width: `${Math.abs(cmdVel.angular.z) * 50}%` }}
                  />
                </div>
              </div>
            </div>
          </div>
        </div>

        {/* Setup Instructions */}
        <div className="mt-6 bg-purple-900/30 backdrop-blur-lg rounded-xl p-6 border border-purple-500/30">
          <h3 className="text-lg font-semibold mb-3 text-purple-300">Setup Instructions</h3>
          <div className="text-sm text-gray-300 space-y-2">
            <p><strong>1. Install rosbridge on Raspberry Pi:</strong></p>
            <code className="block bg-slate-700 px-4 py-2 rounded mb-2">sudo apt install ros-humble-rosbridge-suite</code>
            
            <p><strong>2. Launch rosbridge on Raspberry Pi:</strong></p>
            <code className="block bg-slate-700 px-4 py-2 rounded mb-2">ros2 run rosbridge_server rosbridge_websocket</code>
            
            <p><strong>3. Make sure your topics are running:</strong></p>
            <code className="block bg-slate-700 px-4 py-2 rounded mb-2">ros2 launch carl_controller ps4_controller_launch.py</code>
            
            <p className="text-green-300 mt-4">
              ✓ This dashboard uses native WebSockets - no additional libraries needed!
            </p>
          </div>
        </div>
      </div>
    </div>
  );
}