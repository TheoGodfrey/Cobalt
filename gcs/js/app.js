/**
 * app.js
 * Main GCS application logic.
 * Ties together WebSocket, UI, Map, and Video.
 */

import { connectWebSocket, sendCommand } from './websocket.js';
import { initMap, updateDroneMarker, removeDroneMarker } from './map.js';
import { showVideoFeed } from './video.js';

// --- DOM Elements ---
const log = document.getElementById('log');
const statusIndicator = document.getElementById('statusIndicator');
const fleetList = document.getElementById('fleetList');
const startMissionBtn = document.getElementById('startMissionBtn');
const abortAllBtn = document.getElementById('abortAllBtn');

// --- State ---
const fleet = {}; // Object to store drone states

// --- Initialization ---
document.addEventListener('DOMContentLoaded', () => {
    logMessage('GCS: Initializing...');
    
    // Initialize the map (will use default coords)
    initMap(); 
    
    // Register button listeners
    startMissionBtn.addEventListener('click', onStartMission);
    abortAllBtn.addEventListener('click', onAbortAll);

    // Connect to WebSocket and provide handlers
    connectWebSocket({
        onOpen: onSocketOpen,
        onMessage: onSocketMessage,
        onClose: onSocketClose,
        onError: onSocketError
    });
});

// --- WebSocket Handlers ---
function onSocketOpen() {
    logMessage('GCS: Connected to Hub (WebSocket)');
    statusIndicator.classList.remove('bg-red-500');
    statusIndicator.classList.add('bg-green-500');
    statusIndicator.title = 'Connected';
}

function onSocketMessage(msg) {
    // Log all messages
    logMessage(`HUB: ${JSON.stringify(msg)}`);

    // Update fleet status
    if (msg.type === 'telemetry' || msg.type === 'lwt') {
        updateFleetStatus(msg.payload.drone_id, msg.payload);
    } else if (msg.type === 'detection') {
        updateFleetStatus(msg.payload.drone_id, { 
            last_detection: msg.payload, 
            state: fleet[msg.payload.drone_id]?.state || 'UNKNOWN' 
        });
    } else if (msg.type === 'phase_update') {
         updateFleetStatus(msg.payload.drone_id, { 
            state: msg.payload.phase_name,
            mission_state: msg.payload.mission_state
        });
    }
}

function onSocketClose() {
    logMessage('GCS: Disconnected from Hub. Retrying in 3s...');
    statusIndicator.classList.add('bg-red-500');
    statusIndicator.classList.remove('bg-green-500');
    statusIndicator.title = 'Disconnected';
}

function onSocketError(err) {
    logMessage(`GCS: WebSocket error: ${err.message || 'Unknown error'}`);
}

// --- UI Logic ---
function logMessage(message) {
    if (!log) return;
    log.textContent = `[${new Date().toLocaleTimeString()}] ${message}\n` + log.textContent;
}

function updateFleetStatus(droneId, data) {
    if (!fleet[droneId]) {
        fleet[droneId] = {}; // Create new entry if not exists
    }

    // Update state
    if (data.status) { // LWT message
        fleet[droneId].state = data.status.toUpperCase();
        fleet[droneId].lwt_timestamp = Date.now();
        if(data.status === 'offline') {
            removeDroneMarker(droneId);
        }
    }
    if (data.mission_state) { // From phase update
        fleet[droneId].state = `${data.phase_name} (${data.mission_state})`;
    }
    if (data.battery_percent) { // Telemetry
        fleet[droneId].battery = data.battery_percent;
        fleet[droneId].position = `(${data.latitude.toFixed(4)}, ${data.longitude.toFixed(4)}) @ ${data.relative_altitude.toFixed(0)}m`;
        // Update map
        updateDroneMarker(droneId, data.latitude, data.longitude, data.yaw || 0);
    }
    if (data.last_detection) {
        fleet[droneId].last_detection = `Found ${data.last_detection.class_label} (${data.last_detection.confidence.toFixed(2)})`;
    }
    
    fleet[droneId].last_update = Date.now();
    renderFleetList();
}

function renderFleetList() {
    if (!fleetList) return;
    fleetList.innerHTML = ''; // Clear list
    
    const connectedDrones = Object.keys(fleet).filter(id => fleet[id].state !== 'OFFLINE');

    if (connectedDrones.length === 0) {
         fleetList.innerHTML = '<p class="text-gray-500">No drones connected...</p>';
         return;
    }

    for (const droneId of connectedDrones) {
        const drone = fleet[droneId];
        const stateColor = drone.state === 'ONLINE' ? 'text-green-400' : (drone.state === 'OFFLINE' ? 'text-red-400' : 'text-yellow-400');
        
        const html = `
            <div class="bg-gray-700 p-4 rounded-lg">
                <div class="flex justify-between items-center">
                    <span class="text-lg font-bold">${droneId}</span>
                    <span class="font-mono ${stateColor}">${drone.state || 'UNKNOWN'}</span>
                </div>
                <div class="text-sm text-gray-300 mt-2 space-y-1">
                    ${drone.battery ? `<div><strong>Battery:</strong> ${drone.battery.toFixed(1)}%</div>` : ''}
                    ${drone.position ? `<div><strong>Position:</strong> ${drone.position}</div>` : ''}
                    ${drone.last_detection ? `<div class="text-cyan-300"><strong>Detection:</strong> ${drone.last_detection}</div>` : ''}
                </div>
                <div class="mt-4">
                    <button class="show-video-btn bg-gray-600 hover:bg-gray-500 text-white text-sm py-1 px-3 rounded" data-drone-id="${droneId}">
                        Show Video
                    </button>
                </div>
            </div>
        `;
        fleetList.innerHTML += html;
    }
    
    // Add event listeners to the new video buttons
    document.querySelectorAll('.show-video-btn').forEach(btn => {
        btn.addEventListener('click', (e) => {
            const droneId = e.target.dataset.droneId;
            showVideoFeed(droneId);
        });
    });
}

// --- Command Senders ---
function onStartMission() {
    const command = {
        action: "START_MISSION",
        mission_id: "mob_search_001.json" // Hardcoded for now
    };
    sendCommand(command);
    logMessage("GCS: Sent START_MISSION (mob_search_001.json) command.");
}

function onAbortAll() {
    const command = {
        action: "ABORT_ALL",
        failsafe_action: "EXECUTE_RTH"
    };
    sendCommand(command);
    logMessage("GCS: Sent ABORT_ALL (RTH) command.");
}
