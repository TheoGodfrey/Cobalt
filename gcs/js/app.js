/**
 * app.js
 * Main GCS application logic.
 * Ties together WebSocket, UI, Map, and Video.
 */

import { connectWebSocket, sendCommand } from './websocket.js';
// --- FIX 3.3: Import updateDroneMarker, removeDroneMarker is no longer needed here ---
import { initMap, updateDroneMarker } from './map.js';
// --- End of FIX 3.3 ---
import { showVideoFeed } from './video.js';

// --- DOM Elements ---
const log = document.getElementById('log');
const statusIndicator = document.getElementById('statusIndicator');
const fleetList = document.getElementById('fleetList');
const startMissionBtn = document.getElementById('startMissionBtn');
const abortAllBtn = document.getElementById('abortAllBtn');
const missionFileInput = document.getElementById('missionFile');


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

    // --- FIX 3.3: Simplified message handling ---
    let droneId = msg.payload?.drone_id;
    if (!droneId) {
        // Handle non-drone messages like PONG or ERROR
        return;
    }

    // Ensure drone exists in fleet state
    if (!fleet[droneId]) {
        fleet[droneId] = { state: 'UNKNOWN' }; // Create new entry
    }

    let drone = fleet[droneId];

    // Update state based on message type
    switch (msg.type) {
        case 'lwt':
            drone.state = msg.payload.status ? msg.payload.status.toUpperCase() : 'UNKNOWN';
            drone.lwt_timestamp = Date.now();
            break;
        case 'telemetry':
            drone.battery = msg.payload.battery_percent;
            drone.position = `(${msg.payload.latitude.toFixed(4)}, ${msg.payload.longitude.toFixed(4)}) @ ${msg.payload.relative_altitude.toFixed(0)}m`;
            // Update map
            updateDroneMarker(droneId, msg.payload.latitude, msg.payload.longitude, msg.payload.yaw || 0, drone.state);
            break;
        case 'detection':
            drone.last_detection = `Found ${msg.payload.class_label} (${msg.payload.confidence.toFixed(2)})`;
            break;
        case 'phase_update':
            drone.state = msg.payload.phase_name ? `${msg.payload.phase_name} (${msg.payload.mission_state})` : 'UNKNOWN';
            break;
    }
    
    drone.last_update = Date.now();
    renderFleetList();
    // --- End of FIX 3.3 ---
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

// --- FIX 3.3: This function is now simplified ---
// It just passes data to renderFleetList and updateDroneMarker
function updateFleetStatus(droneId, data) {
    if (!fleet[droneId]) {
        fleet[droneId] = {}; // Create new entry if not exists
    }

    // Update state
    if (data.status) { // LWT message
        fleet[droneId].state = data.status.toUpperCase();
        fleet[droneId].lwt_timestamp = Date.now();
        // --- FIX 3.3: REMOVED removeDroneMarker(droneId); ---
    }
    if (data.mission_state) { // From phase update
        fleet[droneId].state = `${data.phase_name} (${data.mission_state})`;
    }
    if (data.battery_percent) { // Telemetry
        fleet[droneId].battery = data.battery_percent;
        fleet[droneId].position = `(${data.latitude.toFixed(4)}, ${data.longitude.toFixed(4)}) @ ${data.relative_altitude.toFixed(0)}m`;
        // --- FIX 3.3: Pass the drone's state to the map marker ---
        updateDroneMarker(droneId, data.latitude, data.longitude, data.yaw || 0, fleet[droneId].state);
    }
    if (data.last_detection) {
        fleet[droneId].last_detection = `Found ${data.last_detection.class_label} (${data.last_detection.confidence.toFixed(2)})`;
    }
    
    fleet[droneId].last_update = Date.now();
    renderFleetList();
}
// --- End of FIX 3.3 ---

// --- FIX 3.3: renderFleetList now renders ALL drones ---
function renderFleetList() {
    if (!fleetList) return;
    fleetList.innerHTML = ''; // Clear list
    
    // --- FIX 3.3: Get all drones, not just connected ones ---
    const allDrones = Object.keys(fleet);
    // --- End of FIX 3.3 ---

    if (allDrones.length === 0) {
         fleetList.innerHTML = '<p class="text-gray-500">No drones connected...</p>';
         return;
    }

    // --- FIX 3.3: Loop over allDrones ---
    for (const droneId of allDrones) {
        const drone = fleet[droneId];
        // --- FIX 3.3: Check state for styling ---
        const isOffline = drone.state === 'OFFLINE';
        const stateColor = isOffline ? 'text-gray-500' : (drone.state === 'ONLINE' ? 'text-green-400' : 'text-yellow-400');
        const cardOpacity = isOffline ? 'opacity-50' : '';
        // --- End of FIX 3.3 ---
        
        const html = `
            <div class="bg-gray-700 p-4 rounded-lg ${cardOpacity}">
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
    // --- End of FIX 3.3 ---
    
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
    const missionId = missionFileInput.value; // Get value from input
    
    if (!missionId) {
        logMessage("GCS: Error - Mission File Name is empty.");
        return; // Don't send command if input is empty
    }
    
    const command = {
        action: "START_MISSION",
        mission_id: missionId // Use the value
    };
    sendCommand(command);
    logMessage(`GCS: Sent START_MISSION (${missionId}) command.`);
}

function onAbortAll() {
    const command = {
        action: "ABORT_ALL",
        failsafe_action: "EXECUTE_RTH"
    };
    sendCommand(command);
    logMessage("GCS: Sent ABORT_ALL (RTH) command.");
}

