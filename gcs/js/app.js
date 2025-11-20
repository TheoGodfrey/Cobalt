/**
 * app.js
 * Main GCS application logic.
 * Ties together WebSocket, UI, Map, and Video.
 */

import { connectWebSocket, sendCommand } from './websocket.js';
// --- FIX: Import removeDroneMarker ---
import { initMap, updateDroneMarker, removeDroneMarker } from './map.js';
import { showVideoFeed } from './video.js';

// --- DOM Elements ---
const log = document.getElementById('log');
const statusIndicator = document.getElementById('statusIndicator');
const fleetList = document.getElementById('fleetList');
const startMissionBtn = document.getElementById('startMissionBtn');
const abortAllBtn = document.getElementById('abortAllBtn');
const missionFileInput = document.getElementById('missionFile');


// --- State ---
// This object will be the single source of truth for fleet state
let fleet = {}; 
// Keep track of markers on the map
let markersOnMap = new Set();

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
    // --- FIX: This is the new message handling logic ---
    if (msg.type === 'FLEET_STATE_UPDATE') {
        handleFleetUpdate(msg.data);
    } else if (msg.type === 'PONG') {
        // Handle pong messages if needed
    } else {
        // Log any other message types
        logMessage(`HUB: ${JSON.stringify(msg)}`);
    }
    // --- End of FIX ---
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

// --- NEW: Main function to process the fleet snapshot ---
function handleFleetUpdate(data) {
    // 1. Rebuild the fleet state from the snapshot
    const newFleet = {};
    const droneIdsInUpdate = new Set();
    
    for (const drone of data.drones) {
        newFleet[drone.drone_id] = drone;
        droneIdsInUpdate.add(drone.drone_id);
    }
    fleet = newFleet;

    // 2. Re-render the entire fleet list UI
    renderFleetList();
    
    // 3. Update all map markers
    for (const drone of Object.values(fleet)) {
        const telem = drone.last_telemetry;
        
        // Check if telemetry and location data exist
        if (telem && telem.latitude != null && telem.longitude != null) {
            updateDroneMarker(
                drone.drone_id, 
                telem.latitude, 
                telem.longitude, 
                telem.yaw || 0, 
                drone.mission_state
            );
            markersOnMap.add(drone.drone_id);
        }
    }
    
    // 4. Remove any map markers for drones that are no longer in the fleet
    for (const droneId of markersOnMap) {
        if (!droneIdsInUpdate.has(droneId)) {
            removeDroneMarker(droneId);
            markersOnMap.delete(droneId);
        }
    }
}
// --- End of NEW ---


function renderFleetList() {
    if (!fleetList) return;
    fleetList.innerHTML = ''; // Clear list
    
    const allDrones = Object.values(fleet);

    if (allDrones.length === 0) {
         fleetList.innerHTML = '<p class="text-gray-500">No drones connected...</p>';
         return;
    }

    for (const drone of allDrones) {
        // --- FIX: Get state and telemetry from the correct snapshot structure ---
        const state = drone.mission_state || 'UNKNOWN';
        const telem = drone.last_telemetry || {};
        
        const isOffline = (state === 'OFFLINE');
        const stateColor = isOffline ? 'text-gray-500' : (state === 'IDLE' ? 'text-green-400' : 'text-yellow-400');
        const cardOpacity = isOffline ? 'opacity-50' : '';
        
        const battery = telem.battery_percent;
        const position = (telem.latitude != null) ?
            `(${telem.latitude.toFixed(4)}, ${telem.longitude.toFixed(4)}) @ ${telem.relative_altitude?.toFixed(0) || '?'}m` :
            'No position data';
        
        // --- End of FIX ---
        
        const html = `
            <div class="bg-gray-700 p-4 rounded-lg ${cardOpacity}">
                <div class="flex justify-between items-center">
                    <span class="text-lg font-bold">${drone.drone_id}</span>
                    <span class="font-mono ${stateColor}">${state}</span>
                </div>
                <div class="text-sm text-gray-300 mt-2 space-y-1">
                    ${battery != null ? `<div><strong>Battery:</strong> ${battery.toFixed(1)}%</div>` : '<div><strong>Battery:</strong> N/A</div>'}
                    <div><strong>Position:</strong> ${position}</div>
                    ${drone.last_detection ? `<div class="text-cyan-300"><strong>Detection:</strong> ${drone.last_detection}</div>` : ''}
                </div>
                <div class="mt-4">
                    <button class="show-video-btn bg-gray-600 hover:bg-gray-500 text-white text-sm py-1 px-3 rounded" data-drone-id="${drone.drone_id}">
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