/**
 * websocket.js
 * Manages the WebSocket connection to the Hub.
 */

// Store the WebSocket instance
let ws;

// Store callbacks for message handling
const callbacks = {
    onOpen: null,
    onMessage: null,
    onClose: null,
    onError: null
};

/**
 * Connects to the WebSocket server.
 * @param {object} cbs - Callback functions for ws events.
 * @param {function} cbs.onOpen - Called on connection open.
 * @param {function} cbs.onMessage - Called when a message is received.
 * @param {function} cbs.onClose - Called on connection close.
 * @param {function} cbs.onError - Called on WebSocket error.
 */
export function connectWebSocket(cbs) {
    // Assign callbacks
    callbacks.onOpen = cbs.onOpen;
    callbacks.onMessage = cbs.onMessage;
    callbacks.onClose = cbs.onClose;
    callbacks.onError = cbs.onError;

    // Assumes GCS server is on port 8765
    const wsUrl = 'ws://' + (window.location.hostname || 'localhost') + ':8765';
    ws = new WebSocket(wsUrl);

    ws.onopen = () => {
        if (callbacks.onOpen) callbacks.onOpen();
    };

    ws.onmessage = (event) => {
        try {
            const msg = JSON.parse(event.data);
            if (callbacks.onMessage) callbacks.onMessage(msg);
        } catch (e) {
            console.error("Failed to parse WebSocket message:", event.data, e);
            if (callbacks.onError) callbacks.onError({ message: "Failed to parse JSON message" });
        }
    };

    ws.onclose = () => {
        if (callbacks.onClose) callbacks.onClose();
        // Automatic reconnection attempt
        setTimeout(() => connectWebSocket(cbs), 3000);
    };

    ws.onerror = (err) => {
        if (callbacks.onError) callbacks.onError(err);
        ws.close();
    };
}

/**
 * Sends a command object to the WebSocket server.
 * @param {object} command - The command object to send.
 */
export function sendCommand(command) {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify(command));
    } else {
        console.warn("WebSocket not open. Command not sent:", command);
        if (callbacks.onError) callbacks.onError({ message: "WebSocket not open. Command not sent." });
    }
}
