/**
 * map.js
 * Manages the Leaflet map and drone markers.
 */

let map;
const droneMarkers = {}; // Store markers by drone_id

// Custom drone icon
const droneIcon = L.divIcon({
    html: `
        <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="currentColor" class="w-8 h-8 text-cyan-400">
            <path d="M12.001 2.25c.578 0 1.122.164 1.586.444l4.359 2.5c.465.268.86.621.112.985l-4.359 2.5c-.465.268-.86.621-1.12.985l-4.36-2.5c-.464-.268-.859-.621-1.119-.985l4.36-2.5c.464-.268.859-.621 1.119-.985zM12.001 7.119c.578 0 1.122.164 1.586.444l4.359 2.5c.465.268.86.621.112.985l-4.359 2.5c-.465.268-.86.621-1.12.985l-4.36-2.5c-.464-.268-.859-.621-1.119-.985l4.36-2.5c.464-.268.859-.621 1.119-.985zM12.001 11.984c.578 0 1.122.164 1.586.444l4.359 2.5c.465.268.86.621.112.985l-4.359 2.5c-.465.268-.86.621-1.12.985l-4.36-2.5c-.464-.268-.859-.621-1.119-.985l4.36-2.5c.464-.268.859-.621 1.119-.985zM21.75 12.14c.118-.458-.112-.94-.56-1.115l-4.36-2.5a1.875 1.875 0 00-1.688 0l-4.36 2.5c-.448.175-.678.657-.56 1.115l.397 1.548c.118.458.55.764 1.008.688l3.201-.54c.458-.076.89.23 1.008.688l.397 1.548c.118.458.55.764 1.008.688l3.201-.54c.458-.076.89.23 1.008.688l.397 1.548z" />
        </svg>`,
    className: 'bg-transparent border-0',
    iconSize: [32, 32],
    iconAnchor: [16, 16]
});

/**
 * Initializes the Leaflet map.
 * @param {number} lat - Initial latitude.
 * @param {number} lon - Initial longitude.
 */
export function initMap(lat = 44.56, lon = -123.28) {
    if (map) return; // Already initialized

    map = L.map('map').setView([lat, lon], 13);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
    }).addTo(map);

    L.control.scale().addTo(map);
}

/**
 * Adds or updates a drone marker on the map.
 * @param {string} droneId - The unique ID of the drone.
 * @param {number} lat - Latitude.
 * @param {number} lon - Longitude.
 * @param {number} heading - Heading in degrees (optional).
 */
export function updateDroneMarker(droneId, lat, lon, heading = 0) {
    if (!map) initMap(lat, lon); // Initialize map if not already done

    const latLng = [lat, lon];
    const tooltipContent = `<b>${droneId}</b><br>Lat: ${lat.toFixed(4)}<br>Lon: ${lon.toFixed(4)}`;

    if (droneMarkers[droneId]) {
        // Update existing marker
        droneMarkers[droneId].setLatLng(latLng);
        droneMarkers[droneId].setRotationAngle(heading);
        droneMarkers[droneId].setTooltipContent(tooltipContent);
    } else {
        // Create new marker
        droneMarkers[droneId] = L.marker(latLng, {
            icon: droneIcon,
            rotationAngle: heading,
            rotationOrigin: 'center center'
        }).addTo(map);
        
        droneMarkers[droneId].bindTooltip(tooltipContent, {
            permanent: true,
            direction: 'top',
            offset: [0, -15],
            className: 'bg-gray-800 text-white border-0 rounded-md p-2 shadow-lg'
        });
    }

    // Pan map to the latest drone
    map.panTo(latLng);
}

/**
 * Removes a drone marker from the map.
 * @param {string} droneId - The unique ID of the drone.
 */
export function removeDroneMarker(droneId) {
    if (droneMarkers[droneId]) {
        map.removeLayer(droneMarkers[droneId]);
        delete droneMarkers[droneId];
    }
}
