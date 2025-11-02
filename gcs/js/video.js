/**
 * video.js
 * Manages the video feed modal.
 */

const videoModal = document.getElementById('videoModal');
const videoTitle = document.getElementById('videoTitle');
const videoContainer = document.getElementById('videoContainer');
const closeVideoBtn = document.getElementById('closeVideoBtn');

// Close modal logic
closeVideoBtn.addEventListener('click', () => {
    videoModal.classList.add('hidden');
    // Stop video stream (placeholder)
    videoContainer.innerHTML = `
        <div class="loader"></div>
        <p class="text-gray-500 ml-4">Attempting to connect to video stream...</p>`;
});

/**
 * Initializes and shows the video feed modal.
 * @param {string} droneId - The ID of the drone to stream from.
 */
export function showVideoFeed(droneId) {
    videoTitle.textContent = `Live Feed: ${droneId}`;
    videoModal.classList.remove('hidden');

    // --- Placeholder: Simulate loading a video stream ---
    // In a real application, this would involve WebRTC or
    // creating an <img> tag pointing to an MJPEG stream.
    setTimeout(() => {
        // Simulate a successful connection
        videoContainer.innerHTML = `
            <img 
                src="https://placehold.co/800x600/000000/333333?text=Video+Stream+from+${droneId}" 
                alt="Live video feed from ${droneId}"
                class="w-full h-full object-cover"
                onerror="this.src='https://placehold.co/800x600/000000/FF0000?text=Stream+Failed'"
            />`;
    }, 2000); // 2-second delay to simulate connection
}
