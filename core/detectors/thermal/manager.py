class DetectorManager:
    """
    Aggregates results from multiple detector strategies.
    """
    def __init__(self):
        self.detectors = []

    def add_detector(self, detector):
        self.detectors.append(detector)

    def process(self, frame):
        all_results = []
        for d in self.detectors:
            # In a real system, we might pass different frame types (thermal vs visual)
            # based on the detector type.
            results = d.process_frame(frame)
            all_results.extend(results)
        return all_results