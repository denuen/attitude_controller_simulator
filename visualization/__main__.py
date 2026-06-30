# Entry point for "python -m visualization"
import sys
from .AttitudeAnalysisApplication import AttitudeAnalysisApplication

def main():
    app = AttitudeAnalysisApplication()
    return app.run()

if __name__ == "__main__":
    sys.exit(main())
