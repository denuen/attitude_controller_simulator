from .VisualizationConfig import VisualizationType, VisualizationConfig
import logging
import sys
from typing import Optional
from .AttitudeVisualizer import AttitudeVisualizer

# Text menu that loads a CSV and drives the visualizer exports
class AttitudeAnalysisApplication:

    _MENU = {
        '1': VisualizationType.STATIC_SEQUENCE,
        '2': VisualizationType.ANIMATED_3D,
        '3': VisualizationType.TRAJECTORY_3D,
        '4': VisualizationType.ALL_VISUALIZATIONS,
    }

    def __init__(self):
        self.config = VisualizationConfig()
        self.logger = logging.getLogger(__name__)
        self.visualizer = AttitudeVisualizer(self.config)

    def print_header(self):
        print("Attitude Controller Simulator - 3D Body Visualization")
        print("=" * 60)

    def print_menu(self):
        print("\nVisualization Options:")
        print("1. Static sequence (multiple orientations)")
        print("2. Animated 3D visualization")
        print("3. Attitude trajectory in 3D space")
        print("4. All visualizations")
        print("q. Quit")

    def get_user_choice(self) -> Optional[VisualizationType]:
        while True:
            try:
                choice = input("\nEnter choice (1-4, q): ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                return None

            if choice == 'q':
                return None

            picked = self._MENU.get(choice)
            if picked is not None:
                return picked

            print("Invalid choice. Please enter 1-4 or 'q' to quit.")

    def execute_visualization(self, viz_type: VisualizationType) -> bool:
        if viz_type == VisualizationType.STATIC_SEQUENCE:
            self.logger.info("Creating static sequence visualization")
            return self.visualizer.plot_static_sequence()

        if viz_type == VisualizationType.ANIMATED_3D:
            self.logger.info("Creating animated 3D visualization")
            print("Animating the transient up to the settling point...")
            return self.visualizer.create_animated_plot()

        if viz_type == VisualizationType.TRAJECTORY_3D:
            self.logger.info("Creating trajectory visualization")
            return self.visualizer.create_trajectory_plot()

        self.logger.info("Creating all visualizations")
        results = [self.visualizer.plot_static_sequence(),
                   self.visualizer.create_trajectory_plot()]
        print("Creating animated visualization...")
        results.append(self.visualizer.create_animated_plot())
        return all(results)

    # Load data, then loop on the menu; returns 0 on ok, 1 on error
    def run(self, data_file: Optional[str] = None) -> int:
        try:
            self.print_header()

            if data_file is None:
                data_file = sys.argv[1] if len(sys.argv) > 1 else 'simulation_output/simulation_output.csv'

            self.logger.info(f"Loading data from: {data_file}")

            if not self.visualizer.load_simulation_data(data_file):
                print(f"\nError: could not load simulation data from '{data_file}'")
                print("Make sure the file exists and is valid simulation output.")
                print("Run the simulator first: ./attitude_simulator")
                return 1

            while True:
                self.print_menu()
                viz_type = self.get_user_choice()

                if viz_type is None:
                    print("\nExiting application.")
                    break

                success = self.execute_visualization(viz_type)

                if success:
                    print("\nVisualization completed successfully.")
                else:
                    print("\nWarning: some visualizations may have failed.")
                    print("See the log for details.")

            return 0

        except KeyboardInterrupt:
            print("\n\nApplication interrupted by user.")
            return 1

        except Exception as e:
            self.logger.error(f"Unexpected application error: {e}")
            print(f"\nFATAL ERROR: {e}")
            print("Check the log file for detailed error information.")
            return 1


def main():
    app = AttitudeAnalysisApplication()
    return app.run()


if __name__ == "__main__":
    sys.exit(main())
