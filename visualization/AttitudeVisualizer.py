from .VisualizationConfig import VisualizationConfig, ValidationResult
from .SimulationDataLoader import SimulationDataLoader
import os
import matplotlib
if not os.environ.get("MPLBACKEND"):
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
from typing import List, Optional, Union
import numpy as np
import pandas as pd
from .AttitudeMathUtils import AttitudeMathUtils
from .RigidBodyGeometry import RigidBodyGeometry
from pathlib import Path
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
import logging
import matplotlib.lines as mlines
from matplotlib.animation import FuncAnimation, FFMpegWriter, PillowWriter

_INTERACTIVE_BACKEND = matplotlib.get_backend().lower() not in ("agg", "pdf", "ps", "svg", "template")

# Renders attitude data as static plots, 3D animations and trajectories
class AttitudeVisualizer:

    def __init__(self, config: VisualizationConfig):
        self.config = config
        self.logger = logging.getLogger(__name__)
        self.data_loader = SimulationDataLoader(self.logger)
        self.geometry = RigidBodyGeometry(config)
        # Source path, needed by the parallel video renderer
        self._data_path: Optional[str] = None
        # Go parallel past this many frames, capped at this many workers
        self._parallel_frame_threshold: int = 200
        self._parallel_max_workers: int = 8

        Path(self.config.output_dir).mkdir(parents=True, exist_ok=True)

    def load_simulation_data(self, file_path: Union[str, Path]) -> bool:
        validation_result, _ = self.data_loader.load_data(file_path)
        ok = validation_result == ValidationResult.VALID
        self._data_path = str(file_path) if ok else None
        self._torque_series = None
        return ok

    @property
    def simulation_data(self) -> Optional[pd.DataFrame]:
        return self.data_loader.data

    # Settling index/time; unsettled if any channel never holds the band
    def compute_settling(self) -> dict:
        data = self.simulation_data
        time = data['time'].to_numpy()

        indices = []
        settled = True

        for channel in ['roll', 'pitch', 'yaw']:
            idx = AttitudeMathUtils.settling_index(
                data[channel].to_numpy(),
                settle_fraction=self.config.settling_fraction,
                tail_fraction=self.config.settling_tail_fraction,
                floor=self.config.settling_angle_floor)
            if idx < 0:
                settled = False
            indices.append(max(idx, 0))

        index = max(indices)
        return {'settled': settled, 'index': index, 'time': float(time[index])}

    _TORQUE_COLS = ['torque_x', 'torque_y', 'torque_z']
    _GYRO_COLS = ['gyro_x', 'gyro_y', 'gyro_z']

    # True when the CSV has both torque and gyro columns
    def _has_torque_columns(self) -> bool:
        cols = set(self.simulation_data.columns)
        return set(self._TORQUE_COLS) <= cols and set(self._GYRO_COLS) <= cols

    # Centered moving average of the raw torque: drops the noise-dominated
    # high-frequency part (filtered out by inertia anyway) and keeps the slow
    # net torque that drives the motion. CSV is left untouched
    def _display_series(self, cols) -> np.ndarray:
        raw = self.simulation_data[cols].to_numpy(dtype=float)
        window = max(1, int(self.config.torque_smoothing_window))
        if window <= 1:
            return raw
        return pd.DataFrame(raw).rolling(window=window, center=True,
                                         min_periods=1).mean().to_numpy()

    # Build and cache the smoothed control/gyro torque series
    def _ensure_torque_series(self) -> None:
        if getattr(self, '_torque_series', None) is not None:
            return
        if not self._has_torque_columns():
            self._torque_series = {'control': None, 'gyro': None}
            return
        self._torque_series = {
            'control': self._display_series(self._TORQUE_COLS),
            'gyro': self._display_series(self._GYRO_COLS),
        }

    # Per-channel peak over the animated span; control and gyro differ by orders
    # of magnitude, so a shared scale would hide the smaller one
    def _torque_scales(self, end_index: int) -> dict:
        self._ensure_torque_series()
        scales = {}
        for key in ('control', 'gyro'):
            series = self._torque_series[key]
            if series is None:
                scales[key] = 1e-9
            else:
                norms = np.linalg.norm(series[:end_index], axis=1)
                scales[key] = max(float(norms.max()), 1e-9)
        return scales

    # Three 3D views of the orientation sequence plus angle-vs-time history
    def plot_static_sequence(self, num_frames: int = 20) -> bool:
        if self.simulation_data is None:
            self.logger.error("No simulation data loaded")
            return False

        try:
            fig = plt.figure(figsize=(16, 12))
            fig.patch.set_facecolor('white')

            data_length = len(self.simulation_data)
            step_size = max(1, data_length // num_frames)
            time_indices = range(0, data_length, step_size)
            body_count = len(time_indices)

            half_diagonal = 0.5 * float(np.sqrt(self.config.body_length ** 2 + self.config.body_width ** 2 + self.config.body_height ** 2))
            spacing = 2.0 * half_diagonal + 0.4

            gs = fig.add_gridspec(2, 2, hspace=0.4, wspace=0.3,top=0.92, bottom=0.08, left=0.08, right=0.95)
            ax1 = fig.add_subplot(gs[0, 0], projection='3d')  # Isometric
            ax2 = fig.add_subplot(gs[0, 1], projection='3d')  # Top
            ax3 = fig.add_subplot(gs[1, 0], projection='3d')  # Side
            ax4 = fig.add_subplot(gs[1, 1])                   # Time history

            colors = plt.cm.viridis(np.linspace(0, 1, len(time_indices)))

            for i, (idx, color) in enumerate(zip(time_indices, colors)):
                row = self.simulation_data.iloc[idx]
                roll, pitch, yaw = row['roll'], row['pitch'], row['yaw']

                # spread along X to avoid overlap
                position = np.array([i * spacing, 0.0, 0.0])

                vertices = self.geometry.transform_body_vertices(roll, pitch, yaw, position)
                faces = self.geometry.get_body_faces()
                face_vertices = [[vertices[j] for j in face] for face in faces]

                for ax in [ax1, ax2, ax3]:
                    poly3d = Poly3DCollection(face_vertices, alpha=0.4,
                                            facecolors=color, edgecolors='black', linewidths=0.5)
                    ax.add_collection3d(poly3d)

                    axes = self.geometry.create_coordinate_axes(roll, pitch, yaw, position)
                    for axis_name, axis_color in self.config.axis_colors.items():
                        axis_data = axes[axis_name]
                        ax.plot([axis_data[0][0], axis_data[1][0]],
                               [axis_data[0][1], axis_data[1][1]],
                               [axis_data[0][2], axis_data[1][2]],
                               color=axis_color, linewidth=1.2, alpha=0.8)

            margin = half_diagonal + self.config.coordinate_axis_scale + 0.5
            max_range = (body_count - 1) * spacing + margin
            for ax in [ax1, ax2, ax3]:
                ax.set_xlim([-margin, max_range])
                ax.set_ylim([-3, 3])
                ax.set_zlim([-3, 3])
                ax.set_xlabel('X [m]', labelpad=8, fontsize=9)
                ax.set_ylabel('Y [m]', labelpad=8, fontsize=9)
                ax.set_zlabel('Z [m]', labelpad=8, fontsize=9)
                ax.tick_params(axis='x', pad=2, labelsize=8)
                ax.tick_params(axis='y', pad=2, labelsize=8)
                ax.tick_params(axis='z', pad=2, labelsize=8)

            ax1.set_title('Isometric', pad=10, fontsize=10)
            ax1.view_init(elev=20, azim=45)

            ax2.set_title('Top  (X-Y)', pad=10, fontsize=10)
            ax2.view_init(elev=90, azim=0)
            ax2.set_zticklabels([])
            ax2.set_zlabel('')

            ax3.set_title('Side  (X-Z)', pad=10, fontsize=10)
            ax3.view_init(elev=0, azim=-90)
            ax3.set_yticklabels([])
            ax3.set_ylabel('')

            ax4.plot(self.simulation_data['time'],AttitudeMathUtils.rad_to_deg(self.simulation_data['roll']), color=self.config.axis_colors['x'], label='Roll (φ)', linewidth=1.2, alpha=0.9)
            ax4.plot(self.simulation_data['time'],AttitudeMathUtils.rad_to_deg(self.simulation_data['pitch']), color=self.config.axis_colors['y'], label='Pitch (θ)', linewidth=1.2, alpha=0.9)
            ax4.plot(self.simulation_data['time'],AttitudeMathUtils.rad_to_deg(self.simulation_data['yaw']), color=self.config.axis_colors['z'], label='Yaw (ψ)', linewidth=1.2, alpha=0.9)

            # guides at first, middle and last shown frame
            selected_markers = [time_indices[0], time_indices[len(time_indices)//2], time_indices[-1]]
            for idx in selected_markers:
                time_val = self.simulation_data.iloc[idx]['time']
                ax4.axvline(x=time_val, color='gray', alpha=0.5, linestyle='--', linewidth=1.0)

            settling = self.compute_settling()
            if settling['settled']:
                ax4.axvline(x=settling['time'], color='black', alpha=0.8, linestyle='-', linewidth=1.2, label=f"Settled (t={settling['time']:.2f}s)")

            ax4.set_xlabel('Time [s]', fontsize=10)
            ax4.set_ylabel('Angle [°]', fontsize=10)
            ax4.set_title('Euler Angles', pad=10, fontsize=10)
            ax4.legend(loc='upper right', frameon=True, framealpha=0.95, fontsize=9,
                      bbox_to_anchor=(0.98, 0.98), ncol=1)
            ax4.grid(True, alpha=0.3, linewidth=0.5)
            ax4.tick_params(direction='in', which='both', labelsize=8)

            fig.suptitle('Orientation Analysis', fontsize=12, fontweight='bold', y=0.97)

            # tight_layout isn't supported for 3D axes
            plt.subplots_adjust(left=0.05, bottom=0.08, right=0.95, top=0.90, wspace=0.15, hspace=0.25)

            output_path = Path(self.config.output_dir) / self.config.static_plot_filename
            plt.savefig(output_path, dpi=self.config.figure_dpi, bbox_inches='tight', facecolor='white', edgecolor='none')

            self.logger.info(f"Static visualization saved: {output_path}")
            if _INTERACTIVE_BACKEND:
                plt.show()
            return True

        except Exception as e:
            self.logger.error(f"Error creating static plot: {e}")
            return False

    # Animated 3D view with telemetry overlay, exported as MP4 (ffmpeg) or GIF
    # by extension. One frame per CSV row up to settling plus a small margin
    def create_animated_plot(self, interval: Optional[int] = None) -> bool:
        if self.simulation_data is None:
            self.logger.error("No simulation data loaded")
            return False

        if interval is None:
            interval = self.config.animation_interval

        try:
            data_length = len(self.simulation_data)

            # up to settling plus a margin, or the whole record if it never settles
            settling = self.compute_settling()
            settle_index = settling['index']
            if settling['settled']:
                margin = max(int(self.config.settling_margin_fraction * settle_index), self.config.settling_margin_min_frames)
                end_index = min(data_length, settle_index + margin + 1)
                self.logger.info(f"Settling detected at t={settling['time']:.3f}s "
                                 f"(row {settle_index}); animating {end_index} frames")
            else:
                end_index = data_length
                self.logger.info("No settling detected within the log; animating all frames")

            total_frames = end_index

            omega_norms = np.linalg.norm(self.simulation_data[['omega_x', 'omega_y', 'omega_z']].to_numpy()[:end_index], axis=1)
            omega_max = max(float(omega_norms.max()), 1e-9)
            torque_scales = self._torque_scales(end_index)

            output_path = Path(self.config.output_dir) / self.config.animation_filename
            ext = output_path.suffix.lower()

            print(f"Generating {total_frames} animation frames...")
            self.logger.info(f"Creating animation with {total_frames} frames")

            if (ext in ('.mp4', '.m4v', '.mov') and self._data_path is not None
                    and total_frames >= self._parallel_frame_threshold
                    and (os.cpu_count() or 1) > 1):
                if self._render_parallel(output_path, end_index, settling,
                                         settle_index, total_frames, omega_max, torque_scales):
                    if _INTERACTIVE_BACKEND:
                        plt.show()
                    return True
                self.logger.warning("Parallel render failed; falling back to single process")

            # single-process path: GIF, short clips, or fallback
            fig, ax, artists = self._build_anim_figure()

            def animate(frame_idx: int) -> List:
                self._update_anim_frame(ax, artists, frame_idx, settling, settle_index, total_frames, omega_max, torque_scales)
                return [artists['body_poly'], artists['telemetry'], artists['title']]

            anim = FuncAnimation(fig, animate, frames=range(0, end_index), interval=interval, blit=False, repeat=False)

            try:
                self.logger.info(f"Saving animation ({total_frames} frames)...")
                if ext in ('.mp4', '.m4v', '.mov'):
                    # yuv420p for QuickTime/browser playback
                    writer = FFMpegWriter(fps=self.config.animation_fps, codec='libx264', extra_args=['-pix_fmt', 'yuv420p'])
                else:
                    writer = PillowWriter(fps=self.config.animation_fps)

                anim.save(output_path, writer=writer, dpi=self.config.animation_dpi)
                self.logger.info(f"Animation saved: {output_path}")
            except Exception as e:
                self.logger.warning(f"Could not save animation: {e}")
            finally:
                plt.close(fig)

            if _INTERACTIVE_BACKEND:
                plt.show()
            return True

        except Exception as e:
            self.logger.error(f"Error creating animation: {e}")
            return False

    # Render PNGs in a process pool, then stitch them with one ffmpeg encode.
    # The temp PNG dir is always cleaned up
    def _render_parallel(self, output_path: Path, end_index: int, settling: dict, settle_index: int, total_frames: int, omega_max: float, torque_scales: dict) -> bool:
        import multiprocessing as mp
        import subprocess
        import shutil
        import tempfile

        tmp_dir = tempfile.mkdtemp(prefix='anim_frames_', dir=str(self.config.output_dir))
        try:
            n_workers = min(os.cpu_count() or 1, self._parallel_max_workers)

            # near-equal contiguous chunks per worker
            bounds = np.linspace(0, end_index, n_workers + 1).astype(int)
            tasks = []
            for w in range(n_workers):
                start, stop = int(bounds[w]), int(bounds[w + 1])
                if stop > start:
                    tasks.append((self._data_path, tmp_dir, start, stop, settling, settle_index, total_frames, omega_max, torque_scales, self.config))

            self.logger.info(f"Rendering {total_frames} frames on {len(tasks)} processes")
            ctx = mp.get_context('spawn')
            with ctx.Pool(len(tasks)) as pool:
                results = pool.map(_render_frame_range, tasks)
            if not all(results):
                return False

            cmd = ['ffmpeg', '-y', '-loglevel', 'error',
                   '-framerate', str(self.config.animation_fps),
                   '-i', os.path.join(tmp_dir, 'frame_%06d.png'),
                   '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '23',
                   str(output_path)]
            subprocess.run(cmd, check=True)
            self.logger.info(f"Animation saved: {output_path}")
            return True
        except Exception as e:
            self.logger.warning(f"Parallel render error: {e}")
            return False
        finally:
            shutil.rmtree(tmp_dir, ignore_errors=True)

    # Figure with static decorations plus the reusable artists
    def _build_anim_figure(self):
        fig = plt.figure(figsize=(12, 9))
        ax = fig.add_subplot(111, projection='3d')
        fig.patch.set_facecolor('white')

        ax.set_xlim([-2, 2])
        ax.set_ylim([-2, 2])
        ax.set_zlim([-2, 2])
        ax.set_autoscale_on(False)
        ax.set_xlabel('X [m]', labelpad=10)
        ax.set_ylabel('Y [m]', labelpad=10)
        ax.set_zlabel('Z [m]', labelpad=10)
        for axis in ['x', 'y', 'z']:
            ax.tick_params(axis=axis, pad=5)

        axis_colors = self.config.axis_colors
        body_poly = Poly3DCollection([], alpha=0.3, facecolors=self.config.body_face_color, edgecolors=self.config.body_edge_color, linewidths=1.0)
        ax.add_collection3d(body_poly)

        axis_lines = {name: ax.plot([], [], [], color=color, linewidth=self.config.axis_line_width, alpha=self.config.axis_line_alpha)[0] for name, color in axis_colors.items()}

        info_box = dict(boxstyle='round,pad=0.3', facecolor='white',alpha=0.95, edgecolor='gray', linewidth=0.5)
        telemetry = ax.text2D(0.02, 0.95, '', transform=ax.transAxes, fontsize=9,verticalalignment='top', family='monospace', bbox=info_box, zorder=1000)
        title = ax.set_title('', pad=15, fontsize=12)

        legend_elements = [mlines.Line2D([0], [0], color=axis_colors[name],linewidth=2, alpha=0.7, label=f'body {name}-axis') for name in axis_colors.keys()]
        legend_elements.append(mlines.Line2D([0], [0], color=self.config.omega_arrow_color, linewidth=4, label='ω  angular velocity'))
        legend_elements.append(mlines.Line2D([0], [0], color=self.config.control_torque_color, linewidth=3, label='τ  control torque (net)'))
        if self.config.show_gyroscopic_arrow:
            legend_elements.append(mlines.Line2D([0], [0], color=self.config.gyro_torque_color, linewidth=3, label='ω×Iω  gyroscopic'))
        ax.legend(handles=legend_elements, loc='upper right', bbox_to_anchor=(0.98, 0.90), frameon=True, framealpha=0.9, fontsize=8, borderpad=0.4, handlelength=1.6)

        artists = {'body_poly': body_poly, 'axis_lines': axis_lines,
                   'telemetry': telemetry,
                   'title': title, 'quiver_holder': {'artist': None},
                   'control_holder': {'artist': None}, 'gyro_holder': {'artist': None}}
        return fig, ax, artists

    # Redraw one torque arrow: body vector rotated into the inertial frame,
    # scaled by that channel's own peak
    def _draw_torque_arrow(self, ax, holder, rotation_matrix, body_vec, scale, color) -> None:
        if holder['artist'] is not None:
            holder['artist'].remove()
            holder['artist'] = None

        if body_vec is None or scale <= 1e-9:
            return

        magnitude = np.linalg.norm(body_vec)
        if magnitude <= 1e-6:
            return

        direction = (rotation_matrix @ body_vec) / magnitude
        length = (magnitude / scale) * self.config.torque_arrow_max_len
        tip = direction * length
        holder['artist'] = ax.quiver(
            0, 0, 0, tip[0], tip[1], tip[2],
            color=color, linewidth=3.0, alpha=0.9, arrow_length_ratio=0.2)

    # Refresh the reusable artists for one frame; no figure setup, cheap in a loop
    def _update_anim_frame(self, ax, artists, frame_idx, settling, settle_index,total_frames, omega_max, torque_scales=None) -> None:
        faces = self.geometry.get_body_faces()
        row = self.simulation_data.iloc[frame_idx]
        roll, pitch, yaw = row['roll'], row['pitch'], row['yaw']
        time_val = row['time']
        omega_x, omega_y, omega_z = row['omega_x'], row['omega_y'], row['omega_z']

        vertices = self.geometry.transform_body_vertices(roll, pitch, yaw)
        artists['body_poly'].set_verts([[vertices[j] for j in face] for face in faces])

        rotation_matrix = AttitudeMathUtils.rotation_matrix_xyz(roll, pitch, yaw)

        axes = self.geometry.create_coordinate_axes(roll, pitch, yaw)
        for axis_name, line in artists['axis_lines'].items():
            p0, p1 = axes[axis_name][0], axes[axis_name][1]
            line.set_data([p0[0], p1[0]], [p0[1], p1[1]])
            line.set_3d_properties([p0[2], p1[2]])

        quiver_holder = artists['quiver_holder']
        if quiver_holder['artist'] is not None:
            quiver_holder['artist'].remove()
            quiver_holder['artist'] = None

        omega_vec = np.array([omega_x, omega_y, omega_z])
        omega_magnitude = np.linalg.norm(omega_vec)

        if omega_magnitude > 1e-6:
            omega_dir = (rotation_matrix @ omega_vec) / omega_magnitude
            arrow_len = (omega_magnitude / omega_max) * self.config.omega_arrow_max_len
            omega_inertial = omega_dir * arrow_len
            quiver_holder['artist'] = ax.quiver(
                0, 0, 0, omega_inertial[0], omega_inertial[1], omega_inertial[2],
                color=self.config.omega_arrow_color, linewidth=4.5, alpha=0.95,
                arrow_length_ratio=0.2)

        # control torque always; gyroscopic reaction only when enabled
        if torque_scales is not None and self._has_torque_columns():
            self._ensure_torque_series()
            self._draw_torque_arrow(ax, artists['control_holder'], rotation_matrix,
                                    self._torque_series['control'][frame_idx],
                                    torque_scales['control'],
                                    self.config.control_torque_color)
            if self.config.show_gyroscopic_arrow:
                self._draw_torque_arrow(ax, artists['gyro_holder'], rotation_matrix,
                                        self._torque_series['gyro'][frame_idx],
                                        torque_scales['gyro'],
                                        self.config.gyro_torque_color)

        roll_deg = round(AttitudeMathUtils.rad_to_deg(roll), 1)
        pitch_deg = round(AttitudeMathUtils.rad_to_deg(pitch), 1)
        yaw_deg = round(AttitudeMathUtils.rad_to_deg(yaw), 1)

        if settling['settled']:
            settle_line = ("SETTLED" if frame_idx >= settle_index else "settling...")
            settle_line += f"   t_s = {settling['time']:.2f}s"
        else:
            settle_line = "not settled within log"

        torque_line = ""
        if self._has_torque_columns():
            self._ensure_torque_series()
            net_ctrl = float(np.linalg.norm(self._torque_series['control'][frame_idx]))
            torque_line = f"τ : {net_ctrl:5.2f} Nm\n"

        telemetry_text = (f"φ : {roll_deg:+6.1f}°\n"
                          f"θ : {pitch_deg:+6.1f}°\n"
                          f"ψ : {yaw_deg:+6.1f}°\n\n"
                          f"|ω|: {omega_magnitude*180/np.pi:5.1f}°/s\n"
                          f"{torque_line}\n"
                          f"{settle_line}")
        artists['telemetry'].set_text(telemetry_text)
        artists['title'].set_text(f'Attitude Dynamics    t = {time_val:.2f} s')

    # Attitude path as a time-colored 3D curve in roll/pitch/yaw space
    def create_trajectory_plot(self) -> bool:
        if self.simulation_data is None:
            self.logger.error("No simulation data loaded")
            return False

        try:
            fig = plt.figure(figsize=(12, 9), dpi=100)
            ax = fig.add_subplot(111, projection='3d')
            fig.patch.set_facecolor('white')

            roll_deg = AttitudeMathUtils.rad_to_deg(self.simulation_data['roll'])
            pitch_deg = AttitudeMathUtils.rad_to_deg(self.simulation_data['pitch'])
            yaw_deg = AttitudeMathUtils.rad_to_deg(self.simulation_data['yaw'])

            # no downsampling, so the path matches the CSV
            data_length = len(self.simulation_data)
            time = self.simulation_data['time']
            time_normalized = time / time.max()

            # one segment per step, shaded by time
            points = np.array([roll_deg.values, pitch_deg.values, yaw_deg.values]).T
            segments = np.array([points[:-1], points[1:]]).transpose(1, 0, 2)

            colors = plt.cm.plasma(time_normalized[:-1])

            lc = Line3DCollection(segments, colors=colors,linewidths=self.config.trajectory_line_width, alpha=0.8)
            ax.add_collection3d(lc)

            ax.scatter([roll_deg.iloc[0]], [pitch_deg.iloc[0]], [yaw_deg.iloc[0]],
                      c='darkgreen', s=100, marker='o', label='Start',
                      edgecolors='white', linewidth=1.5, alpha=0.9)
            ax.scatter([roll_deg.iloc[-1]], [pitch_deg.iloc[-1]], [yaw_deg.iloc[-1]],
                      c='darkred', s=100, marker='s', label='End',
                      edgecolors='white', linewidth=1.5, alpha=0.9)

            settling = self.compute_settling()
            settle_label = 'Not settled within log'
            if settling['settled']:
                si = settling['index']
                ax.scatter([roll_deg.iloc[si]], [pitch_deg.iloc[si]], [yaw_deg.iloc[si]],
                          c='gold', s=100, marker='D', label=f"Settled (t={settling['time']:.2f}s)",
                          edgecolors='white', linewidth=1.5, alpha=0.9)
                settle_label = f"Settling time: {settling['time']:.2f} s"

            ax.set_xlabel('Roll φ [°]', labelpad=10, fontsize=10)
            ax.set_ylabel('Pitch θ [°]', labelpad=10, fontsize=10)
            ax.set_zlabel('Yaw ψ [°]', labelpad=10, fontsize=10)

            final_time = self.simulation_data['time'].iloc[-1]
            ax.set_title(f'Attitude Trajectory  —  {settle_label}',
                        pad=15, fontsize=11, fontweight='bold')

            ax.legend(loc='upper left', bbox_to_anchor=(0.02, 0.96),frameon=True, framealpha=0.9, fontsize=9)

            mappable = plt.cm.ScalarMappable(cmap=plt.cm.plasma)
            mappable.set_array(time)
            cbar = plt.colorbar(mappable, ax=ax, shrink=0.6, aspect=20, pad=0.1)
            cbar.set_label('Time [s]', rotation=270, labelpad=12, fontsize=9)

            for axis in ['x', 'y', 'z']:
                ax.tick_params(axis=axis, pad=5, labelsize=9)

            output_path = Path(self.config.output_dir) / self.config.trajectory_filename

            # cap DPI to keep the file size down
            save_dpi = min(self.config.figure_dpi, 200)
            plt.savefig(output_path, dpi=save_dpi,bbox_inches='tight', facecolor='white', edgecolor='none')

            self.logger.info(f"Trajectory plot saved: {output_path}")
            self.logger.info(f"Rendered {data_length:,} points (full log)")

            if _INTERACTIVE_BACKEND:
                plt.show()
            return True

        except Exception as e:
            import traceback
            self.logger.error(f"Error creating trajectory plot: {e}")
            print("\n[TRACEBACK - Trajectory Visualization Error]")
            traceback.print_exc()
            return False


# Parallel render worker: owns a figure and writes the PNGs for one frame range. It stays picklable under the 'spawn' start method
def _render_frame_range(task) -> bool:
    (data_path, tmp_dir, start, stop, settling, settle_index,total_frames, omega_max, torque_scales, config) = task
    try:
        viz = AttitudeVisualizer(config)
        if not viz.load_simulation_data(data_path):
            return False
        fig, ax, artists = viz._build_anim_figure()
        try:
            for frame_idx in range(start, stop):
                viz._update_anim_frame(ax, artists, frame_idx, settling,settle_index, total_frames, omega_max, torque_scales)
                fig.savefig(os.path.join(tmp_dir, f"frame_{frame_idx:06d}.png"),dpi=config.animation_dpi)
        finally:
            plt.close(fig)
        return True
    except Exception:
        return False
