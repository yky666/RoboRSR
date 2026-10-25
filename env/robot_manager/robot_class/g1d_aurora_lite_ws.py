"""G1-D body interfaces with the CAD-derived Aurora Lite asset on both wrists."""

from env.robot_manager.aurora_contract import measured_opening, urdf_jaw_positions
from env.robot_manager.robot_class.g1d import G1DLeft, G1DRight
from env.robot_manager.tactile_bridge import XenseTactileBridge


class AuroraObservationMixin:
    """Optional four-pad observation API; arm actions retain their existing size."""

    # The inherited Dex1 lag limiter assumes increasing q closes the fingers.
    # Aurora uses increasing q to open; retain the delivered position targets.
    finger_coupling = None

    opening_m_to_joint_positions = staticmethod(urdf_jaw_positions)
    joint_positions_to_opening_m = staticmethod(measured_opening)

    def configure_tactile_recording(self, directory):
        self._tactile_bridge = XenseTactileBridge(directory)

    def tactile_observation(self, frame_id, simulation_time_s):
        bridge = getattr(self, "_tactile_bridge", None)
        if bridge is None:
            raise RuntimeError("Configure the matching tactile recording before reading observations")
        return bridge.observation(frame_id, simulation_time_s)


class G1DAuroraLeft(AuroraObservationMixin, G1DLeft):
    asset_name = "g1d_aurora_lite_ws"


class G1DAuroraRight(AuroraObservationMixin, G1DRight):
    asset_name = "g1d_aurora_lite_ws"
