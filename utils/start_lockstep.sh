#!/bin/bash
# =============================================================================
# start_lockstep.sh
#
# Starts the flightstack in SIMULATION LOCKSTEP mode. The simulator
# (acsl-chrono-simulator) sends the vehicle state and the reference every
# physics step, and the flightstack only computes and returns the motor commands.
#
# Differences from start_flight.sh:
#   - Runs WITHOUT sudo. Lockstep mode needs no real-time (SCHED_FIFO) thread
#     priority, and sudo would also discard the ROS_DOMAIN_ID exported below.
#   - Sets ROS_DOMAIN_ID only for this run. The ODROID's global ROS setup, and
#     PX4's uXRCE agent on domain 0 for real flights, are left untouched.
#   - Works from any directory (it locates the workspace from its own path).
#
# Requirements:
#   - The flightstack must be built with  #define SIMULATION_LOCKSTEP 1  in
#     src/acsl_flight/include/utils/flightstack_helpers/global_helpers.hpp
#   - Make it executable once:  chmod +x utils/start_lockstep.sh
# =============================================================================

# Must match ROS_DOMAIN_ID on the simulator PC (set in its ~/.bashrc, will be installed when you run setup.py).
LOCKSTEP_DOMAIN_ID=49

# -----------------------------------------------------------------------------
# Workspace root (the parent of utils/), resolved from this script's location.
# sudo keeps the current directory, so the root shell below starts here too.
# -----------------------------------------------------------------------------
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_ROOT" || exit 1
LOG_DIR="$WS_ROOT/src/acsl_flight/flight_log"

# -----------------------------------------------------------------------------
# Safety check: only start if the lockstep switch is ON in the source tree.
# NOTE: this checks the SOURCE, not the compiled binary. Always rebuild with
# colcon after flipping the switch.
# -----------------------------------------------------------------------------
HELPERS="src/acsl_flight/include/config/global_config.hpp"
if ! grep -Eq '^[[:space:]]*#define[[:space:]]+SIMULATION_LOCKSTEP[[:space:]]+1\b' "$HELPERS"; then
    echo "[start_lockstep] SIMULATION_LOCKSTEP is not 1 in $HELPERS - refusing to start."
    echo "[start_lockstep] Set it to 1, rebuild with colcon, then rerun this script."
    exit 1
fi

# -----------------------------------------------------------------------------
# Find the ROS 2 distro the flightstack was BUILT with - no sourcing needed.
# colcon records the underlay it was built against in install/setup.bash as
#     COLCON_CURRENT_PREFIX="<ros prefix>"   (e.g. /opt/ros/jazzy)
# and sourcing install/setup.bash later sources that same underlay for us.
# -----------------------------------------------------------------------------
if [ ! -f install/setup.bash ]; then
    echo "[start_lockstep] install/setup.bash not found - build the flightstack with colcon first."
    exit 1
fi

# The first COLCON_CURRENT_PREFIX in install/setup.bash is the ROS 2 underlay.
ROS_PREFIX=$(grep -m1 -oE 'COLCON_CURRENT_PREFIX="/[^"]+"' install/setup.bash | cut -d'"' -f2)
if [ -z "$ROS_PREFIX" ] || [ ! -f "$ROS_PREFIX/setup.bash" ]; then
    echo "[start_lockstep] Could not find the ROS 2 install this workspace was built with (got: '${ROS_PREFIX:-none}')."
    echo "[start_lockstep] Rebuild the flightstack with colcon, then rerun."
    exit 1
fi

# Read the distro name from that ROS 2 install itself (works for /opt/ros/<distro>
# and for ROS 2 built from source, e.g. ~/ros2_humble/install). Used for the banner.
DISTRO=$(bash -c "source '$ROS_PREFIX/setup.bash' >/dev/null 2>&1 && echo \$ROS_DISTRO")

# The invoking (non-root) user, who gets the log folder back on exit.
OWNER="$(id -u):$(id -g)"

# -----------------------------------------------------------------------------
# Everything the root shell runs. Variables are expanded HERE, before sudo,
# except \$ROS_DOMAIN_ID, which the root shell expands so the banner shows the
# domain the node actually runs on.
#   - trap ... EXIT: when the root shell ends (normal exit or Ctrl+C), give the
#     log folder back to the invoking user. Runs as root, so no second password.
# -----------------------------------------------------------------------------
LOCKSTEP_CMD="trap 'chown -R $OWNER \"$LOG_DIR\" 2>/dev/null' EXIT && \
export ROS_DOMAIN_ID=$LOCKSTEP_DOMAIN_ID && \
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4 && \
source install/setup.bash && \
echo '[start_lockstep] Host: $(hostname) | ROS 2: $DISTRO | ROS_DOMAIN_ID='\$ROS_DOMAIN_ID && \
echo '[start_lockstep] Waiting for the simulator handshake...' && \
ros2 run acsl_flight start"

# Always prompt for the sudo password (no stored passwords).
sudo bash -c "$LOCKSTEP_CMD"