#!/usr/bin/env bash
# fix_gazebo_state_retry.sh
#
# Patches coursework_world_spawner.py (both the installed copy and the source
# copy) to add retry logic so that get_model_state_by_name and
# get_model_state_via_gz wait for Gazebo to fully register a freshly-spawned
# entity before giving up.
#
# Run once after cloning / rebuilding:
#   bash ~/comp0250_s26_labs/src/courseworks/cw2_team_20/scripts/fix_gazebo_state_retry.sh
#
# Environment variables (optional overrides):
#   ROS2_WS        — path to the ROS 2 workspace (default: ~/comp0250_s26_labs)
#   PYTHON_VERSION — e.g. "3.10" (default: auto-detected)
#
# No colcon rebuild is needed for cw2_world_spawner after running this script
# (it is a pure-Python package).  You DO need to rebuild cw2_team_20 if you
# renamed the package (see README.md).

set -euo pipefail

# --- Configurable paths --------------------------------------------------- #
ROS2_WS="${ROS2_WS:-${HOME}/comp0250_s26_labs}"

if [[ -z "${PYTHON_VERSION:-}" ]]; then
    PYTHON_VERSION="$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
fi

INSTALL_FILE="${ROS2_WS}/install/cw2_world_spawner/local/lib/python${PYTHON_VERSION}/dist-packages/cw2_world_spawner_lib/coursework_world_spawner.py"
SOURCE_FILE="${ROS2_WS}/src/courseworks/cw2_world_spawner/src/cw2_world_spawner_lib/coursework_world_spawner.py"
# -------------------------------------------------------------------------- #

patch_file() {
    local TARGET="$1"

    if [[ ! -f "$TARGET" ]]; then
        echo "[SKIP] File not found: $TARGET"
        return
    fi

    python3 - "$TARGET" << 'PYEOF'
import sys

path = sys.argv[1]

with open(path, 'r') as fh:
    lines = fh.readlines()

# ------------------------------------------------------------------ #
# Helper: find the start and end line indices of a method            #
# ------------------------------------------------------------------ #
def find_method_bounds(lines, sig_prefix):
    """Return (start, end) where lines[start:end] is the full method.

    start — index of the 'def' line matching sig_prefix
    end   — index of the next '  def ' at the same 2-space indent,
            or len(lines) if there is none
    """
    start = None
    for i, line in enumerate(lines):
        if line.startswith(sig_prefix):
            start = i
            break
    if start is None:
        return None, None
    for j in range(start + 1, len(lines)):
        if lines[j].startswith('  def '):
            return start, j
    return start, len(lines)

# ------------------------------------------------------------------ #
# 1. Ensure 'import time' is present                                 #
# ------------------------------------------------------------------ #
if not any('import time' in ln for ln in lines):
    insert_at = next(
        (i for i, ln in enumerate(lines)
         if ln.startswith('import ') or ln.startswith('from ')),
        0,
    )
    lines.insert(insert_at, 'import time\n')
    print(f"[OK]   Added 'import time' to {path}")

# ------------------------------------------------------------------ #
# 2. Patch get_model_state_by_name                                    #
# ------------------------------------------------------------------ #
NEW_BY_NAME = [
    '  def get_model_state_by_name(self, name, relname="world"):\n',
    '    # Wait up to 30s for service availability\n',
    '    if not self.get_state_client.wait_for_service(timeout_sec=30.0):\n',
    '      return self.get_model_state_via_gz(name)\n',
    '\n',
    '    request = GetEntityState.Request()\n',
    '    request.name = name\n',
    '    request.reference_frame = relname\n',
    '\n',
    '    # Retry up to 30 times with 1s sleep — entity may not be registered in Gazebo immediately after spawn\n',
    '    max_attempts = 30\n',
    '    for attempt in range(max_attempts):\n',
    '      resp = call_service_sync(self.client_node, self.get_state_client, request, timeout_sec=5.0)\n',
    "      if resp is not None and getattr(resp, 'success', False):\n",
    '        return resp\n',
    '      self.node.get_logger().warn(\n',
    "        f\"get_entity_state for '{name}' attempt {attempt+1}/{max_attempts} failed, retrying in 1s...\")\n",
    '      time.sleep(1.0)\n',
    '\n',
    '    return self.get_model_state_via_gz(name)\n',
]

start, end = find_method_bounds(lines, '  def get_model_state_by_name(')
if start is None:
    print(f"[WARN] Could not find get_model_state_by_name in {path} — skipping.")
elif any('max_attempts = 30' in ln for ln in lines[start:end]):
    print(f"[SKIP] get_model_state_by_name already patched in {path}")
else:
    lines[start:end] = NEW_BY_NAME
    print(f"[OK]   Patched get_model_state_by_name in {path}")

# ------------------------------------------------------------------ #
# 3. Patch get_model_state_via_gz                                     #
# ------------------------------------------------------------------ #
NEW_VIA_GZ_HEAD = [
    '  def get_model_state_via_gz(self, name):\n',
    '    max_attempts = 30\n',
    '    output = None\n',
    '    for attempt in range(max_attempts):\n',
    '      try:\n',
    '        output = subprocess.check_output(\n',
    "          ['gz', 'model', '-m', name, '-i'],\n",
    '          stderr=subprocess.STDOUT,\n',
    '          text=True,\n',
    '          timeout=5.0,\n',
    '        )\n',
    '        break\n',
    '      except Exception:\n',
    '        if attempt < max_attempts - 1:\n',
    '          self.node.get_logger().warn(\n',
    "            f\"gz model query for '{name}' attempt {attempt+1}/{max_attempts} failed, retrying in 1s...\")\n",
    '          time.sleep(1.0)\n',
    '        else:\n',
    '          self.node.get_logger().warn(\n',
    "            f\"Unable to query model state for '{name}' via /gazebo/get_entity_state or gz model\")\n",
    '          return None\n',
    '    if output is None:\n',
    '      return None\n',
]

start, end = find_method_bounds(lines, '  def get_model_state_via_gz(')
if start is None:
    print(f"[WARN] Could not find get_model_state_via_gz in {path} — skipping.")
elif any('max_attempts = 30' in ln for ln in lines[start:end]):
    print(f"[SKIP] get_model_state_via_gz already patched in {path}")
else:
    # Preserve the existing tail starting from 'match = re.search'
    tail_start = None
    for k in range(start, end):
        if 'match = re.search' in lines[k]:
            tail_start = k
            break
    tail = lines[tail_start:end] if tail_start is not None else []
    lines[start:end] = NEW_VIA_GZ_HEAD + tail
    print(f"[OK]   Patched get_model_state_via_gz in {path}")

with open(path, 'w') as fh:
    fh.writelines(lines)

print(f"[DONE] {path}")
print()
PYEOF
}

echo "=== Patching installed copy ==="
patch_file "$INSTALL_FILE"

echo "=== Patching source copy ==="
patch_file "$SOURCE_FILE"

echo "=== All done ==="
echo ""
echo "No rebuild required for cw2_world_spawner."
echo "If you renamed the package to cw2_team_20, rebuild it with:"
echo "  cd ${ROS2_WS} && colcon build --packages-select cw2_team_20"
