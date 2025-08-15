#!/usr/bin/env bash
set -euo pipefail

# ===== Config =====
IMAGE_NAME="steerai_demo_lifecycle_controller:latest"
DOCKERFILE_PATH="./Dockerfile"       # adjust if Dockerfile is elsewhere
CONTAINER_NAME="turtle_lifecycle_dev"

# ===== Colors =====
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# ===== X11 paths & auth =====
XSOCK="/tmp/.X11-unix"
# Put the xauth file in HOME to avoid /tmp permission issues
XAUTH="${XAUTH:-$HOME/.docker.xauth}"

# Revert xhost when we exit
cleanup() {
  echo -e "${GREEN}>>> Reverting X access control...${NC}"
  xhost -local:root >/dev/null 2>&1 || true
}
trap cleanup EXIT

echo -e "${GREEN}>>> Building Docker image...${NC}"
docker build -t "${IMAGE_NAME}" -f "${DOCKERFILE_PATH}" .

echo -e "${GREEN}>>> Allowing local X clients (docker) to connect...${NC}"
xhost +local:root >/dev/null 2>&1 || true

echo -e "${GREEN}>>> Preparing Xauthority at ${XAUTH}${NC}"
# Create/refresh Xauthority file. If no cookie is found (e.g., Wayland-only),
# we’ll still run but skip mounting XAUTH.
touch "${XAUTH}" || echo -e "${YELLOW}WARN:${NC} Could not create ${XAUTH}"
HAVE_COOKIE=false
if command -v xauth >/dev/null 2>&1; then
  if xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f "${XAUTH}" nmerge - 2>/dev/null; then
    chmod 644 "${XAUTH}" || true
    # Check if file is non-empty (cookie merged)
    if [[ -s "${XAUTH}" ]]; then
      HAVE_COOKIE=true
    fi
  else
    echo -e "${YELLOW}WARN:${NC} No X11 cookie found for DISPLAY='${DISPLAY}'. If you’re on Wayland-only, consider an Xorg session or headless fallback."
  fi
else
  echo -e "${YELLOW}WARN:${NC} 'xauth' not found on host; proceeding without XAUTH."
fi

echo -e "${GREEN}>>> Starting container...${NC}"
RUN_ARGS=(
  --rm -it
  --name "${CONTAINER_NAME}"
  --net=host
  -e "DISPLAY=${DISPLAY}"
  -e "QT_X11_NO_MITSHM=1"
  -v "${XSOCK}:${XSOCK}:rw"
)

# Mount XAUTH only if it exists and has content
if [[ "${HAVE_COOKIE}" == "true" ]]; then
  RUN_ARGS+=(-e "XAUTHORITY=${XAUTH}" -v "${XAUTH}:${XAUTH}:ro")
else
  echo -e "${YELLOW}INFO:${NC} Skipping XAUTH mount (file missing or empty)."
fi

# Launch an interactive bash (your Dockerfile’s CMD already sources ROS if you set it that way)
docker run "${RUN_ARGS[@]}" "${IMAGE_NAME}" bash
