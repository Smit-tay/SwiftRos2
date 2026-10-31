#!/usr/bin/env bash
# test_swiftros2.sh — exercise every service and topic on the SwiftRos2
# hardware node. Reports pass/fail per test with live coloured output.
#
# speed is the firmware's F everywhere: 1 F = 80 deg/min of joint-space speed,
# not mm/min. Base angles in requests: 90 = straight ahead.
# Moves stay within about 30 deg of straight ahead.
#
# Schema notes: SetWrist/SetServoAngle/SetPolar speed is int32, not float;
# SetPolar request includes 'relative' and 'wait' fields (default false).
#
# Usage:
#   ros2 run swiftpro_hardware swiftpro_hardware       # in another terminal
#   ./test_swiftros2.sh
#
# Copyright 2026 Jack Sidman Smith — MIT License

set -u

# ── Colours ──────────────────────────────────────────────────────────────────
readonly C_RESET='\033[0m'
readonly C_RED='\033[1;31m'
readonly C_GREEN='\033[1;32m'
readonly C_YELLOW='\033[1;33m'
readonly C_BLUE='\033[1;34m'
readonly C_CYAN='\033[1;36m'
readonly C_GREY='\033[0;37m'

# ── Counters ─────────────────────────────────────────────────────────────────
PASS=0
FAIL=0
SKIP=0
FAILED_TESTS=()

# ── Helpers ──────────────────────────────────────────────────────────────────
section() {
    echo
    echo -e "${C_BLUE}══════════════════════════════════════════════════════════════════════${C_RESET}"
    echo -e "${C_BLUE}  $1${C_RESET}"
    echo -e "${C_BLUE}══════════════════════════════════════════════════════════════════════${C_RESET}"
}

pass() {
    PASS=$((PASS + 1))
    echo -e "  ${C_GREEN}PASS${C_RESET}  $1"
}

fail() {
    FAIL=$((FAIL + 1))
    FAILED_TESTS+=("$1: $2")
    echo -e "  ${C_RED}FAIL${C_RESET}  $1"
    echo -e "        ${C_GREY}$2${C_RESET}"
}

skip() {
    SKIP=$((SKIP + 1))
    echo -e "  ${C_YELLOW}SKIP${C_RESET}  $1"
    echo -e "        ${C_GREY}$2${C_RESET}"
}

info() {
    echo -e "  ${C_CYAN}INFO${C_RESET}  $1"
}

wait_for() {
    local secs=$1
    local msg=${2:-"waiting"}
    for ((i = secs; i > 0; i--)); do
        printf "        ${C_GREY}%s %ds...${C_RESET}\r" "$msg" "$i"
        sleep 1
    done
    printf "                                                  \r"
}

# call_service <name> <srv_type> <request_yaml> [expect_success_regex]
call_service() {
    local svc=$1
    local typ=$2
    local req=$3
    local expect=${4:-"success=True"}
    CALL_OUTPUT=$(ros2 service call "$svc" "$typ" "$req" 2>&1)
    local rc=$?
    if [[ $rc -ne 0 ]]; then
        return 1
    fi
    if [[ $CALL_OUTPUT =~ $expect ]]; then
        return 0
    fi
    return 1
}

# ── Pre-flight ───────────────────────────────────────────────────────────────
section "Pre-flight checks"

# Missing ros2 IS a hard fail — no test can run without it.
if ! command -v ros2 &>/dev/null; then
    fail "ros2 CLI available" "ros2 not in PATH — source /opt/ros/jazzy/setup.bash first"
    exit 1
fi
pass "ros2 CLI available"

# Other pre-flight checks fail loudly but allow the run to continue so
# the operator sees the full picture.
if ! ros2 node list 2>/dev/null | grep -q swiftpro_hardware; then
    fail "swiftpro_hardware node running" "Start the node first: ros2 run swiftpro_hardware swiftpro_hardware"
else
    pass "swiftpro_hardware node running"
fi

CONNECTED=$(timeout 3 ros2 topic echo /swiftpro/connected --once 2>/dev/null | grep -oP 'data:\s*\K\w+')
if [[ ${CONNECTED:-} == "true" ]]; then
    pass "/swiftpro/connected = true"
else
    fail "/swiftpro/connected = true" "got '${CONNECTED:-<empty/timeout>}' — arm may not be connected"
fi

# ── Topics ───────────────────────────────────────────────────────────────────
section "Topic surface"

EXPECTED_TOPICS=(
    "/joint_states"
    "/swiftpro/connected"
    "/swiftpro/gripper_status"
    "/swiftpro/limit_switch"
    "/swiftpro/motion_complete"
    "/swiftpro/position"
    "/swiftpro/power"
    "/swiftpro/pump_status"
)

TOPIC_LIST=$(ros2 topic list 2>/dev/null)
for t in "${EXPECTED_TOPICS[@]}"; do
    if echo "$TOPIC_LIST" | grep -qx "$t"; then
        pass "topic exists: $t"
    else
        fail "topic exists: $t" "topic missing from ros2 topic list"
    fi
done

info "Sampling /joint_states for 2s..."
JS_COUNT=$(timeout 2 ros2 topic echo /joint_states --no-arr 2>/dev/null | grep -c '^---' || true)
if [[ $JS_COUNT -ge 5 ]]; then
    pass "/joint_states streaming (~${JS_COUNT} msgs/2s)"
else
    fail "/joint_states streaming" "got only $JS_COUNT messages in 2s"
fi

info "Sampling /swiftpro/position for 2s..."
POS_COUNT=$(timeout 2 ros2 topic echo /swiftpro/position --no-arr 2>/dev/null | grep -c '^---' || true)
if [[ $POS_COUNT -ge 5 ]]; then
    pass "/swiftpro/position streaming (~${POS_COUNT} msgs/2s)"
else
    fail "/swiftpro/position streaming" "got only $POS_COUNT messages in 2s"
fi

# ── Services — discovery ─────────────────────────────────────────────────────
section "Service surface"

EXPECTED_SERVICES=(
    /swiftpro/angles_to_coord
    /swiftpro/coord_to_angles
    /swiftpro/get_analog
    /swiftpro/get_digital
    /swiftpro/get_encoder_status
    /swiftpro/get_servo_attach
    /swiftpro/is_reachable
    /swiftpro/motion_reset
    /swiftpro/move_to
    /swiftpro/pause_motion
    /swiftpro/reset
    /swiftpro/resume_motion
    /swiftpro/send_raw
    /swiftpro/set_acceleration
    /swiftpro/set_buzzer
    /swiftpro/set_digital_direction
    /swiftpro/set_digital_output
    /swiftpro/set_gripper
    /swiftpro/set_mode
    /swiftpro/set_polar
    /swiftpro/set_pump
    /swiftpro/set_servo_angle
    /swiftpro/set_servo_angles
    /swiftpro/set_servo_attach
    /swiftpro/set_wrist
)

SVC_LIST=$(ros2 service list 2>/dev/null)
for s in "${EXPECTED_SERVICES[@]}"; do
    if echo "$SVC_LIST" | grep -qx "$s"; then
        pass "service exists: $s"
    else
        fail "service exists: $s" "service missing from ros2 service list"
    fi
done

ACTION_LIST=$(ros2 action list 2>/dev/null)
if echo "$ACTION_LIST" | grep -qx "/swiftpro/move_arm"; then
    pass "action exists: /swiftpro/move_arm"
else
    fail "action exists: /swiftpro/move_arm" "action missing from ros2 action list"
fi

# ── Group 1: End effectors ───────────────────────────────────────────────────
section "Group 1 — End effectors"

# YAML 1.1 reserves 'on' as a boolean keyword — must quote the field name
# explicitly to keep it as a string.
if call_service /swiftpro/set_pump swiftpro_resources/srv/SetPump '{"on": true}'; then
    pass "pump ON"
else
    fail "pump ON" "$CALL_OUTPUT"
fi
sleep 1
if call_service /swiftpro/set_pump swiftpro_resources/srv/SetPump '{"on": false}'; then
    pass "pump OFF"
else
    fail "pump OFF" "$CALL_OUTPUT"
fi

if call_service /swiftpro/set_gripper swiftpro_resources/srv/SetGripper '{catch_object: true}'; then
    pass "gripper CATCH"
else
    fail "gripper CATCH" "$CALL_OUTPUT"
fi
sleep 1
if call_service /swiftpro/set_gripper swiftpro_resources/srv/SetGripper '{catch_object: false}'; then
    pass "gripper RELEASE"
else
    fail "gripper RELEASE" "$CALL_OUTPUT"
fi

info "Triggering buzzer (1 kHz for 0.3s)..."
if call_service /swiftpro/set_buzzer swiftpro_resources/srv/SetBuzzer \
    '{frequency: 1000, duration: 0.3}'; then
    pass "buzzer beep"
else
    fail "buzzer beep" "$CALL_OUTPUT"
fi

# ── Group 2: Status / queries ────────────────────────────────────────────────
section "Group 2 — Status & queries"

if call_service /swiftpro/get_encoder_status swiftpro_resources/srv/GetEncoderStatus '{}' \
    "success=True"; then
    STATUS=$(echo "$CALL_OUTPUT" | grep -oP 'status=\K-?\d+')
    if [[ $STATUS == "0" ]]; then
        pass "get_encoder_status (all healthy)"
    else
        fail "get_encoder_status" "status=$STATUS (bit0=base bit1=right bit2=left)"
    fi
else
    fail "get_encoder_status" "$CALL_OUTPUT"
fi

for sid in 0 1 2 3; do
    if call_service /swiftpro/get_servo_attach swiftpro_resources/srv/GetServoAttach \
        "{servo_id: $sid}"; then
        ATT=$(echo "$CALL_OUTPUT" | grep -oP 'attached=\K\w+')
        pass "get_servo_attach servo_id=$sid (attached=$ATT)"
    else
        fail "get_servo_attach servo_id=$sid" "$CALL_OUTPUT"
    fi
done

# Pin 0 is USART0, refused; 37 (PC0) is free to read.
if call_service /swiftpro/get_digital swiftpro_resources/srv/GetDigital '{pin: 37}'; then
    VAL=$(echo "$CALL_OUTPUT" | grep -oP 'value=\K-?\d+')
    pass "get_digital pin=37 (value=$VAL)"
else
    fail "get_digital pin=37" "$CALL_OUTPUT"
fi

if call_service /swiftpro/get_analog swiftpro_resources/srv/GetAnalog '{pin: 0}'; then
    VAL=$(echo "$CALL_OUTPUT" | grep -oP 'value=\K-?\d+')
    pass "get_analog pin=0 (value=$VAL)"
else
    fail "get_analog pin=0" "$CALL_OUTPUT"
fi

# ── Group 3: Configuration ───────────────────────────────────────────────────
section "Group 3 — Configuration"

if call_service /swiftpro/set_mode swiftpro_resources/srv/SetMode '{mode: 0}'; then
    pass "set_mode 0 (normal)"
else
    fail "set_mode 0" "$CALL_OUTPUT"
fi

# M204 is stored in EEPROM: 5 is the firmware default, leave it there.
if call_service /swiftpro/set_acceleration swiftpro_resources/srv/SetAcceleration '{acc: 5.0}'; then
    pass "set_acceleration 5"
else
    fail "set_acceleration 5" "$CALL_OUTPUT"
fi

# ── Group 4: Kinematics queries (no motion) ──────────────────────────────────
section "Group 4 — Kinematics queries"

if call_service /swiftpro/is_reachable swiftpro_resources/srv/IsReachable \
    '{x: 200.0, y: 0.0, z: 150.0}'; then
    REACHABLE=$(echo "$CALL_OUTPUT" | grep -oP 'reachable=\K\w+')
    if [[ $REACHABLE == "True" ]]; then
        pass "is_reachable (200,0,150) = True"
    else
        fail "is_reachable (200,0,150)" "expected True got $REACHABLE"
    fi
else
    fail "is_reachable (200,0,150)" "$CALL_OUTPUT"
fi

if call_service /swiftpro/is_reachable swiftpro_resources/srv/IsReachable \
    '{x: 1000.0, y: 0.0, z: 150.0}'; then
    REACHABLE=$(echo "$CALL_OUTPUT" | grep -oP 'reachable=\K\w+')
    if [[ $REACHABLE == "False" ]]; then
        pass "is_reachable (1000,0,150) = False"
    else
        fail "is_reachable (1000,0,150)" "expected False got $REACHABLE"
    fi
else
    fail "is_reachable (1000,0,150)" "$CALL_OUTPUT"
fi

if call_service /swiftpro/coord_to_angles swiftpro_resources/srv/CoordToAngles \
    '{x: 200.0, y: 0.0, z: 150.0}'; then
    pass "coord_to_angles (200,0,150)"
    echo -e "        ${C_GREY}$(echo "$CALL_OUTPUT" | tail -5 | tr '\n' ' ')${C_RESET}"
else
    fail "coord_to_angles" "$CALL_OUTPUT"
fi

if call_service /swiftpro/angles_to_coord swiftpro_resources/srv/AnglesToCoord \
    '{base: 90.0, left: 90.0, right: 0.0}'; then
    pass "angles_to_coord (90,90,0)"
    echo -e "        ${C_GREY}$(echo "$CALL_OUTPUT" | tail -5 | tr '\n' ' ')${C_RESET}"
else
    fail "angles_to_coord" "$CALL_OUTPUT"
fi

# ── Group 5: GPIO writes ─────────────────────────────────────────────────────
section "Group 5 — GPIO"

# Refusals only: nothing here drives a pin. Pin 36 is PC1, encoder I2C.
if call_service /swiftpro/set_digital_direction swiftpro_resources/srv/SetDigitalDirection \
    '{pin: 36, value: 1}' "success=False.*error_code=21"; then
    pass "set_digital_direction refused on firmware-owned pin"
else
    fail "set_digital_direction refused on firmware-owned pin" "$CALL_OUTPUT"
fi

if call_service /swiftpro/set_digital_output swiftpro_resources/srv/SetDigitalOutput \
    '{pin: 36, value: 1}' "success=False.*error_code=21"; then
    pass "set_digital_output refused on firmware-owned pin"
else
    fail "set_digital_output refused on firmware-owned pin" "$CALL_OUTPUT"
fi

# ── Group 6: SendRaw escape hatch ────────────────────────────────────────────
section "Group 6 — SendRaw"

if call_service /swiftpro/send_raw swiftpro_resources/srv/SendRaw \
    '{command: "M2200", timeout: 5.0}'; then
    pass "send_raw M2200 (get position)"
else
    fail "send_raw M2200" "$CALL_OUTPUT"
fi

# ── Group 7: Motion ──────────────────────────────────────────────────────────
section "Group 7 — Motion (arm will move!)"

info "Moving to home (200, 0, 150)..."
if call_service /swiftpro/move_to swiftpro_resources/srv/MoveTo \
    '{x: 200.0, y: 0.0, z: 150.0, speed: 20.0}'; then
    pass "move_to home"
    wait_for 2 "settling"
else
    fail "move_to home" "$CALL_OUTPUT"
fi

# SetWrist.speed is int32 — must NOT have a decimal point
info "Setting wrist to 45°..."
if call_service /swiftpro/set_wrist swiftpro_resources/srv/SetWrist \
    '{angle: 45.0, speed: 20}'; then
    pass "set_wrist 45°"
    sleep 1
else
    fail "set_wrist 45°" "$CALL_OUTPUT"
fi

info "Setting wrist back to 90°..."
if call_service /swiftpro/set_wrist swiftpro_resources/srv/SetWrist \
    '{angle: 90.0, speed: 20}'; then
    pass "set_wrist 90°"
    sleep 1
else
    fail "set_wrist 90°" "$CALL_OUTPUT"
fi

# SetPolar.speed is int32; relative and wait default to false but supplied
# explicitly for clarity
info "Polar move: stretch=200 rot=70° height=150..."
if call_service /swiftpro/set_polar swiftpro_resources/srv/SetPolar \
    '{stretch: 200.0, rotation: 30.0, height: 150.0, speed: 2000, relative: false, wait: false}'; then
    pass "set_polar"
    wait_for 2 "moving"
else
    fail "set_polar" "$CALL_OUTPUT"
fi

# SetServoAngles.speed is float32 (special case — different from SetServoAngle)
info "Moving via joint angles (set_servo_angles): base=90 left=80 right=20..."
if call_service /swiftpro/set_servo_angles swiftpro_resources/srv/SetServoAngles \
    '{j1: 90.0, j2: 80.0, j3: 20.0, speed: 20.0}'; then
    pass "set_servo_angles"
    wait_for 2 "moving"
else
    fail "set_servo_angles" "$CALL_OUTPUT"
fi

# SetServoAngle.speed is int32
info "Single-servo move: base servo to 70°..."
if call_service /swiftpro/set_servo_angle swiftpro_resources/srv/SetServoAngle \
    '{servo_id: 0, angle: 70.0, speed: 20}'; then
    pass "set_servo_angle servo_id=0"
    wait_for 2 "moving"
else
    fail "set_servo_angle servo_id=0" "$CALL_OUTPUT"
fi

info "Starting a long move (will pause mid-flight)..."
ros2 service call /swiftpro/move_to swiftpro_resources/srv/MoveTo \
    '{x: 200.0, y: 100.0, z: 150.0, speed: 5.0}' >/dev/null 2>&1 &
LONG_MOVE_PID=$!
sleep 1

if call_service /swiftpro/pause_motion swiftpro_resources/srv/PauseMotion '{}'; then
    pass "pause_motion (mid-flight)"
else
    fail "pause_motion" "$CALL_OUTPUT"
fi
sleep 1

if call_service /swiftpro/resume_motion swiftpro_resources/srv/ResumeMotion '{}'; then
    pass "resume_motion"
else
    fail "resume_motion" "$CALL_OUTPUT"
fi

wait $LONG_MOVE_PID 2>/dev/null || true
sleep 1

info "Starting a long move (will stop mid-flight)..."
ros2 service call /swiftpro/move_to swiftpro_resources/srv/MoveTo \
    '{x: 200.0, y: -100.0, z: 150.0, speed: 5.0}' >/tmp/stopped_move.log 2>&1 &
LONG_MOVE_PID=$!
sleep 2

if call_service /swiftpro/motion_reset swiftpro_resources/srv/MotionReset '{}'; then
    pass "motion_reset (mid-flight)"
else
    fail "motion_reset (mid-flight)" "$CALL_OUTPUT"
fi

wait $LONG_MOVE_PID 2>/dev/null || true
if grep -q "error_code=25" /tmp/stopped_move.log; then
    pass "stopped move reports E25"
else
    fail "stopped move reports E25" "$(cat /tmp/stopped_move.log)"
fi

if call_service /swiftpro/move_to swiftpro_resources/srv/MoveTo \
    '{x: 200.0, y: 0.0, z: 150.0, speed: 20.0}'; then
    pass "move_to after motion_reset"
else
    fail "move_to after motion_reset" "$CALL_OUTPUT"
fi

# ── Refused moves: no motion, a real error_code ──────────────────────────────
if call_service /swiftpro/move_to swiftpro_resources/srv/MoveTo \
    '{x: 500.0, y: 0.0, z: 150.0, speed: 20.0}' "success=False.*error_code=22"; then
    pass "unreachable move_to reports E22"
else
    fail "unreachable move_to reports E22" "$CALL_OUTPUT"
fi

if call_service /swiftpro/move_to swiftpro_resources/srv/MoveTo \
    '{x: 200.0, y: 0.0, z: 1.0, speed: 20.0}' "success=False.*error_code=-9"; then
    pass "move_to below z_min refused"
else
    fail "move_to below z_min refused" "$CALL_OUTPUT"
fi

# ── Kinematics node, if it is running: its FK against the firmware's position ─
if ros2 node list 2>/dev/null | grep -q swiftpro_kinematics; then
    sleep 1
    FW_X=$(timeout 3 ros2 topic echo /swiftpro/position --once 2>/dev/null | grep -oP '^x:\s*\K\S+')
    FK_X=$(timeout 3 ros2 topic echo /swiftpro/end_effector_position --once 2>/dev/null | grep -oP '^\s*x:\s*\K\S+')
    if awk -v a="${FW_X:-0}" -v b="${FK_X:-1000}" 'BEGIN { d = a - b; exit !(d < 1 && d > -1) }'; then
        pass "end_effector_position matches position (x: $FK_X, $FW_X)"
    else
        fail "end_effector_position matches position" "x: fk='${FK_X:-}' firmware='${FW_X:-}'"
    fi
else
    skip "end_effector_position" "swiftpro_kinematics not running"
fi

# ── Group 8: Action server ───────────────────────────────────────────────────
section "Group 8 — MoveArm action"

info "Sending action goal (will print feedback to stdout)..."
echo -e "${C_GREY}── feedback ───────────────────────────────────────────────────────────${C_RESET}"
if timeout 30 ros2 action send_goal /swiftpro/move_arm \
    swiftpro_resources/action/MoveArm \
    '{x: 250.0, y: 100.0, z: 100.0, speed: 20.0}' \
    --feedback 2>&1 | tee /tmp/action_output.log; then
    if grep -q "SUCCEEDED" /tmp/action_output.log; then
        pass "move_arm action SUCCEEDED"
    else
        fail "move_arm action" "did not reach SUCCEEDED state"
    fi
else
    fail "move_arm action" "timeout or error"
fi
echo -e "${C_GREY}───────────────────────────────────────────────────────────────────────${C_RESET}"

info "Sending an unreachable goal: must not report success..."
timeout 30 ros2 action send_goal /swiftpro/move_arm \
    swiftpro_resources/action/MoveArm \
    '{x: 500.0, y: 0.0, z: 150.0, speed: 20.0}' >/tmp/action_refused.log 2>&1 || true
if grep -q "ABORTED" /tmp/action_refused.log && grep -q "error_code: 22" /tmp/action_refused.log; then
    pass "unreachable move_arm goal ABORTED with E22"
else
    fail "unreachable move_arm goal" "$(cat /tmp/action_refused.log)"
fi

# ── Final: Reset to home ─────────────────────────────────────────────────────
section "Final — Reset"

info "Resetting to home position..."
if call_service /swiftpro/reset swiftpro_resources/srv/Reset \
    '{x: 200.0, y: 0.0, z: 150.0, speed: 20.0}'; then
    pass "reset to home"
else
    fail "reset to home" "$CALL_OUTPUT"
fi

# ── Summary ──────────────────────────────────────────────────────────────────
echo
echo -e "${C_BLUE}══════════════════════════════════════════════════════════════════════${C_RESET}"
echo -e "${C_BLUE}  Summary${C_RESET}"
echo -e "${C_BLUE}══════════════════════════════════════════════════════════════════════${C_RESET}"
echo
echo -e "  ${C_GREEN}Passed: $PASS${C_RESET}"
echo -e "  ${C_RED}Failed: $FAIL${C_RESET}"
echo -e "  ${C_YELLOW}Skipped: $SKIP${C_RESET}"
echo

if [[ $FAIL -gt 0 ]]; then
    echo -e "${C_RED}Failed tests:${C_RESET}"
    for t in "${FAILED_TESTS[@]}"; do
        echo -e "  ${C_RED}•${C_RESET} $t"
    done
    echo
    exit 1
fi

echo -e "${C_GREEN}All tests passed!${C_RESET}"
exit 0
