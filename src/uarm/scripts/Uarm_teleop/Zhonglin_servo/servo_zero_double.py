import argparse
import re
import time

import numpy as np
import serial


init_qpos = np.array([0, 0, 0, 0, 0, 0, 0, 0])  # Initial joint angles in degrees
init_qpos = np.radians(init_qpos)


SERIAL_PORT_A = "/dev/ttyUSB0"
SERIAL_PORT_B = "/dev/ttyUSB1"
BAUDRATE = 115200
SERVO_IDS_A = list(range(0, 4))
SERVO_IDS_B = list(range(4, 8))
SERVO_IDS = SERVO_IDS_A + SERVO_IDS_B
SERVO_SIGNS = np.array([1.0, 1.0, -1.0, -1.0, -1.0, 1.0, -1.0, 1.0])


def fmt_value(value):
    return "None" if value is None else f"{value:.5g}"


def fmt_values(values):
    return "[" + ", ".join(fmt_value(value) for value in values) + "]"


def send_command(ser, cmd, delay=0.03, timeout=0.1):
    ser.reset_input_buffer()
    ser.write(cmd.encode("ascii"))
    ser.flush()
    time.sleep(delay)
    deadline = time.monotonic() + timeout
    response = b""
    while time.monotonic() < deadline:
        chunk = ser.read_all()
        if chunk:
            response += chunk
            if b"!" in response or re.search(rb"P\d{4}", response):
                break
        time.sleep(0.002)
    return response.decode("ascii", errors="ignore")


def pwm_to_angle(response_str, pwm_min=500, pwm_max=2500, angle_range=270):
    match = re.search(r"P(\d{4})", response_str)
    if not match:
        return None
    pwm_val = int(match.group(1))
    pwm_span = pwm_max - pwm_min
    angle = (pwm_val - pwm_min) / pwm_span * angle_range
    return angle


def angle_to_gripper(angle_deg, angle_range=270, pos_min=50, pos_max=730):
    """
    Map servo angle (degrees) to gripper position.

    Parameters:
    - angle_deg: Servo angle in degrees
    - angle_range: Maximum servo angle (default 270)
    - pos_min: Gripper closed position (default 50)
    - pos_max: Gripper open position (default 730)

    Returns:
    - gripper position (integer)
    """
    ratio = (angle_deg / angle_range) * 3
    position = pos_min + (pos_max - pos_min) * ratio
    return int(np.clip(position, pos_min, pos_max))


def servo_bus(ser_a, ser_b, servo_id):
    return ser_a if servo_id in SERVO_IDS_A else ser_b


def unlock_servos(ser_a, ser_b, delay, timeout):
    for ser, ids, name in ((ser_a, SERVO_IDS_A, "A"), (ser_b, SERVO_IDS_B, "B")):
        version = send_command(ser, "#000PVER!", delay=delay, timeout=timeout)
        print(f"Serial {name} version response: {version.strip()}")
        send_command(ser, "#000PCSK!", delay=delay, timeout=timeout)
        for servo_id in ids:
            response = send_command(ser, f"#{servo_id:03d}PULK!", delay=delay, timeout=timeout)
            print(f"Serial {name} servo {servo_id} torque released: {response.strip()}")


def read_angles(ser_a, ser_b, angle_pos, zero_angles, arm_pos, delay, timeout):
    for servo_id in SERVO_IDS:
        ser = servo_bus(ser_a, ser_b, servo_id)
        response = send_command(ser, f"#{servo_id:03d}PRAD!", delay=delay, timeout=timeout)
        angle = pwm_to_angle(response.strip())
        angle_pos[servo_id] = angle
        if angle is None:
            print(f"Servo {servo_id} response error: {response.strip()}")
            continue

        init_offset = init_qpos[servo_id] if servo_id < len(init_qpos) else 0.0
        angle_offset = SERVO_SIGNS[servo_id] * (angle - zero_angles[servo_id]) + init_offset
        arm_pos[servo_id] = np.radians(angle_offset)


def build_arg_parser():
    parser = argparse.ArgumentParser(description="Read and zero Zhonglin servos from two serial ports.")
    parser.add_argument("--port-a", default=SERIAL_PORT_A, help="Serial port for servo ids 0-3")
    parser.add_argument("--port-b", default=SERIAL_PORT_B, help="Serial port for servo ids 4-7")
    parser.add_argument("--baudrate", type=int, default=BAUDRATE)
    parser.add_argument("--timeout", type=float, default=0.1)
    parser.add_argument("--delay", type=float, default=0.03)
    parser.add_argument("--scan", action="store_true", help="Scan servo ids 0-7 on both serial ports, then exit")
    return parser


def scan_servos(ser_a, ser_b, delay, timeout):
    for ser, name in ((ser_a, "A"), (ser_b, "B")):
        version = send_command(ser, "#000PVER!", delay=delay, timeout=timeout)
        print(f"Serial {name} version response: {version.strip()}")
        send_command(ser, "#000PCSK!", delay=delay, timeout=timeout)
        for servo_id in range(8):
            response = send_command(ser, f"#{servo_id:03d}PRAD!", delay=delay, timeout=timeout)
            angle = pwm_to_angle(response.strip())
            signed_angle = None if angle is None else SERVO_SIGNS[servo_id] * angle
            print(
                f"Serial {name} servo {servo_id}: "
                f"angle={fmt_value(angle)} "
                f"signed_angle={fmt_value(signed_angle)} "
                f"response={response.strip()}"
            )


def main():
    args = build_arg_parser().parse_args()
    arm_pos = [0.0] * len(SERVO_IDS)
    angle_pos = [0.0] * len(SERVO_IDS)
    zero_angles = [0.0] * len(SERVO_IDS)

    with serial.Serial(args.port_a, args.baudrate, timeout=args.timeout) as ser_a, \
            serial.Serial(args.port_b, args.baudrate, timeout=args.timeout) as ser_b:
        print(f"Serial port A opened: {args.port_a} ids={SERVO_IDS_A}")
        print(f"Serial port B opened: {args.port_b} ids={SERVO_IDS_B}")
        if args.scan:
            scan_servos(ser_a, ser_b, args.delay, args.timeout)
            return

        unlock_servos(ser_a, ser_b, args.delay, args.timeout)

        while True:
            read_angles(ser_a, ser_b, angle_pos, zero_angles, arm_pos, args.delay, args.timeout)
            print(fmt_values(angle_pos))


if __name__ == "__main__":
    main()
