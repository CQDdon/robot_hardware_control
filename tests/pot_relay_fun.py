#!/usr/bin/env python3

import serial
import socket
import struct
import time


# ============================================================
# CONFIG
# ============================================================

# ---------- Modbus Analog Input ----------
SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 9600

INPUT_SLAVE_ID = 1

# Physical AI8 => register 0x0007
AI_REGISTER = 0x0007

# AI8 configuration register
AI_MODE_REGISTER = 0x1007

# 0 = 0-5 V mode
AI_MODE_0_5V = 0

ADC_MAX = 4095
INPUT_VOLTAGE_MAX = 5.0


# ---------- CAN / ESP32 relay controller ----------
CAN_INTERFACE = "can0"

# OUTPUT_COMMAND
CAN_COMMAND_ID = 0x500

# Firmware protocol
PROTOCOL_VERSION = 0x01


# ---------- Demo behaviour ----------

# Potentiometer below 5%:
# stop sequence + turn all relays OFF
DEADZONE = 0.05

# Slowest:
# one relay toggle every 2.5 seconds
MAX_INTERVAL = 2.5

# Fastest:
# one relay toggle every 0.5 seconds
#
# Don't make this too fast; these are mechanical relays.
MIN_INTERVAL = 0.2

# Poll potentiometer every 50 ms
POLL_PERIOD = 0.05


# ============================================================
# MODBUS CRC
# ============================================================

def crc16(data: bytes) -> int:
    crc = 0xFFFF

    for byte in data:
        crc ^= byte

        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1

    return crc


def make_modbus_frame(payload: bytes) -> bytes:
    crc = crc16(payload)

    return payload + bytes([
        crc & 0xFF,
        (crc >> 8) & 0xFF
    ])


# ============================================================
# MODBUS TRANSACTION
# ============================================================

def modbus_transaction(
    ser: serial.Serial,
    payload: bytes,
    expected_length: int,
) -> bytes:

    request = make_modbus_frame(payload)

    ser.reset_input_buffer()

    ser.write(request)
    ser.flush()

    response = ser.read(expected_length)

    if len(response) != expected_length:
        raise TimeoutError(
            f"Modbus timeout: expected {expected_length} bytes, "
            f"received {len(response)} bytes\n"
            f"TX: {request.hex(' ').upper()}\n"
            f"RX: {response.hex(' ').upper()}"
        )

    # Check CRC
    received_crc = response[-2] | (response[-1] << 8)
    calculated_crc = crc16(response[:-2])

    if received_crc != calculated_crc:
        raise RuntimeError(
            f"Modbus CRC error: "
            f"RX=0x{received_crc:04X}, "
            f"CALC=0x{calculated_crc:04X}"
        )

    # Modbus exception response
    if response[1] & 0x80:
        raise RuntimeError(
            f"Modbus exception: "
            f"function=0x{response[1]:02X}, "
            f"code=0x{response[2]:02X}"
        )

    # Small RTU gap
    time.sleep(0.005)

    return response


# ============================================================
# ANALOG INPUT
# ============================================================

def configure_ai8_voltage(ser: serial.Serial):
    """
    Input slave ID 1

    Function 06
    Register 0x1007 = 0

    => AI8 = 0-5 V mode
    """

    payload = bytes([
        INPUT_SLAVE_ID,
        0x06,

        (AI_MODE_REGISTER >> 8) & 0xFF,
        AI_MODE_REGISTER & 0xFF,

        0x00,
        AI_MODE_0_5V,
    ])

    response = modbus_transaction(
        ser,
        payload,
        expected_length=8,
    )

    if response[:-2] != payload:
        raise RuntimeError("AI8 configuration echo mismatch")


def read_ai8_raw(ser: serial.Serial) -> int:
    """
    Read AI8.

    Slave ID 1
    Function 04
    Register 0x0007
    """

    payload = bytes([
        INPUT_SLAVE_ID,
        0x04,

        (AI_REGISTER >> 8) & 0xFF,
        AI_REGISTER & 0xFF,

        0x00,
        0x01,
    ])

    response = modbus_transaction(
        ser,
        payload,
        expected_length=7,
    )

    if response[0] != INPUT_SLAVE_ID:
        raise RuntimeError(
            f"Unexpected Modbus slave ID: {response[0]}"
        )

    if response[1] != 0x04:
        raise RuntimeError(
            f"Unexpected Modbus function: 0x{response[1]:02X}"
        )

    if response[2] != 2:
        raise RuntimeError(
            f"Unexpected Modbus byte count: {response[2]}"
        )

    raw = (response[3] << 8) | response[4]

    # Clamp for demo
    return max(0, min(ADC_MAX, raw))


# ============================================================
# SOCKETCAN
# ============================================================

def open_can_socket():
    sock = socket.socket(
        socket.PF_CAN,
        socket.SOCK_RAW,
        socket.CAN_RAW,
    )

    sock.bind((CAN_INTERFACE,))

    return sock


def send_output_command(
    can_sock,
    sequence: int,
    output_mask: int,
):
    """
    CAN OUTPUT_COMMAND

    ID  : 0x500
    DLC : 4

    Byte 0 : sequence
    Byte 1 : output_mask
    Byte 2 : protocol version
    Byte 3 : reserved
    """

    data = bytes([
        sequence & 0xFF,
        output_mask & 0xFF,
        PROTOCOL_VERSION,
        0x00,
    ])

    # struct can_frame:
    #
    # uint32_t can_id
    # uint8_t  can_dlc
    # padding[3]
    # uint8_t  data[8]

    frame = struct.pack(
        "=IB3x8s",
        CAN_COMMAND_ID,
        len(data),
        data.ljust(8, b"\x00"),
    )

    can_sock.send(frame)


# ============================================================
# POT CONVERSION
# ============================================================

def raw_to_percent(raw: int) -> float:
    return raw / ADC_MAX


def raw_to_voltage(raw: int) -> float:
    return raw / ADC_MAX * INPUT_VOLTAGE_MAX


def percent_to_interval(percent: float) -> float:
    """
    Pot position -> relay toggle interval.

    5%      -> ~2.5 sec
    100%    -> 0.5 sec
    """

    normalized = (
        percent - DEADZONE
    ) / (
        1.0 - DEADZONE
    )

    normalized = max(
        0.0,
        min(1.0, normalized)
    )

    return (
        MAX_INTERVAL
        - normalized
        * (MAX_INTERVAL - MIN_INTERVAL)
    )


# ============================================================
# DISPLAY
# ============================================================

def relay_string(mask: int) -> str:
    """
    Display relay state:

    10010000 etc.

    Left-to-right = R1 ... R8
    """

    return "".join(
        "1" if mask & (1 << i) else "0"
        for i in range(8)
    )


# ============================================================
# MAIN
# ============================================================

def main():

    print("============================================")
    print("      POT -> 8 RELAY SEQUENCE DEMO")
    print("============================================")
    print()
    print(f"Modbus port : {SERIAL_PORT}")
    print(f"Input       : Slave {INPUT_SLAVE_ID}, AI8")
    print(f"CAN         : {CAN_INTERFACE}")
    print(f"CAN ID      : 0x{CAN_COMMAND_ID:03X}")
    print()
    print("Pot low  -> slow")
    print("Pot high -> fast")
    print("Pot < 5% -> all relays OFF")
    print()
    print("Sequence:")
    print("R1 -> R2 -> ... -> R8 -> R1 ...")
    print()
    print("Ctrl+C to stop")
    print()

    # ----------------------------------------
    # State
    # ----------------------------------------

    sequence = 0

    # 8-bit relay state
    output_mask = 0x00

    # Next relay to toggle:
    # 0 = R1
    # ...
    # 7 = R8
    current_relay = 0

    last_toggle = time.monotonic()
    last_print = 0.0

    # ----------------------------------------
    # Open serial
    # ----------------------------------------

    with serial.Serial(
        port=SERIAL_PORT,
        baudrate=BAUDRATE,
        bytesize=8,
        parity=serial.PARITY_NONE,
        stopbits=1,
        timeout=0.5,
    ) as ser:

        # ------------------------------------
        # Open CAN
        # ------------------------------------

        can_sock = open_can_socket()

        try:

            # ==================================
            # Configure AI8 = 0-5V
            # ==================================

            configure_ai8_voltage(ser)

            print("AI8 configured to 0-5 V")
            print()

            # ==================================
            # Start from all OFF
            # ==================================

            send_output_command(
                can_sock,
                sequence,
                0x00,
            )

            sequence = (sequence + 1) & 0xFF

            print("All relays OFF")
            print()

            # ==================================
            # Main loop
            # ==================================

            while True:

                loop_start = time.monotonic()

                # ------------------------------
                # Read potentiometer
                # ------------------------------

                raw = read_ai8_raw(ser)

                percent = raw_to_percent(raw)
                voltage = raw_to_voltage(raw)

                now = time.monotonic()

                # ====================================================
                # DEADZONE
                # ====================================================

                if percent < DEADZONE:

                    interval = None

                    # If anything is ON, turn everything OFF
                    if output_mask != 0x00:

                        output_mask = 0x00

                        send_output_command(
                            can_sock,
                            sequence,
                            output_mask,
                        )

                        sequence = (sequence + 1) & 0xFF

                    # Restart sequence from R1
                    current_relay = 0

                    # Prevent instant toggle when leaving deadzone
                    last_toggle = now

                # ====================================================
                # RUN SEQUENCE
                # ====================================================

                else:

                    interval = percent_to_interval(
                        percent
                    )

                    if now - last_toggle >= interval:

                        # --------------------------------
                        # Toggle current relay
                        # --------------------------------

                        output_mask ^= (
                            1 << current_relay
                        )

                        # --------------------------------
                        # Send full state to ESP32
                        # --------------------------------

                        send_output_command(
                            can_sock,
                            sequence,
                            output_mask,
                        )

                        sequence = (
                            sequence + 1
                        ) & 0xFF

                        # --------------------------------
                        # Move to next relay
                        # --------------------------------

                        current_relay = (
                            current_relay + 1
                        ) % 8

                        last_toggle = now

                # ====================================================
                # DISPLAY
                # ====================================================

                if now - last_print >= 0.1:

                    states = relay_string(
                        output_mask
                    )

                    if interval is None:

                        print(
                            f"\r"
                            f"AI8 "
                            f"{voltage:4.2f} V | "
                            f"{percent * 100:5.1f}% | "
                            f"STOP | "
                            f"R[{states}]      ",
                            end="",
                            flush=True,
                        )

                    else:

                        print(
                            f"\r"
                            f"AI8 "
                            f"{voltage:4.2f} V | "
                            f"{percent * 100:5.1f}% | "
                            f"{interval:4.2f}s | "
                            f"R[{states}] | "
                            f"next R{current_relay + 1}    ",
                            end="",
                            flush=True,
                        )

                    last_print = now

                # ====================================================
                # LOOP RATE
                # ====================================================

                elapsed = (
                    time.monotonic()
                    - loop_start
                )

                remaining = (
                    POLL_PERIOD
                    - elapsed
                )

                if remaining > 0:
                    time.sleep(remaining)

        # ======================================
        # CTRL+C
        # ======================================

        except KeyboardInterrupt:

            print()
            print()
            print("Stopping...")

        # ======================================
        # ERROR
        # ======================================

        except Exception as exc:

            print()
            print()
            print(f"ERROR: {exc}")

        # ======================================
        # ALWAYS SAFE OFF
        # ======================================

        finally:

            try:

                send_output_command(
                    can_sock,
                    sequence,
                    0x00,
                )

                print("All relays -> OFF")

            except Exception as exc:

                print(
                    f"WARNING: could not turn "
                    f"relays OFF: {exc}"
                )

            can_sock.close()


# ============================================================
# ENTRY
# ============================================================

if __name__ == "__main__":
    main()