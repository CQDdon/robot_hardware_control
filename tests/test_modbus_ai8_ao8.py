#!/usr/bin/env python3

import serial
import time


# ============================================================
# CONFIG
# ============================================================

PORT = "/dev/ttyUSB0"
BAUDRATE = 9600

INPUT_ID = 1
OUTPUT_ID = 2

# Physical channel 8 -> Modbus register offset 7
AI_REGISTER = 0x0007
AI_MODE_REGISTER = 0x1007

AO_REGISTER = 0x0007

# AI mode 0 = 0-5 V
AI_MODE_0_5V = 0

# Input ADC
ADC_MAX = 4095
INPUT_VOLTAGE_MAX = 5.0

# Output board: 0-20 mA
OUTPUT_MAX_UA = 20000

# Bench-test loop rate
LOOP_PERIOD = 0.1  # 10 Hz


# ============================================================
# MODBUS RTU
# ============================================================

def crc16(data: bytes) -> int:
    """Calculate Modbus RTU CRC16."""
    crc = 0xFFFF

    for byte in data:
        crc ^= byte

        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1

    return crc


def make_frame(payload: bytes) -> bytes:
    """Append Modbus CRC, low byte first."""
    crc = crc16(payload)

    return payload + bytes([
        crc & 0xFF,
        (crc >> 8) & 0xFF
    ])


def check_response_crc(response: bytes):
    if len(response) < 4:
        raise RuntimeError(
            f"Response too short: {response.hex(' ').upper()}"
        )

    received_crc = response[-2] | (response[-1] << 8)
    calculated_crc = crc16(response[:-2])

    if received_crc != calculated_crc:
        raise RuntimeError(
            f"CRC error: RX=0x{received_crc:04X}, "
            f"CALC=0x{calculated_crc:04X}"
        )


def transaction(
    ser: serial.Serial,
    payload: bytes,
    expected_length: int
) -> bytes:

    request = make_frame(payload)

    ser.reset_input_buffer()

    ser.write(request)
    ser.flush()

    response = ser.read(expected_length)

    if len(response) != expected_length:
        raise TimeoutError(
            f"Timeout: expected {expected_length} bytes, "
            f"received {len(response)} bytes\n"
            f"TX: {request.hex(' ').upper()}\n"
            f"RX: {response.hex(' ').upper()}"
        )

    check_response_crc(response)

    # Modbus exception response
    if response[1] & 0x80:
        exception_code = response[2]

        raise RuntimeError(
            f"Modbus exception: "
            f"function=0x{response[1]:02X}, "
            f"code=0x{exception_code:02X}"
        )

    # Small RTU inter-frame gap
    time.sleep(0.005)

    return response


# ============================================================
# INPUT BOARD - ID 1
# ============================================================

def configure_ai8_0_5v(ser: serial.Serial):
    """
    Input ID 1
    Function 06
    Register 0x1007 = 0
    -> AI8 = 0-5 V mode
    """

    payload = bytes([
        INPUT_ID,
        0x06,

        (AI_MODE_REGISTER >> 8) & 0xFF,
        AI_MODE_REGISTER & 0xFF,

        0x00,
        AI_MODE_0_5V,
    ])

    response = transaction(
        ser,
        payload,
        expected_length=8
    )

    if response[:-2] != payload:
        raise RuntimeError("AI8 configuration echo mismatch")

    print(
        f"AI8 configured: 0-5 V "
        f"(register 0x{AI_MODE_REGISTER:04X})"
    )


def read_ai8_raw(ser: serial.Serial) -> int:
    """
    Input ID 1
    Function 04 - Read Input Register
    Register 0x0007 -> AI8
    """

    payload = bytes([
        INPUT_ID,
        0x04,

        (AI_REGISTER >> 8) & 0xFF,
        AI_REGISTER & 0xFF,

        0x00,
        0x01,
    ])

    response = transaction(
        ser,
        payload,
        expected_length=7
    )

    if response[0] != INPUT_ID:
        raise RuntimeError(
            f"Unexpected slave ID: {response[0]}"
        )

    if response[1] != 0x04:
        raise RuntimeError(
            f"Unexpected function: 0x{response[1]:02X}"
        )

    if response[2] != 2:
        raise RuntimeError(
            f"Unexpected byte count: {response[2]}"
        )

    raw = (response[3] << 8) | response[4]

    return raw


# ============================================================
# OUTPUT BOARD - ID 2
# ============================================================

def write_ao8(ser: serial.Serial, output_uA: int):
    """
    Output ID 2
    Function 06
    Register 0x0007 -> AO8

    value:
        0       -> 0 mA
        10000   -> 10 mA
        20000   -> 20 mA
    """

    output_uA = max(
        0,
        min(OUTPUT_MAX_UA, int(output_uA))
    )

    payload = bytes([
        OUTPUT_ID,
        0x06,

        (AO_REGISTER >> 8) & 0xFF,
        AO_REGISTER & 0xFF,

        (output_uA >> 8) & 0xFF,
        output_uA & 0xFF,
    ])

    response = transaction(
        ser,
        payload,
        expected_length=8
    )

    if response[:-2] != payload:
        raise RuntimeError("AO8 write echo mismatch")


# ============================================================
# CONVERSION
# ============================================================

def adc_to_voltage(raw: int) -> float:
    raw = max(0, min(ADC_MAX, raw))

    return (
        raw / ADC_MAX
        * INPUT_VOLTAGE_MAX
    )


def adc_to_output_uA(raw: int) -> int:
    """
    Linear mapping:

        AI8 0 V -> AO8  0 mA
        AI8 1 V -> AO8  4 mA
        AI8 2.5V -> AO8 10 mA
        AI8 5 V -> AO8 20 mA
    """

    raw = max(0, min(ADC_MAX, raw))

    return round(
        raw / ADC_MAX
        * OUTPUT_MAX_UA
    )


# ============================================================
# MAIN
# ============================================================

def main():

    print("==========================================")
    print(" Waveshare AI8 -> AO8 Modbus RTU test")
    print("==========================================")
    print(f"Port      : {PORT}")
    print(f"Input ID  : {INPUT_ID}")
    print(f"Output ID : {OUTPUT_ID}")
    print("AI channel: 8 (0-5 V)")
    print("AO channel: 8 (0-20 mA)")
    print()

    with serial.Serial(
        port=PORT,
        baudrate=BAUDRATE,
        bytesize=8,
        parity=serial.PARITY_NONE,
        stopbits=1,
        timeout=0.5,
    ) as ser:

        time.sleep(0.2)

        # Configure AI8
        configure_ai8_0_5v(ser)

        print()
        print("Running...")
        print("Ctrl+C to stop")
        print()

        try:

            while True:

                start = time.monotonic()

                # ------------------------------
                # Read Input AI8
                # ------------------------------

                raw = read_ai8_raw(ser)

                voltage = adc_to_voltage(raw)

                # ------------------------------
                # Map AI8 -> AO8
                # ------------------------------

                output_uA = adc_to_output_uA(raw)

                # ------------------------------
                # Write Output AO8
                # ------------------------------

                write_ao8(
                    ser,
                    output_uA
                )

                print(
                    f"AI8: "
                    f"raw={raw:4d}  "
                    f"{voltage:5.3f} V"
                    f"   ->   "
                    f"AO8: "
                    f"{output_uA / 1000:5.2f} mA"
                )

                # Maintain ~10 Hz loop
                elapsed = time.monotonic() - start
                remaining = LOOP_PERIOD - elapsed

                if remaining > 0:
                    time.sleep(remaining)

        except KeyboardInterrupt:

            print()
            print("Stopping...")

            # Safe output
            try:
                write_ao8(ser, 0)
                print("AO8 -> 0 mA")
            except Exception as e:
                print(
                    f"WARNING: cannot set AO8 to 0 mA: {e}"
                )

        except Exception:
            # Try safe output before propagating error
            try:
                write_ao8(ser, 0)
            except Exception:
                pass

            raise


if __name__ == "__main__":
    main()