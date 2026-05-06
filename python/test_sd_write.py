#!/usr/bin/env python3
"""
Test for the sd_write shell command on tinySA/NanoVNA devices.

Protocol overview:
  1. Host sends:    sd_write <filename> <bytecount>\r
  2. Device echoes: sd_write <filename> <bytecount>\r\n
  3. Device sends:  <bytecount>\r\n  (confirmed byte count as text)
  4. Device sends:  ch>              (ready prompt, 4 bytes, no newline)
  5. Host sends:    exactly <bytecount> bytes of binary data
  6. Device sends:  ch>              (success acknowledgment)
     On error:     err: ...\r\n followed by ch>

Usage:
    python test_sd_write.py -p /dev/ttyACM0
    python test_sd_write.py --port COM3 --baudrate 115200
    python test_sd_write.py --port /dev/ttyACM0 --verbose
"""

import argparse
import os
import struct
import sys
import serial

PROMPT = b'ch> '
DEFAULT_TIMEOUT = 5.0  # seconds
TEST_FILENAME = 'sdwtest.bin'


# ---------------------------------------------------------------------------
# Low-level helpers
# ---------------------------------------------------------------------------

def read_until_prompt(ser, timeout=DEFAULT_TIMEOUT):
    """Read from *ser* until the 'ch> ' prompt is received.

    Returns all bytes received including the trailing prompt.
    Raises TimeoutError if no data arrives within *timeout* seconds.
    """
    ser.timeout = timeout
    buf = b''
    while True:
        chunk = ser.read(1)
        if not chunk:
            raise TimeoutError(
                f"Timeout waiting for prompt. Received so far: {buf!r}"
            )
        buf += chunk
        if buf.endswith(PROMPT):
            return buf


def flush_prompt(ser, timeout=DEFAULT_TIMEOUT):
    """Discard any pending input and wait for a clean prompt.

    Sends a bare carriage return so the device re-issues 'ch> '.
    """
    ser.reset_input_buffer()
    ser.write(b'\r')
    read_until_prompt(ser, timeout)


# ---------------------------------------------------------------------------
# SD card commands
# ---------------------------------------------------------------------------

def sd_write(ser, filename, data, timeout=DEFAULT_TIMEOUT):
    """Write *data* (bytes) to *filename* on the SD card.

    Returns True on success.
    Raises RuntimeError on a device-reported error.
    Raises TimeoutError on a communication timeout.
    """
    bytecount = len(data)
    cmd = f'sd_write {filename} {bytecount}\r'
    ser.write(cmd.encode())

    # Read echo + confirmed bytecount line + first 'ch> '
    response = read_until_prompt(ser, timeout)

    # Detect errors before the first prompt (mount failure, open failure)
    body = response[: -len(PROMPT)]
    if b'err:' in body:
        error_text = body.decode(errors='replace').strip()
        raise RuntimeError(f"sd_write setup error: {error_text}")

    # The last non-empty line before the prompt is the confirmed bytecount
    lines = [ln for ln in body.split(b'\r\n') if ln.strip()]
    if not lines:
        raise RuntimeError(
            f"sd_write: unexpected response (no bytecount line): {response!r}"
        )
    try:
        confirmed = int(lines[-1])
    except ValueError:
        raise RuntimeError(
            f"sd_write: could not parse bytecount from response: {response!r}"
        )
    if confirmed != bytecount:
        raise RuntimeError(
            f"sd_write: bytecount mismatch: sent {bytecount}, device confirmed {confirmed}"
        )

    # Send binary payload
    ser.write(data)

    # Read final 'ch> ' – this is the success acknowledgment
    final = read_until_prompt(ser, timeout)
    final_body = final[: -len(PROMPT)]
    if b'err:' in final_body:
        error_text = final_body.decode(errors='replace').strip()
        raise RuntimeError(f"sd_write transfer error: {error_text}")

    return True


def sd_read(ser, filename, timeout=DEFAULT_TIMEOUT):
    """Read *filename* from the SD card.

    Returns the file contents as bytes.
    Raises RuntimeError if the file does not exist or on a device error.
    Raises TimeoutError on a communication timeout.
    """
    cmd = f'sd_read {filename}\r'
    ser.write(cmd.encode())

    # The device echoes the command and terminates it with \r\n.
    # After that it sends 4 raw bytes (file size, little-endian uint32)
    # followed by the file data, then the 'ch> ' prompt.
    # We read the echo line first (text), then switch to binary.
    ser.timeout = timeout
    echo = b''
    while not echo.endswith(b'\r\n'):
        c = ser.read(1)
        if not c:
            raise TimeoutError("Timeout reading sd_read echo")
        echo += c

    # If the echo is followed by an error message it will be readable as text;
    # peek at the next bytes to check.
    # Read 4 bytes: either the start of an error ("err:") or the file size.
    header = ser.read(4)
    if len(header) < 4:
        raise TimeoutError("Timeout reading sd_read header")

    if header == b'err:':
        # Consume the rest of the error line and the prompt
        rest = read_until_prompt(ser, timeout)
        error_text = (b'err:' + rest[: -len(PROMPT)]).decode(errors='replace').strip()
        raise RuntimeError(f"sd_read error: {error_text}")

    # 'header' contains the first 4 bytes of the file size (little-endian uint32)
    filesize = struct.unpack('<I', header)[0]

    # Read exactly filesize bytes
    data = b''
    remaining = filesize
    while remaining > 0:
        chunk = ser.read(remaining)
        if not chunk:
            raise TimeoutError(
                f"Timeout reading sd_read data: got {len(data)}/{filesize} bytes"
            )
        data += chunk
        remaining -= len(chunk)

    # Consume the trailing 'ch> ' prompt
    read_until_prompt(ser, timeout)
    return data


def sd_delete(ser, filename, timeout=DEFAULT_TIMEOUT):
    """Delete *filename* from the SD card (best-effort, ignores errors)."""
    cmd = f'sd_delete {filename}\r'
    ser.write(cmd.encode())
    try:
        read_until_prompt(ser, timeout)
    except TimeoutError:
        pass


# ---------------------------------------------------------------------------
# Test cases
# ---------------------------------------------------------------------------

class TestResult:
    def __init__(self):
        self.passed = 0
        self.failed = 0
        self.errors = []

    def record(self, name, ok, msg=''):
        if ok:
            self.passed += 1
            print(f"  PASS  {name}")
        else:
            self.failed += 1
            self.errors.append(f"{name}: {msg}")
            print(f"  FAIL  {name}: {msg}")

    def summary(self):
        total = self.passed + self.failed
        print(f"\n{self.passed}/{total} tests passed")
        if self.errors:
            print("Failures:")
            for e in self.errors:
                print(f"  {e}")
        return self.failed == 0


def run_tests(port, baudrate, verbose):
    results = TestResult()

    ser = serial.Serial(port, baudrate, timeout=DEFAULT_TIMEOUT)
    try:
        # Allow the device a moment to settle and get a clean prompt
        flush_prompt(ser)
        if verbose:
            print(f"Connected to {port} at {baudrate} baud")

        # ------------------------------------------------------------------
        # Test 1: usage message (invalid argument count)
        # ------------------------------------------------------------------
        ser.write(b'sd_write ?\r')
        resp = read_until_prompt(ser)
        ok = b'sd_write' in resp and b'filename' in resp.lower()
        results.record("usage message on '?'", ok,
                       f"unexpected response: {resp!r}")

        # ------------------------------------------------------------------
        # Test 2: write a small binary file (< 512 bytes, single chunk)
        # ------------------------------------------------------------------
        small_data = bytes(range(256))
        try:
            sd_write(ser, TEST_FILENAME, small_data)
            results.record("write small file (256 bytes)", True)
        except Exception as exc:
            results.record("write small file (256 bytes)", False, str(exc))

        # ------------------------------------------------------------------
        # Test 3: verify small file content via sd_read
        # ------------------------------------------------------------------
        try:
            read_back = sd_read(ser, TEST_FILENAME)
            ok = read_back == small_data
            results.record(
                "read back small file matches written data",
                ok,
                f"length {len(read_back)} vs {len(small_data)}" if not ok else '',
            )
        except Exception as exc:
            results.record("read back small file matches written data", False, str(exc))

        # ------------------------------------------------------------------
        # Test 4: overwrite with a larger file (> 512 bytes, multiple chunks)
        # ------------------------------------------------------------------
        large_data = bytes(i & 0xFF for i in range(1500))
        try:
            sd_write(ser, TEST_FILENAME, large_data)
            results.record("write large file (1500 bytes, multi-chunk)", True)
        except Exception as exc:
            results.record("write large file (1500 bytes, multi-chunk)", False, str(exc))

        # ------------------------------------------------------------------
        # Test 5: verify large file content via sd_read
        # ------------------------------------------------------------------
        try:
            read_back = sd_read(ser, TEST_FILENAME)
            ok = read_back == large_data
            results.record(
                "read back large file matches written data",
                ok,
                f"length {len(read_back)} vs {len(large_data)}" if not ok else '',
            )
        except Exception as exc:
            results.record("read back large file matches written data", False, str(exc))

        # ------------------------------------------------------------------
        # Test 6: write a file with exactly 512 bytes (boundary condition)
        # ------------------------------------------------------------------
        boundary_data = os.urandom(512)
        try:
            sd_write(ser, TEST_FILENAME, boundary_data)
            results.record("write exactly 512 bytes (chunk boundary)", True)
        except Exception as exc:
            results.record("write exactly 512 bytes (chunk boundary)", False, str(exc))

        # ------------------------------------------------------------------
        # Test 7: verify 512-byte boundary file content
        # ------------------------------------------------------------------
        try:
            read_back = sd_read(ser, TEST_FILENAME)
            ok = read_back == boundary_data
            results.record(
                "read back 512-byte file matches written data",
                ok,
                f"length {len(read_back)} vs {len(boundary_data)}" if not ok else '',
            )
        except Exception as exc:
            results.record("read back 512-byte file matches written data", False, str(exc))

        # ------------------------------------------------------------------
        # Test 8: write an empty file (zero bytes)
        # ------------------------------------------------------------------
        try:
            sd_write(ser, TEST_FILENAME, b'')
            results.record("write empty file (0 bytes)", True)
        except Exception as exc:
            results.record("write empty file (0 bytes)", False, str(exc))

        # ------------------------------------------------------------------
        # Test 9: verify empty file
        # ------------------------------------------------------------------
        try:
            read_back = sd_read(ser, TEST_FILENAME)
            ok = read_back == b''
            results.record(
                "read back empty file is empty",
                ok,
                f"got {len(read_back)} bytes" if not ok else '',
            )
        except Exception as exc:
            results.record("read back empty file is empty", False, str(exc))

    finally:
        # Best-effort cleanup: delete the test file
        try:
            sd_delete(ser, TEST_FILENAME)
            if verbose:
                print(f"\nCleaned up: deleted {TEST_FILENAME}")
        except Exception:
            pass
        ser.close()

    return results.summary()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Test sd_write command on a tinySA/NanoVNA device"
    )
    parser.add_argument(
        '-p', '--port',
        required=True,
        help="Serial port the device is connected to (e.g. /dev/ttyACM0 or COM3)",
    )
    parser.add_argument(
        '-b', '--baudrate',
        type=int,
        default=115200,
        help="Serial baud rate (default: 115200)",
    )
    parser.add_argument(
        '-v', '--verbose',
        action='store_true',
        help="Enable verbose output",
    )
    args = parser.parse_args()

    print(f"Running sd_write tests on {args.port}...")
    success = run_tests(args.port, args.baudrate, args.verbose)
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
