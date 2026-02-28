from __future__ import annotations

import argparse
import re
import subprocess
import sys
from typing import Iterable

try:
    from serial.tools import list_ports
except Exception as exc:  # pragma: no cover - dependency availability varies
    list_ports = None
    SERIAL_IMPORT_ERROR = exc
else:
    SERIAL_IMPORT_ERROR = None


DEFAULT_XARM_IP = "192.168.0.203"
DEFAULT_KEYWORDS = (
    "rs485",
    "485",
    "ftdi",
    "ch340",
    "cp210",
    "pl2303",
    "usb serial",
    "uart",
)

KNOWN_RS485_ADAPTER_IDS = {
    (0x1A86, 0x7523),  # QinHeng/WCH CH340
    (0x10C4, 0xEA60),  # Silicon Labs CP210x
    (0x0403, 0x6001),  # FTDI FT232
    (0x067B, 0x2303),  # Prolific PL2303
}


def ping_host(ip: str, timeout_seconds: int, count: int) -> tuple[bool, str]:
    cmd = ["ping", "-c", str(count), "-W", str(timeout_seconds), ip]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    output = (proc.stdout + proc.stderr).strip()
    return proc.returncode == 0, output


def _port_blob(port: object) -> str:
    fields = [
        getattr(port, "device", ""),
        getattr(port, "description", ""),
        getattr(port, "manufacturer", ""),
        getattr(port, "product", ""),
        getattr(port, "hwid", ""),
    ]
    return " | ".join([f for f in fields if f]).lower()


def _is_usb_serial(port: object) -> bool:
    device = getattr(port, "device", "") or ""
    blob = _port_blob(port)
    return (
        device.startswith("/dev/ttyUSB")
        or device.startswith("/dev/ttyACM")
        or "usb" in blob
    )


def find_rs485_ports(keywords: Iterable[str]) -> tuple[list[object], list[object]]:
    if list_ports is None:
        return [], []

    all_ports = list(list_ports.comports())
    usb_ports = [p for p in all_ports if _is_usb_serial(p)]
    lowered = tuple(k.lower() for k in keywords)
    matches = []
    for port in usb_ports:
        text_match = any(k in _port_blob(port) for k in lowered)
        id_match = (getattr(port, "vid", None), getattr(port, "pid", None)) in KNOWN_RS485_ADAPTER_IDS
        if text_match or id_match:
            matches.append(port)
    return matches, usb_ports


def _print_port(port: object) -> None:
    device = getattr(port, "device", "unknown")
    description = getattr(port, "description", "n/a")
    manufacturer = getattr(port, "manufacturer", "n/a")
    product = getattr(port, "product", "n/a")
    hwid = getattr(port, "hwid", "n/a")
    print(f"- {device}")
    print(f"  description : {description}")
    print(f"  manufacturer: {manufacturer}")
    print(f"  product     : {product}")
    print(f"  hwid        : {hwid}")


def _print_ping_summary(ping_output: str) -> None:
    summary_match = re.search(r"(\d+)\s+packets transmitted,\s+(\d+)\s+received", ping_output)
    rtt_match = re.search(r"min/avg/max(?:/mdev)?\s*=\s*([0-9./]+)\s*ms", ping_output)

    if summary_match:
        sent, received = summary_match.groups()
        print(f"  packets: sent={sent}, received={received}")
    if rtt_match:
        print(f"  rtt    : {rtt_match.group(1)} ms")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Scan USB serial ports for RS485 adapters and verify xArm network reachability."
    )
    parser.add_argument("--xarm-ip", default=DEFAULT_XARM_IP, help=f"xArm IP (default: {DEFAULT_XARM_IP})")
    parser.add_argument("--timeout", type=int, default=1, help="Ping timeout in seconds per packet")
    parser.add_argument("--count", type=int, default=2, help="Number of ping packets")
    parser.add_argument(
        "--keywords",
        nargs="+",
        default=list(DEFAULT_KEYWORDS),
        help="Keywords used to identify likely RS485 converters",
    )
    parser.add_argument(
        "--show-all-usb-serial",
        action="store_true",
        help="Print all detected USB serial ports in addition to matches",
    )
    args = parser.parse_args()

    print("=== Hardware Verification ===")
    print(f"xArm target IP: {args.xarm_ip}")
    print(f"RS485 keywords: {', '.join(args.keywords)}")
    print("")

    if list_ports is None:
        print("[ERROR] pyserial is unavailable.")
        print(f"Import error: {SERIAL_IMPORT_ERROR}")
        print("Install with: pip install pyserial")
        return 2

    rs485_matches, usb_serial_ports = find_rs485_ports(args.keywords)

    print("USB serial scan:")
    if rs485_matches:
        print(f"[OK] Found {len(rs485_matches)} likely RS485 converter port(s):")
        for port in rs485_matches:
            _print_port(port)
    else:
        print("[WARN] No likely RS485 converter detected.")

    if args.show_all_usb_serial:
        print("")
        print(f"All USB serial ports ({len(usb_serial_ports)}):")
        for port in usb_serial_ports:
            _print_port(port)

    print("")
    print("xArm network check:")
    ping_ok, ping_output = ping_host(args.xarm_ip, args.timeout, args.count)
    if ping_ok:
        print(f"[OK] Ping to {args.xarm_ip} succeeded.")
        _print_ping_summary(ping_output)
    else:
        print(f"[FAIL] Ping to {args.xarm_ip} failed.")
        _print_ping_summary(ping_output)

    print("")
    if rs485_matches and ping_ok:
        print("Overall status: PASS")
        return 0

    if not rs485_matches and not ping_ok:
        print("Overall status: FAIL (RS485 not found + xArm unreachable)")
    elif not rs485_matches:
        print("Overall status: PARTIAL (xArm reachable, RS485 not found)")
    else:
        print("Overall status: PARTIAL (RS485 found, xArm unreachable)")
    return 1


if __name__ == "__main__":
    sys.exit(main())
