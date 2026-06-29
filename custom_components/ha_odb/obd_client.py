"""BLE GATT ELM327 communication client for OBD-II adapters.

Connects to an ELM327-compatible Bluetooth LE OBD-II adapter (e.g. Vgate iCar Pro,
G6) via Home Assistant's Bluetooth integration, sends AT/OBD commands over the
GATT write characteristic, and receives responses via notifications.
"""

from __future__ import annotations

import asyncio
import logging
import time
from typing import Optional

from homeassistant.components.bluetooth import async_ble_device_from_address
from homeassistant.core import HomeAssistant

from .const import (
    BLE_COMMAND_TIMEOUT,
    BLE_CONNECT_TIMEOUT,
    DISCOVERY_SERVICE_UUIDS,
    ELM_CR,
    ELM_HEADER_BMS,
    ELM_HEADER_VCU,
    ELM_INIT_G6,
    ELM_INIT_SAE,
    ELM_PROMPT,
    MAX_COMMAND_RETRIES,
    OBD_CHAR_NOTIFY_UUID,
    OBD_CHAR_WRITE_UUID,
    OBD_SERVICE_UUID,
    VGT_CHAR_UUID,
    VGT_SERVICE_UUID,
)

_LOGGER = logging.getLogger(__package__)

# Try importing bleak; HA bundles it but it may not be available in older installs
try:
    from bleak import BleakClient, BleakScanner  # noqa: F811
    from bleak.exc import BleakDeviceNotFoundError, BleakError
    HAS_BLEAK = True
except ImportError:
    HAS_BLEAK = False
    BleakClient = None  # type: ignore[assignment]
    BleakScanner = None  # type: ignore[assignment]
    BleakDeviceNotFoundError = OBDConnectionError  # type: ignore[assignment,misc]
    BleakError = OBDConnectionError  # type: ignore[assignment,misc]


class OBDClientError(Exception):
    """Base exception for OBD client errors."""


class OBDConnectionError(OBDClientError):
    """BLE connection failed."""


class OBDCommandError(OBDClientError):
    """ELM327 command failed or timed out."""


class OBDClient:
    """BLE GATT client for ELM327 OBD-II adapters."""

    def __init__(
        self,
        hass: HomeAssistant,
        address: str,
        vehicle_profile: str = "sae",  # "sae" or "xpeng_g6"
    ) -> None:
        """Initialise the OBD client.

        Args:
            hass: Home Assistant instance.
            address: BLE MAC address of the OBD-II adapter.
            vehicle_profile: PID profile to use ("sae" or "xpeng_g6").
        """
        self._hass = hass
        self._address = address
        self._profile = vehicle_profile
        self._client: Optional[BleakClient] = None
        self._rx_buffer: bytearray = bytearray()
        self._rx_event = asyncio.Event()
        self._rx_lock = asyncio.Lock()
        self._connected = False

    # ── Connection management ───────────────────────────────────────────

    async def connect(self) -> None:
        """Connect to the BLE adapter, discover services, and initialise ELM327."""
        if not HAS_BLEAK:
            raise OBDConnectionError("bleak is not installed (required for BLE communication)")

        device = await self._find_device()
        if device is None:
            raise OBDConnectionError(
                f"BLE device not found for address {self._address}. "
                "Ensure the adapter is powered on and in range of a Bluetooth proxy."
            )

        _LOGGER.debug("Connecting to %s (%s)", device.name or "OBD-II", self._address)

        try:
            self._client = BleakClient(
                device,
                disconnected_callback=self._on_disconnect,
                timeout=BLE_CONNECT_TIMEOUT,
            )
            await self._client.connect()
        except (BleakError, asyncio.TimeoutError) as exc:
            raise OBDConnectionError(f"Failed to connect to {self._address}: {exc}") from exc

        # Discover the GATT service and characteristics
        svc_uuid = await self._discover_chars()
        _LOGGER.info(
            "Connected to %s (service %s, write=%s, notify=%s)",
            self._address, svc_uuid, self._char_write_uuid, self._char_notify_uuid,
        )

        # Subscribe to notifications on the notify characteristic
        try:
            await self._client.start_notify(self._char_notify_uuid, self._on_notification)
        except BleakError as exc:
            await self.disconnect()
            raise OBDConnectionError(f"Failed to subscribe to notifications: {exc}") from exc

        self._connected = True

        # Run ELM327 initialisation
        await self._init_elm327()

    async def _find_device(self) -> Optional[BLEDevice]:
        """Find the BLE device, using cache first then active scan."""
        # Try HA's cached advertisement history (fast, no radio overhead)
        device = async_ble_device_from_address(self._hass, self._address, connectable=True)
        if device is not None:
            _LOGGER.debug("Found %s in HA cache (%s)", self._address, device.name or "unknown")
            return device

        # Cache miss — actively scan for the device
        _LOGGER.info(
            "Device %s not in cache, starting active scan (up to %ds)...",
            self._address, BLE_CONNECT_TIMEOUT,
        )
        if BleakScanner is None:
            return None

        try:
            scanner = BleakScanner()
            found = await scanner.find_device_by_address(
                self._address.upper(), timeout=BLE_CONNECT_TIMEOUT
            )
            if found is not None:
                _LOGGER.info("Found %s via active scan (%s)", self._address, found.name or "unknown")
                return found
        except Exception as exc:
            _LOGGER.warning("Active scan for %s failed: %s", self._address, exc)

        return None

    async def _discover_chars(self) -> str:
        """Discover GATT service and resolve write + notify characteristic UUIDs.

        Veepeak uses separate chars (FFF1 write, FFF2 notify) under service FFF0.
        Vgate uses a shared char (bef8d6c9) under service e7810a71.
        Returns the resolved service UUID.
        """
        if self._client is None:
            raise OBDConnectionError("Not connected")

        # Try each known service UUID, first match wins
        for svc_uuid in DISCOVERY_SERVICE_UUIDS:
            try:
                svc = self._client.services.get_service(svc_uuid)
                if svc is None:
                    continue
            except Exception:
                continue

            # Check for Veepeak-style split chars (FFF1+FFF2)
            char_w = svc.get_characteristic(OBD_CHAR_WRITE_UUID)
            char_n = svc.get_characteristic(OBD_CHAR_NOTIFY_UUID)
            if char_w is not None and char_n is not None:
                self._char_write_uuid = OBD_CHAR_WRITE_UUID
                self._char_notify_uuid = OBD_CHAR_NOTIFY_UUID
                return svc_uuid

            # Check for Vgate-style shared char (bef8d6c9)
            char_shared = svc.get_characteristic(VGT_CHAR_UUID)
            if char_shared is not None:
                self._char_write_uuid = VGT_CHAR_UUID
                self._char_notify_uuid = VGT_CHAR_UUID
                return svc_uuid

        # Last resort: scan all services for any recognisable OBD characteristic
        for svc in self._client.services:
            for char_uuid in (OBD_CHAR_WRITE_UUID, VGT_CHAR_UUID):
                if svc.get_characteristic(char_uuid) is not None:
                    self._char_write_uuid = char_uuid
                    self._char_notify_uuid = char_uuid
                    return svc.uuid

        raise OBDConnectionError(
            f"No OBD-II GATT service found on {self._address}. "
            f"Tried: {DISCOVERY_SERVICE_UUIDS}"
        )

    async def _init_elm327(self) -> None:
        """Send ELM327 initialisation commands for the selected vehicle profile."""
        init_cmds = ELM_INIT_G6 if self._profile == "xpeng_g6" else ELM_INIT_SAE
        _LOGGER.debug("Initialising ELM327 with %s profile", self._profile)

        for cmd in init_cmds:
            try:
                await self._send_at(cmd)
            except OBDCommandError as exc:
                _LOGGER.warning("ELM327 init command '%s' failed: %s (continuing)", cmd, exc)

        _LOGGER.info("ELM327 initialisation complete (profile: %s)", self._profile)

    async def disconnect(self) -> None:
        """Disconnect from the BLE adapter."""
        self._connected = False
        if self._client is not None:
            try:
                await self._client.disconnect()
            except Exception:
                pass
            self._client = None
        _LOGGER.debug("Disconnected from %s", self._address)

    def _on_disconnect(self, client: BleakClient) -> None:
        """Handle unexpected BLE disconnection."""
        _LOGGER.warning("BLE disconnected from %s", self._address)
        self._connected = False
        self._rx_event.set()  # Unblock any waiting readers

    def _on_notification(self, _sender: int, data: bytearray) -> None:
        """Handle GATT notification — append to receive buffer."""
        self._rx_buffer.extend(data)
        self._rx_event.set()

    @property
    def is_connected(self) -> bool:
        """Return whether the BLE connection is active."""
        return self._connected

    # ── ELM327 command interface ────────────────────────────────────────

    async def _send_raw(self, data: bytes) -> None:
        """Write raw bytes to the GATT write characteristic."""
        if self._client is None or not self._connected:
            raise OBDConnectionError("Not connected")
        try:
            await self._client.write_gatt_char(self._char_write_uuid, data, response=False)
        except BleakError as exc:
            self._connected = False
            raise OBDConnectionError(f"GATT write failed: {exc}") from exc

    async def _send_at(self, command: str, timeout: float = BLE_COMMAND_TIMEOUT) -> str:
        """Send an AT command and return the response string.

        ELM327 AT commands end with CR and the adapter responds with a
        prompt character '>' when ready.
        """
        return await self._send_command(command, at_mode=True, timeout=timeout)

    async def _send_obd(self, command: str, timeout: float = BLE_COMMAND_TIMEOUT) -> str:
        """Send an OBD-II PID query and return the response string."""
        return await self._send_command(command, at_mode=False, timeout=timeout)

    async def _send_command(
        self, command: str, at_mode: bool = False, timeout: float = BLE_COMMAND_TIMEOUT
    ) -> str:
        """Send a command to the ELM327 and wait for the response.

        The ELM327 protocol:
        1. We send: command + CR
        2. Adapter processes and sends back: response + prompt ">"
        3. We collect all data until we see the prompt

        Args:
            command: The AT or OBD command string (without CR).
            at_mode: True for AT commands, False for OBD PIDs.
            timeout: Maximum wait time for response.

        Returns:
            The response string with the prompt character stripped.
        """
        async with self._rx_lock:
            # Flush any stale data
            self._rx_buffer.clear()
            self._rx_event.clear()

            payload = (command + "\r").encode("ascii")
            await self._send_raw(payload)

            # Collect response until prompt or timeout
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                try:
                    await asyncio.wait_for(self._rx_event.wait(), timeout=0.5)
                except asyncio.TimeoutError:
                    if ELM_PROMPT in self._rx_buffer:
                        break
                    continue
                self._rx_event.clear()

                # Check if we got the prompt
                if ELM_PROMPT in self._rx_buffer:
                    break

            response = bytes(self._rx_buffer).decode("ascii", errors="replace").strip()

            # Strip the prompt character(s)
            if response.endswith(">"):
                response = response[:-1].strip()

            # For AT commands, the echo may be included; strip it
            if at_mode and response.upper().startswith(command.upper()):
                response = response[len(command):].strip()

            if not response and not at_mode:
                raise OBDCommandError(f"Empty response for command: {command}")

            return response

    async def send_command(self, command: str) -> str:
        """Send a command and return the raw response string.

        This is the public interface used by the coordinator. It detects
        whether the command is an AT command or OBD PID and handles retries.
        """
        is_at = command.upper().startswith("AT")
        last_exc: Optional[Exception] = None

        for attempt in range(MAX_COMMAND_RETRIES):
            try:
                if is_at:
                    return await self._send_at(command)
                else:
                    return await self._send_obd(command)
            except (OBDCommandError, OBDConnectionError) as exc:
                last_exc = exc
                if attempt < MAX_COMMAND_RETRIES - 1:
                    _LOGGER.debug(
                        "Command '%s' failed (attempt %d/%d): %s",
                        command, attempt + 1, MAX_COMMAND_RETRIES, exc,
                    )
                    await asyncio.sleep(0.5 * (attempt + 1))
                    continue
                raise

        # Should not reach here, but satisfy type checker
        raise last_exc or OBDCommandError(f"Command failed: {command}")

    async def query_pid(self, mode: str, pid: str) -> str:
        """Query a single OBD-II PID.

        Args:
            mode: OBD mode (e.g. "01" for SAE current data, "22" for manufacturer).
            pid: PID hex string (e.g. "0C", "221109").

        Returns:
            Raw ELM327 response string.
        """
        return await self.send_command(f"{mode} {pid}")

    async def switch_header(self, header_cmd: str) -> None:
        """Switch the ELM327 CAN header for multi-ECU vehicles.

        Args:
            header_cmd: ELM327 header command (e.g. ELM_HEADER_BMS or ELM_HEADER_VCU).
        """
        # Split compound commands like "AT SH 704; AT CRA 784; AT FCSH 704"
        for cmd in header_cmd.split(";"):
            cmd = cmd.strip()
            if cmd:
                await self.send_command(cmd)

    async def switch_to_bms(self) -> None:
        """Switch to BMS ECU (CAN 704→784)."""
        await self.switch_header(ELM_HEADER_BMS)

    async def switch_to_vcu(self) -> None:
        """Switch to VCU ECU (CAN 7E0→7E8)."""
        await self.switch_header(ELM_HEADER_VCU)
