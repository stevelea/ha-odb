"""DataUpdateCoordinator for OBD-II BLE integration.

Manages scheduled polling of OBD-II PIDs, ECU header switching for multi-ECU
vehicles (XPENG G6), response parsing, and sensor data distribution.
"""

from __future__ import annotations

import asyncio
import logging
from datetime import timedelta
from typing import Optional

from homeassistant.core import HomeAssistant
from homeassistant.helpers.update_coordinator import DataUpdateCoordinator, UpdateFailed

from .const import (
    DOMAIN,
    PIDPriority,
    POLL_INTERVAL_DEFAULT,
    SAE_PIDS,
    XPENG_G6_PIDS,
    OBDPid,
    OBDMode,
    parse_obd_response,
)
from .obd_client import OBDClient, OBDClientError

_LOGGER = logging.getLogger(__package__)

# Low-priority PIDs are polled every N cycles (multiplier)
_LOW_PRIORITY_INTERVAL = 12


class OBDCoordinator(DataUpdateCoordinator[dict[str, Optional[float]]]):
    """Coordinator that polls OBD-II PIDs on a schedule."""

    def __init__(
        self,
        hass: HomeAssistant,
        client: OBDClient,
        pids: dict[str, OBDPid],
        poll_interval: int = POLL_INTERVAL_DEFAULT,
        vehicle_profile: str = "sae",
    ) -> None:
        """Initialise the coordinator.

        Args:
            hass: Home Assistant instance.
            client: Connected OBDClient.
            pids: PID definitions to poll.
            poll_interval: Update interval in seconds.
            vehicle_profile: Profile name ("sae" or "xpeng_g6").
        """
        self._client = client
        self._pids = pids
        self._profile = vehicle_profile
        self._cycle_count = 0
        self._last_low_values: dict[str, float] = {}
        self._current_header: Optional[str] = None  # "bms", "vcu", or None

        super().__init__(
            hass,
            _LOGGER,
            name=DOMAIN,
            update_interval=timedelta(seconds=poll_interval),
        )

    @staticmethod
    def get_default_pids(profile: str) -> dict[str, OBDPid]:
        """Return the default PID list for a given profile."""
        if profile == "xpeng_g6":
            return dict(XPENG_G6_PIDS)
        return dict(SAE_PIDS)

    async def _async_update_data(self) -> dict[str, Optional[float]]:
        """Poll all configured PIDs and return a sensor_name → value dict."""
        if not self._client.is_connected:
            raise UpdateFailed("OBD-II adapter is not connected")

        self._cycle_count += 1
        data: dict[str, Optional[float]] = {}

        # For SAE profile: simple round-robin poll
        if self._profile == "sae":
            data = await self._poll_sae_pids()
        else:
            # For XPENG G6: poll BMS PIDs first, then switch to VCU
            data = await self._poll_g6_pids()

        # Carry forward stale low-priority values
        for name, value in self._last_low_values.items():
            if name not in data:
                data[name] = value

        return data

    async def _poll_sae_pids(self) -> dict[str, Optional[float]]:
        """Poll standard SAE PIDs (mode 01)."""
        data: dict[str, Optional[float]] = {}

        for pid_key, pid_def in self._pids.items():
            # Skip low-priority PIDs on non-low cycles
            if pid_def.priority == PIDPriority.LOW:
                if self._cycle_count % _LOW_PRIORITY_INTERVAL != 0:
                    continue

            value = await self._query_single_pid(pid_def, pid_def.mode, pid_def.pid)
            if value is not None:
                data[pid_def.name] = value
                if pid_def.priority == PIDPriority.LOW:
                    self._last_low_values[pid_def.name] = value

        return data

    async def _poll_g6_pids(self) -> dict[str, Optional[float]]:
        """Poll XPENG G6 PIDs — BMS first, then VCU, minimising header switches."""
        data: dict[str, Optional[float]] = {}

        # Separate PIDs by CAN header
        bms_pids = [(pid_key, pid_def) for pid_key, pid_def in self._pids.items()
                     if pid_def.can_header == "704"]
        vcu_pids = [(pid_key, pid_def) for pid_key, pid_def in self._pids.items()
                     if pid_def.can_header == "7E0"]

        # ── Poll BMS PIDs ──
        if bms_pids:
            if self._current_header != "bms":
                try:
                    await self._client.switch_to_bms()
                    self._current_header = "bms"
                except OBDClientError as exc:
                    _LOGGER.warning("Failed to switch to BMS header: %s", exc)
                    # Try to continue anyway

            for pid_key, pid_def in bms_pids:
                if pid_def.priority == PIDPriority.LOW and self._cycle_count % _LOW_PRIORITY_INTERVAL != 0:
                    continue
                value = await self._query_single_pid(pid_def, OBDMode.MANUFACTURER, pid_def.pid)
                if value is not None:
                    data[pid_def.name] = value
                    if pid_def.priority == PIDPriority.LOW:
                        self._last_low_values[pid_def.name] = value

        # ── Poll VCU PIDs ──
        if vcu_pids:
            if self._current_header != "vcu":
                try:
                    await self._client.switch_to_vcu()
                    self._current_header = "vcu"
                except OBDClientError as exc:
                    _LOGGER.warning("Failed to switch to VCU header: %s", exc)

            for pid_key, pid_def in vcu_pids:
                if pid_def.priority == PIDPriority.LOW and self._cycle_count % _LOW_PRIORITY_INTERVAL != 0:
                    continue
                value = await self._query_single_pid(pid_def, OBDMode.MANUFACTURER, pid_def.pid)
                if value is not None:
                    data[pid_def.name] = value
                    if pid_def.priority == PIDPriority.LOW:
                        self._last_low_values[pid_def.name] = value

        return data

    async def _query_single_pid(
        self, pid_def: OBDPid, mode: str, pid: str
    ) -> Optional[float]:
        """Query a single PID and parse the response.

        Returns None if the PID is unsupported or communication fails.
        """
        try:
            response = await self._client.query_pid(mode, pid)
        except (OBDClientError, asyncio.TimeoutError) as exc:
            _LOGGER.debug("PID %s query failed: %s", pid_def.name, exc)
            return None

        value = parse_obd_response(response, pid_def.formula)
        if value is None:
            _LOGGER.debug(
                "PID %s (%s): no data or parse error. Response: %s",
                pid_def.name, pid, response[:80],
            )
            return None

        _LOGGER.debug("PID %s (%s): %.2f %s", pid_def.name, pid, value, pid_def.unit or "")
        return value

    async def query_dtcs(self) -> list[str]:
        """Query diagnostic trouble codes (mode 03)."""
        try:
            response = await self._client.query_pid("03", "")
        except OBDClientError:
            return []

        # Parse DTCs from response (standard OBD-II format)
        # Response format: 43 [count] [DTC pairs]...
        # Each DTC is 2 bytes: first character decoded, next 3.5 nibbles
        clean = response.replace(" ", "").replace(">", "").replace("\r", "").upper()

        # Check for no DTCs
        if "NO DATA" in clean or "NODATA" in clean:
            return []

        dtcs = []
        # Skip mode echo bytes ("43") and hex-length byte
        data = clean[4:] if len(clean) > 4 else clean
        for i in range(0, len(data) - 3, 4):
            dtc_hex = data[i : i + 4]
            if len(dtc_hex) == 4:
                dtcs.append(dtc_hex)

        return dtcs

    async def clear_dtcs(self) -> bool:
        """Clear diagnostic trouble codes (mode 04)."""
        try:
            await self._client.query_pid("04", "")
            return True
        except OBDClientError:
            return False

    async def get_vin(self) -> Optional[str]:
        """Read VIN via mode 09 PID 02."""
        try:
            response = await self._client.query_pid("09", "02")
        except OBDClientError:
            return None

        clean = response.replace(" ", "").replace(">", "").replace("\r", "").upper()
        if "49" in clean[:4]:
            # Mode 09 PID 02 response: 49 02 [01] [VIN ASCII]
            vin_start = clean.find("49") + 4  # Skip "4902"
            if len(clean) > vin_start:
                vin_hex = clean[vin_start:]
                try:
                    return bytes.fromhex(vin_hex).decode("ascii", errors="replace").strip()
                except ValueError:
                    pass
        return None
