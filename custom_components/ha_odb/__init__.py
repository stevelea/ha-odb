"""The OBD-II BLE integration.

Connects to ELM327-compatible Bluetooth LE OBD-II adapters via Home Assistant's
Bluetooth integration and exposes vehicle data as sensor entities.
"""

from __future__ import annotations

import logging

from homeassistant.config_entries import ConfigEntry
from homeassistant.const import CONF_ADDRESS, Platform
from homeassistant.core import HomeAssistant, ServiceCall
from homeassistant.exceptions import ConfigEntryNotReady

from .const import (
    CONF_POLL_INTERVAL,
    CONF_VEHICLE_PROFILE,
    DOMAIN,
    OBDPid,
    POLL_INTERVAL_DEFAULT,
)
from .coordinator import OBDCoordinator
from .obd_client import OBDClient, OBDConnectionError

_LOGGER = logging.getLogger(__package__)

PLATFORMS: list[Platform] = [Platform.SENSOR]


async def async_setup_entry(hass: HomeAssistant, entry: ConfigEntry) -> bool:
    """Set up OBD-II BLE from a config entry."""
    address: str = entry.data[CONF_ADDRESS]
    profile: str = entry.data.get(CONF_VEHICLE_PROFILE, "sae")
    poll_interval: int = entry.options.get(
        CONF_POLL_INTERVAL,
        entry.data.get(CONF_POLL_INTERVAL, POLL_INTERVAL_DEFAULT),
    )

    # Build the PID list
    pids: dict[str, OBDPid] = OBDCoordinator.get_default_pids(profile)

    _LOGGER.info(
        "Setting up OBD-II BLE: address=%s profile=%s pids=%d interval=%ds",
        address, profile, len(pids), poll_interval,
    )

    # Create and connect the BLE client
    client = OBDClient(hass, address, vehicle_profile=profile)
    try:
        await client.connect()
    except OBDConnectionError as exc:
        await client.disconnect()
        raise ConfigEntryNotReady(f"Failed to connect to OBD-II adapter at {address}: {exc}") from exc

    # Create the coordinator
    coordinator = OBDCoordinator(
        hass,
        client,
        pids,
        poll_interval=poll_interval,
        vehicle_profile=profile,
    )

    # Fetch initial data
    try:
        await coordinator.async_config_entry_first_refresh()
    except Exception as exc:
        await client.disconnect()
        raise ConfigEntryNotReady(f"Initial OBD data fetch failed: {exc}") from exc

    # Store in hass.data
    hass.data.setdefault(DOMAIN, {})
    hass.data[DOMAIN][entry.entry_id] = {
        "client": client,
        "coordinator": coordinator,
        "pids": pids,
    }

    # Forward to platforms
    await hass.config_entries.async_forward_entry_setups(entry, PLATFORMS)

    # Register services
    _register_services(hass, entry, coordinator)

    return True


async def async_unload_entry(hass: HomeAssistant, entry: ConfigEntry) -> bool:
    """Unload a config entry."""
    # Remove services
    _remove_services(hass, entry.entry_id)

    # Unload platforms
    unload_ok = await hass.config_entries.async_unload_platforms(entry, PLATFORMS)

    if unload_ok:
        data = hass.data[DOMAIN].pop(entry.entry_id)
        client: OBDClient = data["client"]
        await client.disconnect()

    return unload_ok


def _register_services(
    hass: HomeAssistant, entry: ConfigEntry, coordinator: OBDCoordinator
) -> None:
    """Register OBD services (read DTCs, clear DTCs, get VIN)."""

    async def handle_get_dtcs(call: ServiceCall) -> None:
        """Handle the get_dtcs service call."""
        dtcs = await coordinator.query_dtcs()
        call.data.get("callback") or _LOGGER.info("DTCs: %s", dtcs or "None")

    async def handle_clear_dtcs(call: ServiceCall) -> None:
        """Handle the clear_dtcs service call."""
        success = await coordinator.clear_dtcs()
        _LOGGER.info("Clear DTCs: %s", "successful" if success else "failed")

    async def handle_get_vin(call: ServiceCall) -> None:
        """Handle the get_vin service call."""
        vin = await coordinator.get_vin()
        _LOGGER.info("VIN: %s", vin or "Unknown")

    service_base = f"{DOMAIN}_{entry.entry_id}"

    hass.services.async_register(DOMAIN, f"{service_base}_get_dtcs", handle_get_dtcs)
    hass.services.async_register(DOMAIN, f"{service_base}_clear_dtcs", handle_clear_dtcs)
    hass.services.async_register(DOMAIN, f"{service_base}_get_vin", handle_get_vin)


def _remove_services(hass: HomeAssistant, entry_id: str) -> None:
    """Remove registered services for an entry."""
    service_base = f"{DOMAIN}_{entry_id}"
    for svc in (f"{service_base}_get_dtcs", f"{service_base}_clear_dtcs", f"{service_base}_get_vin"):
        hass.services.async_remove(DOMAIN, svc)
