"""Sensor platform for OBD-II BLE integration.

Creates dynamic sensor entities for each configured OBD-II PID.
"""

from __future__ import annotations

import logging
from typing import Any

from homeassistant.components.sensor import (
    SensorDeviceClass,
    SensorEntity,
    SensorStateClass,
)
from homeassistant.config_entries import ConfigEntry
from homeassistant.const import PERCENTAGE, UnitOfSpeed, UnitOfTemperature
from homeassistant.core import HomeAssistant, callback
from homeassistant.helpers.entity_platform import AddEntitiesCallback
from homeassistant.helpers.update_coordinator import CoordinatorEntity

from .const import DOMAIN, OBDPid
from .coordinator import OBDCoordinator

_LOGGER = logging.getLogger(__package__)


async def async_setup_entry(
    hass: HomeAssistant,
    entry: ConfigEntry,
    async_add_entities: AddEntitiesCallback,
) -> None:
    """Set up OBD-II sensors from a config entry."""
    coordinator: OBDCoordinator = hass.data[DOMAIN][entry.entry_id]["coordinator"]
    pids: dict[str, OBDPid] = hass.data[DOMAIN][entry.entry_id]["pids"]

    # Wait for the first successful poll so we know which PIDs are live
    await coordinator.async_config_entry_first_refresh()

    entities: list[OBDSensor] = []
    for pid_key, pid_def in pids.items():
        entities.append(OBDSensor(coordinator, pid_def))

    async_add_entities(entities)


class OBDSensor(CoordinatorEntity[OBDCoordinator], SensorEntity):
    """Sensor entity for a single OBD-II PID."""

    _attr_has_entity_name = True
    _attr_should_poll = False  # Coordinator handles polling

    def __init__(
        self,
        coordinator: OBDCoordinator,
        pid_def: OBDPid,
    ) -> None:
        """Initialise the sensor."""
        super().__init__(coordinator)
        self._pid_def = pid_def
        self._attr_unique_id = f"{coordinator.config_entry.entry_id}_{pid_def.pid}"
        self._attr_name = pid_def.name
        self._attr_native_unit_of_measurement = pid_def.unit
        self._attr_suggested_display_precision = pid_def.suggested_display_precision
        self._attr_icon = pid_def.icon
        self._attr_entity_registry_enabled_default = (
            pid_def.priority == "high"
        )

        # Map device class
        if pid_def.device_class:
            self._attr_device_class = pid_def.device_class

        # Map state class
        self._attr_state_class = pid_def.state_class

        # Device info
        self._attr_device_info = {
            "identifiers": {(DOMAIN, coordinator.config_entry.entry_id)},
            "name": "OBD-II Adapter",
            "manufacturer": "ELM327",
            "model": "BLE OBD-II",
        }

    @callback
    def _handle_coordinator_update(self) -> None:
        """Handle updated data from the coordinator."""
        value = self.coordinator.data.get(self._pid_def.name)
        if value is not None:
            self._attr_native_value = round(value, self._pid_def.suggested_display_precision)
        else:
            self._attr_native_value = None
        self.async_write_ha_state()
