"""Config flow for OBD-II BLE integration.

Supports Bluetooth discovery (dropdown of nearby OBD-II adapters) and
manual MAC address entry with vehicle profile selection.
"""

from __future__ import annotations

import logging
from typing import Any

import voluptuous as vol

from homeassistant.components.bluetooth import (
    BluetoothServiceInfoBleak,
    async_discovered_service_info,
    async_last_service_info,
)
from homeassistant.config_entries import (
    ConfigEntry,
    ConfigFlow,
    ConfigFlowResult,
    OptionsFlow,
)
from homeassistant.const import CONF_ADDRESS
from homeassistant.core import HomeAssistant, callback
from homeassistant.helpers.selector import (
    SelectSelector,
    SelectSelectorConfig,
    SelectSelectorMode,
)

from .const import (
    DISCOVERY_SERVICE_UUIDS,
    DOMAIN,
    POLL_INTERVAL_DEFAULT,
)

_LOGGER = logging.getLogger(__package__)

CONF_VEHICLE_PROFILE = "vehicle_profile"
CONF_POLL_INTERVAL = "poll_interval"
CONF_MANUAL_ADDRESS = "manual_address"
CONF_DEVICE_SELECT = "device_select"

PROFILE_OPTIONS = [
    {"value": "sae", "label": "SAE Standard (ICE vehicles — RPM, Speed, Coolant Temp, etc.)"},
    {"value": "xpeng_g6", "label": "XPENG G6 (EV — SoC, HV Battery, Motor, Charging, etc.)"},
]

DEVICE_SELECT_OPTION = "__manual__"  # Special value for "Enter address manually"


class OBDConfigFlow(ConfigFlow, domain=DOMAIN):
    """Handle a config flow for OBD-II BLE."""

    VERSION = 1

    _discovery_info: BluetoothServiceInfoBleak | None = None

    async def async_step_bluetooth(
        self, discovery_info: BluetoothServiceInfoBleak
    ) -> ConfigFlowResult:
        """Handle a flow initiated by Bluetooth discovery."""
        await self.async_set_unique_id(discovery_info.address)
        self._abort_if_unique_id_configured()

        self._discovery_info = discovery_info
        self.context["title_placeholders"] = {
            "name": discovery_info.name or "OBD-II Adapter",
            "address": discovery_info.address,
        }

        return await self.async_step_user()

    async def async_step_user(
        self, user_input: dict[str, Any] | None = None
    ) -> ConfigFlowResult:
        """Handle the user step: pick a discovered device or enter address manually."""
        errors: dict[str, str] = {}

        if user_input is not None:
            selected = user_input.get(CONF_DEVICE_SELECT)
            profile = user_input.get(CONF_VEHICLE_PROFILE, "sae")

            if selected == DEVICE_SELECT_OPTION:
                # User chose manual entry
                return await self.async_step_manual()

            # User selected a discovered device — address is the key
            address = selected
            if not address:
                errors[CONF_DEVICE_SELECT] = "no_device_selected"
            else:
                await self.async_set_unique_id(address, raise_on_progress=False)
                self._abort_if_unique_id_configured()

                return self.async_create_entry(
                    title=f"OBD-II ({address})",
                    data={
                        CONF_ADDRESS: address,
                        CONF_VEHICLE_PROFILE: profile,
                    },
                )

        # Build the list of discovered devices
        discovered = await _get_discovered_obd_devices(self.hass)

        # Pre-fill from Bluetooth discovery trigger if available
        prefill_address = ""
        if self._discovery_info is not None:
            prefill_address = self._discovery_info.address

        # Build selector options
        device_options: list[dict[str, str]] = []
        seen: set[str] = set()

        for dev in discovered:
            addr = dev["address"]
            if addr in seen:
                continue
            seen.add(addr)
            name = dev.get("name", "Unknown")
            # Truncate long names
            display = f"{name} ({addr})" if name and name != "Unknown" else addr
            device_options.append({"value": addr, "label": display})

        # Always add manual entry option
        device_options.append({
            "value": DEVICE_SELECT_OPTION,
            "label": "⟳ Enter address manually…",
        })

        if not any(o["value"] != DEVICE_SELECT_OPTION for o in device_options):
            # No discovered devices — go straight to manual
            return await self.async_step_manual()

        # Pre-select if we have a discovery trigger
        default = prefill_address if prefill_address in seen else None

        schema = vol.Schema(
            {
                vol.Required(
                    CONF_DEVICE_SELECT,
                    default=default or vol.UNDEFINED,
                ): SelectSelector(
                    SelectSelectorConfig(
                        options=device_options,
                        mode=SelectSelectorMode.DROPDOWN,
                        custom_value=True,
                    )
                ),
                vol.Required(
                    CONF_VEHICLE_PROFILE, default="sae"
                ): SelectSelector(
                    SelectSelectorConfig(
                        options=PROFILE_OPTIONS,
                        mode=SelectSelectorMode.DROPDOWN,
                    )
                ),
            }
        )

        description_placeholders = None
        if self._discovery_info is not None:
            description_placeholders = {
                "name": self._discovery_info.name or "Unknown",
                "address": self._discovery_info.address,
            }

        return self.async_show_form(
            step_id="user",
            data_schema=schema,
            errors=errors,
            description_placeholders=description_placeholders,
        )

    async def async_step_manual(
        self, user_input: dict[str, Any] | None = None
    ) -> ConfigFlowResult:
        """Manual MAC address entry step."""
        errors: dict[str, str] = {}

        if user_input is not None:
            address = user_input[CONF_MANUAL_ADDRESS].strip().upper()
            profile = user_input.get(CONF_VEHICLE_PROFILE, "sae")

            if not _is_valid_mac(address):
                errors[CONF_MANUAL_ADDRESS] = "invalid_mac_address"
            else:
                await self.async_set_unique_id(address, raise_on_progress=False)
                self._abort_if_unique_id_configured()

                return self.async_create_entry(
                    title=f"OBD-II ({address})",
                    data={
                        CONF_ADDRESS: address,
                        CONF_VEHICLE_PROFILE: profile,
                    },
                )

        schema = vol.Schema(
            {
                vol.Required(CONF_MANUAL_ADDRESS): str,
                vol.Required(
                    CONF_VEHICLE_PROFILE, default="sae"
                ): SelectSelector(
                    SelectSelectorConfig(
                        options=PROFILE_OPTIONS,
                        mode=SelectSelectorMode.DROPDOWN,
                    )
                ),
            }
        )

        return self.async_show_form(
            step_id="manual",
            data_schema=schema,
            errors=errors,
            description_placeholders={"hint": "Enter the BLE MAC address of your OBD-II adapter (e.g., 8C:DE:52:DE:FA:CF)."},
        )

    @staticmethod
    @callback
    def async_get_options_flow(config_entry: ConfigEntry) -> OptionsFlow:
        """Get the options flow for this handler."""
        return OBDOptionsFlow(config_entry)


class OBDOptionsFlow(OptionsFlow):
    """Handle options for OBD-II BLE — poll interval."""

    def __init__(self, config_entry: ConfigEntry) -> None:
        """Initialize options flow."""
        self._config_entry = config_entry

    async def async_step_init(
        self, user_input: dict[str, Any] | None = None
    ) -> ConfigFlowResult:
        """Manage the options."""
        if user_input is not None:
            return self.async_create_entry(data=user_input)

        current_interval = self._config_entry.options.get(
            CONF_POLL_INTERVAL,
            self._config_entry.data.get(CONF_POLL_INTERVAL, POLL_INTERVAL_DEFAULT),
        )

        schema = vol.Schema(
            {
                vol.Required(
                    CONF_POLL_INTERVAL, default=current_interval
                ): vol.All(vol.Coerce(int), vol.Range(min=5, max=3600)),
            }
        )

        return self.async_show_form(step_id="init", data_schema=schema)


def _is_valid_mac(address: str) -> bool:
    """Validate a BLE MAC address (XX:XX:XX:XX:XX:XX)."""
    parts = address.replace("-", ":").split(":")
    if len(parts) != 6:
        return False
    try:
        return all(0 <= int(p, 16) <= 255 for p in parts)
    except ValueError:
        return False


async def _get_discovered_obd_devices(hass: HomeAssistant) -> list[dict[str, str]]:
    """Return a list of discovered OBD-II BLE devices.

    Searches for devices advertising known OBD-II service UUIDs.
    """
    discovered: list[dict[str, str]] = []
    seen_addrs: set[str] = set()
    candidate_uuids = list(DISCOVERY_SERVICE_UUIDS)

    # Check by service UUID first
    for uuid in candidate_uuids:
        svc_info = async_last_service_info(hass, uuid)
        if svc_info is not None and svc_info.address not in seen_addrs:
            seen_addrs.add(svc_info.address)
            discovered.append({
                "address": svc_info.address,
                "name": svc_info.name or "Unknown",
            })

    # Also scan all discovered services for our UUIDs
    all_services = async_discovered_service_info(hass, connectable=True)
    for svc_info in all_services:
        if svc_info.address in seen_addrs:
            continue
        for adv_uuid in svc_info.advertisement.service_uuids:
            if adv_uuid.lower() in (u.lower() for u in candidate_uuids):
                seen_addrs.add(svc_info.address)
                discovered.append({
                    "address": svc_info.address,
                    "name": svc_info.name or "Unknown",
                })
                break

    return discovered
