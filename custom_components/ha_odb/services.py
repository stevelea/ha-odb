"""Services for OBD-II BLE integration.

Provides diagnostic services: read DTCs, clear DTCs, and get VIN.
Registered via __init__.py.
"""

from __future__ import annotations

import logging

from homeassistant.core import HomeAssistant, ServiceCall, SupportsResponse
import voluptuous as vol

from .const import DOMAIN

_LOGGER = logging.getLogger(__package__)

# Service schemas
SERVICE_GET_DTCS = "get_dtcs"
SERVICE_CLEAR_DTCS = "clear_dtcs"
SERVICE_GET_VIN = "get_vin"
