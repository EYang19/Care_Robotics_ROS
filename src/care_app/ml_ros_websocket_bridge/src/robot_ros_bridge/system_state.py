#!/usr/bin/env python3
"""
System-Wide State Management - Thread-safe inventory tracking
"""

import threading
from typing import Dict, Any, List, Optional
from datetime import datetime


class SystemState:
    """Thread-safe storage for inventory state"""

    def __init__(self):
        self._lock = threading.Lock()

        # Inventory: {location_id: {sku_id: {stock_level, max_level, par_level, reorder_point, category}}}
        self._location_inventory: Dict[str, Dict[str, Dict[str, Any]]] = {}

        # Category inventory: {location_id: {category: {total_stock, max_stock, num_skus}}}
        self._category_inventory: Dict[str, Dict[str, Dict[str, float]]] = {}

        # Location metadata: {location_id: {name, timestamp}}
        self._location_metadata: Dict[str, Dict[str, Any]] = {}

        # Buffer flag
        self._buffer_inventory: bool = True

        # Pending updates (for buffering)
        self._pending_inventory_updates: List[Dict[str, Any]] = []

    def set_buffering(self, buffer_inventory: bool) -> None:
        """
        Set buffering mode

        Args:
            buffer_inventory: Buffer inventory updates
        """
        with self._lock:
            self._buffer_inventory = buffer_inventory

    def update_location_inventory(
        self,
        location_id: str,
        location_name: str,
        sku_inventory: Dict[str, Dict[str, Any]],
        category_inventory: Dict[str, Dict[str, float]],
    ) -> None:
        """
        Update inventory for a location

        Args:
            location_id: Unique location identifier
            location_name: Human-readable location name
            sku_inventory: Dict of {sku_id: {stock_level, max_level, par_level, reorder_point, category}}
            category_inventory: Dict of {category: {total_stock, max_stock, num_skus}}
        """
        with self._lock:
            self._location_inventory[location_id] = sku_inventory.copy()
            self._category_inventory[location_id] = category_inventory.copy()
            self._location_metadata[location_id] = {
                "name": location_name,
                "timestamp": datetime.utcnow().isoformat(),
            }

            if self._buffer_inventory:
                self._pending_inventory_updates.append({
                    "location_id": location_id,
                    "location_name": location_name,
                    "sku_inventory": sku_inventory,
                    "category_inventory": category_inventory,
                })

    def get_all_location_inventories(self) -> List[Dict[str, Any]]:
        """
        Get all location inventories

        Returns:
            List of location inventory dicts
        """
        with self._lock:
            result = []
            for location_id in self._location_inventory:
                metadata = self._location_metadata.get(location_id, {})
                result.append({
                    "location_id": location_id,
                    "location_name": metadata.get("name", location_id),
                    "sku_inventory": self._location_inventory[location_id].copy(),
                    "category_inventory": self._category_inventory.get(location_id, {}).copy(),
                })
            return result

    def get_location_inventory(self, location_id: str) -> Optional[Dict[str, Any]]:
        """
        Get inventory for a specific location

        Args:
            location_id: Unique location identifier

        Returns:
            Location inventory dict or None
        """
        with self._lock:
            if location_id not in self._location_inventory:
                return None

            metadata = self._location_metadata.get(location_id, {})
            return {
                "location_id": location_id,
                "location_name": metadata.get("name", location_id),
                "sku_inventory": self._location_inventory[location_id].copy(),
                "category_inventory": self._category_inventory.get(location_id, {}).copy(),
            }

    def get_pending_inventory_updates(self) -> List[Dict[str, Any]]:
        """
        Get and clear pending inventory updates

        Returns:
            List of pending updates
        """
        with self._lock:
            updates = self._pending_inventory_updates.copy()
            self._pending_inventory_updates = []
            return updates

    def clear_all(self) -> None:
        """Clear all state"""
        with self._lock:
            self._location_inventory.clear()
            self._category_inventory.clear()
            self._location_metadata.clear()
            self._pending_inventory_updates.clear()
