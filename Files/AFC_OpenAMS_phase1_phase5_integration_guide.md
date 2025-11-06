# AFC_OpenAMS.py - Phase 1 & 5 Integration Guide

This guide shows the key changes to make to AFC_OpenAMS.py to integrate
the Lane Registry (Phase 1) and Event System (Phase 5).

## Step 1: Update Imports (around line 34-38)

### BEFORE:
```python
try:
    from extras.openams_integration import AMSHardwareService, AMSRunoutCoordinator
except Exception:
    AMSHardwareService = None
    AMSRunoutCoordinator = None
```

### AFTER:
```python
try:
    from extras.openams_integration import (
        AMSHardwareService, 
        AMSRunoutCoordinator,
        LaneRegistry,          # PHASE 1
        AMSEventBus,           # PHASE 5
    )
except Exception:
    AMSHardwareService = None
    AMSRunoutCoordinator = None
    LaneRegistry = None
    AMSEventBus = None
```

## Step 2: Update __init__ (around line 225-268)

### Add after line 240 (after self.timer registration):
```python
        # PHASE 1: Use LaneRegistry instead of local _lane_by_index
        self.registry = None
        if LaneRegistry is not None:
            try:
                self.registry = LaneRegistry.for_printer(self.printer)
            except Exception:
                self.logger.exception("Failed to initialize LaneRegistry")
        
        # PHASE 5: Subscribe to events
        self.event_bus = None
        if AMSEventBus is not None:
            try:
                self.event_bus = AMSEventBus.get_instance()
                # Subscribe to spool events
                self.event_bus.subscribe("spool_loaded", self._handle_spool_loaded_event, priority=10)
                self.event_bus.subscribe("spool_unloaded", self._handle_spool_unloaded_event, priority=10)
            except Exception:
                self.logger.exception("Failed to subscribe to AMS events")
```

### REMOVE line 243 (or comment it out):
```python
        #  Build lane index map for O(1) lookup
        # self._lane_by_index: Dict[int, Any] = {}   # REMOVED - using registry now
```

## Step 3: Update handle_connect (around line 347-364)

### BEFORE (lines 353-364):
```python
        #  Build lane index map once
        self._lane_by_index = {}
        for lane in self.lanes.values():
            lane.prep_state = False
            lane.load_state = False
            lane.status = AFCLaneState.NONE
            lane.ams_share_prep_load = getattr(lane, "load", None) is None
            
            # Build index map
            idx = getattr(lane, "index", 0) - 1
            if idx >= 0:
                self._lane_by_index[idx] = lane
```

### AFTER:
```python
        # PHASE 1: Register lanes with registry instead of local map
        for lane in self.lanes.values():
            lane.prep_state = False
            lane.load_state = False
            lane.status = AFCLaneState.NONE
            lane.ams_share_prep_load = getattr(lane, "load", None) is None
            
            # Register with registry
            if self.registry is not None:
                idx = getattr(lane, "index", 0) - 1
                if idx >= 0:
                    lane_name = getattr(lane, "name", None)
                    unit_name = self.name
                    
                    # Extract group from lane map or derive from lane name
                    group = getattr(lane, "map", None)
                    if group is None and lane_name:
                        # Derive: "lane4" -> "T4"
                        lane_num = lane_name.replace("lane", "")
                        group = f"T{lane_num}"
                    
                    extruder_name = getattr(self, "extruder", None) or getattr(lane, "extruder_name", None)
                    
                    if lane_name and group and extruder_name:
                        try:
                            self.registry.register_lane(
                                lane_name=lane_name,
                                unit_name=unit_name,
                                spool_index=idx,
                                group=group,
                                extruder=extruder_name,
                                fps_name=getattr(self, "fps_name", None),
                                hub_name=getattr(lane, "hub", None),
                                led_index=getattr(lane, "led_index", None),
                                custom_load_cmd=getattr(lane, "custom_load_cmd", None),
                                custom_unload_cmd=getattr(lane, "custom_unload_cmd", None),
                            )
                        except Exception:
                            self.logger.exception("Failed to register lane %s with registry", lane_name)
```

## Step 4: Replace _find_lane_by_spool (around line 1524-1526)

### BEFORE:
```python
    def _find_lane_by_spool(self, spool_index):
        """Use indexed lookup."""
        return self._lane_by_index.get(spool_index)
```

### AFTER:
```python
    def _find_lane_by_spool(self, spool_index):
        """Use registry lookup (PHASE 1)."""
        if self.registry is None:
            # Fallback to old behavior if registry not available
            return self._lane_by_index.get(spool_index) if hasattr(self, '_lane_by_index') else None
        
        # PHASE 1: Use registry
        lane_info = self.registry.get_by_spool(self.name, spool_index)
        if lane_info is None:
            return None
        
        # Return the actual lane object
        return self.lanes.get(lane_info.lane_name)
```

## Step 5: Add Event Handlers (add at end of class, before load_config_prefix)

```python
    def _handle_spool_loaded_event(self, event_type: str, **kwargs) -> None:
        """Handle spool_loaded events from the event bus (PHASE 5)."""
        unit_name = kwargs.get("unit_name")
        spool_index = kwargs.get("spool_index")
        eventtime = kwargs.get("eventtime", 0.0)
        
        # Only handle events for this unit
        if unit_name != self.oams_name:
            return
        
        self.logger.debug("Received spool_loaded event: unit=%s, spool=%s", 
                         unit_name, spool_index)
        
        # Update lane state
        lane = self._find_lane_by_spool(spool_index)
        if lane is not None:
            lane.load_state = True
            lane_name = getattr(lane, "name", None)
            if lane_name and self.hardware_service:
                try:
                    self.hardware_service.update_lane_snapshot(
                        self.name, lane_name, True, True, eventtime, 
                        spool_index=spool_index
                    )
                except Exception:
                    self.logger.exception("Failed to update lane snapshot after spool load")
    
    def _handle_spool_unloaded_event(self, event_type: str, **kwargs) -> None:
        """Handle spool_unloaded events from the event bus (PHASE 5)."""
        unit_name = kwargs.get("unit_name")
        spool_index = kwargs.get("spool_index")
        eventtime = kwargs.get("eventtime", 0.0)
        
        # Only handle events for this unit
        if unit_name != self.oams_name:
            return
        
        self.logger.debug("Received spool_unloaded event: unit=%s, spool=%s", 
                         unit_name, spool_index)
        
        # Update lane state
        if spool_index is not None:
            lane = self._find_lane_by_spool(spool_index)
            if lane is not None:
                lane.load_state = False
                lane_name = getattr(lane, "name", None)
                if lane_name and self.hardware_service:
                    try:
                        self.hardware_service.update_lane_snapshot(
                            self.name, lane_name, False, False, eventtime,
                            spool_index=spool_index
                        )
                    except Exception:
                        self.logger.exception("Failed to update lane snapshot after spool unload")
```

## Step 6: Optional - Reduce Polling Overhead

Since we now have events, you can increase the sync intervals to reduce overhead:

### Around line 230-234, UPDATE:
```python
        # PHASE 5: Increased intervals since we have events now
        self.interval = config.getfloat("interval", SYNC_INTERVAL * 2, above=0.0)  # Was SYNC_INTERVAL
        
        # Adaptive polling intervals (less aggressive now that we have events)
        self.interval_idle = self.interval * 3.0  # Was * 2.0
        self.interval_active = self.interval
```

## Summary of Changes

### Phase 1 Benefits:
- **Eliminates `_lane_by_index` dictionary** - use registry instead
- **Single source of truth** for lane identity
- **O(1) lookups** by lane name, spool index, or group
- **Automatic derivation** of groups from lane names (e.g., "lane4" → "T4")

### Phase 5 Benefits:
- **Event-driven updates** instead of constant polling
- **Reduced CPU overhead** - only update when state changes
- **Better debugging** - can see event history
- **Decoupled architecture** - components communicate via events

### Testing Checklist:
1. Verify lanes register on startup (check logs for "Registered lane:")
2. Test lane lookup: `printer['AFC_OpenAMS AMS_1'].registry.get_by_group('T4')`
3. Watch events: `printer['AFC_OpenAMS AMS_1'].event_bus.get_history()`
4. Verify spool load/unload still works
5. Check that polling interval increased (less CPU usage)

### Backward Compatibility:
- Falls back to old `_lane_by_index` if registry not available
- Events are optional - code works without them
- No config file changes required
