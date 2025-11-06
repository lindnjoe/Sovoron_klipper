# oams.py - Phase 5 Event Publishing Integration

This guide shows how to update oams.py to publish events when hardware
state changes occur.

## Step 1: Update Imports (around line 16-19)

### BEFORE:
```python
try:  # pragma: no cover - optional dependency during unit tests
    from extras.ams_integration import AMSHardwareService
except Exception:  # pragma: no cover - best-effort integration only
    AMSHardwareService = None
```

### AFTER:
```python
try:  # pragma: no cover - optional dependency during unit tests
    from extras.ams_integration import AMSHardwareService
except Exception:  # pragma: no cover - best-effort integration only
    AMSHardwareService = None

# PHASE 5: Import event bus
try:
    from extras.openams_integration import AMSEventBus
except Exception:
    AMSEventBus = None
```

## Step 2: Initialize Event Bus in __init__ (around line 158-169)

### ADD after line 169 (after AMSHardwareService registration):
```python
        # PHASE 5: Get event bus instance
        self.event_bus = None
        if AMSEventBus is not None:
            try:
                self.event_bus = AMSEventBus.get_instance()
            except Exception:
                logging.getLogger(__name__).exception(
                    "Failed to get AMSEventBus instance"
                )
```

## Step 3: Publish Events in load_spool (around line 595-608)

### BEFORE:
```python
    def load_spool(self, spool_idx):
        self.action_status = OAMSStatus.LOADING
        self.oams_load_spool_cmd.send([spool_idx])
        while self.action_status is not None:
            self.reactor.pause(self.reactor.monotonic() + 0.1)
        if self.action_status_code == OAMSOpCode.SUCCESS:
            self.current_spool = spool_idx
            return True, "Spool loaded successfully"
        elif self.action_status_code == OAMSOpCode.ERROR_KLIPPER_CALL:
            return False, "Spool loading stopped by klipper monitor"
        elif self.action_status_code == OAMSOpCode.ERROR_BUSY:
            return False, "OAMS is busy"
        else:
            return False, "Unknown error from OAMS with code %d" % self.action_status_code
```

### AFTER:
```python
    def load_spool(self, spool_idx):
        self.action_status = OAMSStatus.LOADING
        self.oams_load_spool_cmd.send([spool_idx])
        while self.action_status is not None:
            self.reactor.pause(self.reactor.monotonic() + 0.1)
        if self.action_status_code == OAMSOpCode.SUCCESS:
            self.current_spool = spool_idx
            
            # PHASE 5: Publish spool_loaded event
            if self.event_bus is not None:
                try:
                    eventtime = self.reactor.monotonic()
                    self.event_bus.publish(
                        "spool_loaded",
                        unit_name=self.section_name,
                        spool_index=spool_idx,
                        eventtime=eventtime
                    )
                except Exception:
                    logging.getLogger(__name__).exception(
                        "Failed to publish spool_loaded event"
                    )
            
            return True, "Spool loaded successfully"
        elif self.action_status_code == OAMSOpCode.ERROR_KLIPPER_CALL:
            return False, "Spool loading stopped by klipper monitor"
        elif self.action_status_code == OAMSOpCode.ERROR_BUSY:
            return False, "OAMS is busy"
        else:
            return False, "Unknown error from OAMS with code %d" % self.action_status_code
```

## Step 4: Publish Events in unload_spool (around line 627-640)

### BEFORE:
```python
    def unload_spool(self):
        self.action_status = OAMSStatus.UNLOADING
        self.oams_unload_spool_cmd.send()
        while self.action_status is not None:
            self.reactor.pause(self.reactor.monotonic() + 0.1)
        if self.action_status_code == OAMSOpCode.SUCCESS:
            self.current_spool = None
            return True, "Spool unloaded successfully"
        elif self.action_status_code == OAMSOpCode.ERROR_KLIPPER_CALL:
            return False, "Spool unloading stopped by klipper monitor"
        elif self.action_status_code == OAMSOpCode.ERROR_BUSY:
            return False, "OAMS is busy"
        else:
            return False, "Unknown error from OAMS"
```

### AFTER:
```python
    def unload_spool(self):
        # Store current spool before unloading (for event)
        old_spool = self.current_spool
        
        self.action_status = OAMSStatus.UNLOADING
        self.oams_unload_spool_cmd.send()
        while self.action_status is not None:
            self.reactor.pause(self.reactor.monotonic() + 0.1)
        if self.action_status_code == OAMSOpCode.SUCCESS:
            self.current_spool = None
            
            # PHASE 5: Publish spool_unloaded event
            if self.event_bus is not None and old_spool is not None:
                try:
                    eventtime = self.reactor.monotonic()
                    self.event_bus.publish(
                        "spool_unloaded",
                        unit_name=self.section_name,
                        spool_index=old_spool,
                        eventtime=eventtime
                    )
                except Exception:
                    logging.getLogger(__name__).exception(
                        "Failed to publish spool_unloaded event"
                    )
            
            return True, "Spool unloaded successfully"
        elif self.action_status_code == OAMSOpCode.ERROR_KLIPPER_CALL:
            return False, "Spool unloading stopped by klipper monitor"
        elif self.action_status_code == OAMSOpCode.ERROR_BUSY:
            return False, "OAMS is busy"
        else:
            return False, "Unknown error from OAMS"
```

## Step 5: Optional - Publish HES State Change Events

If you want even more granular events, you can publish when hall effect sensors change:

### ADD new method after _oams_cmd_stats (around line 695):
```python
    def _oams_cmd_stats(self, params):
        # Store old values for comparison
        old_hub_hes = list(self.hub_hes_value) if hasattr(self, 'hub_hes_value') else None
        
        # Update values
        self.fps_value = self.u32_to_float(params["fps_value"])
        self.f1s_hes_value[0] = params["f1s_hes_value_0"]
        self.f1s_hes_value[1] = params["f1s_hes_value_1"]
        self.f1s_hes_value[2] = params["f1s_hes_value_2"]
        self.f1s_hes_value[3] = params["f1s_hes_value_3"]
        self.hub_hes_value[0] = params["hub_hes_value_0"]
        self.hub_hes_value[1] = params["hub_hes_value_1"]
        self.hub_hes_value[2] = params["hub_hes_value_2"]
        self.hub_hes_value[3] = params["hub_hes_value_3"]
        self.encoder_clicks = params["encoder_clicks"]
        
        # PHASE 5: Publish hub state change events
        if self.event_bus is not None and old_hub_hes is not None:
            eventtime = self.reactor.monotonic()
            for idx in range(4):
                old_state = bool(old_hub_hes[idx])
                new_state = bool(self.hub_hes_value[idx])
                
                if old_state != new_state:
                    event_type = "lane_hub_loaded" if new_state else "lane_hub_unloaded"
                    try:
                        self.event_bus.publish(
                            event_type,
                            unit_name=self.section_name,
                            spool_index=idx,
                            eventtime=eventtime
                        )
                    except Exception:
                        logging.getLogger(__name__).exception(
                            "Failed to publish %s event for spool %d", event_type, idx
                        )
```

## Testing the Event System

### 1. View Event History in Console:
```python
# In Klipper console
event_bus = printer.lookup_object("AFC_OpenAMS AMS_1").event_bus
history = event_bus.get_history()
for event_type, time, data in history[-10:]:  # Last 10 events
    print(f"{event_type}: {data}")
```

### 2. Subscribe to Events for Debugging:
```python
# Add this to your console or a test script
def debug_callback(event_type, **kwargs):
    print(f"EVENT: {event_type} - {kwargs}")

event_bus = printer.lookup_object("AFC_OpenAMS AMS_1").event_bus
event_bus.subscribe("spool_loaded", debug_callback)
event_bus.subscribe("spool_unloaded", debug_callback)

# Now load/unload a spool and watch the console
```

### 3. Check Event Publication:
```python
# After loading a spool, check if event was published
event_bus = printer.lookup_object("AFC_OpenAMS AMS_1").event_bus
recent = event_bus.get_history(event_type="spool_loaded", since=time.time() - 60)
print(f"Spool loaded events in last 60 seconds: {len(recent)}")
```

## Benefits of Event Publishing

### Before (Polling):
```
AFC_OpenAMS: Check state... (every 2 seconds)
AFC_OpenAMS: Check state...
AFC_OpenAMS: Check state...
AFC_OpenAMS: Spool changed! Update...
AFC_OpenAMS: Check state...
AFC_OpenAMS: Check state...
```

### After (Events):
```
OAMS: Spool loaded → Publish event
AFC_OpenAMS: Received spool_loaded event → Update immediately
[No polling needed between state changes]
```

### Performance Impact:
- **Before**: ~0.5 checks/second = constant CPU usage
- **After**: Only when state changes = near-zero idle CPU usage
- **Latency**: Immediate notification vs up to 2 second delay

## Event Types Available

After these changes, the following events will be published:

1. **spool_loaded**: When OAMS successfully loads a spool
   - `unit_name`: OAMS unit name (e.g., "oams1")
   - `spool_index`: Index of loaded spool (0-3)
   - `eventtime`: Reactor monotonic time

2. **spool_unloaded**: When OAMS successfully unloads a spool
   - `unit_name`: OAMS unit name
   - `spool_index`: Index of unloaded spool
   - `eventtime`: Reactor monotonic time

3. **lane_hub_loaded**: When filament enters hub (optional)
   - `unit_name`: OAMS unit name
   - `spool_index`: Spool index
   - `eventtime`: Reactor monotonic time

4. **lane_hub_unloaded**: When filament leaves hub (optional)
   - `unit_name`: OAMS unit name
   - `spool_index`: Spool index
   - `eventtime`: Reactor monotonic time

## Backward Compatibility

All event publishing is optional and wrapped in try/except blocks:
- If AMSEventBus is not available, code runs as before
- No config changes required
- Existing functionality unchanged
- Events are purely additive
