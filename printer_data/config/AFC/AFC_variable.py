[virtual_input my_virtual_pin]
initial_value: 1

[virtual_filament_sensor my_filament_sensor]
pin: my_virtual_pin        # uses the name from the virtual_input section
pause_on_runout: True      # optional, defaults to True
runout_gcode: ...
insert_gcode: ...
