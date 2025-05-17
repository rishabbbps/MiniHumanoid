from pololu_maestro import Maestro

# adjust if your device node is different
m = Maestro("/dev/ttyACM0")

# Read channel 0 position (will clear error flags)
print("Channel 0 position:", m.get_position(0))

# Move channel 0 to 6000 (1.5 ms pulse width)
m.set_target(0, 6000)
print("Moved channel 0 to 6000 units")
