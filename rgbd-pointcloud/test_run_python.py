import subprocess
import time
from evdev import uinput, ecodes as e


# Run the other script
for i in range(0,5):
    print('press key')
    if input():
        subprocess.run(["python", "main2.py"])

        # pressings s
        with uinput.UInput() as ui:

            ui.write(e.EV_KEY, e.KEY_s, 1)
            ui.syn()

        
