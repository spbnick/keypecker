Keypecker
=========

"Keypecker" is a device for measuring key/button switch response speed and
reliability. It uses a stepper motor to (repeatedly) move an actuator rod up
and down, and records the time from the start of the movement to a signal edge
on one or more digital inputs (channels), for each such movement (pass).

[![Prototype front](proto_front_small.jpeg)](proto_front.jpeg)
[![Prototype back](proto_back_small.jpeg)](proto_back.jpeg)

Keypecker can be used to characterize both physical switches and response of
devices controlled by switches, provided they report events as digital signal
edges.

The device firmware is based on the Zephyr RTOS.

Keypecker is controlled via a TTL-level USART using an interactive shell with
an assortment of supported commands (including stock Zephyr commands):

```
  acquire  :Acquire a timing measurement on all enabled channels for specified
            number of passes (default 1)
  adjust   :Adjust the "current" (default), "top", or "bottom" actuator
            positions interactively
  check    :Check reliability of all-channel triggering between the top and
            bottom positions, over the specified number of passes (default is
            one)
  clear    :Clear screen.
  device   :Device commands
  devmem   :Read/write physical memory
            Usage:
            Read memory at address with optional width:
            devmem address [width]
            Write memory at address with mandatory width and value:
            devmem address <width> <value>
  down     :Move actuator down (n steps)
  get      :Get parameters
  help     :Prints the help message.
  history  :Command history.
  kernel   :Kernel commands
  measure  :Acquire a timing measurement on all enabled channels for specified
            number of passes (default 1), and output "brief" (default), or
            "verbose" results
  off      :Turn off actuator
  on       :Turn on actuator
  print    :Print the last timing measurement in a "brief" (default) or
            "verbose" format
  resize   :Console gets terminal screen size or assumes default in case the
            readout fails. It must be executed after each terminal width
            change to ensure correct text display.
  set      :Set parameters
  setup    :Make sure the actuator is on, and setup top and bottom positions
            specified number of steps (default 1) around the trigger point.
            Verify trigger with specified number of passes (default 2).
  shell    :Useful, not Unix-like shell commands.
  swing    :Move actuator back-n-forth within n steps around current position,
            until interrupted
  tighten  :Move the top and bottom positions within the specified number of
            steps (default 1) around the trigger point. Verify trigger with
            specified number of passes (default 2).
  up       :Move actuator up (n steps)
```

Here's an example session beginning at the power-on, configuring two channels
to detect pressure and release of a directly-connected key switch (Cherry MX
Brown), setting up the top and bottom positions tightly around the trigger
point, verifying that the trigger is reliable through 32 passes, and then
measuring timing of 256 passes (half down, and half up):

```
*** Booting Zephyr OS build zephyr-v3.3.0-474-geead89e7f22d ***


keypecker:~$ resize
keypecker:~$ set ch 0 down rising DOWN
keypecker:~$ set ch 1 up falling UP
keypecker:~$ setup
Actuator is off.
Move the actuator manually to a point above the trigger, and press Enter.
Press Ctrl-C to abort.

Actuator is on.
The current position is the top.
Moving one step down.
Press up and down arrow keys to move the actuator to a point below the
trigger, and press Enter.
Press Ctrl-C to abort.

Bottom position is set.
Tightening around the trigger point.
Press Ctrl-C to abort.

Setup complete.


keypecker:~$ check 32
100%


keypecker:~$ measure 256
                      #0              #1
                    DOWN              UP
-------- --------------- ---------------
    Both           Value           Value
-------- --------------- ---------------
Trigs, %            +100            +100
 Min, us           +2680           +1900
Mean, us           +2800           +1930
 Max, us           +2920           +1960
-------- --------------- ---------------
    Time        Triggers        Triggers
-------- --------------- ---------------
Both, us 0           105 0           128
         |             : |_____________:
    1900 |             : |_____________|
    1963 |             : |             :
    2026 |             : |             :
    2089 |             : |             :
    2152 |             : |             :
    2215 |             : |             :
    2278 |             : |             :
    2341 |             : |             :
    2404 |             : |             :
    2467 |             : |             :
    2530 |             : |             :
    2593 |_            : |             :
    2656 | |___________: |             :
    2719 |_____________| |             :
    2782 |             : |             :
    2845 |             : |             :
    2908 |             : |             :
-------- --------------- ---------------


keypecker:~$
```

The plus `+` signs before the measured numbers indicate overcapture
(bouncing), which is normal for physical switches.
