# VibeStation 2 frontend sounds

The frontend looks for these files here (copied next to the executable on every
build) and works without any of them:

| File | Used for | Without it |
|---|---|---|
| `vs2-boot.wav` | Sound of the boot animation; it plays on under the menu reveal. | The boot animation plays silently. |
| `vs2-backtomenu.wav` | The menu animating in without the boot animation: coming back from VibeStation 1, or with Startup animation set to Skip. Off with Menu sounds. | Silent |
| `vs2-highlight.wav` | Moving the selection | Silent |
| `vs2-menuopen.wav` | Opening a screen | Silent |
| `vs2-menuclose.wav` | Going back | Silent |
| `vs2-selected.wav` | Confirming an action | Silent |
| `vs2-ambientbg.wav` | Menu ambience: a quiet loop; every pass after the first is replayed at a new tape speed and pitch. | No background loop |
| `vs2-certainstatic.wav` | Menu ambience: washes in at random intervals like waves, each slowed to 1.8-2.6x its length with a new pitch, low-pass and stereo position, and soft fades. | No static waves |

The boot animation itself is drawn in real time (`ui/vs2/vs2_boot.cpp`).

These files are ignored by git on purpose. Do not commit sounds taken from
Sony's PS2 BIOS; use original or recreated sounds for anything that ships.
