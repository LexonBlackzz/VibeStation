# VibeStation 2 frontend sounds and images

On Windows, VibeStation.exe has these files built in (resources/vibestation.rc
embeds each one as a named resource: its file name in capitals with - and . as
_). The standalone PS2 lab, other platforms, and any file missing from the exe
read them from this folder instead (copied next to the executable on every
build). VibeStation 2 works without any of them:

| File | Used for | Without it |
|---|---|---|
| `vs2-boot.wav` | Sound of the boot animation; it plays on under the menu reveal. | The boot animation plays silently. |
| `vs2-backtomenu.wav` | The menu animating in without the boot animation: coming back from VibeStation 1, or with Startup animation set to Skip. Off with Menu sounds. | Silent |
| `vs2-highlight.wav` | Moving the selection | Silent |
| `vs2-menuopen.wav` | Opening a screen | Silent |
| `vs2-menuclose.wav` | Going back | Silent |
| `vs2-selected.wav` | Confirming an action | Silent |
| `vs2-ambientbg.ogg` | Menu ambience (Ogg Vorbis to keep the 2:48 loop small; a `.wav` also works): a quiet loop; every pass after the first is replayed at a new tape speed and pitch. | No background loop |
| `vs2-certainstatic.wav` | Menu ambience: washes in at random intervals like waves, each slowed to 1.8-2.6x its length with a new pitch, low-pass and stereo position, and soft fades. | No static waves |
| `ps.png` | Logo on PS1 disc tiles in the Browser | The drawn disc |
| `ps2.png` | Logo on PS2 disc tiles in the Browser. A logo on a white background is turned into a light logo on transparency. | The drawn disc |

The boot animation itself is drawn in real time (`ui/vs2/vs2_boot.cpp`).

Some of these sounds and both logos come from Sony's PlayStation consoles. They are
included under fair use, for a non-commercial fan project, and do not imply any
affiliation with or endorsement by Sony Interactive Entertainment. See the
disclaimer in the main README.
