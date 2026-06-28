# VibeStation Compatibility

Compatibility notes are based on limited local testing. Unless noted otherwise,
tests use the interpreter backend.

## Status Legend

- Playable
- Mostly Playable
- Boots
- Intro/Menu
- Broken
- Untested

## Game Compatibility

| Game | Region | Status | Backend | Notes |
|---|---|---|---|---|
| Bloody Roar | Unknown | Mostly Playable | Interpreter | Mostly 60 FPS, but some visual effects can drop to around 40 FPS. |
| Crash Bandicoot | Unknown | Mostly Playable | Interpreter | Tested for a few minutes. Minor texture artifacting is present, and audio drift may still exist. |
| Ridge Racer Type 4 | Unknown | Boots | Interpreter | MDEC/FMV works, and all menus worked as of testing. Not tested long enough to rate full gameplay. |
| Gran Turismo 2 | USA | Playable | Interpreter | Everything works as far as tested. |
| Gran Turismo 2 | Europe | Playable | Interpreter | Everything works as far as tested. |
| A-Train | Unknown | Boots | Interpreter | Used for CPU backend/JIT testing. FMV gets stuck on the last frame; restarting the BIOS is needed to get into the game. |
| Barbie Super Sports | Unknown | Intro/Menu | Interpreter | Works through gear selection, then gets stuck forever on a loading screen when entering the actual game. |
| Tekken 3 | Unknown | Mostly Playable | Interpreter | FMV may look wrong, but the rest of the game seems to work okay. |
