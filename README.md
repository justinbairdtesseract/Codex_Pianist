# Codex Pianist

Version `1.0.0` snapshot of the Codex Pianist project.

This repository currently contains two working tracks:

- `pianist_robot_v1/`: direct arm/hand control plus Isaac simulation assets and assembly
- `Export_Codex_Pianist/`: stripped-down hardware-control export for another machine

## Version 1.0 Scope

Version `1.0.0` freezes the first assembled simulation of:

- UF850 arm
- Inspire RH56DFX-2R hand
- custom printed adapter

The simulation version marker is here:

- `pianist_robot_v1/simulation/VERSION`

## Clone

This repo uses submodules for upstream source dependencies under `pianist_robot_v1/simulation/sources`.

Clone with:

```bash
git clone --recurse-submodules <repo-url>
```

Or, after clone:

```bash
git submodule update --init --recursive
```
