# Robustness Analysis of a Traffic Scenario under Perturbations using STARK

Bachelor Final Project (2ICS00) — Michał Surażyński, Eindhoven University of Technology.

This project models and verifies robustness properties in a two-lane highway scenario with autonomous and human-driven vehicles using [STARK](https://github.com/the-stark-tool/STARK) and Robustness Temporal Logic (RobTL). Three perturbations are studied: brake-checker, reckless merger, and no-merger. Three robustness properties are verified: safety (saf), target (tar), and stay-in-lane (sil).

## Requirements

- Java 17
- Gradle 7.4.2
- [just]
- [uv]

## Setup

### 1. Build STARK locally

STARK is not available on a public package registry and must be built and published to your local Maven repository manually.

```bash
git clone https://github.com/the-stark-tool/STARK
cd STARK
./gradlew publishToMavenLocal -Pversion=1.0.0
```

### 2. Install Python dependencies

```bash
uv sync --project scripts/
```

## Running scenarios

Each run generates output files under `generated_files/<scenario>/`, containing:
- `sampled_trajectories/` — sampled trajectory CSVs (always produced)
- `violating_trajectories/saf|tar|sil/` — violating trajectories (only if violations found)

```bash
just run-no-merger        # No-merger, eta 0.0 to 1.0
just run-brake-checker    # Brake-checker, perturbation applied at t=0,10,...,140
just run-reckless-merger  # Reckless merger
```

## Visualizing results

See `just --list` and `just help` for full usage. Quick reference:

```bash
# 2D snapshot of initial conditions
just graph-init-datastate no-merger-0.5 -s
just graph-init-datastate reckless-merger -v -saf

# 3D trajectory plot (x, y, time)
just graph-traj brake-checker-40 -v -saf
just graph-traj no-merger-0.3 -s --save fig.png

# Animated trajectory playback
just play-traj no-merger-0.5 -s
just play-traj reckless-merger -v -sil
```
