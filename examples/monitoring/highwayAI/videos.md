# Highway AI Simulation Replays

The folder `./src/main/resources/` contains two video replays of the autonomous driving case study presented in our paper on monitoring reinforcement learning agents with **STARK**.

## Playing the videos

GitHub supports embedded playback directly in the browser. Simply click the video files:

- [`short.mp4`](../../../src/main/resources/short.mp4)
- [`long.mp4`](../../../src/main/resources/long.mp4)

You can also clone the repository and play them locally with any standard media player.

## Videos

### `short.mp4`

Simulation controlled by the RL agent **short**, which was trained for a shorter amount of time.

In this scenario, the ego vehicle initially drives on a free lane, but later performs an unsafe lane change behind a slower vehicle. It narrowly avoids a first collision by changing lanes again, but eventually crashes into oncoming traffic.

This simulation showcases:
- unsafe lane changes,
- violations of safety distance requirements,
- eventual crash behaviour,
- non-monotonic monitoring behaviour for some specifications.

---

### `long.mp4`

Simulation controlled by the RL agent **long**, which was trained three times longer than **short**.

Here, the ego vehicle starts behind traffic and gradually approaches slower vehicles ahead. Instead of performing risky manoeuvres, it eventually adapts its speed and maintains a safer following distance.

This simulation showcases:
- smoother longitudinal control,
- improved safety-distance maintenance,
- advisory-speed-limit behaviour,
- increased robustness with respect to monitored specifications.

---

## Related implementation

The implementation of the monitored highway scenario can be found in:

`./src/main/java/monitoring/AIMultipleLanes.java`
