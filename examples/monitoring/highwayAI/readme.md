# Monitoring under uncertainty with DisTL — Case Study Artifact

This repository contains the implementation and experimental material for the autonomous-driving case study presented in “Monitoring under uncertainty with DisTL”, accepted at Runtime Verification 2026.

The artifact is based on the tool STARK. The relevant Gradle project is located at:

```text
examples/monitoring/highwayAI
```


The case study monitors two recorded autonomous-driving simulations, referred to in the paper as SHORT and LONG, using the partial monitors implemented in STARK.

The experiments evaluate the monitored formulae from the paper case study. The main experiment executes the formula-specific monitoring runs sequentially for the SHORT simulation and then repeats them for the LONG simulation.

For each formula and simulation, the monitor processes the input distribution sequence step by step and records:

- the step-wise monitor verdict; and
- the time required to process the current input distribution and produce the corresponding verdict.

Timing measurements are recorded per step in nanoseconds.

## Running the case study

The experiments are implemented in Java and built as part of the STARK Gradle project.

The camera-ready experiments were executed using **Java 17**.
1. Open or import the STARK repository as a Gradle project.
2. Locate the Gradle project:

   ```text
   examples/monitoring/highwayAI
   ```

3. Run the main Java file for the highway monitoring case study.

The complete experiment runs the monitoring tasks sequentially. It first monitors the **SHORT** simulation once for each formula considered in the paper and then performs the corresponding monitoring runs for the **LONG** simulation.

Experimental results are written to:

```text
examples/monitoring/highwayAI/src/main/resources/experimentResults/
```

A separate `.tsv` file is produced for each monitored simulation. Each row corresponds to one simulation step and contains the monitor output together with the measured response time for that step.


## Software

The monitoring framework is implemented in **STARK (Software Tool for the Analysis of Robustness in the unKnown environment)**.

Development repository:

```text
https://github.com/the-stark-tool/STARK
```
