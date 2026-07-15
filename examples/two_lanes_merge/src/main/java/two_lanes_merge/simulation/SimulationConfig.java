package two_lanes_merge.simulation;

public record SimulationConfig(
    int NUM_SAMPLES_EVOLUTION_SEQUENCE,
    double ETA,
    int THREE_VAL_SAMPLE_COUNT,
    double THREE_VAL_CONFIDENCE) {}
