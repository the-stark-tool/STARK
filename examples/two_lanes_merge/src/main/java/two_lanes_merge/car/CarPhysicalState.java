package two_lanes_merge.car;

public record CarPhysicalState(
    double x, double y, double speed, double acceleration, double steerAngle, double lane) {}
