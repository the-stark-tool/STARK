package two_lanes_merge.car;

public record CarConfig(
    String name,
    int id,
    Car.CarType type,
    double initMaxSpeed,
    double initX,
    double initY,
    double initSpeed,
    double initAcceleration,
    double initSteerAngle,
    double initTurnOffset,
    double targetSpeed,
    double initTargetLane,
    double initMergeRSSMultiplier,
    double initMinComfortSpeed) {}
