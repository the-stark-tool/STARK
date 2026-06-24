package two_lanes_merge.car;

public record NearestCars(
    CarPhysicalState leftLaneBehindCar,
    CarPhysicalState leftLaneAheadCar,
    CarPhysicalState rightLaneBehindCar,
    CarPhysicalState rightLaneAheadCar) {}
