package two_lanes_merge.car;

import static two_lanes_merge.constants.Encodings.*;

import two_lanes_merge.util.MyUtil;
import java.util.*;
import stark.ds.DataState;

public class Car {
  public enum CarType {
    CONTROLLED_CAR,
    REGULAR_CAR
  }

  public record CarInitialConditions(
      double initX,
      double initY,
      double initSpeed,
      double initAcceleration,
      double initSteerAngle,
      double initMaxSpeed,
      double initActionQueueOffset,
      double initTargetSpeed,
      double initMergeRSSMultiplier,
      double initTargetLane,
      double minComfortSpeed) {}

  public record CarMetaData(int base, double id, CarType type) {}

  private final CarInitialConditions carInitialConditions;
  private final CarMetaData carMetaData;

  public Car(CarInitialConditions carInitialConditions, CarMetaData carInitialMetaData) {
    this.carInitialConditions = carInitialConditions;
    this.carMetaData = carInitialMetaData;
  }

  public boolean isAnyRssViolated(double lane, DataState dataState, boolean forMerge) {
    record GapCheck(int distIdx, int safeIdx) {}

    List<GapCheck> checks = new ArrayList<>();
    if (lane != RIGHT_LANE) {
      checks.add(new GapCheck(distAheadLeftLaneIdx(), safetyGapAheadLeftLaneIdx()));
      checks.add(new GapCheck(distBehindLeftLaneIdx(), safetyGapBehindLeftLaneIdx()));
    }
    if (lane != LEFT_LANE) {
      checks.add(new GapCheck(distAheadRightLaneIdx(), safetyGapAheadRightLaneIdx()));
      checks.add(new GapCheck(distBehindRightLaneIdx(), safetyGapBehindRightLaneIdx()));
    }

    return checks.stream()
        .anyMatch(
            c ->
                MyUtil.isRssViolated(
                    dataState.get(c.distIdx()),
                    dataState.get(c.safeIdx()),
                    forMerge,
                    this.carInitialConditions.initMergeRSSMultiplier));
  }

  public double getWorstRssViolationDirection(double lane, DataState dataState, boolean forMerge) {
    record GapCheck(int distIdx, int safeIdx, double direction) {}

    List<GapCheck> checks = new ArrayList<>();
    if (lane != RIGHT_LANE) {
      checks.add(
          new GapCheck(distAheadLeftLaneIdx(), safetyGapAheadLeftLaneIdx(), DIRECTION_AHEAD));
      checks.add(
          new GapCheck(distBehindLeftLaneIdx(), safetyGapBehindLeftLaneIdx(), DIRECTION_BEHIND));
    }
    if (lane != LEFT_LANE) {
      checks.add(
          new GapCheck(distAheadRightLaneIdx(), safetyGapAheadRightLaneIdx(), DIRECTION_AHEAD));
      checks.add(
          new GapCheck(distBehindRightLaneIdx(), safetyGapBehindRightLaneIdx(), DIRECTION_BEHIND));
    }

    double worstLength = 0;
    double worstDirection = DIRECTION_UNDEFINED;
    for (GapCheck check : checks) {
      double dist = dataState.get(check.distIdx());
      double safe = dataState.get(check.safeIdx());
      if (MyUtil.isRssViolated(dist, safe, forMerge, carInitialConditions.initMergeRSSMultiplier)) {
        double v = MyUtil.calculateRssViolationLength(dist, safe);
        if (v > worstLength) {
          worstLength = v;
          worstDirection = check.direction();
        }
      }
    }
    return worstDirection;
  }

  public CarInitialConditions getCarInitialConditions() {
    return carInitialConditions;
  }

  public CarMetaData getCarMetaData() {
    return carMetaData;
  }

  // --- Initial Conditions ---

  public int xIdx() {
    return carMetaData.base + 0;
  }

  public int yIdx() {
    return carMetaData.base + 1;
  }

  public int speedIdx() {
    return carMetaData.base + 2;
  }

  public int accelerationIdx() {
    return carMetaData.base + 3;
  }

  public int steerAngleIdx() {
    return carMetaData.base + 4;
  }

  public int maxSpeedIdx() {
    return carMetaData.base + 5;
  }

  public int actionQueueOffsetIdx() {
    return carMetaData.base + 6;
  }

  public int targetSpeedIdx() {
    return carMetaData.base + 7;
  }

  public int mergeRssMultiplierIdx() {
    return carMetaData.base + 8;
  }

  public int targetLaneIdx() {
    return carMetaData.base + 9;
  }

  public int minComfortSpeedIdx() {
    return carMetaData.base + 10;
  }

  // --- Unchangable initial conditions ---

  public int accelerationCommandIdx() {
    return carMetaData.base + 11;
  }

  public int steerCommandIdx() {
    return carMetaData.base + 12;
  }

  // --- Helper variables ---

  public int distAheadRightLaneIdx() {
    return carMetaData.base + 13;
  }

  public int distBehindRightLaneIdx() {
    return carMetaData.base + 14;
  }

  public int distAheadLeftLaneIdx() {
    return carMetaData.base + 15;
  }

  public int distBehindLeftLaneIdx() {
    return carMetaData.base + 16;
  }

  public int safetyGapBehindRightLaneIdx() {
    return carMetaData.base + 17;
  }

  public int safetyGapAheadRightLaneIdx() {
    return carMetaData.base + 18;
  }

  public int safetyGapAheadLeftLaneIdx() {
    return carMetaData.base + 19;
  }

  public int safetyGapBehindLeftLaneIdx() {
    return carMetaData.base + 20;
  }

  public int finishedIdx() {
    return carMetaData.base + 21;
  }

  public int idIdx() {
    return carMetaData.base + 22;
  }

  public int laneIdx() {
    return carMetaData.base + 23;
  }
}
