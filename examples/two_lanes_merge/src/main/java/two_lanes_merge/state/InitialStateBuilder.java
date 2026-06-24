package two_lanes_merge.state;

import static two_lanes_merge.constants.Action.*;
import static two_lanes_merge.constants.Encodings.*;

import two_lanes_merge.car.Car;
import two_lanes_merge.car.CarRegistry;
import two_lanes_merge.util.MyUtil;
import java.util.HashMap;
import java.util.Map;
import stark.ds.DataState;

public final class InitialStateBuilder {

  private InitialStateBuilder() {
    // Utility class
  }

  private static void initDistanceForCar(
      DataState dataState,
      Car self,
      Car other,
      int distIdx,
      int safetyGapIdx,
      boolean selfIsBehind) {
    if (other != null) {
      double dist = MyUtil.calculateDistanceBetweenCars(self, other, dataState);
      double gap =
          selfIsBehind
              ? MyUtil.calculateRSSSafetyDistance(self, other, dataState)
              : MyUtil.calculateRSSSafetyDistance(other, self, dataState);
      dataState.set(distIdx, dist);
      dataState.set(safetyGapIdx, gap);
    } else {
      dataState.set(distIdx, Double.MAX_VALUE);
      dataState.set(safetyGapIdx, 0.0);
    }
  }

  public static DataState buildInitialState(CarRegistry carRegistery) {

    Map<Integer, Double> values = new HashMap<>();

    for (Car car : carRegistery.getCars()) {
      values.put(car.xIdx(), car.getCarInitialConditions().initX());
      values.put(car.yIdx(), car.getCarInitialConditions().initY());
      values.put(car.speedIdx(), car.getCarInitialConditions().initSpeed());
      values.put(car.accelerationIdx(), car.getCarInitialConditions().initAcceleration());
      values.put(car.steerAngleIdx(), car.getCarInitialConditions().initSteerAngle());
      values.put(car.maxSpeedIdx(), car.getCarInitialConditions().initMaxSpeed());
      values.put(car.actionQueueOffsetIdx(), car.getCarInitialConditions().initActionQueueOffset());
      values.put(car.targetSpeedIdx(), car.getCarInitialConditions().initTargetSpeed());
      values.put(
          car.mergeRssMultiplierIdx(), car.getCarInitialConditions().initMergeRSSMultiplier());
      values.put(car.targetLaneIdx(), car.getCarInitialConditions().initTargetLane());
      values.put(car.minComfortSpeedIdx(), car.getCarInitialConditions().minComfortSpeed());

      values.put(car.accelerationCommandIdx(), IDLE);
      values.put(car.steerCommandIdx(), STEER_STRAIGHT);

      values.put(car.finishedIdx(), CAR_NOT_FINISHED);
      values.put(car.idIdx(), car.getCarMetaData().id());

      // Place holder values
      // Easier to calculate when values converted to dataState
      values.put(car.distAheadLeftLaneIdx(), 0.0);
      values.put(car.distBehindLeftLaneIdx(), 0.0);
      values.put(car.safetyGapAheadLeftLaneIdx(), 0.0);
      values.put(car.safetyGapBehindLeftLaneIdx(), 0.0);
      values.put(car.distAheadRightLaneIdx(), 0.0);
      values.put(car.distBehindRightLaneIdx(), 0.0);
      values.put(car.safetyGapAheadRightLaneIdx(), 0.0);
      values.put(car.safetyGapBehindRightLaneIdx(), 0.0);
    }

    values.put(carRegistery.saf(), 0.0);
    values.put(carRegistery.tar(), 0.0);
    values.put(carRegistery.sil(), 0.0);

    DataState dataState =
        new DataState(carRegistery.numberOfVariables(), i -> values.getOrDefault(i, Double.NaN));

    for (Car self : carRegistery.getCars()) {
      Car ahead = carRegistery.getNearestCar(self, dataState, DIRECTION_AHEAD);
      Car behind = carRegistery.getNearestCar(self, dataState, DIRECTION_BEHIND);

      initDistanceForCar(
          dataState,
          self,
          ahead,
          self.distAheadLeftLaneIdx(),
          self.safetyGapAheadLeftLaneIdx(),
          true);
      initDistanceForCar(
          dataState,
          self,
          behind,
          self.distBehindLeftLaneIdx(),
          self.safetyGapBehindLeftLaneIdx(),
          false);
      initDistanceForCar(
          dataState,
          self,
          ahead,
          self.distAheadRightLaneIdx(),
          self.safetyGapAheadRightLaneIdx(),
          true);
      initDistanceForCar(
          dataState,
          self,
          behind,
          self.distBehindRightLaneIdx(),
          self.safetyGapBehindRightLaneIdx(),
          false);
    }
    return dataState;
  }
}
