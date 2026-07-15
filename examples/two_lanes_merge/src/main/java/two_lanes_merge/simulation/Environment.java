package two_lanes_merge.simulation;

import static two_lanes_merge.constants.Action.*;
import static two_lanes_merge.constants.Encodings.*;
import static two_lanes_merge.constants.PhysicsConstants.*;

import two_lanes_merge.car.Car;
import two_lanes_merge.car.Car.CarType;
import two_lanes_merge.car.CarPhysicalState;
import two_lanes_merge.car.CarRegistry;
import two_lanes_merge.car.NearestCars;
import two_lanes_merge.util.MyUtil;
import java.util.*;
import org.apache.commons.math3.random.RandomGenerator;
import stark.ds.DataState;
import stark.ds.DataStateUpdate;

record CarDecision(double accelerationCommand, double steerCommand) {}

interface DrivingStrategy {
  CarDecision decide(RandomGenerator rng, DataState dataState, Car self, CarRegistry carRegistery);
}

class ControlledCarStrategy implements DrivingStrategy {
  @Override
  public CarDecision decide(
      RandomGenerator rng, DataState dataState, Car self, CarRegistry carRegistery) {
    double accelerationCommand = dataState.get(self.accelerationCommandIdx());
    double steerCommand = dataState.get(self.steerCommandIdx());

    return new CarDecision(accelerationCommand, steerCommand);
  }
}

class RegularCarStrategy implements DrivingStrategy {
  @Override
  public CarDecision decide(
      RandomGenerator rng, DataState dataState, Car self, CarRegistry carRegistery) {
    double steerCommand = dataState.get(self.steerCommandIdx());
    double selfLane = MyUtil.getMyLane(dataState.get(self.yIdx()));
    double targetLane = dataState.get(self.targetLaneIdx());
    double selfSpeed = dataState.get(self.speedIdx());
    double targetSpeed = dataState.get(self.targetSpeedIdx());
    double accelerationCommand;

    // Get state
    boolean inMerge =
        steerCommand == STEER_LEFT
            || steerCommand == STEER_RIGHT
            || selfLane == BETWEEN_LEFT_AND_RIGHT_LANE;
    boolean onTargetLane = !inMerge && selfLane == targetLane;

    // While merging
    if (inMerge) {
      // Reached target lane — stop merging
      if (selfLane == targetLane) {
        return new CarDecision(IDLE, STEER_STRAIGHT);
      }

      // RSS violated during merge
      if (self.isAnyRssViolated(BETWEEN_LEFT_AND_RIGHT_LANE, dataState, true)) {
        steerCommand = targetLane == LEFT_LANE ? STEER_LEFT : STEER_RIGHT;
        if (self.getWorstRssViolationDirection(BETWEEN_LEFT_AND_RIGHT_LANE, dataState, true)
            == DIRECTION_AHEAD) {
          return new CarDecision(SLOWER, steerCommand);
        } else {
          return new CarDecision(FASTER, steerCommand);
        }
      }

      // Maintain speed while merging
      steerCommand = targetLane == LEFT_LANE ? STEER_LEFT : STEER_RIGHT;
      if (selfSpeed < targetSpeed) {
        accelerationCommand = FASTER;
      } else if (selfSpeed > targetSpeed) {
        accelerationCommand = SLOWER;
      } else {
        accelerationCommand = IDLE;
      }
      return new CarDecision(accelerationCommand, steerCommand);
    }

    // On targret lane stay safe
    if (onTargetLane) {
      steerCommand = STEER_STRAIGHT;
      if (self.isAnyRssViolated(selfLane, dataState, false)) {
        if (self.getWorstRssViolationDirection(selfLane, dataState, false) == DIRECTION_AHEAD) {
          return new CarDecision(SLOWER, steerCommand);
        } else {
          return new CarDecision(FASTER, steerCommand);
        }
      }

      if (selfSpeed < targetSpeed) accelerationCommand = FASTER;
      else if (selfSpeed > targetSpeed) accelerationCommand = SLOWER;
      else accelerationCommand = IDLE;
      return new CarDecision(accelerationCommand, steerCommand);
    }

    // WantToMerge
    // Check own lane first
    if (self.isAnyRssViolated(selfLane, dataState, true)) {
      if (self.getWorstRssViolationDirection(selfLane, dataState, true) == DIRECTION_AHEAD) {
        return new CarDecision(SLOWER, STEER_STRAIGHT);
      } else {
        return new CarDecision(FASTER, STEER_STRAIGHT);
      }
    }
    // Then check other lane
    if (self.isAnyRssViolated(targetLane, dataState, true)) {
      if (self.getWorstRssViolationDirection(targetLane, dataState, true) == DIRECTION_AHEAD) {
        accelerationCommand = selfSpeed <= dataState.get(self.minComfortSpeedIdx()) ? IDLE : SLOWER;
      } else {
        accelerationCommand = FASTER;
      }
      return new CarDecision(accelerationCommand, STEER_STRAIGHT);
    }

    // Safe to merge
    steerCommand = targetLane == LEFT_LANE ? STEER_LEFT : STEER_RIGHT;
    if (selfSpeed < targetSpeed) {
      accelerationCommand = FASTER;
    } else if (selfSpeed > targetSpeed) {
      accelerationCommand = SLOWER;
    } else {
      accelerationCommand = IDLE;
    }
    return new CarDecision(accelerationCommand, steerCommand);
  }
}

public class Environment {

  private Environment() {
    // Utility class
  }

  private static CarPhysicalState applyPhysics(
      List<DataStateUpdate> updates,
      DataState dataState,
      CarDecision decision,
      Car self,
      CarRegistry carRegistery,
      RandomGenerator rng,
      SimulationConfig simulationConfig) {

    double oldSpeed = dataState.get(self.speedIdx());
    double oldAcceleration = dataState.get(self.accelerationIdx());
    double oldSteeringAngle = dataState.get(self.steerAngleIdx());

    // --- New coordinates
    double travelX = (oldAcceleration / 2 + oldSpeed) * Math.cos(oldSteeringAngle);
    double travelY = (oldAcceleration / 2 + oldSpeed) * Math.sin(oldSteeringAngle);

    double newX = dataState.get(self.xIdx()) + travelX;

    if (newX > ROAD_LENGHT) {
      updates.add(new DataStateUpdate(self.finishedIdx(), CAR_FINISHED));
    }

    double newY = Math.min(8, Math.max(0, dataState.get(self.yIdx()) + travelY));
    double newLane = MyUtil.getMyLane(newY);

    // --- New acceleration
    double accelerationCommand = decision.accelerationCommand();
    double newAcceleration;
    if (accelerationCommand == FASTER) {
      newAcceleration = MyUtil.sampleAcceleration(rng);
    } else if (accelerationCommand == SLOWER) {
      newAcceleration = MyUtil.sampleBrake(rng);
    } else {
      newAcceleration = MyUtil.sampleIdle(rng);
    }

    // --- New speed
    double newSpeed = dataState.get(self.speedIdx()) + newAcceleration;

    if (newSpeed > dataState.get(self.maxSpeedIdx())) {
      newSpeed = dataState.get(self.maxSpeedIdx());
    }
    if (newSpeed < 0) {
      newSpeed = 0;
    }

    // --- New steering angle
    double newSteerAngle = 0;
    if (decision.steerCommand() == STEER_LEFT) {
      newSteerAngle = Math.PI / 18;
    } else if (decision.steerCommand() == STEER_RIGHT) {
      newSteerAngle = -Math.PI / 18;
    }

    // --- New action queue offset
    double oldActionQueueOffset = dataState.get(self.actionQueueOffsetIdx());
    double newActionQueueOffset =
        oldActionQueueOffset == 0 ? ACTION_QUEUE_LENGTH : oldActionQueueOffset - 1;

    updates.add(new DataStateUpdate(self.xIdx(), newX));
    updates.add(new DataStateUpdate(self.yIdx(), newY));
    updates.add(new DataStateUpdate(self.speedIdx(), newSpeed));
    updates.add(new DataStateUpdate(self.laneIdx(), newLane));
    updates.add(new DataStateUpdate(self.accelerationIdx(), newAcceleration));
    updates.add(new DataStateUpdate(self.steerAngleIdx(), newSteerAngle));
    updates.add(new DataStateUpdate(self.actionQueueOffsetIdx(), newActionQueueOffset));
    updates.add(new DataStateUpdate(self.steerCommandIdx(), decision.steerCommand()));
    updates.add(new DataStateUpdate(self.accelerationCommandIdx(), decision.accelerationCommand()));

    return new CarPhysicalState(newX, newY, newSpeed, newAcceleration, newSteerAngle, newLane);
  }

  private static void updateDistanceForCar(
      List<DataStateUpdate> updates,
      CarPhysicalState self,
      CarPhysicalState other,
      int distIdx,
      int safetyGapIdx,
      boolean selfIsBehind) {
    if (other != null) {
      updates.add(new DataStateUpdate(distIdx, MyUtil.calculateDistanceBetweenCars(self, other)));
      double gap =
          selfIsBehind
              ? MyUtil.calculateRSSSafetyDistance(self, other)
              : MyUtil.calculateRSSSafetyDistance(other, self);
      updates.add(new DataStateUpdate(safetyGapIdx, gap));
    } else {
      updates.add(new DataStateUpdate(distIdx, Double.MAX_VALUE));
      updates.add(new DataStateUpdate(safetyGapIdx, Double.MAX_VALUE));
    }
  }

  private static void updateDistances(
      CarPhysicalState selfNew, Car self, NearestCars nearestCars, List<DataStateUpdate> updates) {

    updateDistanceForCar(
        updates,
        selfNew,
        nearestCars.leftLaneAheadCar(),
        self.distAheadLeftLaneIdx(),
        self.safetyGapAheadLeftLaneIdx(),
        true);
    updateDistanceForCar(
        updates,
        selfNew,
        nearestCars.leftLaneBehindCar(),
        self.distBehindLeftLaneIdx(),
        self.safetyGapBehindLeftLaneIdx(),
        false);
    updateDistanceForCar(
        updates,
        selfNew,
        nearestCars.rightLaneAheadCar(),
        self.distAheadRightLaneIdx(),
        self.safetyGapAheadRightLaneIdx(),
        true);
    updateDistanceForCar(
        updates,
        selfNew,
        nearestCars.rightLaneBehindCar(),
        self.distBehindRightLaneIdx(),
        self.safetyGapBehindRightLaneIdx(),
        false);
  }

  private static void checkCrashWithCar(
      CarPhysicalState self,
      CarPhysicalState other,
      CarRegistry carRegistery,
      List<DataStateUpdate> updates) {
    if (other != null && MyUtil.doesCrashHappen(self, other)) {
      updates.add(new DataStateUpdate(carRegistery.saf(), 1));
    }
  }

  private static void updateSaf(
      CarPhysicalState self,
      NearestCars nearestCars,
      List<DataStateUpdate> updates,
      CarRegistry carRegistery,
      DataState dataState) {
    // INFO: We only care about the first crash and only about the first two cars that participate
    // in the crash
    if (dataState.get(carRegistery.saf()) > 0) return;

    checkCrashWithCar(self, nearestCars.leftLaneAheadCar(), carRegistery, updates);
    checkCrashWithCar(self, nearestCars.leftLaneBehindCar(), carRegistery, updates);
    checkCrashWithCar(self, nearestCars.rightLaneAheadCar(), carRegistery, updates);
    checkCrashWithCar(self, nearestCars.rightLaneBehindCar(), carRegistery, updates);
  }

  private static void updateTar(
      CarPhysicalState self,
      double targetLane,
      NearestCars nearestCars,
      List<DataStateUpdate> updates,
      CarRegistry carRegistery,
      DataState dataState) {
    if (self.x() >= ROAD_LENGHT && MyUtil.getMyLane(self) != targetLane) {
      updates.add(new DataStateUpdate(carRegistery.tar(), 1));
    }
  }

  private static void updateSil(
      CarPhysicalState self,
      NearestCars nearestCars,
      List<DataStateUpdate> updates,
      CarRegistry carRegistery,
      DataState dataState) {
    if (self.x() >= ROAD_LENGHT && MyUtil.getMyLane(self) == BETWEEN_LEFT_AND_RIGHT_LANE) {
      updates.add(new DataStateUpdate(carRegistery.sil(), 1));
    }

    if (MyUtil.getMyLane(self) == RIGHT_FROM_RIGHT_LANE
        || MyUtil.getMyLane(self) == LEFT_FROM_LEFT_LANE) {
      updates.add(new DataStateUpdate(carRegistery.sil(), 1));
    }
  }

  public static List<DataStateUpdate> getEnvironmentUpdates(
      RandomGenerator rng,
      DataState dataState,
      CarRegistry carRegistery,
      SimulationConfig simulationConfig) {
    List<DataStateUpdate> updates = new LinkedList<>();

    List<Car> cars = carRegistery.getCars();

    Map<Car, CarPhysicalState> physicalStates = new LinkedHashMap<>();

    for (Car self : cars) {
      // Don't update finished cars
      if (dataState.get(self.finishedIdx()) == CAR_FINISHED) {
        continue;
      }

      if (dataState.get(self.actionQueueOffsetIdx()) == 0) {
        // timer expired - make new decision

        DrivingStrategy strategy =
            self.getCarMetaData().type() == Car.CarType.CONTROLLED_CAR
                ? new ControlledCarStrategy()
                : new RegularCarStrategy();

        CarDecision carDecision = strategy.decide(rng, dataState, self, carRegistery);

        physicalStates.put(
            self,
            applyPhysics(
                updates, dataState, carDecision, self, carRegistery, rng, simulationConfig));
      } else {
        // carry forward pervious values, decrease timer
        CarDecision carDecision =
            new CarDecision(
                dataState.get(self.accelerationCommandIdx()),
                dataState.get(self.steerCommandIdx()));

        physicalStates.put(
            self,
            applyPhysics(
                updates, dataState, carDecision, self, carRegistery, rng, simulationConfig));
      }
    }

    for (Car self : cars) {
      // Don't update finished cars
      if (dataState.get(self.finishedIdx()) == CAR_FINISHED) {
        continue;
      }

      CarPhysicalState selfNew = physicalStates.get(self);
      NearestCars nearestCars = carRegistery.getNearestCars(selfNew, physicalStates);
      if (self.getCarMetaData().type() == CarType.CONTROLLED_CAR) {
        updateSaf(selfNew, nearestCars, updates, carRegistery, dataState);
        updateTar(
            selfNew,
            dataState.get(self.targetLaneIdx()),
            nearestCars,
            updates,
            carRegistery,
            dataState);
        updateSil(selfNew, nearestCars, updates, carRegistery, dataState);
      }
      updateDistances(selfNew, self, nearestCars, updates);
    }

    return updates;
  }
}
