package two_lanes_merge.car;

import static two_lanes_merge.constants.Encodings.*;

import two_lanes_merge.car.Car.*;
import two_lanes_merge.util.MyUtil;
import java.util.*;
import stark.ds.DataState;

public class CarRegistry {

  private static final int NUM_VARS_PER_CAR = 24;
  private final List<Car> cars = new ArrayList<>();
  private int nextBase = 0;
  private int actionQueueLength;

  public CarRegistry(int actionQueueLength) {
    this.actionQueueLength = actionQueueLength;
  }

  public Car register(CarConfig carConfig) {

    CarInitialConditions carInitialConditions =
        new CarInitialConditions(
            carConfig.initX(),
            carConfig.initY(),
            carConfig.initSpeed(),
            carConfig.initAcceleration(),
            carConfig.initSteerAngle(),
            carConfig.initMaxSpeed(),
            carConfig.initTurnOffset(),
            carConfig.targetSpeed(),
            carConfig.initMergeRSSMultiplier(),
            carConfig.initTargetLane(),
            carConfig.initMinComfortSpeed());

    CarMetaData carInitialMetaData = new CarMetaData(nextBase, carConfig.id(), carConfig.type());

    Car car = new Car(carInitialConditions, carInitialMetaData);

    cars.add(car);
    nextBase += NUM_VARS_PER_CAR;
    return car;
  }

  public int getActionQueueLenght() {
    return actionQueueLength;
  }

  public List<Car> getCars() {
    return Collections.unmodifiableList(cars);
  }

  public Car getCarByType(Car.CarType type) {
    return cars.stream()
        .filter(c -> c.getCarMetaData().type() == type)
        .findFirst()
        .orElseThrow(() -> new IllegalStateException("No car of provided type registered"));
  }

  public Car getCarById(int id, DataState dataState) {
    for (Car car : cars) {
      if (dataState.get(car.idIdx()) == id) {
        return car;
      }
    }
    return null;
  }

  public CarPhysicalState getNearestCar(
      CarPhysicalState self, Map<Car, CarPhysicalState> physicalStates, double relativeTo) {
    double distToNearestCar = Double.MAX_VALUE;
    CarPhysicalState neareastCar = null;

    for (Map.Entry<Car, CarPhysicalState> entry : physicalStates.entrySet()) {
      CarPhysicalState other = entry.getValue();
      if (other == self) {
        continue;
      }
      if (entry.getKey().finishedIdx() == CAR_FINISHED) {
        continue;
      }

      // If otherCar ahead and we look for car behind skip it
      if (other.x() >= self.x() && relativeTo == DIRECTION_BEHIND) {
        continue;
      }

      // If otherCar behind and we look for car ahead skip it
      if (other.x() <= self.x() && relativeTo == DIRECTION_AHEAD) {
        continue;
      }

      double distance = MyUtil.calculateDistanceBetweenCars(self, other);

      if (distance < distToNearestCar) {
        distToNearestCar = distance;
        neareastCar = other;
      }
    }

    return neareastCar;
  }

  public NearestCars getNearestCars(
      CarPhysicalState self, Map<Car, CarPhysicalState> physicalStates) {
    CarPhysicalState leftLaneBehindCar = null;
    double distToLeftLaneBehindCar = Double.MAX_VALUE;

    CarPhysicalState leftLaneAheadCar = null;
    double distToLeftLaneAheadCar = Double.MAX_VALUE;

    CarPhysicalState rightLaneBehindCar = null;
    double distToRightLaneBehindCar = Double.MAX_VALUE;

    CarPhysicalState rightLaneAheadCar = null;
    double distToRightLaneAheadCar = Double.MAX_VALUE;

    for (Map.Entry<Car, CarPhysicalState> entry : physicalStates.entrySet()) {
      CarPhysicalState other = entry.getValue();

      if (other == self) {
        continue;
      }
      if (entry.getKey().finishedIdx() == CAR_FINISHED) {
        continue;
      }

      double otherLane = MyUtil.getMyLane(other);
      double distToOther = MyUtil.calculateDistanceBetweenCars(self, other);

      if (otherLane == BETWEEN_LEFT_AND_RIGHT_LANE) {
        // Add to BOTH buckets
        if (other.x() >= self.x()) {
          if (distToOther < distToLeftLaneAheadCar) {
            distToLeftLaneAheadCar = distToOther;
            leftLaneAheadCar = other;
          }
          if (distToOther < distToRightLaneAheadCar) {
            distToRightLaneAheadCar = distToOther;
            rightLaneAheadCar = other;
          }
        } else {
          if (distToOther < distToLeftLaneBehindCar) {
            distToLeftLaneBehindCar = distToOther;
            leftLaneBehindCar = other;
          }
          if (distToOther < distToRightLaneBehindCar) {
            distToRightLaneBehindCar = distToOther;
            rightLaneBehindCar = other;
          }
        }
      } else if (otherLane == LEFT_FROM_LEFT_LANE || otherLane == LEFT_LANE) {
        if (other.x() >= self.x() && distToOther < distToLeftLaneAheadCar) {
          // Other is on the left lane and ahead of self
          distToLeftLaneAheadCar = distToOther;
          leftLaneAheadCar = other;
        } else if (other.x() < self.x() && distToOther < distToLeftLaneBehindCar) {
          // Other is on the left lane and behind self
          distToLeftLaneBehindCar = distToOther;
          leftLaneBehindCar = other;
        }
      } else if (otherLane == RIGHT_FROM_RIGHT_LANE || otherLane == RIGHT_LANE) {
        if (other.x() >= self.x() && distToOther < distToRightLaneAheadCar) {
          // Other is on the right lane and ahead of self
          distToRightLaneAheadCar = distToOther;
          rightLaneAheadCar = other;
        } else if (other.x() < self.x() && distToOther < distToRightLaneBehindCar) {
          // Other is on the right lane and behind self
          distToRightLaneBehindCar = distToOther;
          rightLaneBehindCar = other;
        }
      }
    }

    return new NearestCars(
        leftLaneBehindCar, leftLaneAheadCar, rightLaneBehindCar, rightLaneAheadCar);
  }

  public Car getNearestCar(Car self, DataState dataState, double relativeTo) {
    double distToNearestCar = Double.MAX_VALUE;
    Car neareastCar = null;

    for (Car otherCar : this.cars) {
      if (otherCar == self) {
        continue;
      }
      if (otherCar.finishedIdx() == CAR_FINISHED) {
        continue;
      }

      // If otherCar ahead and we look for car behind skip it
      if (dataState.get(otherCar.xIdx()) >= dataState.get(self.xIdx())
          && relativeTo == DIRECTION_BEHIND) {
        continue;
      }

      // If otherCar behind and we look for car ahead skip it
      if (dataState.get(otherCar.xIdx()) < dataState.get(self.xIdx())
          && relativeTo == DIRECTION_AHEAD) {
        continue;
      }

      double distance = MyUtil.calculateDistanceBetweenCars(self, otherCar, dataState);

      if (distance < distToNearestCar) {
        distToNearestCar = distance;
        neareastCar = otherCar;
      }
    }

    return neareastCar;
  }

  public Car getNearestControlledCar(Car self, DataState dataState, double relativeTo) {
    double distToNearestCar = Double.MAX_VALUE;
    Car neareastCar = null;

    for (Car otherCar : this.cars) {
      if (otherCar == self) {
        continue;
      }

      if (otherCar.getCarMetaData().type() != Car.CarType.CONTROLLED_CAR) {
        continue;
      }

      // If otherCar ahead and we look for car behind skip it
      if (dataState.get(otherCar.xIdx()) >= dataState.get(self.xIdx())
          && relativeTo == DIRECTION_BEHIND) {
        continue;
      }

      // If otherCar behind and we look for car ahead skip it
      if (dataState.get(otherCar.xIdx()) <= dataState.get(self.xIdx())
          && relativeTo == DIRECTION_AHEAD) {
        continue;
      }

      double distance = MyUtil.calculateDistanceBetweenCars(self, otherCar, dataState);

      if (distance < distToNearestCar) {
        distToNearestCar = distance;
        neareastCar = otherCar;
      }
    }

    return neareastCar;
  }

  public int saf() {
    return nextBase;
  }

  public int tar() {
    return nextBase + 1;
  }

  public int sil() {
    return nextBase + 2;
  }

  public int numberOfVariables() {
    return nextBase + 3;
  }
}
