package two_lanes_merge.util;

import static two_lanes_merge.constants.Encodings.*;
import static two_lanes_merge.constants.PhysicsConstants.*;

import two_lanes_merge.car.Car;
import two_lanes_merge.car.CarPhysicalState;
import two_lanes_merge.scenario.ScenarioConfig;
import java.io.File;
import java.util.*;
import org.apache.commons.math3.random.RandomGenerator;
import stark.ds.DataState;

public final class MyUtil {

  private MyUtil() {
    // Utility class
  }

  public static void prepareOutputDirectories(ScenarioConfig scenarioConfig) {
    String base = "generated_files/" + scenarioConfig.getScenarioName();
    List<String> dirs =
        List.of(
            base + "/safety",
            base + "/sampled_trajectories",
            base + "/violating_trajectories",
            base + "/violating_trajectories/saf",
            base + "/violating_trajectories/tar",
            base + "/violating_trajectories/sil");

    for (String dir : dirs) {
      File folder = new File(dir);
      if (folder.exists()) {
        for (File file : Objects.requireNonNull(folder.listFiles())) {
          file.delete();
        }
      } else {
        folder.mkdirs();
      }
    }
  }

  public static double sampleAcceleration(RandomGenerator rng) {
    return MAX_ACCELERATION_SCALED - rng.nextDouble() * ACCELERATION_SPREAD_SCALED;
  }

  public static double sampleBrake(RandomGenerator rng) {
    return -Math.min(
        MAX_BRAKE_SCALED,
        Math.max(MIN_BRAKE_SCALED, MAX_BRAKE_SCALED - rng.nextDouble() * BRAKE_SPREAD_SCALED));
  }

  public static double sampleIdle(RandomGenerator rng) {
    return rng.nextDouble() * (2 * IDLE_SPREAD_SCALED) - IDLE_SPREAD_SCALED;
  }

  public static boolean isRssViolated(
      double distance, double safeDistance, boolean forMerge, double mergeRSSMultiplier) {
    double multiplier = forMerge == true ? mergeRSSMultiplier : 1.0;
    if (distance < safeDistance * multiplier) {
      return true;
    }

    return false;
  }

  public static double calculateRssViolationLength(double distance, double safeDistance) {
    return Math.abs(distance - safeDistance);
  }

  public static String writeViolatingTrajectoryTo(
      String fileName, String robPropName, ScenarioConfig scenarioConfig) {
    return "generated_files/"
        + scenarioConfig.getScenarioName()
        + "/violating_trajectories/"
        + "/"
        + robPropName
        + "/"
        + fileName;
  }

  public static String writeSampledTrajectoryTo(String fileName, ScenarioConfig scenarioConfig) {
    return "generated_files/"
        + scenarioConfig.getScenarioName()
        + "/sampled_trajectories/"
        + fileName;
  }

  public static String writeSaftyTo(String fileName, ScenarioConfig scenarioConfig) {
    return "generated_files/" + scenarioConfig.getScenarioName() + "/safety/" + fileName;
  }

  public static double calculateRSSSafetyDistance(double behindCarSpeed, double aheadCarSpeed) {
    double vR = behindCarSpeed / TICK_DURATION;
    double vF = aheadCarSpeed / TICK_DURATION;
    double aMax = MAX_ACCELERATION_SCALED / (TICK_DURATION * TICK_DURATION);
    double bMin = MIN_BRAKE_SCALED / (TICK_DURATION * TICK_DURATION);
    double bMax = MAX_BRAKE_SCALED / (TICK_DURATION * TICK_DURATION);

    double reactionTime = TICK_DURATION * ACTION_QUEUE_LENGTH;

    double d1 = reactionTime * vR;
    double d2 = 0.5 * aMax * reactionTime * reactionTime;
    double d3 = Math.pow(vR + reactionTime * aMax, 2) / (2 * bMin);
    double d4 = -(vF * vF) / (2 * bMax);
    double rss = Math.max(d1 + d2 + d3 + d4, 0);
    return rss;
  }

  public static double calculateRSSSafetyDistance(
      Car behindCar, Car aheadCar, DataState dataState) {
    return calculateRSSSafetyDistance(
        dataState.get(behindCar.speedIdx()), dataState.get(aheadCar.speedIdx()));
  }

  public static double calculateRSSSafetyDistance(
      CarPhysicalState behindCar, CarPhysicalState aheadCar) {
    return calculateRSSSafetyDistance(behindCar.speed(), aheadCar.speed());
  }

  public static double calculateDistanceBetweenCars(double selfX, double otherX) {
    return Math.max(0, Math.abs(selfX - otherX) - VEHICLE_LENGTH);
  }

  public static double calculateDistanceBetweenCars(CarPhysicalState self, CarPhysicalState other) {
    return calculateDistanceBetweenCars(self.x(), other.x());
  }

  public static double calculateDistanceBetweenCars(Car self, Car other, DataState dataState) {
    return calculateDistanceBetweenCars(dataState.get(self.xIdx()), dataState.get(other.xIdx()));
  }

  public static double getMyLane(double selfCenterY) {
    double selfLeftSideY = selfCenterY + VEHICLE_WIDTH / 2;
    double selfRightSideY = selfCenterY - VEHICLE_WIDTH / 2;

    if (selfRightSideY < 0) return RIGHT_FROM_RIGHT_LANE;
    if (selfLeftSideY <= LANE_WIDTH) return RIGHT_LANE;
    if (selfRightSideY >= LANE_WIDTH) return LEFT_LANE;
    if (selfLeftSideY > 2 * LANE_WIDTH) return LEFT_FROM_LEFT_LANE;
    return BETWEEN_LEFT_AND_RIGHT_LANE;
  }

  public static double getMyLane(CarPhysicalState self) {
    return getMyLane(self.y());
  }

  public static double getMyLane(Car self, DataState dataState) {
    double selfCenterY = dataState.get(self.yIdx());
    return getMyLane(selfCenterY);
  }

  public static boolean doesCrashHappen(CarPhysicalState self, CarPhysicalState other) {
    if (Math.abs(self.x() - other.x()) < VEHICLE_LENGTH
        && Math.abs(self.y() - other.y()) < VEHICLE_WIDTH) {
      return true;
    }

    return false;
  }
}
