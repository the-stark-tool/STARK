package two_lanes_merge.scenario;

import static two_lanes_merge.constants.Encodings.*;
import static two_lanes_merge.constants.PhysicsConstants.*;

import two_lanes_merge.car.Car;
import two_lanes_merge.car.Car.*;
import two_lanes_merge.car.CarConfig;
import two_lanes_merge.perturbation.AbstractPerturbationFactory;
import two_lanes_merge.perturbation.BrakeCheckerFactory;
import two_lanes_merge.simulation.SimulationConfig;
import two_lanes_merge.util.MyUtil;
import java.util.*;
import org.apache.commons.math3.random.RandomGenerator;
import stark.DefaultRandomGenerator;

record Interval(double min, double max) {
  public Interval {
    if (min > max) throw new IllegalArgumentException("min must be <= max");
  }

  public double span() {
    return max - min;
  }

  public double sample(RandomGenerator rng) {
    return min + rng.nextDouble() * span();
  }

  public int sampleInt(RandomGenerator rng) {
    return (int) Math.round(min + rng.nextDouble() * span());
  }
}

public class RandomBrakeCheckerScenario implements ScenarioConfig {

  private static final RandomGenerator rng = new DefaultRandomGenerator();

  private int IcSimId;

  private Interval xInterval;
  private Interval yInterval;
  private Interval speedInterval;
  private Interval targetSpeedInterval;
  private Interval actionQueueOffsetInterval;

  private Interval pertrurbationDurationInterval;
  private Interval pertrubationStartTimeInterval;
  private Interval perturbedCarIdInterval;

  public RandomBrakeCheckerScenario(int IcSimId) {
    this.IcSimId = IcSimId;

    this.xInterval = new Interval(0, ROAD_LENGHT);
    this.yInterval = new Interval(RIGHT_LANE_Y_CENTER, LEFT_LANE_Y_CENTER);
    this.speedInterval = new Interval(8, 12);
    this.targetSpeedInterval = new Interval(8, 12);

    this.perturbedCarIdInterval = new Interval(1, 3);
    this.pertrurbationDurationInterval = new Interval(1, 15);
    this.pertrubationStartTimeInterval = new Interval(0, TIME_HORIZON);
  }

  @Override
  public String getScenarioName() {
    return String.valueOf(this.IcSimId);
  }

  private Optional<List<CarConfig>> generateCarConfigs() {
    List<CarConfig> carConfigs = new ArrayList<>();

    for (int i = 0; i < 4; i++) {
      double initX = xInterval.sample(rng);
      double initY = yInterval.sample(rng);
      double initSpeed = speedInterval.sample(rng);
      double initTargetSpeed = targetSpeedInterval.sample(rng);
      double initActionQueueOffset = actionQueueOffsetInterval.sampleInt(rng);

      double initLane = MyUtil.getMyLane(initY);
      double initTargetLane = -1;

      if (initLane == RIGHT_LANE || initLane == LEFT_LANE) {
        initTargetLane = initLane == RIGHT_LANE ? LEFT_LANE : RIGHT_LANE;
      }

      if (initLane == BETWEEN_LEFT_AND_RIGHT_LANE) {
        initTargetLane = rng.nextDouble() > 0.5 ? RIGHT_LANE : LEFT_LANE;
      }

      CarType type = Car.CarType.REGULAR_CAR;

      // First index always controlled car
      if (i == 0) {
        initX = 0;
        initY = RIGHT_LANE_Y_CENTER;
        initTargetLane = LEFT_LANE;

        type = Car.CarType.CONTROLLED_CAR;
      }

      for (CarConfig carConfig : carConfigs) {
        double distanceX = Math.abs(carConfig.initX() - initX);
        double distanceY = Math.abs(carConfig.initY() - initY);

        // Physical overlap — always reject
        if (distanceX < VEHICLE_LENGTH && distanceY < VEHICLE_WIDTH) {
          return Optional.empty();
        }

        // Same lane — check RSS on x axis
        if (distanceY < VEHICLE_WIDTH) {
          boolean newCarIsBehind = initX < carConfig.initX();
          double rearSpeed = newCarIsBehind ? initSpeed * TICK_DURATION : carConfig.initSpeed();
          double frontSpeed = newCarIsBehind ? carConfig.initSpeed() : initSpeed * TICK_DURATION;
          double rssGap = MyUtil.calculateRSSSafetyDistance(rearSpeed, frontSpeed);
          if (distanceX < rssGap) {
            return Optional.empty();
          }
        }
      }

      // TODO need to sample
      double minComfortSpeed = 4 * TICK_DURATION;

      carConfigs.add(
          new CarConfig(
              String.valueOf(i),
              i,
              type,
              MAX_SPEED_SCALED,
              initX,
              initY,
              initSpeed * TICK_DURATION,
              0,
              0,
              initActionQueueOffset,
              initTargetSpeed * TICK_DURATION,
              initTargetLane,
              0.2,
              minComfortSpeed));
    }

    return Optional.of(carConfigs);
  }

  @Override
  public List<CarConfig> getCarConfigs() {

    List<CarConfig> carConfigs = new ArrayList<>();

    while (carConfigs.size() == 0) {

      Optional<List<CarConfig>> optionalCarConfigs = generateCarConfigs();
      if (optionalCarConfigs.isEmpty()) {
        continue;
      }

      carConfigs = optionalCarConfigs.get();
    }

    return carConfigs;
  }

  @Override
  public AbstractPerturbationFactory getPerturbationFactory() {
    return new BrakeCheckerFactory(
        perturbedCarIdInterval.sampleInt(rng),
        pertrubationStartTimeInterval.sampleInt(rng),
        pertrurbationDurationInterval.sampleInt(rng));
  }

  @Override
  public SimulationConfig getSimulationConfig() {
    int NUM_SAMPLES_EVOLUTION_SEQUENCE = 100;
    double ETA = 0.00;
    int THREE_VAL_SAMPLE_COUNT = 50;
    double THREE_VAL_CONFIDENCE = 1.96;

    return new SimulationConfig(
        NUM_SAMPLES_EVOLUTION_SEQUENCE, ETA, THREE_VAL_SAMPLE_COUNT, THREE_VAL_CONFIDENCE);
  }
}
