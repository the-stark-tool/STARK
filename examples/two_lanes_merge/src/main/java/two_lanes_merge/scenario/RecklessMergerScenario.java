package two_lanes_merge.scenario;

import static two_lanes_merge.constants.Encodings.*;
import static two_lanes_merge.constants.PhysicsConstants.*;

import two_lanes_merge.car.*;
import two_lanes_merge.car.CarConfig;
import two_lanes_merge.perturbation.AbstractPerturbationFactory;
import two_lanes_merge.perturbation.RecklessMergerFactory;
import two_lanes_merge.simulation.SimulationConfig;
import java.util.List;

public class RecklessMergerScenario implements ScenarioConfig {

  @Override
  public String getScenarioName() {
    return "reckless-merger";
  }

  @Override
  public List<CarConfig> getCarConfigs() {
    return List.of(
        new CarConfig(
            "controlled",
            1,
            Car.CarType.CONTROLLED_CAR,
            MAX_SPEED_SCALED,
            55,
            RIGHT_LANE_Y_CENTER,
            8 * TICK_DURATION,
            0,
            0,
            4,
            8 * TICK_DURATION,
            LEFT_LANE,
            0.22,
            3 * TICK_DURATION),
        new CarConfig(
            "otherRight1",
            2,
            Car.CarType.REGULAR_CAR,
            MAX_SPEED_SCALED,
            5,
            RIGHT_LANE_Y_CENTER,
            12 * TICK_DURATION,
            0,
            0,
            3,
            11 * TICK_DURATION,
            LEFT_LANE,
            0.24,
            4 * TICK_DURATION),
        new CarConfig(
            "otherLeft1",
            3,
            Car.CarType.REGULAR_CAR,
            MAX_SPEED_SCALED,
            30,
            LEFT_LANE_Y_CENTER,
            11 * TICK_DURATION,
            0,
            0,
            2,
            10 * TICK_DURATION,
            RIGHT_LANE,
            0.28,
            3 * TICK_DURATION),
        new CarConfig(
            "otherLeft2",
            4,
            Car.CarType.REGULAR_CAR,
            MAX_SPEED_SCALED,
            80,
            LEFT_LANE_Y_CENTER,
            10 * TICK_DURATION,
            0,
            0,
            1,
            10 * TICK_DURATION,
            RIGHT_LANE,
            0.25,
            5 * TICK_DURATION));
  }

  @Override
  public AbstractPerturbationFactory getPerturbationFactory() {
    return new RecklessMergerFactory(2, 0, TIME_HORIZON);
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
