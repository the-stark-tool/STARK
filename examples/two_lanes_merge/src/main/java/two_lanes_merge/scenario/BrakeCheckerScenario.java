package two_lanes_merge.scenario;

import static two_lanes_merge.constants.Encodings.*;
import static two_lanes_merge.constants.PhysicsConstants.*;

import two_lanes_merge.car.*;
import two_lanes_merge.car.CarConfig;
import two_lanes_merge.perturbation.AbstractPerturbationFactory;
import two_lanes_merge.perturbation.BrakeCheckerFactory;
import two_lanes_merge.simulation.SimulationConfig;
import java.util.List;

public class BrakeCheckerScenario implements ScenarioConfig {

  public int perturbationStartTimeStep;

  public BrakeCheckerScenario(int perturbationStartTimeStep) {
    this.perturbationStartTimeStep = perturbationStartTimeStep;
  }

  @Override
  public String getScenarioName() {
    return "brake-checker-" + perturbationStartTimeStep;
  }

  @Override
  public List<CarConfig> getCarConfigs() {
    return List.of(
        new CarConfig(
            "controlled",
            1,
            Car.CarType.CONTROLLED_CAR,
            MAX_SPEED_SCALED,
            20,
            RIGHT_LANE_Y_CENTER,
            10 * TICK_DURATION,
            0,
            0,
            4,
            11 * TICK_DURATION,
            LEFT_LANE,
            0.26,
            3 * TICK_DURATION),
        new CarConfig(
            "otherRight1",
            2,
            Car.CarType.REGULAR_CAR,
            MAX_SPEED_SCALED,
            40,
            RIGHT_LANE_Y_CENTER,
            10 * TICK_DURATION,
            0,
            0,
            3,
            11 * TICK_DURATION,
            LEFT_LANE,
            0.22,
            4 * TICK_DURATION),
        new CarConfig(
            "otherLeft1",
            3,
            Car.CarType.REGULAR_CAR,
            MAX_SPEED_SCALED,
            10,
            LEFT_LANE_Y_CENTER,
            12 * TICK_DURATION,
            0,
            0,
            2,
            13 * TICK_DURATION,
            RIGHT_LANE,
            0.3,
            3 * TICK_DURATION),
        new CarConfig(
            "otherLeft2",
            4,
            Car.CarType.REGULAR_CAR,
            MAX_SPEED_SCALED,
            55,
            LEFT_LANE_Y_CENTER,
            8 * TICK_DURATION,
            0,
            0,
            1,
            9 * TICK_DURATION,
            RIGHT_LANE,
            0.29,
            5 * TICK_DURATION));
  }

  @Override
  public AbstractPerturbationFactory getPerturbationFactory() {
    return new BrakeCheckerFactory(2, perturbationStartTimeStep, 15);
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
