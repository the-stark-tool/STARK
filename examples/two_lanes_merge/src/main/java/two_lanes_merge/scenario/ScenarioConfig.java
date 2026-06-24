package two_lanes_merge.scenario;

import two_lanes_merge.car.CarConfig;
import two_lanes_merge.perturbation.AbstractPerturbationFactory;
import two_lanes_merge.simulation.SimulationConfig;
import java.util.*;

public interface ScenarioConfig {

  String getScenarioName();

  List<CarConfig> getCarConfigs();

  AbstractPerturbationFactory getPerturbationFactory();

  SimulationConfig getSimulationConfig();
}
