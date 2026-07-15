package two_lanes_merge.scenario;

import static two_lanes_merge.constants.PhysicsConstants.*;
import static two_lanes_merge.state.InitialStateBuilder.*;

import two_lanes_merge.car.Car;
import two_lanes_merge.car.CarConfig;
import two_lanes_merge.car.CarRegistry;
import two_lanes_merge.perturbation.AbstractPerturbationFactory;
import two_lanes_merge.simulation.ControllerFactory;
import two_lanes_merge.simulation.Environment;
import two_lanes_merge.simulation.SimulationConfig;
import two_lanes_merge.util.MyUtil;
import two_lanes_merge.verification.PenaltyFunctions;
import java.io.IOException;
import java.util.*;
import org.apache.commons.math3.random.RandomGenerator;
import stark.*;
import stark.distance.AtomicDistanceExpression;
import stark.distance.DistanceExpression;
import stark.distance.MaxIntervalDistanceExpression;
import stark.ds.DataState;
import stark.ds.DataStateExpression;
import stark.ds.RelationOperator;
import stark.perturbation.Perturbation;
import stark.robtl.*;

public class ScenarioRunner {

  private static final RandomGenerator rng = new DefaultRandomGenerator();

  public ScenarioRunner() throws IOException {}

  public void run(ScenarioConfig scenarioConfig) throws IOException {
    SimulationConfig simulationConfig = scenarioConfig.getSimulationConfig();

    MyUtil.prepareOutputDirectories(scenarioConfig);

    AbstractPerturbationFactory perturbationFactory = scenarioConfig.getPerturbationFactory();

    CarRegistry carRegistry = new CarRegistry(ACTION_QUEUE_LENGTH);

    for (CarConfig carConfig : scenarioConfig.getCarConfigs()) {
      carRegistry.register(carConfig);
    }

    DataState dataState = buildInitialState(carRegistry);

    // INFO: Only one controlled car allowed
    ControlledSystem controlledSystem =
        new ControlledSystem(
            ControllerFactory.create(carRegistry.getCarByType(Car.CarType.CONTROLLED_CAR)),
            (rg, ds) ->
                ds.apply(Environment.getEnvironmentUpdates(rg, ds, carRegistry, simulationConfig)),
            dataState);

    EvolutionSequence evolutionSequence =
        new EvolutionSequence(
            rng, rg -> controlledSystem, simulationConfig.NUM_SAMPLES_EVOLUTION_SEQUENCE());

    PenaltyFunctions penaltyFunctions = new PenaltyFunctions(carRegistry);

    ArrayList<DataStateExpression> dataStateExpressions = buildExpressions(carRegistry);

    saveTrajectories(
        perturbationFactory,
        scenarioConfig,
        dataStateExpressions,
        controlledSystem,
        carRegistry,
        simulationConfig,
        dataState);

    runVerification(
        perturbationFactory,
        penaltyFunctions,
        evolutionSequence,
        simulationConfig,
        carRegistry,
        controlledSystem,
        scenarioConfig,
        dataState);
  }

  private ArrayList<DataStateExpression> buildExpressions(CarRegistry carRegistery) {
    ArrayList<DataStateExpression> expressions = new ArrayList<>();

    for (Car self : carRegistery.getCars()) {
      expressions.add(ds -> ds.get(self.xIdx()));
      expressions.add(ds -> ds.get(self.yIdx()));
      expressions.add(ds -> ds.get(self.speedIdx()));
      expressions.add(ds -> ds.get(self.accelerationIdx()));
      expressions.add(ds -> ds.get(self.steerCommandIdx()));
      expressions.add(ds -> ds.get(self.finishedIdx()));
    }
    expressions.add(ds -> ds.get(carRegistery.saf()));
    expressions.add(ds -> ds.get(carRegistery.tar()));
    expressions.add(ds -> ds.get(carRegistery.sil()));
    return expressions;
  }

  private void saveTrajectories(
      AbstractPerturbationFactory perturbationFactory,
      ScenarioConfig scenarioConfig,
      ArrayList<DataStateExpression> dataStateExpressions,
      ControlledSystem controlledSystem,
      CarRegistry carRegistry,
      SimulationConfig simulationConfig,
      DataState dataState)
      throws IOException {

    Map<String, double[][]> samples = new LinkedHashMap<>();

    Perturbation perturbation = perturbationFactory.buildPerturbation(carRegistry, dataState);

    samples.put(
        "_extra_trajectory_p.csv",
        SystemState.sample(
            rng, dataStateExpressions, perturbation, controlledSystem, TIME_HORIZON, 1));

    List<Car> cars = carRegistry.getCars();
    for (int i = 0; i < cars.size(); i++) {
      Car car = cars.get(i);
      int base = i * 6;
      for (Map.Entry<String, double[][]> entry : samples.entrySet()) {
        Util.writeToCSV(
            MyUtil.writeSampledTrajectoryTo(
                dataState.get(car.idIdx()) + entry.getKey(), scenarioConfig),
            extractData(
                entry.getValue(),
                base,
                base + 1,
                base + 2,
                base + 3,
                base + 4,
                base + 5,
                (int) dataState.get(car.idIdx())));
      }
    }
  }

  private double[][] extractData(
      double[][] data,
      int xCol,
      int yCol,
      int speedCol,
      int accelerationCol,
      int laneChangeCommandCol,
      int finishedCol,
      int carID) {
    double[][] trajectory = new double[data.length][7];

    for (int i = 0; i < data.length; i++) {
      trajectory[i][0] = carID;
      trajectory[i][1] = data[i][xCol];
      trajectory[i][2] = data[i][yCol];
      // INFO: Scale back from per tick to per second
      trajectory[i][3] = data[i][speedCol] / TICK_DURATION;
      trajectory[i][4] = data[i][accelerationCol] / (TICK_DURATION * TICK_DURATION);
      trajectory[i][5] = data[i][laneChangeCommandCol];
      trajectory[i][6] = data[i][finishedCol];
    }

    return trajectory;
  }

  private void runVerification(
      AbstractPerturbationFactory perturbationFactory,
      PenaltyFunctions penaltyFunctions,
      EvolutionSequence evolutionSequence,
      SimulationConfig simulationConfig,
      CarRegistry carRegistry,
      ControlledSystem controlledSystem,
      ScenarioConfig scenarioConfig,
      DataState dataState)
      throws IOException {
    Perturbation perturbation = perturbationFactory.buildPerturbation(carRegistry, dataState);

    record RobProp(String name, DataStateExpression penaltyFunction) {}
    List<RobProp> robProps =
        List.of(
            new RobProp("saf", penaltyFunctions::rhoCrash),
            new RobProp("tar", penaltyFunctions::rhoTargetLaneAchieved),
            new RobProp("sil", penaltyFunctions::rhoInLane));

    for (RobProp robProp : robProps) {
      DistanceExpression distExp =
          new AtomicDistanceExpression(robProp.penaltyFunction(), (v1, v2) -> Math.max(0, v2 - v1));

      // For boolean check: full horizon from step 0
      RobustnessFormula phiGlobal =
          new AtomicRobustnessFormula(
              perturbation,
              new MaxIntervalDistanceExpression(distExp, 0, TIME_HORIZON),
              RelationOperator.LESS_OR_EQUAL_THAN,
              simulationConfig.ETA());

      String check = evalBoolean(phiGlobal, evolutionSequence, simulationConfig);
      System.out.println(robProp.name + " : " + check);

      if (check.equals("False")) {
        saveAndFindViolatingCase(
            perturbationFactory,
            carRegistry,
            controlledSystem,
            simulationConfig,
            robProp.name,
            scenarioConfig,
            robProp.penaltyFunction(),
            evolutionSequence,
            dataState,
            carRegistry);
      }
    }
  }

  private void saveAndFindViolatingCase(
      AbstractPerturbationFactory perturbationFactory,
      CarRegistry carRegistery,
      ControlledSystem controlledSystem,
      SimulationConfig simulationConfig,
      String robustnessPropertyName,
      ScenarioConfig scenarioConfig,
      DataStateExpression penaltyFunction,
      EvolutionSequence evolutionSequence,
      DataState dataState,
      CarRegistry carRegistry)
      throws IOException {

    int numCars = carRegistery.getCars().size();
    int safCol = numCars * 6;
    int tarCol = numCars * 6 + 1;
    int silCol = numCars * 6 + 2;

    int violationCol =
        switch (robustnessPropertyName) {
          case "saf" -> safCol;
          case "tar" -> tarCol;
          case "sil" -> silCol;
          default ->
              throw new IllegalArgumentException("Unknown property: " + robustnessPropertyName);
        };

    System.out.println("Searching for violating trajectory for " + robustnessPropertyName + "...");
    int attempt = 0;
    while (true) {
      attempt++;
      Perturbation freshPerturbation =
          perturbationFactory.buildPerturbation(carRegistry, dataState);
      double[][] perturbedData =
          SystemState.sample(
              rng,
              buildExpressions(carRegistery),
              freshPerturbation,
              controlledSystem,
              TIME_HORIZON,
              1);

      boolean hasViolation = false;
      for (int t = 0; t < TIME_HORIZON; t++) {
        if (perturbedData[t][violationCol] > simulationConfig.ETA()) {
          hasViolation = true;
          break;
        }
      }

      if (hasViolation) {
        System.out.println("Found violating trajectory on attempt " + attempt);
        List<Car> cars = carRegistery.getCars();
        for (int c = 0; c < cars.size(); c++) {
          Car car = cars.get(c);
          int base = c * 6;
          Util.writeToCSV(
              MyUtil.writeViolatingTrajectoryTo(
                  dataState.get(car.idIdx()) + "_violation.csv",
                  robustnessPropertyName,
                  scenarioConfig),
              extractData(
                  perturbedData,
                  base,
                  base + 1,
                  base + 2,
                  base + 3,
                  base + 4,
                  base + 5,
                  (int) dataState.get(car.idIdx())));
        }
        break;
      }

      if (attempt > 20000) {
        System.out.println(
            "WARNING: Could not find violating trajectory after 20000 attempts for "
                + robustnessPropertyName);
        break;
      }
    }
  }

  private String evalBoolean(
      RobustnessFormula formula,
      EvolutionSequence evolutionSequence,
      SimulationConfig simulationConfig) {
    // Argument 0 means apply from the first step
    TruthValues v =
        new ThreeValuedSemanticsVisitor()
            .eval(formula)
            .eval(simulationConfig.NUM_SAMPLES_EVOLUTION_SEQUENCE(), 0, evolutionSequence);

    switch (v) {
      case TRUE:
        return "True";
      case FALSE:
        return "False";
      case UNKNOWN:
        return "Unknown";
    }
    return "Error";
  }
}
