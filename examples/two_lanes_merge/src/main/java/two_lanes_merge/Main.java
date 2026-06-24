package two_lanes_merge;

import static two_lanes_merge.constants.PhysicsConstants.TIME_HORIZON;

import two_lanes_merge.scenario.*;
import two_lanes_merge.scenario.BrakeCheckerScenario;
import two_lanes_merge.scenario.NoMergeScenario;
import two_lanes_merge.scenario.RecklessMergerScenario;
import java.io.IOException;

public class Main {

  public static void main(String[] args) throws IOException {
    ScenarioRunner scenarioRunner = new ScenarioRunner();

    String scenario = args.length > 0 ? args[0] : "no-merger";

    switch (scenario) {
      case "brake-checker" -> runBrakeCheckerAnalysis(scenarioRunner);
      case "no-merger" -> runNoMergerAnalysis(scenarioRunner);
      case "reckless-merger" -> runRecklesMergerAnalysis(scenarioRunner);
      default -> {
        System.err.println("Unknown scenario: " + scenario);
        System.err.println("Valid options: brake-checker, no-merger, reckless-merger");
        System.exit(1);
      }
    }
  }

  public static void runRecklesMergerAnalysis(ScenarioRunner scenarioRunner) throws IOException {
    System.out.println("Running reckless merger analysis");
    try {
      scenarioRunner.run(new RecklessMergerScenario());
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  public static void runNoMergerAnalysis(ScenarioRunner scenarioRunner) throws IOException {
    System.out.println("Running no-merger analysis");
    try {
      for (int i = 0; i <= 10; i += 1) {
        double eta = ((double) i) / 10;
        System.out.println("(" + eta + ") - eta used");
        NoMergeScenario scenario = new NoMergeScenario(eta);
        scenarioRunner.run(scenario);
        System.out.println("");
        System.out.println("-----");
        System.out.println("");
      }
    } catch (RuntimeException e) {
      e.printStackTrace();
    }
  }

  public static void runBrakeCheckerAnalysis(ScenarioRunner scenarioRunner) throws IOException {
    System.out.println("Running brake checker analysis");
    try {
      for (int i = 0; i < TIME_HORIZON; i += 10) {
        System.out.println("(" + i + ") - perturbation applied at this time step");
        BrakeCheckerScenario scenario = new BrakeCheckerScenario(i);
        scenarioRunner.run(scenario);
        System.out.println("");
        System.out.println("-----");
        System.out.println("");
      }
    } catch (RuntimeException e) {
      e.printStackTrace();
    }
  }
}
