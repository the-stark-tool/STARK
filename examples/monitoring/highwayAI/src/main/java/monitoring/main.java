package monitoring;

public class main {
    public static void main(String[] args) {

        MonitoringAIStateProvider shortReader = new JSONFileReader("examples/monitoring/highwayAI/src/main/resources/short.json");
        HighwayEnvScenario SHORT =  new HighwayEnvScenario(shortReader, 160, 100, "examples/monitoring/highwayAI/src/main/resources/experimentResults/short.tsv");
        SHORT.runCrashExperiment();
        SHORT.runSafetyGapExperiment();
        SHORT.runDesiredSpeedExperiment();
        SHORT.runDesiredSpeedExperiment();
        SHORT.runSpeedLimitUntilFreeLaneExperiment();

        MonitoringAIStateProvider longReader = new JSONFileReader("examples/monitoring/highwayAI/src/main/resources/long.json");
        HighwayEnvScenario LONG =  new HighwayEnvScenario(shortReader, 1500, 100, "examples/monitoring/highwayAI/src/main/resources/experimentResults/long.tsv");
        LONG.runCrashExperiment();
        LONG.runSafetyGapExperiment();
        LONG.runDesiredSpeedExperiment();
        LONG.runDesiredSpeedExperiment();
        LONG.runSpeedLimitUntilFreeLaneExperiment();

    }
}
