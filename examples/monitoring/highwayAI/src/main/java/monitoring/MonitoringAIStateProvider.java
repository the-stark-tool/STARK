package monitoring;

import java.util.ArrayList;

public interface MonitoringAIStateProvider {

    ArrayList<MonitoringAiState> readInitialState();

    ArrayList<MonitoringAiState> getAIStates(int timestep);
}