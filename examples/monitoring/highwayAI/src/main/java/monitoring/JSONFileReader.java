package monitoring;

import org.json.JSONObject;
import org.json.JSONArray;
import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.ArrayList;

public class JSONFileReader implements MonitoringAIStateProvider {
    private final ArrayList<ArrayList<MonitoringAiState>> aiStates;
    private final BufferedReader reader;

    public JSONFileReader(String filePath){
        aiStates = new ArrayList<>();
        try {
            reader = new BufferedReader(new FileReader(filePath));
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public ArrayList<MonitoringAiState> readInitialState() {
        aiStates.clear();
        return getNext();
    }

    @Override
    public ArrayList<MonitoringAiState> getAIStates(int timestep) {
        while (aiStates.size() <= timestep){
            getNext();
        }
        return aiStates.get(timestep);
    }



    private ArrayList<MonitoringAiState> getNext(){
        JSONObject jsonStates = null;
        try {
            String line = reader.readLine();
            if(line == null){
                ArrayList<MonitoringAiState> prev = aiStates.get(aiStates.size()-1);
                aiStates.add(prev);
                return prev;
            }
            jsonStates = new JSONObject(line);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        org.json.JSONArray array = jsonStates.getJSONArray("states");
        ArrayList<MonitoringAiState> states = new ArrayList<>();
        for (int i = 0; i < array.length(); i++) {
            states.add(new MonitoringAiState(array.getJSONObject(i), aiStates.size()));
        }
        aiStates.add(states);
        return states;
    }
}