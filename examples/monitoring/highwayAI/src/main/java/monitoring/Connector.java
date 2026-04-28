/*
 * JSpear: a SimPle Environment for statistical estimation of Adaptation and Reliability.
 *
 *              Copyright (C) 2020.
 *
 * See the NOTICE file distributed with this work for additional information
 * regarding copyright ownership.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *             http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express
 * or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package monitoring;

import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;

public class Connector {
    private final String baseUrl;
    private final ArrayList<ArrayList<AiState>> aiStates;

    public Connector(String baseUrl) {
        this.baseUrl = baseUrl;
        aiStates = new ArrayList<>();
    }

    public ArrayList<AiState> readInitialState(){
        aiStates.clear();
        return doNext(getInitConnection());
    }

    public synchronized ArrayList<AiState> getAIStates(int timestep){
        while (aiStates.size() <= timestep){
            doNext(getStepConnection());
        }
        return aiStates.get(timestep);
    }

    private HttpURLConnection getInitConnection() {
        HttpURLConnection initConnection;
        try {
            initConnection = (HttpURLConnection) new URL(baseUrl + "/reset").openConnection();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        return initConnection;
    }

    private ArrayList<AiState> doNext(HttpURLConnection connection){
        JSONObject jsonStates = doGET(connection);
        org.json.JSONArray array = jsonStates.getJSONArray("states");
        ArrayList<AiState> states = new ArrayList<>();
        for (int i = 0; i < array.length(); i++) {
            states.add(new AiState(array.getJSONObject(i), aiStates.size()));
        }
        aiStates.add(states);
        return states;
    }

    private HttpURLConnection getStepConnection() {
        HttpURLConnection stepConnection;
        try {
            stepConnection = (HttpURLConnection) new URL(baseUrl + "/step").openConnection();
            stepConnection.setDoOutput(true);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        return stepConnection;
    }

    private JSONObject doGET(HttpURLConnection connection) {
        try {
            connection.setRequestMethod("GET");
            connection.setRequestProperty("Content-Type", "application/json");

            int responseCode = connection.getResponseCode();
            StringBuilder response = getResponse(connection, responseCode);
            if (responseCode == HttpURLConnection.HTTP_OK) {
                return new JSONObject(response.toString());
            } else {
                throw new RuntimeException("Failed to get a successful response: " + responseCode + " " + response);
            }
        } catch (Exception e) {
            throw new RuntimeException(e.toString());
        }
    }

    private static StringBuilder getResponse(HttpURLConnection connection, int responseCode) throws IOException {
        StringBuilder response = new StringBuilder();
        try (BufferedReader reader = new BufferedReader(new InputStreamReader(
                responseCode == HttpURLConnection.HTTP_OK
                        ? connection.getInputStream()
                        : connection.getErrorStream(),
                StandardCharsets.UTF_8))) {
            String line;
            while ((line = reader.readLine()) != null) {
                response.append(line.trim());
            }
        }
        return response;
    }
}
