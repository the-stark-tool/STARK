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

import org.json.JSONArray;
import org.json.JSONObject;
import stark.PerceivedSystemState;
import stark.ds.DataState;

import java.util.HashMap;
import java.util.Map;


public class MonitoringAiState {
    // expected features: ['presence', 'x', 'y', 'vx', 'vy']
    // sample observation: Relative to ego car, unnormalized
//    [ [  1.        180.58331     8.         29.14556     0.       ]
//      [  1.         19.14256    -4.         -6.714924    0.       ]
//      [  1.         40.12648    -8.        -12.395373    0.       ]
//      [  1.         63.717155   -8.        -11.642069    0.       ]
//      [  1.         87.08581    -8.        -13.0664015   0.       ]]

    private static final int PRESENCE_COLUMN = 0;
    private static final int X_POSITION_COLUMN = 1;
    private static final int Y_POSITION_COLUMN =  2;
    private static final int X_SPEED_COLUMN = 3;
    private static final int Y_SPEED_COLUMN = 4;



    private int originalValuesCount = -1; // includes non car values

    // Index of the datastate element that stores the simulation step that the datastate belongs to
    public static final int DATASTATE_INDEX_FOR_TIMESTEP = 0;
    protected static final int DATASTATE_INDEX_FOR_CRASHES = 1;

    private static final int NUMBER_OF_NON_CAR_VALUES = 2;

    private final int timestep;

    protected final JSONObject state;

    public MonitoringAiState(JSONObject state, int timestep) {
        this.state = state;
        this.timestep = timestep;
    }

   protected JSONObject toJson(){
        return state;
    }

    private JSONArray getCarState(){
        return state.getJSONArray("state");
    }

    public int getCarCount(){
        return this.getCarState().length();
    }

    public int getCrashes() {
        return state.getInt("crashes");
    }

    public JSONArray getFeatures(){
        return state.getJSONArray("features");
    }

    public int getCrashesIndex() {
        return DATASTATE_INDEX_FOR_CRASHES;
    }

    public int[] getRealDataStatePresenceIndexes(){
        return getRealDataStateIndexes(PRESENCE_COLUMN);
    }

    public int[] getPerturbedDataStatePresenceIndexes(){
        return getPerturbedDataStateIndexes(PRESENCE_COLUMN);
    }

    public int[] getRealDataStateXSpeedIndexes(){
        return getRealDataStateIndexes(X_SPEED_COLUMN);
    }

    public int[] getRealDataStateXPositionIndexes(){
        return getRealDataStateIndexes(X_POSITION_COLUMN);
    }

    public int[] getPerturbedDataStateXSpeedIndexes(){
        return getPerturbedDataStateIndexes(X_SPEED_COLUMN);
    }

    public int[] getPerturbedDataStateXPositionIndexes(){
        return getPerturbedDataStateIndexes(X_POSITION_COLUMN);
    }

    public int[] getRealDataStateYSpeedIndexes(){
        return getRealDataStateIndexes(Y_SPEED_COLUMN);
    }

    public int[] getRealDataStateYPositionIndexes(){
        return getRealDataStateIndexes(Y_POSITION_COLUMN);
    }

    public int[] getPerturbedDataStateYSpeedIndexes(){
        return getPerturbedDataStateIndexes(Y_SPEED_COLUMN);
    }

    public int[] getPerturbedDataStateYPositionIndexes(){
        return getPerturbedDataStateIndexes(Y_POSITION_COLUMN);
    }

    public int getControlledVehicleIndex(){
        return 0;
    }

    // The AI reports the simulation is truncated if the established duration is reached
    public boolean isTruncated(){
        return state.getBoolean("truncated");
    }

    // The AI reports the simulation is done if the controlled vehicle crashed
    public boolean isDone(){
        return state.getBoolean("done");
    }

    private int[] getRealDataStateIndexes(int column){
        return getDataStateIndexes(column, NUMBER_OF_NON_CAR_VALUES);
    }

    private int[] getPerturbedDataStateIndexes(int column){
        return getDataStateIndexes(column, originalValuesCount);
    }

    private int[] getDataStateIndexes(int column, int offset) {
        if (offset == originalValuesCount && originalValuesCount==-1) {
            throw new RuntimeException("Calling for perturbed datastate indexes before creating a datastate");
        }
        int[] m = new int[this.getCarCount()];
        int colCount = this.getFeatures().length();
        for (int i = 0; i < colCount; i++) {
            m[i] = offset + (i*colCount) + column;
        }
        return m;
    }

    public void setPerturbedDataState(DataState pds) {
        state.put("state", perturbedDataStateToJson(pds));
    }

    private JSONArray perturbedDataStateToJson(DataState pds){
        int carCount = this.getCarCount();
        int colCount = this.getFeatures().length();
        JSONArray perturbedJson = new JSONArray(carCount);

        for (int i = 0; i < carCount; i++) {
            JSONArray row = new JSONArray(colCount);
            for (int j = 0; j < colCount; j++) {
                double value = pds.get(originalValuesCount+i*carCount+j);
                row.put(j, value);
            }
            perturbedJson.put(i, row);
        }
        return perturbedJson;
    }

    /**
     * @return the observable state
     */
    public DataState getDataState(){
        JSONArray carState = this.getCarState();
        int carCount = carState.length();
        int featureCount = this.getFeatures().length();

        Map<Integer, Double> values = new HashMap<>();

        values.put(DATASTATE_INDEX_FOR_TIMESTEP, (double) this.timestep);
        values.put(DATASTATE_INDEX_FOR_CRASHES, (double) getCrashes());

        for (int i = 0; i < carCount; i++) {
            JSONArray row = carState.getJSONArray(i);
            if (row.length() != featureCount) {
                System.out.println("AIServer: Rows of different sizes in the received observation");
            }
            for (int j = 0; j < featureCount; j++) {
                double value;
                if (j < row.length()){
                    value = row.getDouble(j);
                } else {
                    value = Double.NaN;
                }
                values.put(NUMBER_OF_NON_CAR_VALUES + (i * featureCount) + j, value);
            }
        }
        // create a duplicate of all car values to be perturbed
        originalValuesCount = values.size();
        for (int i = 0; i < originalValuesCount-NUMBER_OF_NON_CAR_VALUES; i++){
            values.put(originalValuesCount + i, values.get(NUMBER_OF_NON_CAR_VALUES+i));
        }
        return new DataState(values.size(), i -> values.getOrDefault(i, Double.NaN));
    }

    public PerceivedSystemState getAsPerceivedSystemState(){
        return new PerceivedSystemState(this.getDataState());
    }

}
