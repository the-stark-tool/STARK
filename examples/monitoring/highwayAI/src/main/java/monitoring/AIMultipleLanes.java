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

/*
THIS CASE STUDY REQUIRES AN AI SERVER
Set up and run the following project: https://github.com/the-stark-tool/highway-env-ai-server
 */
package monitoring;

import org.apache.commons.math3.random.RandomGenerator;
import stark.*;
import stark.controller.Controller;
import stark.controller.ControllerRegistry;
import stark.controller.ExecController;
import stark.distl.*;
import stark.ds.*;
import stark.monitors.DefaultMonitorBuilder;
import stark.monitors.DefaultUDisTLMonitor;
import stark.udistl.UDisTLFormula;
import stark.udistl.UnboundedUntiluDisTLFormula;

import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintWriter;
import java.util.*;
import java.util.function.Function;

public class AIMultipleLanes {
    private static final double RESPONSE_TIME = 1;

    // VEHICLE DIMENSIONS
    private static final double VEHICLE_LENGTH = 5;
    private static final double VEHICLE_WIDTH = 2;

    // VARIABLE BOUNDS
    private static final double MAX_SPEED = 40;
    private static final double MAX_ACCELERATION = 5;
    private static final double MAX_BRAKE = 5;
    private static final double MIN_BRAKE = 3;

    // PERTURBATION PARAMETERS
    private static final int STARTING_STEP = 1;
    private static final int FREQUENCY = 2;
    private static final int TIMES_TO_APPLY = 14;
    private static final int STEPS_TO_SAMPLE = 30;
    private static final int EVOLUTION_SEQUENCE_SIZE = 10;

    private static final Connector AI = new Connector("http://127.0.0.1:6000");
    private String experimentName;
    private static final String RESULTS_FOLDER = "./monitoring";;

    // DATASTATE INDEXES
    private int[] presence;
    private int[] xPosition;
    private int[] xSpeed;
    private int[] yPosition;
    private int[] ySpeed;
    private int crashes;

    private int observedCarCount;


    public AIMultipleLanes() {
        run();
    }

    private void run() {
        EvolutionSequence sequence = new EvolutionSequence(new SilentMonitor("AIMultipleLanes"), new DefaultRandomGenerator(), readInitialState(), EVOLUTION_SEQUENCE_SIZE);

        experimentName = "test1";
        runDisTLExperiments(sequence, 0.0);
//        printSummary(sequence, STEPS_TO_SAMPLE, "UNPERTURBED", System.out);
    }

    private void runDisTLExperiments(EvolutionSequence sequence, double threshold) {
        DefaultMonitorBuilder defaultMonitorBuilder = new DefaultMonitorBuilder(EVOLUTION_SEQUENCE_SIZE, false);

//         crash
        DataStateFunction mu = (rg, ds) -> {
            List<DataStateUpdate> updates = new LinkedList<>();
            updates.add(new DataStateUpdate(crashes, 0.0));
            return ds.apply(updates);
        };
        TargetDisTLFormula atomicCrash = new TargetDisTLFormula(mu, getCrashPenalty(), threshold);
        UDisTLFormula alwaysCrash = new NegationDisTLFormula(new UnboundedUntiluDisTLFormula(
                new FalseDisTLFormula(),
                new NegationDisTLFormula(
                    atomicCrash
        )));
        UDisTLFormula bAlwaysCrash = new AlwaysDisTLFormula(atomicCrash,0,50);
        DefaultUDisTLMonitor mCrash = defaultMonitorBuilder.build(alwaysCrash);
        DefaultUDisTLMonitor mbCrash = defaultMonitorBuilder.build(bAlwaysCrash);

        printMonitoringSummary(sequence, mCrash, STEPS_TO_SAMPLE, "mCrash", System.out);
        printMonitoringSummary(sequence, mbCrash, STEPS_TO_SAMPLE, "mbCrash", System.out);

//        // safety gap
//        TargetDisTLFormula atomicSG = new TargetDisTLFormula(getSafetyGapMu(), getClosestVehiclePenalty(), threshold);
//        UDisTLFormula alwaysSG = new AlwaysDisTLFormula(atomicSG,0,50);
//        DefaultUDisTLMonitor mSG = defaultMonitorBuilder.build(alwaysSG);
//
//        printMonitoringSummary(sequence, mSG, STEPS_TO_SAMPLE, "mSG", System.out);
//
//        // max speed
//        double DESIRED_SPEED = 40.0;
//        DataStateFunction muDesiredSpeed = (rg, ds) -> {
//            int controlledVehicle = AI.getAIStates((int) ds.get(AiState.DATASTATE_INDEX_FOR_HISTORY_INDEX)).get(0).getControlledVehicleIndex();
//            List<DataStateUpdate> updates = new LinkedList<>();
//            updates.add(new DataStateUpdate(xSpeed[controlledVehicle], 0.3*rg.nextGaussian()+DESIRED_SPEED));
//            return ds.apply(updates);
//        };
//        DataStateExpression penaltyDesiredSpeed = (ds) -> {
//            int controlledVehicle = AI.getAIStates((int) ds.get(AiState.DATASTATE_INDEX_FOR_HISTORY_INDEX)).get(0).getControlledVehicleIndex();
//            double egoSpeed = ds.get(xSpeed[controlledVehicle]);
//            if (egoSpeed >= DESIRED_SPEED) {
//                return 0.0;
//            } else {
//                return (DESIRED_SPEED - egoSpeed) / DESIRED_SPEED;
//            }
//        };
//        TargetDisTLFormula atomicDesiredSpeed = new TargetDisTLFormula(muDesiredSpeed, penaltyDesiredSpeed, threshold);
//        UDisTLFormula alwaysDesiredSpeed = new AlwaysDisTLFormula(atomicDesiredSpeed,0,50);
//        DefaultUDisTLMonitor mDesiredSpeed = defaultMonitorBuilder.build(alwaysDesiredSpeed);
//
//        printMonitoringSummary(sequence, mDesiredSpeed, STEPS_TO_SAMPLE, "mDesiredSpeed", System.out);
//
//        // speed limit
//        double SPEED_LIMIT = 30.0;
//        DataStateFunction muSpeedLimit = (rg, ds) -> {
//            int controlledVehicle = AI.getAIStates((int) ds.get(AiState.DATASTATE_INDEX_FOR_HISTORY_INDEX)).get(0).getControlledVehicleIndex();
//            List<DataStateUpdate> updates = new LinkedList<>();
//            updates.add(new DataStateUpdate(xSpeed[controlledVehicle], rg.nextDouble()*SPEED_LIMIT));
//            return ds.apply(updates);
//        };
//        DataStateExpression penaltySpeedLimit = (ds) -> {
//            int controlledVehicle = AI.getAIStates((int) ds.get(AiState.DATASTATE_INDEX_FOR_HISTORY_INDEX)).get(0).getControlledVehicleIndex();
//            double egoSpeed = ds.get(xSpeed[controlledVehicle]);
//            if (egoSpeed >= SPEED_LIMIT) {
//                return 0.0;
//            } else {
//                return (SPEED_LIMIT - egoSpeed) / SPEED_LIMIT;
//            }
//
//        };
//        TargetDisTLFormula atomicSpeedLimit = new TargetDisTLFormula(muSpeedLimit, penaltySpeedLimit, threshold);
//        UDisTLFormula alwaysSpeedLimit = new AlwaysDisTLFormula(atomicSpeedLimit,0,50);
//        DefaultUDisTLMonitor mSpeedLimit = defaultMonitorBuilder.build(alwaysSpeedLimit);
//
//        printMonitoringSummary(sequence, mSpeedLimit, STEPS_TO_SAMPLE, "mSpeedLimit", System.out);
//
//         // free way
//        double Y_DISTANCE = 4;
//        DataStateFunction muFW = (rg, ds) -> {
//            int controlledVehicle = AI.getAIStates((int) ds.get(AiState.DATASTATE_INDEX_FOR_HISTORY_INDEX)).get(0).getControlledVehicleIndex();
//            List<DataStateUpdate> updates = new LinkedList<>();
//            double yPos = ds.get(yPosition[controlledVehicle]);
//            for (int vehicle = 0; vehicle < observedCarCount; vehicle++) {
//                if (vehicle != controlledVehicle) {
//                    if(yPos < ds.get(yPosition[vehicle])){
//                        updates.add(new DataStateUpdate(yPosition[vehicle], yPosition[vehicle] + Y_DISTANCE));
//                    } else{
//                        updates.add(new DataStateUpdate(yPosition[vehicle], yPosition[vehicle] - Y_DISTANCE));
//                    }
//                }
//            }
//            return ds.apply(updates);
//        };
//
//        DataStateExpression penaltyFW = (DataState ds) -> {
//            int controlledVehicle = AI.getAIStates((int) ds.get(AiState.DATASTATE_INDEX_FOR_HISTORY_INDEX)).get(0).getControlledVehicleIndex();
//            double yPos = ds.get(yPosition[controlledVehicle]);
//            double closestVehicleDist = 0;
//            for (int vehicle = 0; vehicle < observedCarCount; vehicle++) {
//                if (vehicle != controlledVehicle) {
//                    double vehicleDist = Math.abs(yPos - ds.get(yPosition[vehicle]));
//                    if (closestVehicleDist < vehicleDist) {
//                        closestVehicleDist = vehicleDist;
//                    }
//                }
//            }
//            if (closestVehicleDist >= Y_DISTANCE) {
//                return 0.0;
//            } else {
//                return (Y_DISTANCE - closestVehicleDist) / Y_DISTANCE;
//            }
//        };
//        TargetDisTLFormula atomicFW = new TargetDisTLFormula(muFW, penaltyFW, threshold);
//        UDisTLFormula FWreleasesSpeed = new NegationDisTLFormula(
//                new UnboundedUntiluDisTLFormula(
//                        new NegationDisTLFormula(atomicFW),
//                        new NegationDisTLFormula(atomicSpeedLimit)
//                ));
//        DefaultUDisTLMonitor mFW = defaultMonitorBuilder.build(FWreleasesSpeed);
//        printMonitoringSummary(sequence, mFW, STEPS_TO_SAMPLE, "mFWrS", System.out);
    }

    private DataStateExpression getClosestVehiclePenalty() {
        return (ds) -> {
            int controlledVehicle = AI.getAIStates((int) ds.get(AiState.DATASTATE_INDEX_FOR_HISTORY_INDEX)).get(0).getControlledVehicleIndex();
            int closestVehicle = 0;
            for (int vehicle = 0; vehicle < observedCarCount; vehicle++) {
                if (vehicle != controlledVehicle) {
                    double vehiclePos = ds.get(xPosition[vehicle]);
                    if (vehiclePos < ds.get(xPosition[closestVehicle])) {
                        closestVehicle = vehicle;
                    }
                }
            }
            return ds.get(xPosition[closestVehicle]);
        };
    }


    private Function<RandomGenerator, SystemState> readInitialState() {
        ArrayList<AiState> initialAiState = AI.readInitialState();
        updateDataStateIndexes(initialAiState.get(0));
        return (RandomGenerator rg) -> {
            DataState ds = initialAiState.get(rg.nextInt(initialAiState.size())).getDataState();
            return new ControlledSystem(getController(), this::getEnvironmentUpdates, ds);
        };
    }

    private Controller getController(){
        ControllerRegistry registry = new ControllerRegistry();
        registry.set("Control", Controller.doTick(registry.reference("Control")));
        return new ExecController(registry.reference("Control"));
    }

    public DataState getEnvironmentUpdates(RandomGenerator rg, DataState ds) {
        int currentTimestep = (int) ds.get(AiState.DATASTATE_INDEX_FOR_HISTORY_INDEX);
        ArrayList<AiState> nextTimestepStates = AI.getAIStates(currentTimestep + 1);
        AiState newAiState = nextTimestepStates.get(rg.nextInt(nextTimestepStates.size()));
        DataState newState = newAiState.getDataState();
        updateDataStateIndexes(newAiState);
        return newState;
    }

    private void updateDataStateIndexes(AiState aiState) {
        presence = aiState.getRealDataStatePresenceIndexes();
        xSpeed = aiState.getRealDataStateXSpeedIndexes();
        xPosition = aiState.getRealDataStateXPositionIndexes();

        ySpeed = aiState.getRealDataStateYSpeedIndexes();
        yPosition = aiState.getRealDataStateYPositionIndexes();

        crashes = aiState.getCrashesIndex();
        observedCarCount = aiState.getCarCount();
    }

    private void printMonitoringSummary(EvolutionSequence sequence, DefaultUDisTLMonitor m, int stepsToPrint, String title, OutputStream outputStream){
        PrintWriter writer = new PrintWriter(outputStream);
        writer.printf("%s%n" + ("%" + 6 + "s, ").repeat(18) + "%n", title,
                "i", "mon", "samples", "crash", "p0", "x0", "v0", "p1", "x1", "v1", "p2", "x2", "v2", "p3", "x3", "v3", "p4", "x4", "v4");
        int i = 0;
        while(i < stepsToPrint) {
            SampleSet<PerceivedSystemState> distribution = sequence.getAsPerceivedSystemStates(i);
            OptionalDouble monitorEval = m.evalNext(distribution);
            ArrayList<String> s = new ArrayList<>();

            s.add(String.format("%" + 6 + "d,", i));
            s.add(String.format("%" + 6 + ".2f,", monitorEval.orElseThrow()));
            s.add(String.format("%" + 6 + "d,", distribution.size()));
            OptionalDouble c = Arrays.stream(distribution.evalPenaltyFunction(ds -> ds.get(crashes))).average();
            s.add(String.format("%" + 6 + ".2f,", c.orElse(Double.NaN)));
            for (int j = 0; j < observedCarCount; j++) {
                int finalJ = j;
                OptionalDouble p = Arrays.stream(distribution.evalPenaltyFunction(ds -> ds.get(presence[finalJ]))).average();
                s.add(String.format("%" + 6 + ".2f,", p.orElse(Double.NaN)));
                OptionalDouble x = Arrays.stream(distribution.evalPenaltyFunction(ds -> ds.get(xPosition[finalJ]))).average();
                s.add(String.format("%" + 6 + ".2f,", x.orElse(Double.NaN)));
                OptionalDouble v = Arrays.stream(distribution.evalPenaltyFunction(ds -> ds.get(xSpeed[finalJ]))).average();
                s.add(String.format("%" + 6 + ".2f,", v.orElse(Double.NaN)));
            }

            writer.println(String.join(" ", s));
            writer.flush();

            i++;
        }
    }

    private void printSummary(EvolutionSequence sequence, int stepsToPrint, String title, OutputStream outputStream) {
        int colw = 6;
        PrintWriter writer = new PrintWriter(outputStream);
        try {
            writer.printf("%s%n" + ("%" + colw + "s, ").repeat(17) + "%n", title,
                    "i", "samples", "crash", "p0", "x0", "v0", "p1", "x1", "v1", "p2", "x2", "v2", "p3", "x3", "v3", "p4", "x4", "v4");

            for (int i = 0; i < stepsToPrint; i++) {
                SampleSet<SystemState> dss = sequence.get(i);
                ArrayList<String> s = new ArrayList<>();
                s.add(String.format("%" + colw + "d,", i));
                s.add(String.format("%" + colw + "d,", dss.size()));
                OptionalDouble c = Arrays.stream(dss.evalPenaltyFunction(ds -> ds.get(crashes))).average();
                s.add(String.format("%" + colw + ".2f,", c.orElse(Double.NaN)));
                for (int j = 0; j < observedCarCount; j++) {
                    int finalJ = j;
                    OptionalDouble p = Arrays.stream(dss.evalPenaltyFunction(ds -> ds.get(presence[finalJ]))).average();
                    s.add(String.format("%" + colw + ".2f,", p.orElse(Double.NaN)));
                    OptionalDouble x = Arrays.stream(dss.evalPenaltyFunction(ds -> ds.get(xPosition[finalJ]))).average();
                    s.add(String.format("%" + colw + ".2f,", x.orElse(Double.NaN)));
                    OptionalDouble v = Arrays.stream(dss.evalPenaltyFunction(ds -> ds.get(xSpeed[finalJ]))).average();
                    s.add(String.format("%" + colw + ".2f,", v.orElse(Double.NaN)));
                }

                writer.println(String.join(" ", s));
                writer.flush(); // Ensure data is written after every line
            }
        } finally {
            writer.flush(); // Extra flush in case of an out-of-memory situation
        }
    }

    public void printDistanceArray(OutputStream outputStream, String title, double[] distances) {
        PrintWriter writer = new PrintWriter(outputStream);
        writer.println(title); // Write the provided text

        for (double distance : distances) {
            writer.printf("%.5f ", distance); // Format each double to two decimal places
        }

        writer.println();
        writer.flush();
    }

    private DataStateExpression getCrashPenalty() {
        // Penalizes when the controlled car crashes with the vehicle in front or behind
        return (ds) -> ds.get(crashes) > 0.0 ? 1.0 : 0.0;
    }

    private DataStateFunction getSafetyGapMu() {
        // mu distribution where closest car to ego is exactly at RSS Safety Gap distance
        return (rg, ds) -> {
            int controlledVehicle = AI.getAIStates((int) ds.get(AiState.DATASTATE_INDEX_FOR_HISTORY_INDEX)).get(0).getControlledVehicleIndex();
            int closestVehicle = 0;
            for (int vehicle = 0; vehicle < observedCarCount; vehicle++) {
                if (vehicle != controlledVehicle) {
                    double vehiclePos = ds.get(xPosition[vehicle]);
                    if (vehiclePos < ds.get(xPosition[closestVehicle])){
                        closestVehicle = vehicle;
                    }
                }
            }

            double vehiclePos = ds.get(xPosition[closestVehicle]);
            double safetyGap;
            if (vehiclePos < 0) { // controlled vehicle is behind
                safetyGap = calculateRSSSafetyDistance(RESPONSE_TIME, ds.get(xSpeed[controlledVehicle]), ds.get(xSpeed[closestVehicle]));
            } else { // controlled vehicle is ahead
                safetyGap = calculateRSSSafetyDistance(RESPONSE_TIME, ds.get(xSpeed[closestVehicle]), ds.get(xSpeed[controlledVehicle]));
            }
//            System.out.println(safetyGap);
            List<DataStateUpdate> updates = new LinkedList<>();
            updates.add(new DataStateUpdate(xPosition[closestVehicle], safetyGap));
            return ds.apply(updates);
        };
    }

    private DataStateExpression getSafetyGapViolationPenaltyFn() {
        // Penalizes when the controlled car violates the safety gap with any vehicle in the observation
        return (ds) -> {
            int controlledVehicle = AI.getAIStates((int) ds.get(AiState.DATASTATE_INDEX_FOR_HISTORY_INDEX)).get(0).getControlledVehicleIndex();
            for (int vehicle = 0; vehicle < observedCarCount; vehicle++) {
                if (vehicle != controlledVehicle) {
                    double vehiclePos = ds.get(xPosition[vehicle]);
                    double safetyGap;
                    if (vehiclePos < 0) { // controlled vehicle is behind
                        safetyGap = calculateRSSSafetyDistance(RESPONSE_TIME, ds.get(xSpeed[controlledVehicle]), ds.get(xSpeed[vehicle]));
                    } else { // controlled vehicle is ahead
                        safetyGap = calculateRSSSafetyDistance(RESPONSE_TIME, ds.get(xSpeed[vehicle]), ds.get(xSpeed[controlledVehicle]));
                    }
                    if (Math.abs(vehiclePos) <= VEHICLE_LENGTH + safetyGap) {
                        return 1.0;
                    }
                }
            }
            return 0.0;
        };
    }

    private static double calculateRSSSafetyDistance(double responseTime, double rearVehicleSpeed, double frontVehicleSpeed){
        /* Formula of safety distance presented by the Responsibility-Sensitive Safety (RSS) model
         * Shalev-Shwartz, S., Shammah, S., Shashua, A.: On a formal model of safe and scalable self-driving cars.
         * CoRR abs/1708.06374 (2017), http://arxiv.org/abs/1708.0637
         */
        double d1 = responseTime*rearVehicleSpeed;
        double d2 = 0.5 * MAX_ACCELERATION*responseTime*responseTime;
        double d3 = Math.pow((rearVehicleSpeed+responseTime*MAX_ACCELERATION),2)/(2*MIN_BRAKE);
        double d4 = - (frontVehicleSpeed*frontVehicleSpeed)/(2*MAX_BRAKE);
        double rssSafetyDistance = Math.max(d1 + d2 + d3 + d4, 0);
        // The RSS model assumes vehicles as points, but ABZ case study vehicles have dimensions.
        // We add the distances from each vehicle's center to its front/rear bumpers.
        return rssSafetyDistance + VEHICLE_LENGTH;
    }
}