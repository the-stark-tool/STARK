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
import stark.udistl.*;

import java.io.*;
import java.util.*;
import java.util.function.Function;

public class AIMultipleLanes {
    private static final double RESPONSE_TIME = 1.0/60.0; // 1 Hz

    // VEHICLE DIMENSIONS
    private static final double VEHICLE_LENGTH = 5; // m
    private static final double VEHICLE_WIDTH = 2; // m

    // VARIABLE BOUNDS
    private static final double MAX_SPEED = 80; // m/s
    private static final double MAX_ACCELERATION = 25; // m/s^2
    private static final double MAX_DEACCELERATION = 30; // m/s^2, implicitly negative
    private static final double MIN_DEACCELERATION = 15; // m/s^2
    public static final double[] VX_RANGE = {-80.0, 80.0};
    public static final double[] VY_RANGE = {-80.0, 80.0};
    public static final double[] X_RANGE  = {-1000.0, 1000.0};
    public static final double[] Y_RANGE  = {-12.0, 12.0};

    private static final int STEPS_TO_SAMPLE = 160;
    private static final int EVOLUTION_SEQUENCE_SIZE = 100;

    private static final MonitoringAIStateProvider provider =
            new JSONFileReader("examples/monitoring/highwayAI/src/main/resources/short.json");
//    new JSONFileReader("examples/monitoring/highwayAI/src/main/resources/long.json");

    private String experimentName;
    private static final String RESULTS_FOLDER = "examples/monitoring/highwayAI/src/main/resources";

    /**
     * Set this to System.out to print to console, or to a FileOutputStream to write to a file.
     * Example (file): OUTPUT = new FileOutputStream("results.txt");
     * Example (console): OUTPUT = System.out;
     */
    private static final OutputStream OUTPUT = System.out;
//    private static final OutputStream OUTPUT; // append mode
//
//    static {
//        try {
//            OUTPUT = new FileOutputStream(RESULTS_FOLDER + "/long_release.txt", true);
//        } catch (FileNotFoundException e) {
//            throw new RuntimeException(e);
//        }
//    }

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

        double threshold = 0.0;
        DefaultMonitorBuilder builder = new DefaultMonitorBuilder(EVOLUTION_SEQUENCE_SIZE, false);

        runCrashExperiment(sequence, threshold, builder);
        runSafetyGapExperiment(sequence, threshold, builder);
        run5CarsExperiment(sequence, threshold, builder);
        run5CarsImpliesNoCrashExperiment(sequence, threshold, builder);
        runDesiredSpeedExperiment(sequence, threshold, builder);
        runSpeedLimitUntilFreeLaneExperiment(sequence, threshold, builder);

    }

    private void runCrashExperiment(EvolutionSequence sequence, double threshold, DefaultMonitorBuilder builder) {
        //crash
        DataStateFunction muCrash = (rg, ds) -> {
            List<DataStateUpdate> updates = new LinkedList<>();
            updates.add(new DataStateUpdate(crashes, 1.0));
            return ds.apply(updates);
        };
        // Penalizes when the controlled car crashes with the vehicle in front or behind
        DataStateExpression penaltyCrash = (ds) -> ds.get(crashes) > 0.0 ? 1.0 : 0.0;
        BrinkDisTLFormula atomicNoCrash = new BrinkDisTLFormula(muCrash, penaltyCrash, threshold);

        UDisTLFormula alwaysCrash = new UnboundedAlwaysuDisTLFormula(atomicNoCrash, 0);
        DefaultUDisTLMonitor mCrash = builder.build(alwaysCrash);
        printMonitoringSummary(sequence, mCrash, STEPS_TO_SAMPLE, "G nocrash", OUTPUT);
    }


    private DataState safetyGapMu(RandomGenerator rg, DataState ds){
        // safety gap
        // mu : dirac around datastate where all cars on the same lane as ego are exactly at RSS Safety Gap distance
        List<DataStateUpdate> updates = new LinkedList<>();
        updates.add(new DataStateUpdate(crashes, 0.0));
        int controlledVehicle = provider.getAIStates((int) ds.get(MonitoringAiState.DATASTATE_INDEX_FOR_TIMESTEP)).get(0).getControlledVehicleIndex();
        for (int other = 0; other < observedCarCount; other++) {
            double otherY = ds.get(yPosition[other]);
            // other is in the same lane as ego if other's relative distance to ego's center is less than VEHICLE WIDTH
            if (other != controlledVehicle && Math.abs(otherY) < VEHICLE_WIDTH) {
                double safetyGap = computeSafetyDistance(ds, controlledVehicle, other);
                updates.add(new DataStateUpdate(xPosition[other], safetyGap));
            }
        }
        return ds.apply(updates);
    }

    private Double safetyGapPenalty(DataState ds){
        // penalty is max of the value r of cars in the same lane as ego, where
        // for each car, the value r is 1 minus the ratio of the x distance to the ego car and the safety gap they should have, clipped to [0,1]
        if(ds.get(crashes) > 0.0){
            return 1.0;
        }
        int controlledVehicle = provider.getAIStates((int) ds.get(MonitoringAiState.DATASTATE_INDEX_FOR_TIMESTEP)).get(0).getControlledVehicleIndex();
        double maxPenalty = 0;
        for (int other = 0; other < observedCarCount; other++) {
            if (other != controlledVehicle) {
                double vehicleDist = ds.get(xPosition[other]);
                double safetyGap = computeSafetyDistance(ds, controlledVehicle, other);
                // if other is on the same lane as ego
                if (Math.abs(ds.get(yPosition[other])) < VEHICLE_WIDTH) {
                    double ratio = (Math.abs(vehicleDist) - VEHICLE_LENGTH) / (Math.abs(safetyGap));
                    double r = 1 - Math.min(ratio, 1);
                    if(r > maxPenalty){
                        maxPenalty = r;
                    }
                }
            }
        }
        return maxPenalty;
    }

    private void runSafetyGapExperiment(EvolutionSequence sequence, double threshold, DefaultMonitorBuilder builder) {
        TargetDisTLFormula atomicSG = new TargetDisTLFormula(this::safetyGapMu, this::safetyGapPenalty, threshold);
        UDisTLFormula alwaysSG = new UnboundedAlwaysuDisTLFormula(atomicSG, 0);
        DefaultUDisTLMonitor mSG = builder.build(alwaysSG);
        printMonitorEval(sequence, mSG, STEPS_TO_SAMPLE, "G RSSsd", OUTPUT);
    }

    private DataState fiveCarsLengthMu(RandomGenerator rg, DataState ds){
        // safety gap
        // mu : dirac around datastate where all cars on the same lane as ego are exactly at 3 seconds distance
        List<DataStateUpdate> updates = new LinkedList<>();
        updates.add(new DataStateUpdate(crashes, 0.0));
        int controlledVehicle = provider.getAIStates((int) ds.get(MonitoringAiState.DATASTATE_INDEX_FOR_TIMESTEP)).get(0).getControlledVehicleIndex();
        for (int other = 0; other < observedCarCount; other++) {
            double otherY = ds.get(yPosition[other]);
            // other is in the same lane as ego if other's relative distance to ego's center is less than VEHICLE WIDTH
            if (other != controlledVehicle && Math.abs(otherY) < VEHICLE_WIDTH) {
                double threeCarsDistance = compute3CarsDistance(ds, controlledVehicle, other);
                updates.add(new DataStateUpdate(xPosition[other], threeCarsDistance));
            }
        }
        return ds.apply(updates);
    }

    private Double fiveCarsPenalty(DataState ds){
        // penalty is max of the value r of cars in the same lane as ego, where
        // for each car, the value r is 1 minus the ratio of the x distance to the ego car and the safety gap they should have, clipped to [0,1]
        if(ds.get(crashes) > 0.0){
            return 1.0;
        }
        int controlledVehicle = provider.getAIStates((int) ds.get(MonitoringAiState.DATASTATE_INDEX_FOR_TIMESTEP)).get(0).getControlledVehicleIndex();
        double maxPenalty = 0;
        for (int other = 0; other < observedCarCount; other++) {
            if (other != controlledVehicle) {
                double vehicleDist = ds.get(xPosition[other]);
                double threeCars = compute3CarsDistance(ds, controlledVehicle, other);
                // if other is on the same lane as ego
                if (Math.abs(ds.get(yPosition[other])) < VEHICLE_WIDTH) {
                    double ratio = (Math.abs(vehicleDist)) / Math.abs(threeCars);
                    double r = 1 - Math.min(ratio, 1);
                    if(r > 0){
                        r = r;
                    }
                    if(r > maxPenalty){
                        maxPenalty = r;
                    }
                }
            }
        }
        return maxPenalty;
    }

    private void run5CarsExperiment(EvolutionSequence sequence, double threshold, DefaultMonitorBuilder builder) {
        TargetDisTLFormula atomicSG = new TargetDisTLFormula(this::fiveCarsLengthMu, this::fiveCarsPenalty, threshold);
        UDisTLFormula alwaysSG = new UnboundedAlwaysuDisTLFormula(atomicSG, 0);
        DefaultUDisTLMonitor mSG = builder.build(alwaysSG);
        printMonitorEval(sequence, mSG, STEPS_TO_SAMPLE, "G 5csd", OUTPUT);
    }

    private double compute3CarsDistance(DataState ds, int ego, int other){
        return VEHICLE_LENGTH*5;
    }

    private void run5CarsImpliesNoCrashExperiment(EvolutionSequence sequence, double threshold, DefaultMonitorBuilder builder) {

        //crash
        DataStateFunction muCrash = (rg, ds) -> {
            List<DataStateUpdate> updates = new LinkedList<>();
            updates.add(new DataStateUpdate(crashes, 1.0));
            return ds.apply(updates);
        };
        // Penalizes when the controlled car crashes with the vehicle in front or behind
        DataStateExpression penaltyCrash = (ds) -> ds.get(crashes) > 0.0 ? 1.0 : 0.0;
        TargetDisTLFormula atomicCrash = new TargetDisTLFormula(muCrash, penaltyCrash, threshold);

        TargetDisTLFormula atomic5csd = new TargetDisTLFormula(this::fiveCarsLengthMu, this::fiveCarsPenalty, threshold);

        UDisTLFormula disjunction = new DisjunctionDisTLFormula(atomic5csd, new UnboundedEventuallyuDisTLFormula(atomicCrash, 0));
        UDisTLFormula safetyGapImpliesNoCrash = new UnboundedAlwaysuDisTLFormula(disjunction, 0);
        DefaultUDisTLMonitor mSGimpliesNoCrash = builder.build(safetyGapImpliesNoCrash);

        printMonitorEval(sequence, mSGimpliesNoCrash, STEPS_TO_SAMPLE, "G (sg->nocrash)", OUTPUT);
        printStepwiseMonitorEval(sequence, atomicCrash, builder, STEPS_TO_SAMPLE, "stepwise crash", OUTPUT);
        printStepwiseMonitorEval(sequence, atomic5csd, builder, STEPS_TO_SAMPLE, "stepwise 5csd", OUTPUT);
//        printStepwiseEvaluationSummary(sequence, disjunction, builder, STEPS_TO_SAMPLE, "disj", OUTPUT);
    }

    private void runDesiredSpeedExperiment(EvolutionSequence sequence, double threshold, DefaultMonitorBuilder builder) {
        // desired speed
        double DESIRED_SPEED = 25.0;
        // muDesiredSpeed: gaussian with mean DESIRED_SPEED
        DataStateFunction muDesiredSpeed = (rg, ds) -> {
            int controlledVehicle = provider.getAIStates((int) ds.get(MonitoringAiState.DATASTATE_INDEX_FOR_TIMESTEP)).get(0).getControlledVehicleIndex();
            List<DataStateUpdate> updates = new LinkedList<>();
            updates.add(new DataStateUpdate(xSpeed[controlledVehicle], 0.3*rg.nextGaussian()+DESIRED_SPEED));
            return ds.apply(updates);
        };
        // penalty: difference between speed of ego and desired speed, normalized
        DataStateExpression penaltyDesiredSpeed = (ds) -> {
            int controlledVehicle = provider.getAIStates((int) ds.get(MonitoringAiState.DATASTATE_INDEX_FOR_TIMESTEP)).get(0).getControlledVehicleIndex();
            double egoSpeed = ds.get(xSpeed[controlledVehicle]);
            return Math.abs((DESIRED_SPEED - egoSpeed)/(VX_RANGE[1] - VX_RANGE[0]));
        };
        TargetDisTLFormula atomicDesiredSpeed = new TargetDisTLFormula(muDesiredSpeed, penaltyDesiredSpeed, threshold);
        UDisTLFormula alwaysDesiredSpeed = new EventuallyDisTLFormula(atomicDesiredSpeed,20,100);
        DefaultUDisTLMonitor mDesiredSpeed = builder.build(alwaysDesiredSpeed);

        printMonitoringSummary(sequence, mDesiredSpeed, STEPS_TO_SAMPLE, "F desiredspeed", OUTPUT);
    }

    private DataStateFunction createMuFW() {
        return (rg, ds) -> {
            int controlledVehicle = provider.getAIStates((int) ds.get(MonitoringAiState.DATASTATE_INDEX_FOR_TIMESTEP)).get(0).getControlledVehicleIndex();
            List<DataStateUpdate> updates = new LinkedList<>();
            for (int other = 0; other < observedCarCount; other++) {
                double xOtherDistance = ds.get(xPosition[other]);
                double yOtherDistance = ds.get(yPosition[other]);
                if (other != controlledVehicle
                        && 0 < xOtherDistance
                        && Math.abs(yOtherDistance) < VEHICLE_WIDTH
                ) {
                    if (yOtherDistance < 0) {
                        updates.add(new DataStateUpdate(yPosition[other], yPosition[other] - VEHICLE_WIDTH));
                    } else {
                        updates.add(new DataStateUpdate(yPosition[other], yPosition[other] + VEHICLE_WIDTH));
                    }
                }
            }
            return ds.apply(updates);
        };
    }

    private DataStateExpression createPenaltyFW() {
        return (DataState ds) -> {
            int controlledVehicle = provider.getAIStates((int) ds.get(MonitoringAiState.DATASTATE_INDEX_FOR_TIMESTEP)).get(0).getControlledVehicleIndex();
            double minXDistance = X_RANGE[1];
            for (int other = 0; other < observedCarCount; other++) {
                double xOtherDistance = ds.get(xPosition[other]);
                double yOtherDistance = ds.get(yPosition[other]);
                if (other != controlledVehicle
                        && 0 < xOtherDistance
                        && Math.abs(yOtherDistance) < VEHICLE_WIDTH
                ) {
                    if (xOtherDistance < minXDistance) {
                        minXDistance = xOtherDistance;
                    }
                }
            }
            return 1 - (minXDistance / X_RANGE[1]);
        };
    }

    private DataStateFunction createMuSpeedLimit(double speedLimit, double speedLowerBound, double s, double decay) {
        return (rg, ds) -> {
            int controlledVehicle = provider.getAIStates((int) ds.get(MonitoringAiState.DATASTATE_INDEX_FOR_TIMESTEP)).get(0).getControlledVehicleIndex();
            List<DataStateUpdate> updates = new LinkedList<>();
            double u = rg.nextDouble();
            double idealSpeed;
            if (u < s) {
                idealSpeed = speedLimit + (-Math.log(1.0 - rg.nextDouble()) / decay);
            } else {
                idealSpeed = speedLowerBound + (rg.nextDouble() * (speedLimit - speedLowerBound));
            }
            updates.add(new DataStateUpdate(xSpeed[controlledVehicle], idealSpeed));
            return ds.apply(updates);
        };
    }

    private DataStateExpression createPenaltySpeedLimit(double speedLimit) {
        return (ds) -> {
            int controlledVehicle = provider.getAIStates((int) ds.get(MonitoringAiState.DATASTATE_INDEX_FOR_TIMESTEP)).get(0).getControlledVehicleIndex();
            double egoSpeed = ds.get(xSpeed[controlledVehicle]);
            if(egoSpeed <= speedLimit){
                return 0;
            }
            return (Math.abs(egoSpeed)-speedLimit)/MAX_SPEED;
        };
    }

    private void runSpeedLimitUntilFreeLaneExperiment(EvolutionSequence sequence, double threshold, DefaultMonitorBuilder builder) {
        double SPEED_LIMIT = 50.0;
        double SPEED_LOWERBOUND = -30.0;
        double s = 0.1;
        double decay = 0.01;

        TargetDisTLFormula atomicFW = new TargetDisTLFormula(createMuFW(), createPenaltyFW(), threshold);
        TargetDisTLFormula atomicSpeedLimit = new TargetDisTLFormula(createMuSpeedLimit(SPEED_LIMIT, SPEED_LOWERBOUND, s, decay), createPenaltySpeedLimit(SPEED_LIMIT), threshold);

        UDisTLFormula formula = new UnboundedReleaseuDisTLFormula(new UnboundedAlwaysuDisTLFormula(atomicSpeedLimit,0), atomicFW,0);

        DefaultUDisTLMonitor mRelease = builder.build(formula);
        printMonitorEval(sequence, mRelease, STEPS_TO_SAMPLE, "sl U fl",OUTPUT);
        printStepwiseMonitorEval(sequence, atomicFW, builder, STEPS_TO_SAMPLE, "stepwise fl", OUTPUT);
        printStepwiseMonitorEval(sequence, atomicSpeedLimit, builder, STEPS_TO_SAMPLE, "stepwise sl", OUTPUT);

    }


    private double computeSafetyDistance(DataState ds, int ego, int other) {
        double vehicleDist = ds.get(xPosition[other]);
        double egoSpeed = ds.get(xSpeed[ego]);
        double vehicleSpeed = egoSpeed + ds.get(xSpeed[other]);
        double safetyGap;
        if (vehicleDist > 0) { // controlled vehicle is behind
            safetyGap = calculateRSSSafetyDistanceFormula(RESPONSE_TIME, egoSpeed, vehicleSpeed);
        } else { // controlled vehicle is ahead, so distance is negative
            safetyGap = -calculateRSSSafetyDistanceFormula(RESPONSE_TIME, vehicleSpeed, egoSpeed);
        } // VEHICLE_LENGTH is added because RSS formula computes distance between front-most point of rear car
        // and rear-most point of front while our model computes distances from center of cars
        return safetyGap;
    }

    private Function<RandomGenerator, SystemState> readInitialState() {
        ArrayList<MonitoringAiState> initialAiState = provider.readInitialState();
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
        int currentTimestep = (int) ds.get(MonitoringAiState.DATASTATE_INDEX_FOR_TIMESTEP);
        ArrayList<MonitoringAiState> nextTimestepStates = provider.getAIStates(currentTimestep + 1);
        MonitoringAiState newAiState = nextTimestepStates.get(rg.nextInt(nextTimestepStates.size()));
        DataState newState = newAiState.getDataState();
        updateDataStateIndexes(newAiState);
        return newState;
    }

    private void updateDataStateIndexes(MonitoringAiState aiState) {
        presence = aiState.getRealDataStatePresenceIndexes();
        xSpeed = aiState.getRealDataStateXSpeedIndexes();
        xPosition = aiState.getRealDataStateXPositionIndexes();

        ySpeed = aiState.getRealDataStateYSpeedIndexes();
        yPosition = aiState.getRealDataStateYPositionIndexes();

        crashes = aiState.getCrashesIndex();
        observedCarCount = aiState.getCarCount();
    }

    @FunctionalInterface
    private interface MonitorFeeder {
        OptionalDouble feed(DefaultUDisTLMonitor m, SampleSet<PerceivedSystemState> distribution, int i);
    }

    private void printStepwiseEvaluationSummary(EvolutionSequence sequence, UDisTLFormula formula, DefaultMonitorBuilder builder, int stepsToPrint, String title, OutputStream outputStream) {
        feedAndPrintMonitoringSummary(sequence, builder.build(formula, 0), stepsToPrint, title, outputStream,
                (monitor, distribution, i) -> {
                    return builder.build(formula, 0).evalNext(distribution);
                });
    }


    private void printMonitoringSummary(EvolutionSequence sequence, DefaultUDisTLMonitor m, int stepsToPrint, String title, OutputStream outputStream) {
        feedAndPrintMonitoringSummary(sequence, m, stepsToPrint, title, outputStream,
                (monitor, distribution, i) -> monitor.evalNext(distribution));
    }

    private void feedAndPrintMonitoringSummary(EvolutionSequence sequence, DefaultUDisTLMonitor m, int stepsToPrint, String title, OutputStream outputStream, MonitorFeeder feeder) {
        PrintWriter writer = new PrintWriter(outputStream);
        writer.printf("%s%n" + ("%6s, ").repeat(24) + "%n", title,
                "i", "mon", "n", "crash", "p0", "x0", "y0", "v0", "p1", "x1", "y1", "v1", "p2", "x2", "y2", "v2", "p3", "x3", "y3", "v3", "p4", "x4", "y4", "v4");
        int i = 0;
        while (i < stepsToPrint) {
            SampleSet<PerceivedSystemState> distribution = sequence.getAsPerceivedSystemStates(i);
            OptionalDouble monitorEval = feeder.feed(m, distribution, i);
            ArrayList<String> s = new ArrayList<>();

            OptionalDouble selfPerceivedI = Arrays.stream(distribution.evalPenaltyFunction
                    (ds -> ds.get(MonitoringAiState.DATASTATE_INDEX_FOR_TIMESTEP))).average();
            s.add(String.format("%" + 6 + ".2f,", selfPerceivedI.orElse(Double.NaN)));
            s.add(monitorEval.isPresent() ? String.format("%" + 6 + ".2f,", monitorEval.getAsDouble()) : "n");
            s.add(String.format("%" + 6 + "d,", distribution.size()));
            OptionalDouble c = Arrays.stream(distribution.evalPenaltyFunction(ds -> ds.get(crashes))).average();
            s.add(String.format("%" + 6 + ".2f,", c.orElse(Double.NaN)));
//            OptionalDouble sd = Arrays.stream(distribution.evalPenaltyFunction(this::minimumSG)).average();
//            s.add(String.format("%" + 6 + ".2f,", sd.orElse(Double.NaN)));

            for (int j = 0; j < observedCarCount; j++) {
                int finalJ = j;
                OptionalDouble p = Arrays.stream(distribution.evalPenaltyFunction(ds -> ds.get(presence[finalJ]))).average();
                s.add(String.format("%" + 6 + ".2f,", p.orElse(Double.NaN)));
                OptionalDouble x = Arrays.stream(distribution.evalPenaltyFunction(ds -> ds.get(xPosition[finalJ]))).average();
                s.add(String.format("%" + 6 + ".2f,", x.orElse(Double.NaN)));
                OptionalDouble y = Arrays.stream(distribution.evalPenaltyFunction(ds -> ds.get(yPosition[finalJ]))).average();
                s.add(String.format("%" + 6 + ".2f,", y.orElse(Double.NaN)));
                OptionalDouble v = Arrays.stream(distribution.evalPenaltyFunction(ds -> ds.get(xSpeed[finalJ]))).average();
                s.add(String.format("%" + 6 + ".2f,", v.orElse(Double.NaN)));
            }

            writer.println(String.join(" ", s));
            writer.flush();

            i++;
        }
    }

    private void printMonitorEval(
            EvolutionSequence sequence,
            DefaultUDisTLMonitor m,
            int stepsToPrint, String title,
            OutputStream outputStream
    ) {
        PrintWriter writer = new PrintWriter(outputStream);
        writer.println(title);
        for (int i = 0; i < stepsToPrint; i++) {
            SampleSet<PerceivedSystemState> distribution =
                    sequence.getAsPerceivedSystemStates(i);

            OptionalDouble monitorEval = m.evalNext(distribution);

            writer.println(
                    monitorEval.isPresent()
                            ? String.format(Locale.GERMAN, "%.6f", monitorEval.getAsDouble())
                            : "u"
            );

            writer.flush();
        }
    }

    private void printStepwiseMonitorEval(
            EvolutionSequence sequence,
            UDisTLFormula formula,
            DefaultMonitorBuilder builder,
            int stepsToPrint, String title,
            OutputStream outputStream
    ) {
        PrintWriter writer = new PrintWriter(outputStream);
        writer.println(title);
        for (int i = 0; i < stepsToPrint; i++) {
            SampleSet<PerceivedSystemState> distribution =
                    sequence.getAsPerceivedSystemStates(i);
            builder.build(formula, 0).evalNext(distribution);
            OptionalDouble monitorEval =
                    builder.build(formula, 0).evalNext(distribution);

            writer.println(
                    monitorEval.isPresent()
                            ? String.format(Locale.GERMAN, "%.6f", monitorEval.getAsDouble())
                            : "u"
            );

            writer.flush();
        }
    }

    private static double calculateRSSSafetyDistanceFormula(double responseTime, double rearVehicleSpeed, double frontVehicleSpeed){
        /* Formula of safety distance presented by the Responsibility-Sensitive Safety (RSS) model
         * Shalev-Shwartz, S., Shammah, S., Shashua, A.: On a formal model of safe and scalable self-driving cars.
         * CoRR abs/1708.06374 (2017), http://arxiv.org/abs/1708.0637
         */
        double d1 = responseTime*rearVehicleSpeed;
        double d2 = 0.5 * MAX_ACCELERATION*responseTime*responseTime;
        double d3 = Math.pow((rearVehicleSpeed+(responseTime*MAX_ACCELERATION)),2)/(2* MIN_DEACCELERATION);
        double d4 = - (frontVehicleSpeed*frontVehicleSpeed)/(2* MAX_DEACCELERATION);
        double rssSafetyDistance = Math.max(d1 + d2 + d3 + d4, 0);
        // The RSS model assumes vehicles as points, but ABZ case study vehicles have dimensions.
        // We add the distances from each vehicle's center to its front/rear bumpers.
        return rssSafetyDistance + VEHICLE_LENGTH;
    }
}