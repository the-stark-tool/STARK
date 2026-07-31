package Scenarios;

import stark.*;
import stark.controller.Controller;
import stark.controller.ControllerRegistry;
import stark.controller.ExecController;
import stark.distance.*;
import stark.ds.DataState;
import stark.ds.DataStateExpression;
import stark.ds.DataStateFunction;
import stark.ds.DataStateUpdate;
import stark.ds.RelationOperator;
import stark.perturbation.AtomicPerturbation;
import stark.perturbation.Perturbation;
import stark.perturbation.SequentialPerturbation;
import stark.robtl.AtomicRobustnessFormula;
import stark.robtl.AlwaysRobustnessFormula;
import stark.robtl.ConjunctionRobustnessFormula;
import stark.robtl.RobustnessFormula;
import stark.robtl.ThreeValuedSemanticsVisitor;
import stark.robtl.TruthValues;
import org.apache.commons.math3.random.RandomGenerator;

import java.util.*;
import java.util.function.BinaryOperator;
import java.util.function.Function;
import java.util.stream.Collectors;


public class OneLaneMerge {

    //-------------Variables---------------

    private static final double RESPONSE_TIME = 1;

    private static final int NUMBER_OF_VEHICLES = 3;

    // Vehicle dimensions
    private static final double VEHICLE_LENGTH = 5;
    private static final double VEHICLE_WIDTH = 2;

    // Vehicle limits
    private static final double MAX_SPEED = 40;
    private static final double MAX_ACCELERATION = 5;
    private static final double MAX_ACCEL_OFFSET = 1;
    private static final double MAX_BRAKE = 5;
    private static final double MIN_BRAKE = 3;
    private static final double IDLE_DELTA = 1;
    // Max |acceleration| the uncontrolled (rear/front) cars apply while cruising.
    private static final double UNCONTROLLED_ACCEL_DELTA = 2;

    // PERTURBATION PARAMETERS
    private static final int PERTURBATION_STARTING_STEP = 0;
    private static final int PERTURBATION_TIMES = 40;
    private static final int PERTURBATION_SCALE = 120;
    private static final double FRONT_PERTURB_CHANCE  = 0.5;
    private static final double REAR_PERTURB_CHANCE  = 0.5;
    private static final int EGO_DELAY_STEPS = 3;

    //Init Values
    // 0 -> rear vehicle, 1 -> ego vehicle, 2 -> front vehicle
    private static final double[] INIT_SPEED = {20, 18, 20};
    private static final double[] INIT_ACCEL = {0, 0, 0};



    // Case 1: front gap unsafe, ego slows down first
    private static final double[] INIT_DISTANCE_BETWEEN = {130, 60};

    // Case 2: immediate merge
    //private static final double[] INIT_DISTANCE_BETWEEN = {110, 80};

    // Case 3: borderline unsafe gap
    //  private static final double[] INIT_DISTANCE_BETWEEN = {90, 65};

    private static final int CONTROLLED_VEHICLE = 1;

    // -------------Datastate variable indexes-------------
    private static final int intention = 0;
    private static final int[] speed = new int[]{1,2,3};
    private static final int[] safety_gap = new int[]{4,5};
    private static final int[] accel = new int[]{6,7,8};
    private static final int[] distance = new int[]{9,10};
    // Merge-specific variables
    private static final int merged = 11;
    private static final int lat_safe = 12;
    private static final int dist_end = 13;
    private static final int crit_dist = 14;

    // PERTURBATION PARAMETERS
    private static final int STARTING_STEP = 0;
    private static final int EGO_ACCEL_BUF_BASE = 24;

    // ROBUSTNESS FORMULAE PARAMETERS
    private static final double ETA_SAFETY_GAP_VIOLATION = 0.5; // Maximum acceptable risk of violating safety gap
    private static final double ETA_MERGE_DELAY = 0.3; // Maximum acceptable risk of merge delay
    private static final double ETA_LAT_SAFETY = 0.1;  // Maximum acceptable risk of lateral-safety violation
    private static final int ONSET_WINDOW = 15;
    private static final int BOOTSTRAP_SAMPLES = 100;
    private static final double CI_Z = 1.96; // 95% confidence level

    // LANE/MERGE RELATED VARIABLES
    private static final double LANE_WIDTH = 4;
    private static final double RAMP_Y = 0;
    private static final double HIGHWAY_Y = 4;
    private static final double MERGE_TIMER_INIT = 4;

    // Lateral merge variables
    private static final int ego_lane = 15;
    private static final int merge_timer = 17;

    // x[i] = longitudinal position, y[i] = lateral position
    private static final int[] x = new int[]{18, 19, 20};
    private static final int[] y = new int[]{21, 22, 23};

    private static final int NUMBER_OF_VARIABLES = EGO_ACCEL_BUF_BASE + EGO_DELAY_STEPS;

    // controller intentions
    private static final double FASTER = 1.0;
    private static final double SLOWER = -1.0;
    private static final double IDLE = 0.0;
    private static final double MERGE = 2.0;

    private static final int TIME_HORIZON = 45;
    private static final int EVOLUTION_SEQUENCE_SIZE = 250;

    public OneLaneMerge() {
        DataState state = getInitialState();
        RandomGenerator rand = new DefaultRandomGenerator();
        ControlledSystem system = new ControlledSystem(
                getController(),
                (rg, ds) -> ds.apply(getEnvironmentUpdates(rg, ds)),
                state
        );

        EvolutionSequence sequence = new EvolutionSequence(
                new SilentMonitor("OneLaneMerge"),
                rand,
                rg -> system,
                EVOLUTION_SEQUENCE_SIZE
        );

        printSummary(sequence, TIME_HORIZON, "UNPERTURBED ONE-LANE MERGE");
/*
        Perturbation suddenSpeedChange = getSuddenSpeedChangePerturbation(PERTURBATION_TIMES);
        EvolutionSequence suddenSpeedChangeSequence = sequence.apply(suddenSpeedChange, PERTURBATION_STARTING_STEP, PERTURBATION_SCALE);
        printSummary(suddenSpeedChangeSequence, TIME_HORIZON, "PERTURBED - SUDDEN SPEED CHANGE");

        Perturbation egoDelay = getEgoDelayPerturbation(PERTURBATION_TIMES, EGO_DELAY_STEPS);
        EvolutionSequence egoDelaySequence = sequence.apply(egoDelay, PERTURBATION_STARTING_STEP, PERTURBATION_SCALE);
        printSummary(egoDelaySequence, TIME_HORIZON, "PERTURBED - EGO ACTUATION DELAY");

        //----------------------------------------ROBTL-----------------------------------//
        evaluateRobustness("SUDDEN SPEED CHANGE", suddenSpeedChange, sequence, rand, "");
        evaluateRobustness("EGO ACTUATION DELAY", egoDelay, sequence, rand, "_egodelay");
        //----------------------------------------ROBTL-----------------------------------//
*/
        launchAnimation();
    }

    private void evaluateRobustness(String tag, Perturbation perturbation,
                                    EvolutionSequence sequence, RandomGenerator rand,
                                    String fileSuffix) {
        System.out.println("==================== ROBUSTNESS: " + tag + " ====================");

        RobustnessFormula safetygap = getSafetyGapViolationFormula(perturbation);
        DistanceExpression sgDist = getSafetyGapDistance();
        double[][] eta_crash = new double[20][6];           // rich eta CSV
        for (int i = 0; i < 20; i++) {
            TruthValues value = new ThreeValuedSemanticsVisitor(rand, BOOTSTRAP_SAMPLES, CI_Z)
                    .eval(safetygap).eval(PERTURBATION_SCALE, i, sequence);
            double[] ci = sgDist.evalCI(rand, i, sequence,
                    sequence.apply(perturbation, i, PERTURBATION_SCALE), BOOTSTRAP_SAMPLES, CI_Z);
            System.out.printf("[%s] safety-gap   step %2d: verdict=%-7s dist=%.4f  CI=[%.4f, %.4f]  (eta=%.2f)%n",
                    tag, i, value, ci[0], ci[1], ci[2], ETA_SAFETY_GAP_VIOLATION);
            eta_crash[i] = new double[]{i, verdictToNum(value), ci[0], ci[1], ci[2], ETA_SAFETY_GAP_VIOLATION};
        }

        //----------------------------------------VERIFICATION-----------------------------------//
        RobustnessFormula mergeDelay = getMergeDelayFormula(perturbation);

        DistanceExpression mdDist = getMergeDelayDistance();
        double[][] eta_merge = new double[20][6];
        for (int i = 0; i < 20; i++) {
            TruthValues v = new ThreeValuedSemanticsVisitor(rand, BOOTSTRAP_SAMPLES, CI_Z)
                    .eval(mergeDelay).eval(PERTURBATION_SCALE, i, sequence);
            double[] ci = mdDist.evalCI(rand, i, sequence,
                    sequence.apply(perturbation, i, PERTURBATION_SCALE), BOOTSTRAP_SAMPLES, CI_Z);
            System.out.printf("[%s] merge-delay  step %2d: verdict=%-7s dist=%.4f  CI=[%.4f, %.4f]  (eta=%.2f)%n",
                    tag, i, v, ci[0], ci[1], ci[2], ETA_MERGE_DELAY);
            eta_merge[i] = new double[]{i, verdictToNum(v), ci[0], ci[1], ci[2], ETA_MERGE_DELAY};
        }

        RobustnessFormula latSafety = getLatSafetyFormula(perturbation);
        DistanceExpression latDist = getLatSafetyDistance();
        double[][] eta_lat = new double[10][6];
        for (int i = 0; i < 10; i++) {
            TruthValues v = new ThreeValuedSemanticsVisitor(rand, BOOTSTRAP_SAMPLES, CI_Z)
                    .eval(latSafety).eval(PERTURBATION_SCALE, i, sequence);
            double[] ci = latDist.evalCI(rand, i, sequence,
                    sequence.apply(perturbation, i, PERTURBATION_SCALE), BOOTSTRAP_SAMPLES, CI_Z);
            System.out.printf("[%s] lat-safety   step %2d: verdict=%-7s dist=%.4f  CI=[%.4f, %.4f]  (eta=%.2f)%n",
                    tag, i, v, ci[0], ci[1], ci[2], ETA_LAT_SAFETY);
            eta_lat[i] = new double[]{i, verdictToNum(v), ci[0], ci[1], ci[2], ETA_LAT_SAFETY};
        }

        // Phi_safe = Always[0,K]( phi_long AND phi_lat )            -- safety group only
        RobustnessFormula phiSafe = new AlwaysRobustnessFormula(
                new ConjunctionRobustnessFormula(
                        getSafetyGapViolationFormula(perturbation),
                        getLatSafetyFormula(perturbation)),
                0, ONSET_WINDOW);
        TruthValues verdictSafe = new ThreeValuedSemanticsVisitor(rand, BOOTSTRAP_SAMPLES, CI_Z)
                .eval(phiSafe).eval(PERTURBATION_SCALE, 0, sequence);
        System.out.println("[" + tag + "] Combined verdict (SAFE) - Always[0," + ONSET_WINDOW + "](long AND lat): " + verdictSafe);
        double[][] val_combined_safe = {{ verdictToNum(verdictSafe) }};

        // Phi_all = Always[0,K]( phi_long AND phi_lat AND phi_md )  -- all three
        RobustnessFormula phiAll = new AlwaysRobustnessFormula(
                new ConjunctionRobustnessFormula(
                        new ConjunctionRobustnessFormula(
                                getSafetyGapViolationFormula(perturbation),
                                getLatSafetyFormula(perturbation)),
                        getMergeDelayFormula(perturbation)),
                0, ONSET_WINDOW);
        TruthValues verdictAll = new ThreeValuedSemanticsVisitor(rand, BOOTSTRAP_SAMPLES, CI_Z)
                .eval(phiAll).eval(PERTURBATION_SCALE, 0, sequence);
        System.out.println("[" + tag + "] Combined verdict (ALL)  - Always[0," + ONSET_WINDOW + "](long AND lat AND mergeDelay): " + verdictAll);
        double[][] val_combined_all = {{ verdictToNum(verdictAll) }};

        try {
            // CSVs
            Util.writeToCSV("./three_val_combined_safe" + fileSuffix + ".csv", val_combined_safe);
            Util.writeToCSV("./three_val_combined_all" + fileSuffix + ".csv", val_combined_all);
            writeEtaCSV("./eta_safety_gap" + fileSuffix + ".csv", eta_crash, ETA_SAFETY_GAP_VIOLATION);
            writeEtaCSV("./eta_merge_delay" + fileSuffix + ".csv", eta_merge, ETA_MERGE_DELAY);
            writeEtaCSV("./eta_lat_safety" + fileSuffix + ".csv", eta_lat, ETA_LAT_SAFETY);
        } catch (java.io.IOException e) {
            System.out.println("Failed to write robustness CSV for " + tag + ": " + e.getMessage());
        }
    }

    // .
    private static double verdictToNum(TruthValues v) {
        return (v == TruthValues.TRUE) ? 1 : (v == TruthValues.UNKNOWN ? 0 : -1);
    }

    // Writes a robustness CSV
    private static void writeEtaCSV(String fileName, double[][] rows, double eta) throws java.io.IOException {
        StringBuilder sb = new StringBuilder("step, verdict, dist, ci_lo, ci_hi, eta\n");
        for (double[] r : rows) {
            sb.append(String.format(java.util.Locale.US, "%.0f, %.0f, %.6f, %.6f, %.6f, %.4f%n",
                    r[0], r[1], r[2], r[3], r[4], eta));
        }
        java.nio.file.Files.writeString(java.nio.file.Path.of(fileName), sb.toString());
    }

    // Runs one single-sample evolution of the scenario
    private void launchAnimation() {
        RandomGenerator rand = new DefaultRandomGenerator();
        ControlledSystem system = new ControlledSystem(
                getController(),
                (rg, ds) -> ds.apply(getEnvironmentUpdates(rg, ds)),
                getInitialState()
        );
        EvolutionSequence animSeq = new EvolutionSequence(
                new SilentMonitor("OneLaneMergeGUI"),
                rand,
                rg -> system,
                1
        );

        List<GUI.Frame> frames = new ArrayList<>();
        for (int i = 0; i < TIME_HORIZON; i++) {
            SampleSet<SystemState> dss = animSeq.get(i);
            double[] px = new double[NUMBER_OF_VEHICLES];
            double[] py = new double[NUMBER_OF_VEHICLES];
            double[] pv = new double[NUMBER_OF_VEHICLES];
            for (int k = 0; k < NUMBER_OF_VEHICLES; k++) {
                final int kk = k;
                px[k] = dss.evalPenaltyFunction(ds -> ds.get(x[kk]))[0];
                py[k] = dss.evalPenaltyFunction(ds -> ds.get(y[kk]))[0];
                pv[k] = dss.evalPenaltyFunction(ds -> ds.get(speed[kk]))[0];
            }
            double mergedVal = dss.evalPenaltyFunction(ds -> ds.get(merged))[0];
            frames.add(new GUI.Frame(px, py, pv, mergedVal));
        }

        GUI.show(frames, CONTROLLED_VEHICLE, VEHICLE_LENGTH, VEHICLE_WIDTH, RAMP_Y, HIGHWAY_Y);
    }

    //----------------------------------------ROBTL-----------------------------------//
    private static DistanceExpression getSafetyGapDistance() {
        return new MaxIntervalDistanceExpression(
                new AtomicDistanceExpression(getSafetyGapEntityPenaltyFn(), (v1, v2) -> Math.abs(v1 - v2)),
                STARTING_STEP, TIME_HORIZON - 1);
    }

    private static DistanceExpression getLatSafetyDistance() {
        // penalizes when the perturbed run is MORE laterally unsafe than nominal
        return new MaxIntervalDistanceExpression(
                new AtomicDistanceExpressionLeq(getLatSafetyPenaltyFn()),
                STARTING_STEP, TIME_HORIZON - 1);
    }

    private static DistanceExpression getMergeDelayDistance() {
        // penalizes when nominal merge progress exceeds the perturbed one
        return new MaxIntervalDistanceExpression(
                new AtomicDistanceExpressionGeq(getMergeProgressPenaltyFn()),
                STARTING_STEP, TIME_HORIZON - 1);
    }

    private static RobustnessFormula getSafetyGapViolationFormula(Perturbation perturbation) {
        // Penalizes when the controlled car violates de safety gap
        return new AtomicRobustnessFormula(
                perturbation,
                getSafetyGapDistance(),
                RelationOperator.LESS_OR_EQUAL_THAN,
                ETA_SAFETY_GAP_VIOLATION
        );
    }

    private static RobustnessFormula getLatSafetyFormula(Perturbation perturbation) {
        // one-sided penalty that fires when the perturbed run is MORE laterally unsafe than the nominal one
        return new AtomicRobustnessFormula(
                perturbation, getLatSafetyDistance(), RelationOperator.LESS_OR_EQUAL_THAN, ETA_LAT_SAFETY);
    }

    private static RobustnessFormula getMergeDelayFormula(Perturbation perturbation) {
        // penalizes when nominal merge progress exceeds the perturbed
        return new AtomicRobustnessFormula(
                perturbation, getMergeDelayDistance(), RelationOperator.LESS_OR_EQUAL_THAN, ETA_MERGE_DELAY);
    }
    
    private static DataStateExpression getSafetyGapEntityPenaltyFn() {
        return ds ->
                ds.get(distance[CONTROLLED_VEHICLE]) > ds.get(safety_gap[CONTROLLED_VEHICLE])
                        && ds.get(distance[CONTROLLED_VEHICLE - 1]) > ds.get(safety_gap[CONTROLLED_VEHICLE - 1])
                        ? 0.0
                        : Math.min(1, Math.max(0, Math.max(
                        (ds.get(safety_gap[CONTROLLED_VEHICLE]) - ds.get(distance[CONTROLLED_VEHICLE])) / ds.get(safety_gap[CONTROLLED_VEHICLE]),
                        (ds.get(safety_gap[CONTROLLED_VEHICLE - 1]) - ds.get(distance[CONTROLLED_VEHICLE - 1])) / ds.get(safety_gap[CONTROLLED_VEHICLE - 1]))));
    }

    private static DataStateExpression getMergeProgressPenaltyFn() {
        return ds -> (ds.get(y[CONTROLLED_VEHICLE]) - RAMP_Y) / (HIGHWAY_Y - RAMP_Y); // 0..1
    }

    private static DataStateExpression getLatSafetyPenaltyFn() {
        return ds -> 1.0 - ds.get(lat_safe); // 1 = laterally unsafe, 0 = safe
    }
    //----------------------------------------ROBTL-----------------------------------//

    // Physics changes
    private DataState getInitialState() {
        Map<Integer, Double> values = new HashMap<>();

        values.put(intention, IDLE);

        for (int i = 0; i < NUMBER_OF_VEHICLES; i++) {
            values.put(speed[i], INIT_SPEED[i]);
            values.put(accel[i], INIT_ACCEL[i]);

            if (i < NUMBER_OF_VEHICLES - 1) {
                values.put(distance[i], INIT_DISTANCE_BETWEEN[i]);

                double initialSafetyGap = calculateRSSSafetyDistance(
                        RESPONSE_TIME,
                        INIT_SPEED[i],
                        INIT_SPEED[i + 1]
                );

                values.put(safety_gap[i], initialSafetyGap);
            }
        }

        values.put(merged, 0.0);
        values.put(lat_safe, 1.0);
        values.put(dist_end, 700.0);
        values.put(crit_dist, 300.0);

        values.put(ego_lane, 0.0);
        values.put(merge_timer, 0.0);

        // Ego actuation-delay
        for (int i = 0; i < EGO_DELAY_STEPS; i++) {
            values.put(EGO_ACCEL_BUF_BASE + i, 0.0);
        }

        //  x = 0.
        // Vehicles ahead of ego get positive x, behind get negative x,
        // INIT_DISTANCE_BETWEEN (gap) + VEHICLE_LENGTH
        values.put(x[CONTROLLED_VEHICLE], 0.0);
        double cumulative = 0.0;
        for (int i = CONTROLLED_VEHICLE + 1; i < NUMBER_OF_VEHICLES; i++) {
            cumulative += INIT_DISTANCE_BETWEEN[i - 1] + VEHICLE_LENGTH;
            values.put(x[i], cumulative);
        }
        cumulative = 0.0;
        for (int i = CONTROLLED_VEHICLE - 1; i >= 0; i--) {
            cumulative -= INIT_DISTANCE_BETWEEN[i] + VEHICLE_LENGTH;
            values.put(x[i], cumulative);
        }

        // ego starts on the ramp, all others on the highway.
        for (int i = 0; i < NUMBER_OF_VEHICLES; i++) {
            values.put(y[i], i == CONTROLLED_VEHICLE ? RAMP_Y : HIGHWAY_Y);
        }

        return new DataState(NUMBER_OF_VARIABLES, i -> values.getOrDefault(i, Double.NaN));
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

    private Controller getController() {
        ControllerRegistry registry = new ControllerRegistry();

        //Conditions
        Function<DataState, Boolean> longSafe = ds ->
                ds.get(distance[1]) >= ds.get(safety_gap[1]) && ds.get(distance[0]) >= ds.get(safety_gap[0]);

        Function<DataState, Boolean> latSafe = ds ->
                ds.get(lat_safe) == 1;

        Function<DataState, Boolean> mergeSafe = ds ->
                longSafe.apply(ds) && latSafe.apply(ds);

        Function<DataState, Boolean> frontGapUnsafe = ds ->
                ds.get(distance[1]) < ds.get(safety_gap[1]);

        Function<DataState, Boolean> rearGapUnsafe = ds ->
                ds.get(distance[0]) < ds.get(safety_gap[0]);

        Function<DataState, Boolean> frontGapSafe = ds ->
                ds.get(distance[1]) >= ds.get(safety_gap[1]);

        Function<DataState, Boolean> rearGapSafe = ds ->
                ds.get(distance[0]) >= ds.get(safety_gap[0]);

        Function<DataState, Boolean> criticalDistanceReached = ds ->
                ds.get(dist_end) <= ds.get(crit_dist);

        // CASE 3: merge behind it
        Function<DataState, Boolean> rearCarPassed = ds -> {
            // rear car's x has passed ego's x means rear car is now ahead
            double rearX = ds.get(x[0]);
            double egoX = ds.get(x[CONTROLLED_VEHICLE]);

            if (rearX <= egoX) return false; // rear hasn't passed yet

            // rear is now in front; check if gap behind it is safe
            double gapBehindRear = rearX - egoX - VEHICLE_LENGTH;
            double requiredGap = calculateRSSSafetyDistance(
                    RESPONSE_TIME,
                    ds.get(speed[CONTROLLED_VEHICLE]),
                    ds.get(speed[0])
            );
            return gapBehindRear >= requiredGap;
        };

        // If collision is unavoidable:
        Function<DataState, Boolean> frontViolationWorse = ds -> {
            double frontViolation =
                    (ds.get(safety_gap[1]) - ds.get(distance[1])) / ds.get(safety_gap[1]);

            double rearViolation =
                    (ds.get(safety_gap[0]) - ds.get(distance[0])) / ds.get(safety_gap[0]);

            return frontViolation >= rearViolation;
        };

        //Intentions
        Controller goFasterToRamp = Controller.doAction(
                (_rg, _ds) -> List.of(new DataStateUpdate(intention, FASTER)),
                registry.reference("RampControl"));
        Controller goSlowerToRamp = Controller.doAction(
                (_rg, _ds) -> List.of(new DataStateUpdate(intention, SLOWER)),
                registry.reference("RampControl")
        );
        Controller goFasterToHighway = Controller.doAction(
                (_rg, _ds) -> List.of(new DataStateUpdate(intention, FASTER)),
                registry.reference("HighwayControl"));
        Controller goSlowerToHighway = Controller.doAction(
                (_rg, _ds) -> List.of(new DataStateUpdate(intention, SLOWER)),
                registry.reference("HighwayControl")
        );
        Controller goIdleToHighway = Controller.doAction(
                (_rg, _ds) -> List.of(new DataStateUpdate(intention, IDLE)),
                registry.reference("HighwayControl")
        );
        Controller goSlowerToCritical = Controller.doAction(
                (_rg, _ds) -> List.of(new DataStateUpdate(intention, SLOWER)),
                registry.reference("CriticalMergeControl")
        );
        Controller doMerge = Controller.doAction(
                (_rg, _ds) -> List.of(
                        new DataStateUpdate(intention, MERGE),
                        new DataStateUpdate(merge_timer, MERGE_TIMER_INIT)
                ),
                registry.reference("MergingControl")
        );

        //----------------------------------------ALGORITHMS-----------------------------------//
        // detailed explanations and high level descriptions of the algorithms are on the report

        // Algorithm 1: Control
        registry.set("Control",
                Controller.ifThenElse(
                        (rg, ds) -> ds.get(merged) == 0.0,
                        registry.reference("RampControl"),
                        registry.reference("HighwayControl")
                )
        );

        // Algorithm 2: RampControl
        registry.set("RampControl",
                Controller.ifThenElse(
                        (rg, ds) -> mergeSafe.apply(ds),
                        doMerge,

                        Controller.ifThenElse(
                                (rg, ds) -> criticalDistanceReached.apply(ds),
                                registry.reference("CriticalMergeControl"),
                                registry.reference("GapAdjustControl")
                        )
                )
        );

        // Algorithm 3: CriticalMergeControl
        registry.set("CriticalMergeControl",
                Controller.ifThenElse(
                        (rg, ds) -> rearCarPassed.apply(ds),
                        doMerge,
                        goSlowerToCritical
                )
        );

        // MergingControl:
        registry.set("MergingControl",
                Controller.ifThenElse(
                        (rg, ds) -> ds.get(merge_timer) > 0.0,
                        Controller.doTick(registry.reference("MergingControl")),

                        Controller.doAction(
                                (_rg, _ds) -> List.of(
                                        new DataStateUpdate(merged, 1.0),
                                        new DataStateUpdate(ego_lane, 1.0),
                                        new DataStateUpdate(y[CONTROLLED_VEHICLE], HIGHWAY_Y)
                                ),
                                registry.reference("HighwayControl")
                        )
                )
        );

        // Algorithm 6: GapAdjustControl
        registry.set("GapAdjustControl",
                Controller.ifThenElse(
                        (rg, ds) -> frontGapUnsafe.apply(ds) && rearGapSafe.apply(ds),
                        goSlowerToRamp,

                        Controller.ifThenElse(
                                (rg, ds) -> rearGapUnsafe.apply(ds) && frontGapSafe.apply(ds),
                                goFasterToRamp,

                                Controller.ifThenElse(
                                        (rg, ds) -> frontViolationWorse.apply(ds),
                                        goSlowerToRamp,
                                        goFasterToRamp
                                )
                        )
                )
        );

        // Algorithm 7: HighwayControl
        registry.set("HighwayControl",
                Controller.ifThenElse(
                        (rg, ds) -> ds.get(distance[1]) < ds.get(safety_gap[1]),
                        goSlowerToHighway,

                        Controller.ifThenElse(
                                (rg, ds) -> ds.get(distance[1]) > ds.get(safety_gap[1]),
                                goFasterToHighway,
                                goIdleToHighway
                        )
                )
        );
        return new ExecController(registry.reference("Control"));
    }
    //----------------------------------------ALGORITHMS-----------------------------------//

    public static List<DataStateUpdate> getEnvironmentUpdates(RandomGenerator rg, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();

        // Update controlled vehicle's acceleration based on the controller's intention
        double intent = state.get(intention);
        double newAccel;

        if (intent == FASTER) { // Controller wants to do action FASTER
            double offset = rg.nextDouble() * MAX_ACCEL_OFFSET;
            newAccel = MAX_ACCELERATION - offset;

        } else if (intent == SLOWER) { // Controller wants to do action SLOWER
            newAccel = -1 * (rg.nextDouble() * (MAX_BRAKE - MIN_BRAKE) + MIN_BRAKE);

        } else if (intent == IDLE) { // Controller wants to do action IDLE
            newAccel = rg.nextDouble() * (2 * IDLE_DELTA) - IDLE_DELTA; // random jitter around zero acceleration

        } else if (intent == MERGE) { // Controller wants to do action MERGE
            newAccel = 0.0;
        } else {
            System.out.println("Unknown controller intention: " + intent);
            newAccel = 0.0;
        }

        updates.add(new DataStateUpdate(accel[CONTROLLED_VEHICLE], newAccel));

        // random jitter around their initial acceleration
        for (int i = 0; i < NUMBER_OF_VEHICLES; i++) {
            if (i != CONTROLLED_VEHICLE) {
                updates.add(new DataStateUpdate(accel[i], uncontrolledAccel(rg, state, i)));
            }
        }

        includePhysicsUpdates(state, updates);

        // Update lateral merge movement
        if (state.get(merge_timer) > 0.0) {
            double newEgoY = Math.min(
                    HIGHWAY_Y,
                    state.get(y[CONTROLLED_VEHICLE]) + LANE_WIDTH / MERGE_TIMER_INIT
            );

            double newMergeTimer = Math.max(
                    0.0,
                    state.get(merge_timer) - 1.0
            );

            updates.add(new DataStateUpdate(y[CONTROLLED_VEHICLE], newEgoY));
            updates.add(new DataStateUpdate(merge_timer, newMergeTimer));

            if (newMergeTimer == 0.0) {
                updates.add(new DataStateUpdate(merged, 1.0));
                updates.add(new DataStateUpdate(ego_lane, 1.0));
                updates.add(new DataStateUpdate(y[CONTROLLED_VEHICLE], HIGHWAY_Y));
            }
        }

        //lat safe 2d adittions
        if (state.get(merged) == 0.0) {
            double ex = state.get(x[CONTROLLED_VEHICLE]);
            // Check the lane the ego wants to merge INTO (the highway) to see any highway car longitudinally overlapping the slot the ego would move into?
            double targetY = HIGHWAY_Y;
            boolean anyBeside = false;
            for (int i = 0; i < NUMBER_OF_VEHICLES; i++) {
                if (i == CONTROLLED_VEHICLE) continue;
                boolean beside = Math.abs(state.get(x[i]) - ex) < VEHICLE_LENGTH
                        && Math.abs(state.get(y[i]) - targetY) < VEHICLE_WIDTH;
                if (beside) { anyBeside = true; break; }
            }
            updates.add(new DataStateUpdate(lat_safe, anyBeside ? 0.0 : 1.0));
        } else {
            updates.add(new DataStateUpdate(lat_safe, 1.0));
        }


        if (state.get(merged) == 0.0) {
            double newDistEnd = Math.max(
                    0.0,
                    state.get(dist_end) - state.get(speed[CONTROLLED_VEHICLE])
            );
            updates.add(new DataStateUpdate(dist_end, newDistEnd));
        } else {
            updates.add(new DataStateUpdate(dist_end, state.get(dist_end)));
        }

        return updates;
    }

    // a small random acceleration that keeps it near its target
    // so it neither runs away nor stops
    private static double uncontrolledAccel(RandomGenerator rg, DataState state, int i) {
        double speedError = INIT_SPEED[i] - state.get(speed[i]);
        double jitter = rg.nextDouble() * (2 * UNCONTROLLED_ACCEL_DELTA) - UNCONTROLLED_ACCEL_DELTA;
        double a = speedError + jitter;
        return Math.max(-UNCONTROLLED_ACCEL_DELTA, Math.min(UNCONTROLLED_ACCEL_DELTA, a));
    }

    private static void includePhysicsUpdates(DataState state, List<DataStateUpdate> updates) {
        // gaps distance[] are derived from the Positions x[]
        DataState eff = state.apply(updates);

        // Update every vehicle's speed from its (new) acceleration.
        double[] newSpeed = new double[NUMBER_OF_VEHICLES];
        for (int i = 0; i < NUMBER_OF_VEHICLES; i++) {
            double v = eff.get(speed[i]);
            double a = eff.get(accel[i]);
            newSpeed[i] = Math.min(Math.max(0, v + a), MAX_SPEED);
            updates.add(new DataStateUpdate(speed[i], newSpeed[i]));
        }

        // advance each vehicle's x by the distance travelled this step.
        // travel = a/2 + v  (this step's accel and the old speed)
        double[] newX = new double[NUMBER_OF_VEHICLES];
        for (int i = 0; i < NUMBER_OF_VEHICLES; i++) {
            double travel = eff.get(accel[i]) / 2 + eff.get(speed[i]);
            newX[i] = eff.get(x[i]) + travel;
            updates.add(new DataStateUpdate(x[i], newX[i]));
            updates.add(new DataStateUpdate(y[i], eff.get(y[i]))); // lateral: ego y handled by merge logic
        }

        // derive longitudinal gaps and RSS safety gaps from the NEW positions/speeds.
        for (int i = 0; i < NUMBER_OF_VEHICLES - 1; i++) {
            double newDistance = newX[i + 1] - newX[i] - VEHICLE_LENGTH;
            updates.add(new DataStateUpdate(distance[i], newDistance));

            double newSafetyGap = calculateRSSSafetyDistance(RESPONSE_TIME, newSpeed[i], newSpeed[i + 1]);
            updates.add(new DataStateUpdate(safety_gap[i], newSafetyGap));
        }
    }

    // Direction of a per-step perturbation, decided once and shared by every sample in that step
    private static final double PERTURB_NONE = 0.0;
    private static final double PERTURB_ACCEL = 1.0;
    private static final double PERTURB_DECEL = -1.0;

    // 50% no perturbation, 25% accel, 25% deccel
    private static double decidePerturbationDirection(RandomGenerator rg, double chance) {
        if (rg.nextDouble() < chance) {
            return rg.nextDouble() < 0.5 ? PERTURB_ACCEL : PERTURB_DECEL;
        }
        return PERTURB_NONE;
    }

    // The directions are fixed, but each sample rolls its own magnitude
    private static DataStateFunction buildStepPerturbation(double rearDir, double frontDir) {
        return (rg, state) -> {
            List<DataStateUpdate> updates = new LinkedList<>();
            if (rearDir == PERTURB_ACCEL) {
                updates.add(new DataStateUpdate(accel[0], rg.nextDouble() * MAX_ACCELERATION));
            } else if (rearDir == PERTURB_DECEL) {
                updates.add(new DataStateUpdate(accel[0], -rg.nextDouble() * MAX_BRAKE));
            }
            if (frontDir == PERTURB_ACCEL) {
                updates.add(new DataStateUpdate(accel[2], rg.nextDouble() * MAX_ACCELERATION));
            } else if (frontDir == PERTURB_DECEL) {
                updates.add(new DataStateUpdate(accel[2], -rg.nextDouble() * MAX_BRAKE));
            }
            includePhysicsUpdates(state, updates);
            return state.apply(updates);
        };
    }

    // Getter method
    // the accel/decel direction is the same for the whole sample ensemble, while magnitudes still vary per sample.
    private static Perturbation getSuddenSpeedChangePerturbation(int timesToApply) {
        RandomGenerator decisionRg = new DefaultRandomGenerator();
        Perturbation chain = Perturbation.NONE;
        for (int k = timesToApply - 1; k >= 0; k--) {
            double rearDir = decidePerturbationDirection(decisionRg, REAR_PERTURB_CHANCE);
            double frontDir = decidePerturbationDirection(decisionRg, FRONT_PERTURB_CHANCE);
            AtomicPerturbation step = new AtomicPerturbation(0, buildStepPerturbation(rearDir, frontDir));
            chain = new SequentialPerturbation(step, chain);
        }
        return chain;
    }

    private static DataStateFunction buildEgoDelayStep(int delaySteps) {
        return (rg, state) -> {
            List<DataStateUpdate> updates = new LinkedList<>();
            // Freshly computed control command
            double cmd = state.get(accel[CONTROLLED_VEHICLE]);
            // Oldest buffered command
            double applied = state.get(EGO_ACCEL_BUF_BASE);
            // Shift the register left (drop the oldest) and append this step's command at the end.
            for (int i = 0; i < delaySteps - 1; i++) {
                updates.add(new DataStateUpdate(EGO_ACCEL_BUF_BASE + i,
                        state.get(EGO_ACCEL_BUF_BASE + i + 1)));
            }
            updates.add(new DataStateUpdate(EGO_ACCEL_BUF_BASE + delaySteps - 1, cmd));
            // Ego actuates the delayed command instead of the fresh one, then physics is re-run.
            updates.add(new DataStateUpdate(accel[CONTROLLED_VEHICLE], applied));
            includePhysicsUpdates(state, updates);
            return state.apply(updates);
        };
    }

    // Applies one delay step at every step of the horizon
    private static Perturbation getEgoDelayPerturbation(int timesToApply, int delaySteps) {
        DataStateFunction step = buildEgoDelayStep(delaySteps);
        Perturbation chain = Perturbation.NONE;
        for (int k = timesToApply - 1; k >= 0; k--) {
            chain = new SequentialPerturbation(new AtomicPerturbation(0, step), chain);
        }
        return chain;
    }

    private void printSummary(EvolutionSequence sequence, int stepsToPrint, String title) {
        String[] headers = new String[]{
                "step",
                "a0", "a1", "a2",
                "v0", "v1", "v2",
                "gap_r", "gap_f",
                "safe_r", "safe_f",
                "merged",
                "latSafe",
                "distEnd",
                "egoY",
                "lane",
                "mTimer",
                "intent"
        };

        System.out.println(title);
        for (String header : headers) {
            System.out.printf("%7s, ", header);
        }
        System.out.println();

        for (int i = 0; i < stepsToPrint; i++) {
            SampleSet<SystemState> dss = sequence.get(i);

            ArrayList<String> s = new ArrayList<>();
            s.add(String.format("%7d,", i));

            int[] variablesToPrint = new int[]{
                    accel[0], accel[1], accel[2],
                    speed[0], speed[1], speed[2],
                    distance[0], distance[1],
                    safety_gap[0], safety_gap[1],
                    merged,
                    lat_safe,
                    dist_end,
                    y[CONTROLLED_VEHICLE],
                    ego_lane,
                    merge_timer
            };

            for (int variable : variablesToPrint) {
                OptionalDouble avg = Arrays.stream(dss.evalPenaltyFunction(ds -> ds.get(variable))).average();
                s.add(String.format("%7.2f,", avg.orElse(Double.NaN)));
            }

            double[] intentions = dss.evalPenaltyFunction(ds -> ds.get(intention));
            Double intentionMode = Arrays.stream(intentions)
                    .boxed()
                    .collect(Collectors.groupingBy(t -> t, Collectors.counting()))
                    .entrySet()
                    .stream()
                    .reduce(BinaryOperator.maxBy(Comparator.comparingLong(Map.Entry::getValue)))
                    .map(Map.Entry::getKey)
                    .orElse(Double.NaN);
            s.add(String.format("%7.2f", intentionMode));
            System.out.println(String.join(" ", s));
        }
    }
}
