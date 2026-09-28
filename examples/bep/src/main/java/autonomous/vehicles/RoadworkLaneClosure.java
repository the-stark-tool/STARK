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

package Scenarios;

import stark.*;
import stark.controller.ControllerRegistry;
import stark.controller.ExecController;
import stark.controller.Controller;
import stark.distance.AtomicDistanceExpressionLeq;
import stark.distance.DistanceExpression;
import stark.distance.MaxIntervalDistanceExpression;
import stark.distance.UntilDistanceExpression;
import stark.ds.DataState;
import stark.ds.DataStateExpression;
import stark.ds.DataStateUpdate;
import stark.ds.RelationOperator;
import stark.perturbation.AtomicPerturbation;
import stark.perturbation.IterativePerturbation;
import stark.perturbation.Perturbation;
import stark.robtl.AlwaysRobustnessFormula;
import stark.robtl.AtomicRobustnessFormula;
import stark.robtl.BooleanSemanticsVisitor;
import stark.robtl.RobustnessFormula;
import java.io.IOException;
import stark.Util;

import org.apache.commons.math3.random.RandomGenerator;

import java.util.*;

/**
 *  Scenario where a controlled vehicle (A) is on the right lane and has to merge into the left lane 
 *  because of a roadwork lane closure in front.
 *  There is one uncontrolled vehicle in front in the left lane (F), and one uncontrolled vehicle 
 *  behind in the left lane (R).
 *
 *  System State:
 *    - Ego vehicle A: position x_A, velocity v_A, lane l_A (0=right, 1=left)
 *    - Front vehicle F: position x_F, velocity v_F, lane l_F (always 1=left)
 *    - Rear vehicle R: position x_R, velocity v_R, lane l_R (always 1=left)
 *    - Roadwork position: x_rw
 *
 *  Controller Modes:
 *    - Cruise: maintain lane and speed
 *    - PrepareMerge: slow down to prepare for merge
 *    - Merge: execute lane change from right to left
 *    - Brake: emergency braking
 */
public class RoadworkLaneClosure {

    private static final double RESPONSE_TIME = 1.0; //theta

    // VEHICLE DIMENSIONS
    private static final double VEHICLE_LENGTH = 5.0; //len_A

    // CONTROLLER PARAMETERS
    private static final double V_DES    = 20.0;

    // VEHICLE DYNAMICS
    private static final double MAX_SPEED        = 40.0; //v_max
    private static final double MAX_ACCELERATION =  2.0; //a_max
    private static final double MAX_BRAKE        =  4.0; //b_max

    // RSS GAP PARAMETERS
    private static final double MIN_BRAKE = 3.0;   // b_min
    private static final double GAP_SAFETY_MARGIN = 1.3; //theta_gap

    // ROADWORK
    private static final double INIT_ROADWORK_POS = 150.0; //x_rw^start
    private static final double INIT_ROADWORK_END = 400.0; //x_rw^end

    // INITIAL VALUES
    private static final double INIT_POS_A = 0.0;
    private static final double INIT_SPEED_A = 12.0;

    private static final double INIT_POS_F = 12.0; //positive: ahead
    private static final double INIT_SPEED_F = 17.0;

    private static final double INIT_POS_R = -15.0; //negative: behind
    private static final double INIT_SPEED_R = 10.0;

    // PERTURBATION PARAMETERS
    private static final int    STARTING_STEP = 6; //tau_0: time at which perturbation begins to act
    private static final int    FREQUENCY = 1; //steps the perturbation waits before applying f
    private static final int    TIMES_TO_APPLY = 4; //n: total number of times the perturbation is applied
    private static final double POS_NOISE = 2.0;   //sigma_x: magnitude of noise
    private static final double VEL_NOISE = 1.0;   //sigma_v: magnitude of noise
    private static final double SN = 0.5; //SN: probability of sensor noise occurring

    // ROBUSTNESS FORMULA PARAMETERS
    private static final double ETA_COL = 0.2;
    private static final double ETA_GAP = 0.2;
    private static final double ETA_RW = 0.0;
    private static final int H = 200;

    // VARIABLE INDEXES
    private static final int x_A = 0;
    private static final int v_A = 1;
    private static final int l_A = 2;
    private static final int mode_A = 3;
    private static final int x_F = 4;
    private static final int v_F = 5;
    private static final int l_F = 6;
    private static final int x_R = 7;
    private static final int v_R = 8;
    private static final int l_R = 9;
    private static final int x_rw = 10;
    private static final int action_A = 11;
    private static final int x_F_prev = 12;
    private static final int v_F_prev = 13;
    private static final int x_R_prev = 14;
    private static final int v_R_prev = 15;
    private static final int x_F_true = 16;
    private static final int v_F_true = 17;
    private static final int x_R_true = 18;
    private static final int v_R_true = 19;
    private static final int NUMBER_OF_VARIABLES = 20;

    // CONTROLLER MODES
    private static final double MODE_CRUISE = 0.0;
    private static final double MODE_PREPARE_MERGE = 1.0;
    private static final double MODE_MERGE = 2.0;
    private static final double MODE_BRAKE = 3.0;

    // CONTROLLER ACTIONS
    private static final double ACTION_ACCELERATE = 0.0;
    private static final double ACTION_KEEP = 1.0;
    private static final double ACTION_BRAKE = 2.0;
    private static final double ACTION_MERGE = 3.0;
    private static final double ACTION_SOFT_BRAKE = 4.0;

    // LOOK-AHEAD BRAKING PARAMETERS
    private static final double SOFT_BRAKE_DECEL = 1.5; //b_soft
    private static final double ROADWORK_BUFFER = VEHICLE_LENGTH; //stop at least one vehicle length before the roadwork
    private static final double DECEL_PREPARE_THRESHOLD = 0.3; //a_prep
    private static final double DECEL_KEEP_THRESHOLD = 1.0; //a_keep
    private static final double DECEL_HARD_THRESHOLD = 2.0; //a_hard
    private static final double STOPPED_SPEED = 0.3; //v_stop

    //SIMULATION PARAMETERS
    private static final int EVOLUTION_SEQUENCE_SIZE = 100;
    private static final int PERTURBATION_SCALE = 100;


    public RoadworkLaneClosure() throws IOException{

        try {
            int EXTRA_SIZE = 1000; // Number of sampled runs over which the exported trajectory CSVs are averaged.

            RandomGenerator rand = new DefaultRandomGenerator();
            DataState state = getInitialState();
            ControlledSystem system = new ControlledSystem(getController(), (rg, ds) -> ds.apply(getEnvironmentUpdates(rg, ds)), state); //controller + environment dynamics + initial state
            EvolutionSequence sequence = new EvolutionSequence(rand, rg -> system, EVOLUTION_SEQUENCE_SIZE); //nominal evolution sequence

            //Labels of logged values
            ArrayList<String> L = new ArrayList<>();
            L.add("x_A"); L.add("x_F"); L.add("x_R");
            L.add("v_A"); L.add("l_A"); L.add("mode_A"); L.add("v_F"); L.add("v_R");
            L.add("rho_col"); L.add("rho_rw");

            ArrayList<DataStateExpression> F = new ArrayList<>();
            F.add(ds -> ds.get(x_A)); F.add(ds -> ds.get(x_F)); F.add(ds -> ds.get(x_R));
            F.add(ds -> ds.get(v_A)); F.add(ds -> ds.get(l_A)); F.add(ds -> ds.get(mode_A)); F.add(ds -> ds.get(v_F)); F.add(ds -> ds.get(v_R));
            F.add(RoadworkLaneClosure::rho_col); F.add(RoadworkLaneClosure::rho_rw);

            //PERTURBATIONS
            Perturbation frontBraking     = getFrontBrakingPerturbation(TIMES_TO_APPLY, FREQUENCY);
            Perturbation rearAcceleration  = getRearAccelerationPerturbation(TIMES_TO_APPLY, FREQUENCY);
            Perturbation sensorNoise       = getSensorNoisePerturbation(TIMES_TO_APPLY, FREQUENCY);
            Perturbation delayedPerception = getDelayedPerceptionPerturbation(TIMES_TO_APPLY, FREQUENCY);

            //One sample run per scenario
            System.out.println("Simulation of nominal behaviour:");
            printLData(rand, L, F, system, H, 1);

            System.out.println("Simulation under front braking:");
            printLData(rand, L, F, frontBraking, system, H, 1);

            System.out.println("Simulation under rear acceleration:");
            printLData(rand, L, F, rearAcceleration, system, H, 1);

            System.out.println("Simulation under sensor noise:");
            printLData(rand, L, F, sensorNoise, system, H, 1);

            System.out.println("Simulation under delayed perception:");
            printLData(rand, L, F, delayedPerception, system, H, 1);

            // trajectories: nominal
            double[][] nomData = SystemState.sample(rand, F, system, H, EXTRA_SIZE);
            double[][] nomPos = new double[H][3]; double[][] nomState = new double[H][5]; double[][] nomRho = new double[H][2];
            for (int i = 0; i < H; i++) {
                nomPos[i][0] = nomData[i][0]; nomPos[i][1] = nomData[i][1]; nomPos[i][2] = nomData[i][2];
                nomState[i][0] = nomData[i][3]; nomState[i][1] = nomData[i][4]; nomState[i][2] = nomData[i][5];; nomState[i][3] = nomData[i][6]; nomState[i][4] = nomData[i][7];
                nomRho[i][0] = nomData[i][8]; nomRho[i][1] = nomData[i][9];
            }
            Util.writeToCSV("./rw_nominal_positions.csv", nomPos);
            Util.writeToCSV("./rw_nominal_state.csv",     nomState);
            Util.writeToCSV("./rw_nominal_rho.csv",       nomRho);

            // trajectories: front braking
            double[][] fbData = SystemState.sample(rand, F, frontBraking, system, H, EXTRA_SIZE);
            double[][] fbPos = new double[H][3]; double[][] fbState = new double[H][5]; double[][] fbRho = new double[H][2];
            for (int i = 0; i < H; i++) {
                fbPos[i][0] = fbData[i][0]; fbPos[i][1] = fbData[i][1]; fbPos[i][2] = fbData[i][2];
                fbState[i][0] = fbData[i][3]; fbState[i][1] = fbData[i][4]; fbState[i][2] = fbData[i][5];; fbState[i][3] = fbData[i][6]; fbState[i][4] = fbData[i][7];
                fbRho[i][0] = fbData[i][8]; fbRho[i][1] = fbData[i][9];
            }
            Util.writeToCSV("./rw_fb_positions.csv", fbPos);
            Util.writeToCSV("./rw_fb_state.csv",     fbState);
            Util.writeToCSV("./rw_fb_rho.csv",       fbRho);

            // trajectories: rear acceleration
            double[][] raData = SystemState.sample(rand, F, rearAcceleration, system, H, EXTRA_SIZE);
            double[][] raPos = new double[H][3]; double[][] raState = new double[H][5]; double[][] raRho = new double[H][2];
            for (int i = 0; i < H; i++) {
                raPos[i][0] = raData[i][0]; raPos[i][1] = raData[i][1]; raPos[i][2] = raData[i][2];
                raState[i][0] = raData[i][3]; raState[i][1] = raData[i][4]; raState[i][2] = raData[i][5]; raState[i][3] = raData[i][6]; raState[i][4] = raData[i][7];
                raRho[i][0] = raData[i][8]; raRho[i][1] = raData[i][9];
            }
            Util.writeToCSV("./rw_ra_positions.csv", raPos);
            Util.writeToCSV("./rw_ra_state.csv",     raState);
            Util.writeToCSV("./rw_ra_rho.csv",       raRho);

            // trajectories: sensor noise
            double[][] snData = SystemState.sample(rand, F, sensorNoise, system, H, EXTRA_SIZE);
            double[][] snPos = new double[H][3]; double[][] snState = new double[H][5]; double[][] snRho = new double[H][2];
            for (int i = 0; i < H; i++) {
                snPos[i][0] = snData[i][0]; snPos[i][1] = snData[i][1]; snPos[i][2] = snData[i][2];
                snState[i][0] = snData[i][3]; snState[i][1] = snData[i][4]; snState[i][2] = snData[i][5]; snState[i][3] = snData[i][6]; snState[i][4] = snData[i][7];
                snRho[i][0] = snData[i][8]; snRho[i][1] = snData[i][9];
            }
            Util.writeToCSV("./rw_sn_positions.csv", snPos);
            Util.writeToCSV("./rw_sn_state.csv",     snState);
            Util.writeToCSV("./rw_sn_rho.csv",       snRho);

            // trajectories: delayed perception
            double[][] dpData = SystemState.sample(rand, F, delayedPerception, system, H, EXTRA_SIZE);
            double[][] dpPos = new double[H][3]; double[][] dpState = new double[H][5]; double[][] dpRho = new double[H][2];
            for (int i = 0; i < H; i++) {
                dpPos[i][0] = dpData[i][0]; dpPos[i][1] = dpData[i][1]; dpPos[i][2] = dpData[i][2];
                dpState[i][0] = dpData[i][3]; dpState[i][1] = dpData[i][4]; dpState[i][2] = dpData[i][5]; dpState[i][3] = dpData[i][6]; dpState[i][4] = dpData[i][7];
                dpRho[i][0] = dpData[i][8]; dpRho[i][1] = dpData[i][9];
            }
            Util.writeToCSV("./rw_dp_positions.csv", dpPos);
            Util.writeToCSV("./rw_dp_state.csv",     dpState);
            Util.writeToCSV("./rw_dp_rho.csv",       dpRho);

            //Single representative run used for the phi_gap illustration to retain the collision-risk spikes
            Util.writeToCSV("./rw_nominal_single.csv", SystemState.sample(rand, F, system, H, 1));
            Util.writeToCSV("./rw_fb_single.csv",      SystemState.sample(rand, F, frontBraking,      system, H, 1));
            Util.writeToCSV("./rw_ra_single.csv",      SystemState.sample(rand, F, rearAcceleration,  system, H, 1));
            Util.writeToCSV("./rw_sn_single.csv",      SystemState.sample(rand, F, sensorNoise,       system, H, 1));
            Util.writeToCSV("./rw_dp_single.csv",      SystemState.sample(rand, F, delayedPerception, system, H, 1));

            //ROBUSTNESS VERIFICATION
            EvolutionSequence frontBrakingSequence     = sequence.apply(frontBraking,     STARTING_STEP, PERTURBATION_SCALE);
            EvolutionSequence rearAccelerationSequence  = sequence.apply(rearAcceleration,  STARTING_STEP, PERTURBATION_SCALE);
            EvolutionSequence sensorNoiseSequence       = sequence.apply(sensorNoise,       STARTING_STEP, PERTURBATION_SCALE);
            EvolutionSequence delayedPerceptionSequence = sequence.apply(delayedPerception, STARTING_STEP, PERTURBATION_SCALE);

            RobustnessFormula phiColFrontBraking    = getPhiCol(frontBraking);
            RobustnessFormula phiColRearAcceleration = getPhiCol(rearAcceleration);
            RobustnessFormula phiColSensorNoise      = getPhiCol(sensorNoise);
            RobustnessFormula phiColDelayed          = getPhiCol(delayedPerception);

            RobustnessFormula phiGapFrontBraking    = getPhiGap(frontBraking);
            RobustnessFormula phiGapRearAcceleration = getPhiGap(rearAcceleration);
            RobustnessFormula phiGapSensorNoise      = getPhiGap(sensorNoise);
            RobustnessFormula phiGapDelayed          = getPhiGap(delayedPerception);

            RobustnessFormula phiRwFrontBraking    = getPhiRw(frontBraking);
            RobustnessFormula phiRwRearAcceleration = getPhiRw(rearAcceleration);
            RobustnessFormula phiRwSensorNoise      = getPhiRw(sensorNoise);
            RobustnessFormula phiRwDelayed          = getPhiRw(delayedPerception);

            //Boolean verdicts (satisfied/violated)
            System.out.println("=== phi_col (collision avoidance) ===");
            System.out.println("Front braking:      " + new BooleanSemanticsVisitor().eval(phiColFrontBraking).eval(PERTURBATION_SCALE, 0, sequence));
            System.out.println("Rear acceleration:   " + new BooleanSemanticsVisitor().eval(phiColRearAcceleration).eval(PERTURBATION_SCALE, 0, sequence));
            System.out.println("Sensor noise:        " + new BooleanSemanticsVisitor().eval(phiColSensorNoise).eval(PERTURBATION_SCALE, 0, sequence));
            System.out.println("Delayed perception:  " + new BooleanSemanticsVisitor().eval(phiColDelayed).eval(PERTURBATION_SCALE, 0, sequence));

            System.out.println("=== phi_gap (safe merge decision) ===");
            System.out.println("Front braking:      " + new BooleanSemanticsVisitor().eval(phiGapFrontBraking).eval(PERTURBATION_SCALE, 0, sequence));
            System.out.println("Rear acceleration:   " + new BooleanSemanticsVisitor().eval(phiGapRearAcceleration).eval(PERTURBATION_SCALE, 0, sequence));
            System.out.println("Sensor noise:        " + new BooleanSemanticsVisitor().eval(phiGapSensorNoise).eval(PERTURBATION_SCALE, 0, sequence));
            System.out.println("Delayed perception:  " + new BooleanSemanticsVisitor().eval(phiGapDelayed).eval(PERTURBATION_SCALE, 0, sequence));

            System.out.println("=== phi_rw (roadwork zone entry prevention) ===");
            System.out.println("Front braking:      " + new BooleanSemanticsVisitor().eval(phiRwFrontBraking).eval(PERTURBATION_SCALE, 0, sequence));
            System.out.println("Rear acceleration:   " + new BooleanSemanticsVisitor().eval(phiRwRearAcceleration).eval(PERTURBATION_SCALE, 0, sequence));
            System.out.println("Sensor noise:        " + new BooleanSemanticsVisitor().eval(phiRwSensorNoise).eval(PERTURBATION_SCALE, 0, sequence));
            System.out.println("Delayed perception:  " + new BooleanSemanticsVisitor().eval(phiRwDelayed).eval(PERTURBATION_SCALE, 0, sequence));

        } catch (RuntimeException e) {
            e.printStackTrace();
        }
    }

    // ROBUSTNESS FORMULAE
    //RSS safe following distance d_RSS(v_r, v_f)
    private static double calculateRSSSafetyDistance(double responseTime, double rearSpeed, double frontSpeed) {
        double d = responseTime * rearSpeed
                + 0.5 * MAX_ACCELERATION * responseTime * responseTime
                + Math.pow(rearSpeed + responseTime * MAX_ACCELERATION, 2) / (2.0 * MIN_BRAKE)
                - (frontSpeed * frontSpeed) / (2.0 * MAX_BRAKE);
        return Math.max(d, 0.0) + VEHICLE_LENGTH;
    }

    //a_needed
    private static double computeNeededDecel(DataState ds) {
        double dist = ds.get(x_rw) - ds.get(x_A) - ROADWORK_BUFFER;
        if (dist <= 0) return Double.MAX_VALUE;
        return (ds.get(v_A) * ds.get(v_A)) / (2.0 * dist);
    }

    //roadworkBraking
    private static double roadworkBrakingAction(DataState ds) {
        if (ds.get(v_A) <= STOPPED_SPEED) return ACTION_KEEP;
        double aNeeded = computeNeededDecel(ds);
        if (aNeeded <= DECEL_KEEP_THRESHOLD) return ACTION_KEEP;
        if (aNeeded <= DECEL_HARD_THRESHOLD) return ACTION_SOFT_BRAKE;
        return ACTION_BRAKE;
    }

    //Collision penalty rho_col
    private static double rho_col(DataState ds) {
        if (ds.get(l_A) == 0.0) return 0.0;
        if (ds.get(x_R_true) > ds.get(x_A)) {
            double dAR = ds.get(x_R_true) - ds.get(x_A);
            double safeAR = calculateRSSSafetyDistance(RESPONSE_TIME, ds.get(v_A), ds.get(v_R_true));
            return Math.min(Math.max(1.0 - dAR / safeAR, 0.0), 1.0);
        }
        double dAF = ds.get(x_F_true) - ds.get(x_A);
        double dRA = ds.get(x_A) - ds.get(x_R_true);
        double safeAF = calculateRSSSafetyDistance(RESPONSE_TIME, ds.get(v_A), ds.get(v_F_true));
        double safeRA = calculateRSSSafetyDistance(RESPONSE_TIME, ds.get(v_R_true), ds.get(v_A));
        double ratioAF = (safeAF > 0.0) ? dAF / safeAF : Double.MAX_VALUE;
        double ratioRA = (safeRA > 0.0) ? dRA / safeRA : Double.MAX_VALUE;
        double r = 1.0 - Math.min(ratioAF, ratioRA);
        return Math.min(Math.max(r, 0.0), 1.0);
    }

    //Safe merge decision penalty rho_merge
    private static double rho_merge(DataState ds) {
        return ds.get(mode_A) == MODE_MERGE ? 1.0 : 0.0;
    }

    //Roadwork zone entry penalty rho_rw
    private static double rho_rw(DataState ds) {
        double xA = ds.get(x_A);
        return (ds.get(l_A) == 0.0 && xA >= ds.get(x_rw) && xA <= INIT_ROADWORK_END) ? 1.0 : 0.0;
    }

    //phi_col
    private static RobustnessFormula getPhiCol(Perturbation perturbation) {
        DistanceExpression M_col = new MaxIntervalDistanceExpression(
                new AtomicDistanceExpressionLeq(RoadworkLaneClosure::rho_col), 0, H);
        return new AlwaysRobustnessFormula(
                new AtomicRobustnessFormula(perturbation, M_col, RelationOperator.LESS_OR_EQUAL_THAN, ETA_COL), 0, H);
    }

    //phi_gap
    private static RobustnessFormula getPhiGap(Perturbation perturbation) {
        DistanceExpression untilExpr = new UntilDistanceExpression(
                new AtomicDistanceExpressionLeq(RoadworkLaneClosure::rho_col), 0, H,
                new AtomicDistanceExpressionLeq(RoadworkLaneClosure::rho_merge));
        return new AlwaysRobustnessFormula(
                new AtomicRobustnessFormula(perturbation, untilExpr, RelationOperator.LESS_OR_EQUAL_THAN, ETA_GAP), 0, H);
    }

    //phi_rw
    private static RobustnessFormula getPhiRw(Perturbation perturbation) {
        DistanceExpression M_rw = new MaxIntervalDistanceExpression(
                new AtomicDistanceExpressionLeq(RoadworkLaneClosure::rho_rw), 0, H);
        return new AlwaysRobustnessFormula(
                new AtomicRobustnessFormula(perturbation, M_rw, RelationOperator.LESS_OR_EQUAL_THAN, ETA_RW), 0, H);
    }

    // INITIAL STATE

    private DataState getInitialState() {
        Map<Integer, Double> values = new HashMap<>();

        values.put(x_A,INIT_POS_A);
        values.put(v_A,INIT_SPEED_A);
        values.put(l_A,0.0);
        values.put(mode_A,MODE_CRUISE);
        values.put(action_A,ACTION_KEEP);

        values.put(x_F,INIT_POS_F);
        values.put(v_F,INIT_SPEED_F);
        values.put(l_F,1.0);

        values.put(x_R,INIT_POS_R);
        values.put(v_R,INIT_SPEED_R);
        values.put(l_R,1.0);

        values.put(x_rw,INIT_ROADWORK_POS);

        values.put(x_F_prev,INIT_POS_F);
        values.put(v_F_prev,INIT_SPEED_F);
        values.put(x_R_prev,INIT_POS_R);
        values.put(v_R_prev,INIT_SPEED_R);

        values.put(x_F_true,INIT_POS_F);
        values.put(v_F_true,INIT_SPEED_F);
        values.put(x_R_true,INIT_POS_R);
        values.put(v_R_true,INIT_SPEED_R);

        return new DataState(NUMBER_OF_VARIABLES, i -> values.getOrDefault(i, Double.NaN));
    }


    // CONTROLLER HELPERS

    //bothGapsSafe
    private static boolean bothGapsSafe(DataState ds) {
        return ds.get(x_F) - ds.get(x_A) >= GAP_SAFETY_MARGIN * calculateRSSSafetyDistance(RESPONSE_TIME, ds.get(v_A), ds.get(v_F))
                && ds.get(x_A) - ds.get(x_R) >= GAP_SAFETY_MARGIN * calculateRSSSafetyDistance(RESPONSE_TIME, ds.get(v_R), ds.get(v_A));
    }

    //canMergeBehindR
    private static boolean canMergeBehindR(DataState ds) {
        return ds.get(x_R) > ds.get(x_A)
                && ds.get(x_R) - ds.get(x_A) >= GAP_SAFETY_MARGIN * calculateRSSSafetyDistance(RESPONSE_TIME, ds.get(v_A), ds.get(v_R));
    }

    // CONTROLLER
    private Controller getController() {
        ControllerRegistry registry = new ControllerRegistry();
        //CRUISE: default
        registry.set("Cruise",
                Controller.ifThenElse(
                        //R is ahead of A in the left lane and closer than the RSS safe gap: brake if approaching R, keep otherwise.
                        (rg, ds) -> ds.get(l_A) == 1.0 && ds.get(x_R) > ds.get(x_A) &&
                                (ds.get(x_R) - ds.get(x_A) < calculateRSSSafetyDistance(RESPONSE_TIME, ds.get(v_A), ds.get(v_R))),
                        Controller.doAction(
                                (_rg, ds) -> {
                                    double action;
                                    if (ds.get(v_A) > ds.get(v_R)) action = ACTION_BRAKE;
                                    else action = ACTION_KEEP;
                                    return List.of(
                                            new DataStateUpdate(action_A, action),
                                            new DataStateUpdate(mode_A, MODE_CRUISE));
                                },
                                registry.reference("Cruise")),
                        Controller.ifThenElse(
                                // if A is in left lane and the gap to F is smaller than the RSS safe gap, slow down/keep.
                                (rg, ds) -> ds.get(l_A) == 1.0 &&
                                        (ds.get(x_F) - ds.get(x_A) < calculateRSSSafetyDistance(RESPONSE_TIME, ds.get(v_A), ds.get(v_F))),
                                Controller.doAction(
                                        (_rg, ds) -> {
                                            // If A is faster than F, brake; if slower or equal, keep.
                                            double action = (ds.get(v_A) > ds.get(v_F)) ? ACTION_BRAKE : ACTION_KEEP;
                                            return List.of(
                                                    new DataStateUpdate(action_A, action),
                                                    new DataStateUpdate(mode_A, MODE_CRUISE));
                                        },
                                        registry.reference("Cruise")),
                                // if A is in left lane and R is too close (within RSS safe gap)
                                Controller.ifThenElse(
                                        (rg, ds) -> ds.get(l_A) == 1.0 &&
                                                (ds.get(x_A) - ds.get(x_R) < calculateRSSSafetyDistance(RESPONSE_TIME, ds.get(v_R), ds.get(v_A))),
                                        Controller.doAction(
                                                (_rg, ds) -> {
                                                    // if v_R > v_A and A is below desired speed, accelerate to increase gap and otherwise keep the speed.
                                                    double action = (ds.get(v_R) > ds.get(v_A) && ds.get(v_A) < V_DES) ? ACTION_ACCELERATE : ACTION_KEEP;
                                                    return List.of(
                                                            new DataStateUpdate(action_A, action),
                                                            new DataStateUpdate(mode_A, MODE_CRUISE));
                                                },
                                                registry.reference("Cruise")),
                                        Controller.ifThenElse(
                                                //Still in the right lane and the roadwork zone starts to demand braking; (a_needed > a_prep) -> apply look-ahead braking and enter PrepareMerge.
                                                (rg, ds) -> ds.get(l_A) == 0.0 && computeNeededDecel(ds) > DECEL_PREPARE_THRESHOLD,
                                                Controller.doAction(
                                                        (_rg, ds) -> List.of(
                                                                new DataStateUpdate(action_A, roadworkBrakingAction(ds)),
                                                                new DataStateUpdate(mode_A, MODE_PREPARE_MERGE)),
                                                        registry.reference("PrepareMerge")),
                                                //Default: hold position behind R if it is ahead; otherwise, acelerate toward v_des/keep speed.
                                                Controller.doAction(
                                                        (_rg, ds) -> {
                                                            double action = (ds.get(l_A) == 1.0 && ds.get(x_R) > ds.get(x_A))
                                                                    ? ACTION_KEEP
                                                                    : (ds.get(v_A) < V_DES ? ACTION_ACCELERATE : ACTION_KEEP);
                                                            return List.of(
                                                                    new DataStateUpdate(action_A, action),
                                                                    new DataStateUpdate(mode_A, MODE_CRUISE));
                                                        },
                                                        registry.reference("Cruise")))))));
        //PREPARE MERGE
        registry.set("PrepareMerge",
                Controller.ifThenElse(
                        //Merge between R and F if both gaps are safe.
                        (rg, ds) -> bothGapsSafe(ds),
                        Controller.doAction(
                                (_rg, _ds) -> List.of(
                                        new DataStateUpdate(action_A, ACTION_MERGE),
                                        new DataStateUpdate(mode_A, MODE_MERGE)),
                                registry.reference("Merge")),
                        Controller.ifThenElse(
                                // If R already overtook A and there is a safe gap, merge behind R.
                                (rg, ds) -> canMergeBehindR(ds),
                                Controller.doAction(
                                        (_rg, _ds) -> List.of(
                                                new DataStateUpdate(action_A, ACTION_MERGE),
                                                new DataStateUpdate(mode_A, MODE_MERGE)),
                                        registry.reference("Merge")),
                                // No safe gap yet.
                                Controller.doAction(
                                        (_rg, ds) -> List.of(
                                                new DataStateUpdate(action_A, roadworkBrakingAction(ds)),
                                                new DataStateUpdate(mode_A, MODE_BRAKE)),
                                        registry.reference("Brake")))));
        //MERGE
        registry.set("Merge",
                Controller.ifThenElse(
                        (rg, ds) -> ds.get(l_A) == 0.0,
                        Controller.doAction(
                                (_rg, _ds) -> List.of(
                                        new DataStateUpdate(action_A, ACTION_MERGE),
                                        new DataStateUpdate(mode_A, MODE_MERGE)),
                                registry.reference("Merge")),
                        Controller.doAction(
                                (_rg, _ds) -> List.of(
                                        new DataStateUpdate(action_A, ACTION_KEEP),
                                        new DataStateUpdate(mode_A, MODE_CRUISE)),
                                registry.reference("Cruise"))));
        //BRAKE
        registry.set("Brake",
                Controller.ifThenElse(
                        // R has overtaken: stop braking, try PrepareMerge
                        (rg, ds) -> ds.get(x_R) > ds.get(x_A),
                        Controller.doAction(
                                (_rg, _ds) -> List.of(
                                        new DataStateUpdate(action_A, ACTION_KEEP),
                                        new DataStateUpdate(mode_A, MODE_PREPARE_MERGE)),
                                registry.reference("PrepareMerge")),
                        Controller.ifThenElse(
                                // Both gaps satisfy the RSS safe gap.
                                (rg, ds) -> bothGapsSafe(ds),
                                Controller.doAction(
                                        (_rg, _ds) -> List.of(
                                                new DataStateUpdate(action_A, ACTION_KEEP),
                                                new DataStateUpdate(mode_A, MODE_PREPARE_MERGE)),
                                        registry.reference("PrepareMerge")),
                                // No safe gap, so apply look-ahead braking and keep waiting.
                                Controller.doAction(
                                        (_rg, ds) -> List.of(
                                                new DataStateUpdate(action_A, roadworkBrakingAction(ds)),
                                                new DataStateUpdate(mode_A, MODE_BRAKE)),
                                        registry.reference("Brake")))));

        return new ExecController(registry.reference("Cruise"));
    }


    // ENVIRONMENT

    public static List<DataStateUpdate> getEnvironmentUpdates(RandomGenerator rg, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();

        // Store true F and R values as previous step; used by delayed perception perturbation
        updates.add(new DataStateUpdate(x_F_prev, state.get(x_F_true)));
        updates.add(new DataStateUpdate(v_F_prev, state.get(v_F_true)));
        updates.add(new DataStateUpdate(x_R_prev, state.get(x_R_true)));
        updates.add(new DataStateUpdate(v_R_prev, state.get(v_R_true)));

        // Ego vehicle A
        double action = state.get(action_A);
        double curr_v_a = state.get(v_A);
        double new_a_a = action == ACTION_ACCELERATE ? MAX_ACCELERATION
                : action == ACTION_BRAKE      ? -MAX_BRAKE
                : action == ACTION_SOFT_BRAKE ? -SOFT_BRAKE_DECEL
                : 0.0;
        updates.add(new DataStateUpdate(v_A, Math.min(Math.max(0, curr_v_a + new_a_a * RESPONSE_TIME), MAX_SPEED)));
        updates.add(new DataStateUpdate(x_A, state.get(x_A) + curr_v_a * RESPONSE_TIME));
        updates.add(new DataStateUpdate(l_A, action == ACTION_MERGE ? 1.0 : state.get(l_A)));

        // Front vehicle F: true state evolves by physics; perceived state re-syncs from true each step
        double acc_f = (rg.nextDouble() * 2 - 1) * 0.5;
        double curr_v_f_true = state.get(v_F_true);
        double new_v_f_true = Math.min(Math.max(0, curr_v_f_true + acc_f * RESPONSE_TIME), MAX_SPEED);
        updates.add(new DataStateUpdate(v_F_true, new_v_f_true));
        updates.add(new DataStateUpdate(x_F_true, state.get(x_F_true) + curr_v_f_true * RESPONSE_TIME));
        updates.add(new DataStateUpdate(v_F, new_v_f_true));
        updates.add(new DataStateUpdate(x_F, state.get(x_F_true) + curr_v_f_true * RESPONSE_TIME));

        // Rear vehicle R: same pattern as F
        double acc_r = (rg.nextDouble() * 2 - 1) * 0.5;
        double curr_v_r_true = state.get(v_R_true);
        double new_v_r_true = Math.min(Math.max(0, curr_v_r_true + acc_r * RESPONSE_TIME), MAX_SPEED);
        updates.add(new DataStateUpdate(v_R_true, new_v_r_true));
        updates.add(new DataStateUpdate(x_R_true, state.get(x_R_true) + curr_v_r_true * RESPONSE_TIME));
        updates.add(new DataStateUpdate(v_R, new_v_r_true));
        updates.add(new DataStateUpdate(x_R, state.get(x_R_true) + curr_v_r_true * RESPONSE_TIME));

        return updates;
    }


    // PERTURBATIONS

    //FRONT BRAKING
    private static Perturbation getFrontBrakingPerturbation(int timesToApply, int frequency) {
        return new IterativePerturbation(timesToApply, new AtomicPerturbation(frequency, RoadworkLaneClosure::applyFrontBraking));
    }

    //f_fb:
    private static DataState applyFrontBraking(RandomGenerator rg, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();
        double curr_v_f = state.get(v_F_true);
        updates.add(new DataStateUpdate(v_F, Math.max(0, curr_v_f - MAX_BRAKE * RESPONSE_TIME)));
        updates.add(new DataStateUpdate(v_F_true, Math.max(0, curr_v_f - MAX_BRAKE * RESPONSE_TIME)));
        return state.apply(updates);
    }

    //REAR ACCELERATION
    private static Perturbation getRearAccelerationPerturbation(int timesToApply, int frequency) {
        return new IterativePerturbation(timesToApply, new AtomicPerturbation(frequency, RoadworkLaneClosure::applyRearAcceleration));
    }

    //f_ra
    private static DataState applyRearAcceleration(RandomGenerator rg, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();
        double curr_v_r = state.get(v_R_true);
        updates.add(new DataStateUpdate(v_R, Math.min(curr_v_r + MAX_ACCELERATION * RESPONSE_TIME, MAX_SPEED)));
        updates.add(new DataStateUpdate(v_R_true, Math.min(curr_v_r + MAX_ACCELERATION * RESPONSE_TIME, MAX_SPEED)));
        return state.apply(updates);
    }

    //SENSOR NOISE
    private static Perturbation getSensorNoisePerturbation(int timesToApply, int frequency) {
        return new IterativePerturbation(timesToApply, new AtomicPerturbation(frequency, RoadworkLaneClosure::applySensorNoise));
    }

    //f_sn
    private static DataState applySensorNoise(RandomGenerator rg, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();
        if (rg.nextDouble() <= SN) {
            updates.add(new DataStateUpdate(x_F, state.get(x_F) + (rg.nextDouble() * 2 - 1) * POS_NOISE));
            updates.add(new DataStateUpdate(v_F, Math.max(0, state.get(v_F) + (rg.nextDouble() * 2 - 1) * VEL_NOISE)));
        }
        if (rg.nextDouble() <= SN) {
            updates.add(new DataStateUpdate(x_R, state.get(x_R) + (rg.nextDouble() * 2 - 1) * POS_NOISE));
            updates.add(new DataStateUpdate(v_R, Math.max(0, state.get(v_R) + (rg.nextDouble() * 2 - 1) * VEL_NOISE)));
        }
        return state.apply(updates);
    }

    //DELAYED PERCEPTION
    private static Perturbation getDelayedPerceptionPerturbation(int timesToApply, int frequency) {
        return new IterativePerturbation(timesToApply, new AtomicPerturbation(frequency, RoadworkLaneClosure::applyDelayedPerception));
    }

    //f_dp
    private static DataState applyDelayedPerception(RandomGenerator rg, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();
        updates.add(new DataStateUpdate(x_F, state.get(x_F_prev)));
        updates.add(new DataStateUpdate(v_F, state.get(v_F_prev)));
        updates.add(new DataStateUpdate(x_R, state.get(x_R_prev)));
        updates.add(new DataStateUpdate(v_R, state.get(v_R_prev)));
        return state.apply(updates);
    }

    // Utility methods

    private static void printLData(RandomGenerator rg, ArrayList<String> label, ArrayList<DataStateExpression> F, SystemState s, int steps, int size) {
        System.out.println(label);
        double[][] data = SystemState.sample(rg, F, s, steps, size);
        for (int i = 0; i < data.length; i++) {
            System.out.printf("%d>   ", i);
            for (int j = 0; j < data[i].length - 1; j++) {
                System.out.printf("%f   ", data[i][j]);
            }
            System.out.printf("%f\n", data[i][data[i].length - 1]);
        }
    }

    private static void printLData(RandomGenerator rg, ArrayList<String> label, ArrayList<DataStateExpression> F, Perturbation p, SystemState s, int steps, int size) {
        System.out.println(label);
        double[][] data = SystemState.sample(rg, F, p, s, steps, size);
        for (int i = 0; i < data.length; i++) {
            System.out.printf("%d>   ", i);
            for (int j = 0; j < data[i].length - 1; j++) {
                System.out.printf("%f   ", data[i][j]);
            }
            System.out.printf("%f\n", data[i][data[i].length - 1]);
        }
    }
}