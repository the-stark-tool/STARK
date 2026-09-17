/*
 * STARK: Software Tool for the Analysis of Robustness in the unKnown environment
 *
 *                Copyright (C) 2023.
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


package tutorial;

import stark.*;
import stark.controller.ControllerRegistry;
import stark.controller.ExecController;
import stark.controller.Controller;
import stark.distance.AtomicDistanceExpression;
import stark.distance.DistanceExpression;
import stark.distance.MaxIntervalDistanceExpression;
import stark.ds.DataState;
import stark.ds.DataStateExpression;
import stark.ds.DataStateUpdate;

import stark.ds.RelationOperator;
import stark.perturbation.AfterPerturbation;
import stark.perturbation.AtomicPerturbation;
import stark.perturbation.IterativePerturbation;
import stark.perturbation.Perturbation;
import stark.robtl.*;
import org.apache.commons.math3.random.RandomGenerator;

import java.io.IOException;
import java.util.*;



public class AutonomousDriving {

    // VEHICLE DIMENSIONS
    private static final double VEHICLE_LENGTH = 5;
    private static final double VEHICLE_WIDTH = 2;
    private static final double TIMER = 2;

    // VARIABLE BOUNDS
    private static final double MAX_SPEED = 40;
    private static final double MAX_ACCELERATION = 5;
    private static final double FAST_OFFSET = 2;
    private static final double MAX_BRAKE = 5;
    private static final double MIN_BRAKE = 3;
    private static final double SLOW_OFFSET = 2;
    private static final double IDLE_OFFSET = 0.4;
    private static final double DIST_OFFSET = VEHICLE_LENGTH*VEHICLE_WIDTH;
    private static final int H = 300;

    // INITIAL VALUES
    private static final double MY_INIT_SPEED = 15;
    private static final double OTHER_INIT_SPEED = 15;
    private static final double MY_INIT_X = 0;
    private static final double MY_INIT_Y = 2;
    private static final double OTHER_INIT_X = 150;
    private static final double OTHER_INIT_Y = 2;


    // VARIABLE INDEXES
    private static final int my_x = 0;
    private static final int my_y = 1;
    private static final int my_speed = 2;
    private static final int intention = 3;
    private static final int my_acc = 4;
    private static final int my_lane = 5;
    private static final int my_move = 6;
    private static final int my_timer = 7;
    private static final int my_position = 8;

    private static final int other_x = 9;
    private static final int other_y = 10;
    private static final int other_speed = 11;
    private static final int other_acc = 12;
    private static final int other_lane = 13;
    private static final int other_move = 14;
    private static final int other_timer = 15;

    private static final int dist = 16;
    private static final int safety_gap = 17;
    private static final int crash = 18;

    private static final int NUMBER_OF_VARIABLES = 19;

    // POSSIBLE CONTROLLER ACTIONS
    private static final double FASTER = 1;
    private static final double SLOWER = -1;
    private static final double IDLE  = 0;
    private static final double LANE_RIGHT = -1;
    private static final double LANE_LEFT = 1;

    public static void main(String[] args) throws IOException{
        try {
            int EVOLUTION_SEQUENCE_SIZE = 10; // was 100;
            int PERTURBATION_SIZE = 10; // was 100;
            int EXTRA_SIZE = 10; //was 100000

            RandomGenerator rand = new DefaultRandomGenerator();
            DataState state = getInitialState();

            ControlledSystem system;
            system = new ControlledSystem(getController(), (rg, ds) -> ds.apply(getEnvironmentUpdates(rg, ds)), state);

            EvolutionSequence sequence = new EvolutionSequence(rand, rg -> system, EVOLUTION_SEQUENCE_SIZE);

            ArrayList<String> L = new ArrayList<>();
            L.add("my_x");
            L.add("my_y");
            L.add("other_x");
            L.add("other_y");
            L.add("dist");
            L.add("RSS_gap");
            L.add("crash");

            ArrayList<DataStateExpression> F = new ArrayList<>();
            F.add(ds -> ds.get(my_x));
            F.add(ds -> ds.get(my_y));
            F.add(ds -> ds.get(other_x));
            F.add(ds -> ds.get(other_y));
            F.add(ds -> ds.get(dist));
            F.add(ds -> ds.get(safety_gap));
            F.add(ds->ds.get(crash));

            System.out.println("Simulation of single nominal behaviour");
            printLData(rand, L, F, system, H, 1);

            System.out.println("Simulation of single perturbed behaviour");
            printLData(rand, L, F, get_reckless_driver(), system, H, 1);

            // trajectories

            double[][] my_trajectory = new double[H][2];
            double[][] other_trajectory = new double[H][2];
            double[][] data = SystemState.sample(rand, F, system, H, 1);
            for (int i = 0; i < H; i++) {
                my_trajectory[i][0] = data[i][0];
                my_trajectory[i][1] = data[i][1];
                other_trajectory[i][0] = data[i][2];
                other_trajectory[i][1] = data[i][3];
            }

            double[][] my_extra_trajectory = new double[H][2];
            double[][] other_extra_trajectory = new double[H][2];
            double[][] extra_data = SystemState.sample(rand, F, system, H, EXTRA_SIZE);
            for (int i = 0; i < H; i++) {
                my_extra_trajectory[i][0] = extra_data[i][0];
                my_extra_trajectory[i][1] = extra_data[i][1];
                other_extra_trajectory[i][0] = extra_data[i][2];
                other_extra_trajectory[i][1] = extra_data[i][3];
            }

            Util.writeToCSV("./my_trajectory_scen3.csv", my_trajectory);
            Util.writeToCSV("./other_trajectory_scen3.csv", other_trajectory);
            Util.writeToCSV("./my_extra_trajectory_scen3.csv", my_extra_trajectory);
            Util.writeToCSV("./other_extra_trajectory_scen3.csv", other_extra_trajectory);

            double[][] my_extra_trajectory_p = new double[H][2];
            double[][] other_extra_trajectory_p = new double[H][2];
            double[][] extra_data_p = SystemState.sample(rand, F, get_reckless_driver(), system, H, EXTRA_SIZE);
            for (int i = 0; i < H; i++) {
                my_extra_trajectory_p[i][0] = extra_data_p[i][0];
                my_extra_trajectory_p[i][1] = extra_data_p[i][1];
                other_extra_trajectory_p[i][0] = extra_data_p[i][2];
                other_extra_trajectory_p[i][1] = extra_data_p[i][3];
            }


            Util.writeToCSV("./my_extra_trajectory_p_scen3.csv", my_extra_trajectory_p);
            Util.writeToCSV("./other_extra_trajectory_p_scen3.csv", other_extra_trajectory_p);



            // APPLICATION OF PERTURBATION reckless_driver

            EvolutionSequence perturbedSequence = sequence.apply(get_reckless_driver(),0,PERTURBATION_SIZE);

            DistanceExpression crash_speed = new AtomicDistanceExpression(AutonomousDriving::rho_si,(v1, v2)->Math.abs(v1-v2));

            double[][] direct_evaluation_crash_speed = new double[H][1];

            for (int i = 0; i<H; i++){
                direct_evaluation_crash_speed[i][0] = crash_speed.compute(i, sequence, perturbedSequence);
            }


            Util.writeToCSV("./atomic_crash_speed_scen3.csv",direct_evaluation_crash_speed);


            double[][] step1_crash_speed = new double[H][1];
            double[][] step2_crash_speed = new double[H][1];
            double[][] step3_crash_speed = new double[H][1];
            double[][] step4_crash_speed = new double[H][1];
            double[][] step5_crash_speed = new double[H][1];

            EvolutionSequence step1_pert = sequence.apply(get_reckless_driver(),1,PERTURBATION_SIZE);
            EvolutionSequence step2_pert = sequence.apply(get_reckless_driver(),2,PERTURBATION_SIZE);
            EvolutionSequence step3_pert = sequence.apply(get_reckless_driver(),3,PERTURBATION_SIZE);
            EvolutionSequence step4_pert = sequence.apply(get_reckless_driver(),4,PERTURBATION_SIZE);
            EvolutionSequence step5_pert = sequence.apply(get_reckless_driver(),5,PERTURBATION_SIZE);

            DistanceExpression crash = new AtomicDistanceExpression(AutonomousDriving::rho_crash,(v1, v2)->Math.abs(v1-v2));

            double[][] step1_crash = new double[H][1];
            double[][] step2_crash = new double[H][1];
            double[][] step3_crash = new double[H][1];
            double[][] step4_crash = new double[H][1];
            double[][] step5_crash = new double[H][1];


            for (int i = 0; i<H; i++){
                step1_crash_speed[i][0] = crash_speed.compute(i+1, sequence, step1_pert);
                step1_crash[i][0] = crash.compute(i, sequence, step1_pert);
            }

            Util.writeToCSV("./step1_crash_speed_scen3.csv", step1_crash_speed);
            Util.writeToCSV("./step1_crash_scen3.csv", step1_crash);


            for (int i = 0; i<H; i++){
                step2_crash_speed[i][0] = crash_speed.compute(i+2, sequence, step2_pert);
                step2_crash[i][0] = crash.compute(i, sequence, step2_pert);
            }

            Util.writeToCSV("./step2_crash_speed_scen3.csv", step2_crash_speed);
            Util.writeToCSV("./step2_crash_scen3.csv", step2_crash);


            for (int i = 0; i<H; i++){
                step3_crash_speed[i][0] = crash_speed.compute(i+3, sequence, step3_pert);
                step3_crash[i][0] = crash.compute(i+3, sequence, step3_pert);
            }

            Util.writeToCSV("./step3_crash_speed_scen3.csv", step3_crash_speed);
            Util.writeToCSV("./step3_crash_scen3.csv", step3_crash);


            for (int i = 0; i<H; i++){
                step4_crash_speed[i][0] = crash_speed.compute(i+4, sequence, step4_pert);
                step4_crash[i][0] = crash.compute(i+4, sequence, step4_pert);
            }

            Util.writeToCSV("./step4_crash_speed_scen3.csv", step4_crash_speed);
            Util.writeToCSV("./step4_crash_scen3.csv", step4_crash);


            for (int i = 0; i<H; i++){
                step5_crash_speed[i][0] = crash_speed.compute(i+5, sequence, step5_pert);
                step5_crash[i][0] = crash.compute(i+5, sequence, step5_pert);
            }

            Util.writeToCSV("./step5_crash_speed_scen3.csv", step5_crash_speed);
            Util.writeToCSV("./step5_crash_scen3.csv", step5_crash);



            // VERIFICATION

            // collision avoidance

            double eta = 0.01;

            DistanceExpression max_si = new MaxIntervalDistanceExpression(
                    crash_speed,
                    0,
                    H
            );

            DistanceExpression max_crash = new MaxIntervalDistanceExpression(
                    crash,
                    0,
                    H
            );


            RobustnessFormula phi_si = new AtomicRobustnessFormula(
                    get_reckless_driver(),
                    max_si,
                    RelationOperator.LESS_OR_EQUAL_THAN,
                    eta
            );

            RobustnessFormula phi_crash = new AtomicRobustnessFormula(
                    get_reckless_driver(),
                    max_crash,
                    RelationOperator.LESS_OR_EQUAL_THAN,
                    eta
            );

            RobustnessFormula phi_combined = new ConjunctionRobustnessFormula(phi_si, phi_crash);

            RobustnessFormula phi_SAF = new AlwaysRobustnessFormula(
                    phi_combined,
                    0,
                    100
            );

            double[][] val_crash = new double[10][1];
            for(int i = 0; i<10; i++) {
                TruthValues value = new ThreeValuedSemanticsVisitor(rand,50,1.96).eval(phi_combined).eval(PERTURBATION_SIZE, i, sequence);
                System.out.println("Evaluation of phi_combined at step "+i+": " + value);
                if (value == TruthValues.TRUE) {
                    val_crash[i][0] = 1;
                } else {
                    if (value == TruthValues.UNKNOWN) {
                        val_crash[i][0] = 0;
                    } else {
                        val_crash[i][0] = -1;
                    }
                }
            }


            Util.writeToCSV("./three_val_crash_scen3.csv",val_crash);


            Boolean SAF = new BooleanSemanticsVisitor().eval(phi_SAF).eval(PERTURBATION_SIZE,0,sequence);

            System.out.println("Evaluation of SAF robustness  with response time "+ TIMER +": "+SAF);

            // don't go off-road

            DistanceExpression atomic_r2l = new AtomicDistanceExpression(AutonomousDriving::rho_r2l,(v1, v2)->Math.abs(v1-v2));

            RobustnessFormula phi_R2L = new AlwaysRobustnessFormula(
                    new AtomicRobustnessFormula(
                            get_reckless_driver(),
                            atomic_r2l,
                            RelationOperator.LESS_OR_EQUAL_THAN,
                            0
                    ),
                    0,
                    300
            );

            Boolean R2L = new BooleanSemanticsVisitor().eval(phi_R2L).eval(PERTURBATION_SIZE,0,sequence);

            System.out.println("Evaluation of R2L robustness  with response time "+ TIMER +": "+R2L);

            // kip it right

            DistanceExpression atomic_kir = new AtomicDistanceExpression(AutonomousDriving::rho_kir,(v1, v2)->Math.abs(v1-v2));

            RobustnessFormula phi_KIR = new AlwaysRobustnessFormula(
                    new AtomicRobustnessFormula(
                            get_reckless_driver(),
                            atomic_kir,
                            RelationOperator.LESS_OR_EQUAL_THAN,
                            0
                    ),
                    0,
                    300
            );

            Boolean KIR = new BooleanSemanticsVisitor().eval(phi_KIR).eval(PERTURBATION_SIZE,0,sequence);

            System.out.println("Evaluation of KIR robustness  with response time "+ TIMER +": "+KIR);

            DistanceExpression max_so = new MaxIntervalDistanceExpression(
                    new AtomicDistanceExpression(AutonomousDriving::rho_so,(v1, v2)->Math.abs(v1-v2)),
                    0,
                    300
            );

            RobustnessFormula phi_SO = new AlwaysRobustnessFormula(
                    new AtomicRobustnessFormula(
                            get_reckless_driver(),
                            max_so,
                            RelationOperator.LESS_OR_EQUAL_THAN,
                            0.1
                    ),
                    0,
                    200
            );

            // safe overtake

            Boolean SO = new BooleanSemanticsVisitor().eval(phi_SO).eval(PERTURBATION_SIZE,0,sequence);

            System.out.println("Evaluation of SO robustness  with response time "+ TIMER +": "+SO);

        }
        catch (RuntimeException e) {
            e.printStackTrace();
        }
    }









    // INITIAL DATA STATE

    private static DataState getInitialState() {
        Map<Integer, Double> values = new HashMap<>();

        values.put(my_speed, MY_INIT_SPEED);
        values.put(other_speed, OTHER_INIT_SPEED);
        values.put(intention, IDLE);
        values.put(my_acc, IDLE);
        values.put(other_acc, IDLE);
        values.put(my_move, 0.0);
        values.put(other_move,0.0);
        values.put(my_timer, 0.0);
        values.put(other_timer, TIMER -1);
        values.put(my_x, MY_INIT_X);
        values.put(other_x, OTHER_INIT_X);
        values.put(my_y, MY_INIT_Y);
        values.put(other_y, OTHER_INIT_Y);
        values.put(my_lane, (MY_INIT_Y <= 4) ? 0.0 : 1.0); // 0 = right / 1 = left
        values.put(other_lane, (OTHER_INIT_Y <= 4) ? 0.0 : 1.0);
        values.put(my_position, (MY_INIT_X <= OTHER_INIT_X) ? -1.0 : 1.0); // -1.0 = behind / 1.0 = ahead
        values.put(dist, Math.sqrt(Math.pow((OTHER_INIT_X - MY_INIT_X), 2) + Math.pow((OTHER_INIT_Y - MY_INIT_Y), 2)));
        double initialSafetyGap;
        if (MY_INIT_X <= OTHER_INIT_X) {
            initialSafetyGap = calculateRSSSafetyDistance(MY_INIT_SPEED, OTHER_INIT_SPEED);
        } else {
            initialSafetyGap = calculateRSSSafetyDistance(OTHER_INIT_SPEED, MY_INIT_SPEED);
        }
        values.put(safety_gap, initialSafetyGap);
        values.put(crash,0.0);

        return new DataState(NUMBER_OF_VARIABLES, i -> values.getOrDefault(i, Double.NaN));

    }  // close getInitialState















    // FORMULA FOR RSS GAP
    private static double calculateRSSSafetyDistance(double rearVehicleSpeed, double frontVehicleSpeed){
        double d1 = TIMER*rearVehicleSpeed;
        double d2 = 0.5 * MAX_ACCELERATION*TIMER*TIMER;
        double d3 = Math.pow((rearVehicleSpeed+TIMER*MAX_ACCELERATION),2)/(2*MIN_BRAKE);
        double d4 = - (frontVehicleSpeed*frontVehicleSpeed)/(2*MAX_BRAKE);
        double rssSafetyDistance = Math.max(d1 + d2 + d3 + d4, 0);
        return rssSafetyDistance + VEHICLE_LENGTH;
    }















    // CONTROLLER

    private static Controller getController(){
        ControllerRegistry registry = new ControllerRegistry();

        registry.set("Control", // agent Control implements the task of the controller when the vehicle is not changing lane
                Controller.ifThenElse(
                        DataState.greaterThan(my_timer,0),
                        Controller.doTick(registry.reference("Control")), // case TIMER not expired: no decision has to be taken
                        Controller.ifThenElse( // case TIMER has expired: decision on lane/speed change have to be taken
                                DataState.equalsTo(my_lane,1),
                                Controller.ifThenElse( // case my_lane = LEFT
                                        (rg, ds) -> ds.get(dist) > ds.get(safety_gap),
                                        Controller.doAction( // case distance > safety gap ==> moving to RIGHT is safe
                                                (rg, ds) -> List.of(new DataStateUpdate(intention, IDLE), new DataStateUpdate(my_move, LANE_RIGHT), new DataStateUpdate(my_timer, TIMER)),
                                                registry.reference("Moving_right")
                                        ),
                                        Controller.ifThenElse( // case distance <= safety gap
                                                DataState.equalsTo(my_position,1),
                                                Controller.ifThenElse( // case I'm ahead
                                                        DataState.equalsTo(other_lane,1),
                                                        Controller.doAction( // case other vehicle on the LEFT lane ==> moving to RIGHT lane is safe
                                                                (rg, ds) -> List.of(new DataStateUpdate(intention, IDLE), new DataStateUpdate(my_move,LANE_RIGHT), new DataStateUpdate(my_timer, TIMER)),
                                                                registry.reference("Moving_right")
                                                        ),
                                                        Controller.doAction( // case other vehicle on the RIGHT lane ==> pushing is safe
                                                                (rg, ds) -> List.of(new DataStateUpdate(intention, FASTER), new DataStateUpdate(my_timer, TIMER)),
                                                                registry.reference("Idling")
                                                        )
                                                ),
                                                Controller.ifThenElse( // case I'm behind
                                                        DataState.equalsTo(other_lane,1),
                                                        Controller.ifThenElse( // case other vehicle on the LEFT lane
                                                                (rg, ds) -> ds.get(dist) == ds.get(safety_gap),
                                                                Controller.doAction( // case distance == safety gap ==> do nothing
                                                                        (rg, ds) -> List.of(new DataStateUpdate(intention, IDLE), new DataStateUpdate(my_timer, TIMER)),
                                                                        registry.reference("Idling")
                                                                ),
                                                                Controller.doAction( // case distance > safety gap ==> slow down  XXXXXXXXXXXXXX
                                                                        (rg, ds) -> List.of(new DataStateUpdate(intention, SLOWER), new DataStateUpdate(my_timer, TIMER)),
                                                                        registry.reference("Idling")
                                                                )
                                                        ),
                                                        Controller.doAction( // case other vehicle on the RIGHT lane ==> pushing is safe
                                                                (rg, ds) -> List.of(new DataStateUpdate(intention, FASTER), new DataStateUpdate(my_timer, TIMER)),
                                                                registry.reference("Idling")
                                                        )
                                                )
                                        )
                                ),
                                Controller.ifThenElse( // case my_lane = RIGHT
                                        (rg,ds) -> ds.get(dist) > ds.get(safety_gap) || ds.get(my_position) == 1,
                                        Controller.doAction( // case distance > safety gap OR I'm ahead ==> pushing is safe
                                                (rg, ds) -> List.of(new DataStateUpdate(intention, FASTER), new DataStateUpdate(my_timer, TIMER)),
                                                registry.reference("Idling")
                                        ),
                                        Controller.ifThenElse( // case distance <= safety gap AND I'm behind
                                                DataState.equalsTo(other_lane,0),
                                                Controller.ifThenElse( // case other vehicle on the RIGHT lane
                                                        (rg,ds) -> ds.get(dist) > ds.get(safety_gap)*0.8,
                                                        Controller.doAction( // case distance > 80% of safety gap ==> moving to LEFT is safe
                                                                (rg, ds) -> List.of(new DataStateUpdate(intention, IDLE), new DataStateUpdate(my_move,LANE_LEFT), new DataStateUpdate(my_timer, TIMER)),
                                                                registry.reference("Moving_left")
                                                        ),
                                                        Controller.doAction( // case distance <= 80% of safety gap ==> I have to slow down
                                                                (rg, ds) -> List.of(new DataStateUpdate(intention, SLOWER), new DataStateUpdate(my_timer, TIMER)),
                                                                registry.reference("Idling")
                                                        )
                                                ),
                                                Controller.doAction( // case other vehicle on the LEFT lane ==> do nothing
                                                        (rg, ds) -> List.of(new DataStateUpdate(intention, IDLE), new DataStateUpdate(my_timer, TIMER)),
                                                        registry.reference("Idling")
                                                )
                                        )
                                )
                        ))
        );

        registry.set("Idling", // agent Idling is waiting for expiring of TIMER
                Controller.ifThenElse(
                        DataState.greaterThan(my_timer, 0),
                        Controller.doTick(registry.reference("Idling")),
                        registry.reference("Control")
                )
        );

        registry.set("Moving_right", // agent Moving_right manages the change from the LEFT to the RIGHT lane
                Controller.ifThenElse(
                        DataState.greaterThan(my_timer,0),
                        Controller.doTick(registry.reference("Moving_right")), // case timer not expired yet: no decision to take, lane crossing will go on
                        Controller.ifThenElse( // case timer expired:
                                (rg,ds) -> ds.get(my_position) == 1 || ds.get(dist) > ds.get(safety_gap),
                                Controller.doAction( // case I'm ahead OR distance > safety gap ==> pushing is safe
                                        (rg, ds) -> List.of(new DataStateUpdate(intention, FASTER), new DataStateUpdate(my_move,0), new DataStateUpdate(my_lane,0), new DataStateUpdate(my_timer, TIMER)),
                                        registry.reference("Idling")
                                ),
                                Controller.ifThenElse( // case I'm behind AND distance <= safety gap
                                        (rg,ds) -> ds.get(dist) == ds.get(safety_gap),
                                        Controller.doAction( // case distance = safety gap ==> do nothing
                                                (rg,ds)-> List.of(new DataStateUpdate(intention,IDLE), new DataStateUpdate(my_move,0), new DataStateUpdate(my_lane,0), new DataStateUpdate(my_timer, TIMER)),
                                                registry.reference("Idling")
                                        ),
                                        Controller.doAction(// if distance < safety gap ==> slow down
                                                (rg,ds)-> List.of(new DataStateUpdate(intention,SLOWER), new DataStateUpdate(my_move,0), new DataStateUpdate(my_lane,0), new DataStateUpdate(my_timer, TIMER)),
                                                registry.reference("Idling")
                                        )
                                )
                        )
                )
        );

        registry.set("Moving_left", // agent Moving_left manages the change from the RIGHT to the LEFT lane
                Controller.ifThenElse(
                        DataState.greaterThan(my_timer, 0),
                        Controller.doTick(registry.reference("Moving_left")), // case timer not expired yet: no decision to take, lane crossing will go on
                        Controller.ifThenElse( // case timer expired
                                DataState.equalsTo(other_lane, 0).and(DataState.equalsTo(my_position,-1)),
                                Controller.doAction( // case other vehicle on the RIGHT lane: pushing is safe
                                        (rg, ds) -> List.of(new DataStateUpdate(intention, FASTER), new DataStateUpdate(my_move,0), new DataStateUpdate(my_lane,1), new DataStateUpdate(my_timer, TIMER)),
                                        registry.reference("Idling")
                                ),
                                Controller.doAction( // case other vehicle on the LEFT lane: slow down is safe
                                        (rg, ds) -> List.of(new DataStateUpdate(intention, SLOWER), new DataStateUpdate(my_move,0), new DataStateUpdate(my_lane,1), new DataStateUpdate(my_timer, TIMER)),
                                        registry.reference("Idling")
                                )
                        )
                )
        );

        return new ExecController(registry.reference("Control"));
    }










    // ENVIRONMENT FUNCTION

    public static List<DataStateUpdate> getEnvironmentUpdates(RandomGenerator rg, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();

        // updating the data of the controlled vehicle
        double my_new_acc;
        // probabilistic update of the acceleration
        if(state.get(intention) == FASTER){ // case Controller wants to do action FASTER
            my_new_acc = MAX_ACCELERATION - rg.nextDouble() * FAST_OFFSET;
        } else if(state.get(intention) == SLOWER){ // case Controller wants to do action SLOWER
                 my_new_acc = - Math.min(MAX_BRAKE, Math.max(MIN_BRAKE, MAX_BRAKE - rg.nextDouble() * SLOW_OFFSET));
              } else if(state.get(intention) == IDLE) { // case Controller wants to do action IDLE
                        my_new_acc = rg.nextDouble() * (2*IDLE_OFFSET) - IDLE_OFFSET; // Accel "close to 0"
                     } else {
                           System.out.println("Controller wants to do something else besides FASTER, SLOWER or IDLE");
                           my_new_acc = 0.0;
                            }
        updates.add(new DataStateUpdate(my_acc, my_new_acc));

        // update for speed / x_and_y position / lane depending on the acceleration
        double my_travel_x = (my_new_acc/2 + state.get(my_speed))*Math.cos((Math.PI/9)*state.get(my_move));
        double my_new_x = state.get(my_x) + my_travel_x;
        double my_new_y = Math.min(8,Math.max(0,state.get(my_y) + (4/ TIMER)*state.get(my_move)));
        double my_new_lane;
        if (my_new_y >= 4){
            my_new_lane = 1;
        } else {
            my_new_lane = 0;
        }
        double my_new_speed = Math.min(Math.max(0,state.get(my_speed) + my_new_acc), MAX_SPEED);

        updates.add(new DataStateUpdate(my_speed, my_new_speed));
        updates.add(new DataStateUpdate(my_x,my_new_x));
        updates.add(new DataStateUpdate(my_y,my_new_y));
        updates.add(new DataStateUpdate(my_lane,my_new_lane));

        // updating the data of the uncontrolled vehicle (UC)
        double other_new_timer;
        double other_new_acc;
        double other_new_move;

        if (state.get(other_timer) == 0){ // case timer of UC expired
            other_new_timer = TIMER -1;
            if (state.get(other_lane)==1){ // case UC-lane = LEFT
                if ((state.get(dist) > state.get(safety_gap) || state.get(my_position)==-1)){
                    //  case distance > safety gap OR UC ahead ==> UC moves to RIGHT lane
                    other_new_acc = rg.nextDouble() * (2*IDLE_OFFSET) - IDLE_OFFSET;
                    other_new_move = LANE_RIGHT;
                } else {  // XXXXXXXXXXXXXXX
                    if (state.get(dist)>state.get(safety_gap)){
                            // case distance > safety gap AND UC behind ==> everything is safe
                        double token = rg.nextDouble();
                        if (token >= 0.60){
                                 // UC accelerates with probability 40%
                            other_new_acc = MAX_ACCELERATION - rg.nextDouble() * FAST_OFFSET;
                            other_new_move = 0.0;
                        } else {
                            if (token >= 0.20) {
                                  // UC moves to RIGHT lane with probability 40%, unless UC was moving to he LEFT
                                other_new_acc = rg.nextDouble() * (2 * IDLE_OFFSET) - IDLE_OFFSET;
                                if (state.get(other_move)!=LANE_LEFT){
                                    other_new_move = LANE_RIGHT;
                                } else {
                                    other_new_move = 0.0;
                                }
                            } else { // UC brakes with probability 20%
                                other_new_acc = - (rg.nextDouble() * (MAX_BRAKE - MIN_BRAKE) + MIN_BRAKE);
                                other_new_move = 0.0;
                            }
                        }
                    } else { // case distance <= safety gap AND UC ahead
                        other_new_move = 0.0;
                        if (state.get(my_position)==1){
                            other_new_acc = - (rg.nextDouble() * (MAX_BRAKE - MIN_BRAKE) + MIN_BRAKE);
                        } else {
                            other_new_acc = MAX_ACCELERATION - rg.nextDouble() * FAST_OFFSET;
                        }
                    }
                }
            } else { // case other_lane = RIGHT
                if (state.get(dist)>state.get(safety_gap)){ // case distance > safety gap: everything is safe
                    double token = rg.nextDouble();
                    if (token >= 0.45){  // UC accelerates with probability 55%
                        other_new_acc = MAX_ACCELERATION - rg.nextDouble() * FAST_OFFSET;
                        other_new_move = 0.0;
                    } else { // UC brakes with probability 25%
                        if (token >= 0.20) {
                            other_new_acc = - (rg.nextDouble() * (MAX_BRAKE - MIN_BRAKE) + MIN_BRAKE);
                            other_new_move = 0.0;
                        } else { // UC moves to the LEFT lane, unless it was moving to the RIGHT, with probability 20%
                            other_new_acc = rg.nextDouble() * (2 * IDLE_OFFSET) - IDLE_OFFSET;
                            if (state.get(other_move)!=LANE_RIGHT){
                                other_new_move = LANE_LEFT;
                            } else {
                                other_new_move = 0.0;
                            }
                        }
                    }
                } else { // case 0.8*safety gap < distance <= safety gap AND UC behind and controlled vehicle on RIGHT lane
                         // ==> UC moves to the LEFT lane
                    if (state.get(dist)>state.get(safety_gap)*0.8 && state.get(my_position)==1 && state.get(my_lane)==0) {
                        other_new_acc = rg.nextDouble() * (2*IDLE_OFFSET) - IDLE_OFFSET;
                        other_new_move = LANE_LEFT;
                    } else { // otherwise, if UC is behind it brakes, whereas if it is ahead it pushes
                        other_new_move = 0.0;
                        if (state.get(my_position)==1) {
                            other_new_acc = - (rg.nextDouble() * (MAX_BRAKE - MIN_BRAKE) + MIN_BRAKE);
                        } else {
                            other_new_acc = MAX_ACCELERATION - rg.nextDouble() * FAST_OFFSET;
                        }
                    }
                }
            }
        } else { // case timer of UC not expired
            other_new_timer = state.get(other_timer)-1;
            other_new_acc = state.get(other_acc);
            if ((state.get(other_y) >= 6 && state.get(other_move)==LANE_LEFT) ||
                    (state.get(other_y) <= 2 && state.get(other_move)==LANE_RIGHT)) {
                other_new_move = 0;
            } else {
                other_new_move = state.get(other_move);
            }
        }
        updates.add(new DataStateUpdate(other_acc,other_new_acc));
        updates.add(new DataStateUpdate(other_timer,other_new_timer));
        updates.add(new DataStateUpdate(other_move,other_new_move));

        double other_new_speed = Math.min(Math.max(0, state.get(other_speed) + other_new_acc), MAX_SPEED-5);

        double other_travel_x = (other_new_acc/2 + other_new_speed)*Math.cos((Math.PI/9)*other_new_move);
        double other_new_x = state.get(other_x) + other_travel_x;
        double other_new_y = Math.min(8,Math.max(0,state.get(other_y) + (4/ TIMER)*other_new_move));
        double other_new_lane;
        if (other_new_y >= 4){
            other_new_lane = 1;
        } else {
            other_new_lane = 0;
        }

        updates.add(new DataStateUpdate(other_speed, other_new_speed));
        updates.add(new DataStateUpdate(other_x,other_new_x));
        updates.add(new DataStateUpdate(other_y,other_new_y));
        updates.add(new DataStateUpdate(other_lane,other_new_lane));


        // updating of distance / safety gap / timer / crash
        double new_dist = Math.sqrt(Math.pow((other_new_x-my_new_x),2) + Math.pow((other_new_y-my_new_y),2));
        updates.add(new DataStateUpdate(dist, new_dist));

        double my_new_position = (my_new_x>= other_new_x)?1:-1;
        updates.add(new DataStateUpdate(my_position,my_new_position));

        double new_safety_gap;
        if(my_new_position==-1){
            new_safety_gap = calculateRSSSafetyDistance(my_new_speed,other_new_speed);
        } else {
            new_safety_gap = calculateRSSSafetyDistance(other_new_speed,my_new_speed);
        }
        updates.add(new DataStateUpdate(safety_gap, new_safety_gap));

        double my_new_timer = state.get(my_timer) - 1;
        updates.add(new DataStateUpdate(my_timer, my_new_timer));

        if(my_new_lane==other_new_lane && Math.abs(my_new_x-other_new_x)<= VEHICLE_LENGTH){
            updates.add(new DataStateUpdate(crash,1));
        } else {
            if (my_new_lane!=other_new_lane && Math.abs(my_new_x-other_new_x)<= VEHICLE_LENGTH && Math.abs(my_new_y-other_new_y)<= VEHICLE_WIDTH){
                updates.add(new DataStateUpdate(crash,1));
            }
        }

        return updates;
    }










    // RECKLESS DRIVER PERTURBATION

    private static Perturbation get_reckless_driver(){
        return new AfterPerturbation(5,new IterativePerturbation(50,new AtomicPerturbation(2, AutonomousDriving::reckless_driver)));
    }

    private static DataState reckless_driver(RandomGenerator rg, DataState state){
        List<DataStateUpdate> updates = new LinkedList<>();
        if(state.get(dist) > state.get(safety_gap)*0.25) {
            double token = rg.nextDouble();
            double other_new_move;
            if (token > 0.4) {
                if (state.get(other_lane) == 0){
                    other_new_move = LANE_LEFT;
                } else {
                    other_new_move = LANE_RIGHT;
                }
                double other_new_speed;
                double other_new_acc = rg.nextDouble() * (2*IDLE_OFFSET) - IDLE_OFFSET;

                other_new_speed = Math.min(Math.max(0, state.get(other_speed) + other_new_acc), MAX_SPEED-5);

                double other_travel_x = (other_new_acc/2 + other_new_speed)*Math.cos((Math.PI/9)*other_new_move);
                double other_new_x = state.get(other_x) + other_travel_x;
                double other_new_y = Math.min(8,Math.max(0,state.get(other_y) + 3*other_new_move));
                double other_new_lane;
                if (other_new_y >= 4){
                    other_new_lane = 1;
                } else {
                    other_new_lane = 0;
                }
                updates.add(new DataStateUpdate(other_move,other_new_move));
                updates.add(new DataStateUpdate(other_speed, other_new_speed));
                updates.add(new DataStateUpdate(other_x,other_new_x));
                updates.add(new DataStateUpdate(other_y,other_new_y));
                updates.add(new DataStateUpdate(other_lane,other_new_lane));
                double new_dist = Math.sqrt(Math.pow((other_new_x-state.get(my_x)),2) + Math.pow((other_new_y-state.get(my_y)),2));
                updates.add(new DataStateUpdate(dist, new_dist));
                double my_new_position = (state.get(my_x)>= other_new_x)?1:-1;
                updates.add(new DataStateUpdate(my_position,my_new_position));
                double new_safety_gap;
                if(my_new_position==-1){
                    new_safety_gap = calculateRSSSafetyDistance(state.get(my_speed),other_new_speed);
                } else {
                    new_safety_gap = calculateRSSSafetyDistance(other_new_speed,state.get(my_speed));
                }
                updates.add(new DataStateUpdate(safety_gap, new_safety_gap));
                //updates.add(new DataStateUpdate(other_timer,RESPONSE_TIME-1));
            } else {
                updates.add(new DataStateUpdate(dist,state.get(dist)+rg.nextDouble()*DIST_OFFSET));
            }
        } else {
            updates.add(new DataStateUpdate(dist,state.get(dist)+rg.nextDouble()*DIST_OFFSET));
        }
        return state.apply(updates);
    }

    // PENALTY FUNCTIONS

    public static double rho_si(DataState state) {
        if (state.get(crash) == 1){
            return 0.5*Math.sqrt(state.get(my_speed)*state.get(my_speed) + state.get(other_speed)*state.get(other_speed) - 2*state.get(my_speed)*state.get(other_speed))/MAX_SPEED;
        }
        else{
            return 0.0;
        }
    }

    public static double rho_crash(DataState state) {
        return state.get(crash);
    }

    public static double rho_r2l(DataState state){
        if (state.get(my_lane)>6 || state.get(my_lane)<2){
            return 1;
        } else {
            return 0;
        }
    }

    public static double rho_so(DataState state){
        if (state.get(my_timer)== TIMER -1 && state.get(my_move)==1 && state.get(other_y)%2==0 && state.get(other_lane)==1){
            return Math.max(0,(state.get(safety_gap)-state.get(dist))/state.get(safety_gap));
        } else {
            return 0;
        }
    }
    public static double rho_kir(DataState state){
        if (state.get(my_timer)== TIMER -1 && state.get(my_move)==1 && state.get(other_y)%2==0 && state.get(other_lane)==1){
            return 1;
        } else {
            return 0;
        }
    }


    // Utility methods

    private static void printLData(RandomGenerator rg, ArrayList<String> label, ArrayList<DataStateExpression> F, SystemState s, int steps, int size) {
        System.out.println(label);
        double[][] data = SystemState.sample(rg, F, s, steps, size);
        for (int i = 0; i < data.length; i++) {
            System.out.printf("%d>   ", i);
            for (int j = 0; j < data[i].length -1; j++) {
                System.out.printf("%f   ", data[i][j]);
            }
            System.out.printf("%f\n", data[i][data[i].length -1]);
        }
    }

    private static void printLData(RandomGenerator rg, ArrayList<String> label, ArrayList<DataStateExpression> F, Perturbation p, SystemState s, int steps, int size) {
        System.out.println(label);
        double[][] data = SystemState.sample(rg, F, p, s, steps, size);
        for (int i = 0; i < data.length; i++) {
            System.out.printf("%d>   ", i);
            for (int j = 0; j < data[i].length -1; j++) {
                System.out.printf("%f   ", data[i][j]);
            }
            System.out.printf("%f\n", data[i][data[i].length -1]);
        }
    }



}