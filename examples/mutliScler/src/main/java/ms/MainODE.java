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

package ms;

import stark.controller.*;
import stark.*;
import stark.controller.Controller;
import stark.controller.NilController;
import stark.ds.DataState;
import stark.ds.DataStateExpression;
import stark.ds.DataStateUpdate;
import stark.perturbation.Perturbation;
import org.apache.commons.math3.random.RandomGenerator;

import java.io.FileWriter;
import java.io.IOException;
import java.util.*;

public class MainODE {

    public static final int APC = 0; // self-antigen presenting cells (self-APCs)
    public static final int T = 1; // normal regulatory cells
    public static final int S = 2; // strong regulatory cells
    public static final int B = 3; // immune memory B cells
    public static final int E = 4; // effector cells
    public static final int O = 5; // oligodendrocytes
    public static final int M = 6; // myelin
    public static final int a = 7; // T+S recruitment from D
    public static final int b = 8; // T recruitment from D
    public static final int c = 9; // loss of T
    public static final int d = 10; // creation of S
    public static final int e = 11; // D generation
    //public static final int e0 = 11; // basal D generation
    //public static final int eb = 12; // B impact on D
    public static final int alphab = 12; // creation of B

    public static final int ebv1 = 13; // scaled ebv parameter
    public static final int ebv2 = 14; // scaled ebv parameter
    public static final int chi = 15; // vitamin D deficiency
    public static final int estro = 16; // estrogen parameter

    public static final int p_gender = 17;

    public static final int EBV_counter = 18;
    public static final int EBV_active = 19;
    public static final int EBV_likelihood = 20;
    public static final int vD_counter = 21;
    public static final int vD_active = 22;
    public static final int vD_likelihood = 23;
    public static final int estro_counter = 24;
    public static final int estro_active = 25;
    public static final int estro_likelihood = 26;

    private static final int NUMBER_OF_VARIABLES = 27;

    public static final double d0 = 5.0; // relation between D and S
    public static final double f = 0.2; // creation of D from T+S
    public static final double g = 0.45; // creation of D from T
    public static final double h = 2.25; // loss of D
    public static final double alpha = 0.625; // relation between D and S
    public static final double mub = 0.0001; // loss of B
    public static final double beta1 = 0.001; // O loss due to D
    public static final double beta2 = 0.001; // O loss due to E
    public static final double gamma = 0.15; // M loss due to E
    public static final double delta1 = h/2; // reduction of E by T
    public static final double delta2 = h; // reduction of E by S
    public static final double deltaE = 0.125; // E decay
    public static final double re = 0.025; // generation of E from D
    public static final double ro = 0.05; // O generation
    public static final double rm = 1.2; // remyelination from O
    public static final double sigma = 1.0;

    public static final double INIT_B = 1;

    private static final double delta_t = Math.pow(10,-4); //
    private static final double env_gran = 1;
    private static final double ctrl_step = 0;
    private static final double ctrl_gran = 0;

    public static double INIT_EBV_TIME = 200;
    public static double INIT_vD_TIME = 100;
    public static double INIT_estro_TIME_m = 150;
    public static double INIT_estro_TIME_f = 28;
    public static double CONTACT = 0.2;

    public static void main(String[] args) throws IOException {
        try {

            RandomGenerator rand = new DefaultRandomGenerator();

            DataState state = getInitialState(rand, env_gran, 0.0, 0.0, delta_t);

            DecoupledTimedSystem system = new DecoupledTimedSystem(new NilController(), (rg, ds) -> ds.apply(odeEnv(rg, ds)), state, ds -> selectTime(rand,ds));

            int steps = 2000;
            int size = 1;

            ArrayList<String> LS = new ArrayList<>();
            LS.add("APC");
            LS.add("T");
            LS.add("S");
            LS.add("B");
            LS.add("E");
            LS.add("O");
            LS.add("M");
            LS.add("gen");
            LS.add("EBV");
            LS.add("estro");
            LS.add("vD");


            ArrayList<DataStateExpression> F = new ArrayList<>();
            F.add(ds -> ds.get(APC));
            F.add(ds -> ds.get(T));
            F.add(ds -> ds.get(S));
            F.add(ds -> ds.get(B));
            F.add(ds -> ds.get(E));
            F.add(ds -> ds.get(O));
            F.add(ds -> ds.get(M));
            F.add(ds -> ds.get(p_gender));
            F.add(ds -> ds.get(EBV_active));
            F.add(ds -> ds.get(estro_active));
            F.add(ds -> ds.get(vD_active));


            double[][] data_avgH = SystemState.sample(rand, F, system, steps, size);

            System.out.println("Average value for APC, T, S, B, E, O, M, gender, EBV activity, estro activity, vD activity in " +size+ " runs "+ steps +" days ");
            for (int i = 0; i < data_avgH.length; i++) {
                System.out.printf("%d>   ", i);
                for (int j = 0; j < data_avgH[i].length - 1; j++) {
                    System.out.printf("%f   ", data_avgH[i][j]);
                }
                System.out.printf("%f\n", data_avgH[i][data_avgH[i].length - 1]);
            }



        } catch(RuntimeException e){
            e.printStackTrace();
        }

    }

    public static DataState getInitialState(RandomGenerator rg, double gran, double Tstep, double Ttot, double Tshift) {
        Map<Integer, Double> values = new HashMap<>();

        values.put(APC, 0.1);
        values.put(T, 0.0);
        values.put(S, 0.0);
        //values.put(B, Math.min(2,rg.nextDouble()*2+0.01));
        values.put(B, INIT_B);
        values.put(E, 0.0);
        values.put(O, 1.0);
        values.put(M, 1.0);
        values.put(a, 40000.0 / 3.0);
        values.put(b, 30000.0);
        values.put(c, 0.125);
        values.put(d, d0 - alpha*(1.0+0.4*INIT_B));
        values.put(e, 1.0+0.4*INIT_B);
        values.put(alphab, 0.01);
        values.put(ebv1, 1.0);
        values.put(ebv2, 1.0);
        values.put(chi, 1.0);
        values.put(EBV_counter, INIT_EBV_TIME);
        values.put(vD_counter, INIT_vD_TIME);
        double gen = rg.nextDouble();
        if (gen < 0.5){
            values.put(p_gender,0.0);
            values.put(estro_counter, INIT_estro_TIME_m);
            values.put(estro_likelihood,0.01);
            values.put(vD_likelihood,0.45);
        } else {
            values.put(p_gender,1.0);
            values.put(estro_counter, INIT_estro_TIME_f);
            values.put(estro_likelihood,0.1);
            values.put(vD_likelihood,0.55);
        }
        double like = rg.nextDouble();
        if (like <= 0.1){
            values.put(EBV_likelihood, 0.0);
        } else {
            values.put(EBV_likelihood, 0.1);
        }
        values.put(EBV_active, 0.0);
        values.put(vD_active, 0.0);
        values.put(estro_active, 0.0);
        return new DataState(NUMBER_OF_VARIABLES, i -> values.getOrDefault(i, Double.NaN), gran, Tstep, Ttot, Tshift);
    }


    public static List<DataStateUpdate> odeEnv(RandomGenerator rg, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();

        double new_alphab = state.get(alphab);
        double new_e = state.get(e);
        double new_d = state.get(d);

        if (state.get(EBV_counter) <= 0){
            if (state.get(EBV_active) == 0) {
                double token = rg.nextDouble();
                if (token < state.get(EBV_likelihood)) {
                    double new_ebv1 = 1+rg.nextDouble();
                    double new_ebv2 = 1+rg.nextDouble();
                    new_alphab = state.get(alphab)*new_ebv1;
                    new_e = state.get(e)*new_ebv2;
                    new_d = Math.max(d0 - alpha*new_e,0.0);
                    double new_ebv_timer = 14 + rg.nextInt(15);
                    updates.add(new DataStateUpdate(ebv1,new_ebv1));
                    updates.add(new DataStateUpdate(ebv2,new_ebv2));
                    updates.add(new DataStateUpdate(alphab, new_alphab));
                    updates.add(new DataStateUpdate(e, new_e));
                    updates.add(new DataStateUpdate(d, new_d));
                    updates.add(new DataStateUpdate(EBV_active,1));
                    updates.add(new DataStateUpdate(EBV_counter, new_ebv_timer));
                } else {
                    updates.add(new DataStateUpdate(EBV_counter,INIT_EBV_TIME));
                }
            } else {
                new_alphab = state.get(alphab)/ state.get(ebv1);
                new_e = state.get(e)/ state.get(ebv2);
                new_d = Math.max(d0 - alpha*new_e,0.0);
                updates.add(new DataStateUpdate(ebv1,1));
                updates.add(new DataStateUpdate(ebv2,1));
                updates.add(new DataStateUpdate(alphab, new_alphab));
                updates.add(new DataStateUpdate(e, new_e));
                updates.add(new DataStateUpdate(d, new_d));
                updates.add(new DataStateUpdate(EBV_active,0));
                updates.add(new DataStateUpdate(EBV_counter, INIT_EBV_TIME));
            }
        } else {
            updates.add(new DataStateUpdate(EBV_counter, state.get(EBV_counter)-delta_t));
        }

        double new_a = state.get(a);
        double new_b = state.get(b);

        if (state.get(estro_counter) <= 0){
            if (state.get(estro_active) == 0){
                double token = rg.nextDouble();
                if (token < state.get(estro_likelihood)) {
                    double new_estro = 1+rg.nextDouble();
                    new_a = state.get(a)*new_estro;
                    new_b = state.get(e)*new_estro;
                    updates.add(new DataStateUpdate(a, new_a));
                    updates.add(new DataStateUpdate(b, new_b));
                    updates.add(new DataStateUpdate(estro,new_estro));
                    updates.add(new DataStateUpdate(estro_active,1));
                    if (state.get(p_gender)==1) {
                        updates.add(new DataStateUpdate(estro_counter, 7));
                    } else {
                        updates.add(new DataStateUpdate(estro_counter, 1));
                    }
                } else {
                    if (state.get(p_gender)==1) {
                        updates.add(new DataStateUpdate(estro_counter, INIT_estro_TIME_f));
                    } else {
                        updates.add(new DataStateUpdate(estro_counter, INIT_estro_TIME_m));
                    }
                }
            } else {
                new_a = state.get(a)/state.get(estro);
                new_b = state.get(b)/state.get(estro);
                updates.add(new DataStateUpdate(a, new_a));
                updates.add(new DataStateUpdate(b, new_b));
                updates.add(new DataStateUpdate(estro,1));
                updates.add(new DataStateUpdate(estro_active,0));
                if (state.get(p_gender)==1) {
                    updates.add(new DataStateUpdate(estro_counter, 21));
                } else {
                    updates.add(new DataStateUpdate(estro_counter, INIT_estro_TIME_m));
                }
            }
        } else {
            updates.add(new DataStateUpdate(estro_counter, state.get(estro_counter)-delta_t));
        }

        double new_new_a = new_a;
        double new_new_b = new_b;

        if (state.get(vD_counter) <= 0){
            if (state.get(vD_active) == 0) {
                double token = rg.nextDouble();
                if (token < state.get(vD_likelihood)) {
                    double new_chi = 1+rg.nextDouble();
                    new_new_a = new_a*new_chi;
                    new_new_b = new_b*new_chi;
                    double new_vd_timer = 60 + rg.nextInt(121);
                    updates.add(new DataStateUpdate(chi,new_chi));
                    updates.add(new DataStateUpdate(a, new_new_a));
                    updates.add(new DataStateUpdate(b, new_new_b));
                    updates.add(new DataStateUpdate(vD_active,1));
                    updates.add(new DataStateUpdate(vD_counter, new_vd_timer));
                } else {
                    updates.add(new DataStateUpdate(EBV_counter,INIT_EBV_TIME));
                }
            } else {
                new_new_a = new_a/ state.get(chi);
                new_new_b = new_b/ state.get(chi);
                updates.add(new DataStateUpdate(chi,1));
                updates.add(new DataStateUpdate(a, new_new_a));
                updates.add(new DataStateUpdate(b, new_new_b));
                updates.add(new DataStateUpdate(vD_active,0));
                updates.add(new DataStateUpdate(vD_counter, INIT_vD_TIME));
            }
        } else {
            updates.add(new DataStateUpdate(vD_counter, state.get(vD_counter)-delta_t));
        }

        double dAPC = new_e*state.get(APC)/(f*(state.get(T)+ state.get(S)) + g) - ((state.get(T)+ sigma*state.get(S))*CONTACT+h)* state.get(APC);
        double new_APC = state.get(APC) + dAPC*delta_t;
        updates.add(new DataStateUpdate(APC, new_APC));

        double dT = Math.pow(state.get(APC), 2)/(new_new_a*(state.get(T)+state.get(S)) + new_new_b) - state.get(c)* state.get(T);
        double new_T = state.get(T) + dT*delta_t;
        updates.add(new DataStateUpdate(T, new_T));

        double dS = new_d*state.get(T) - state.get(S);
        double new_S = state.get(S) + dS*delta_t;
        updates.add(new DataStateUpdate(S, new_S));

        double dB = new_alphab*Math.max(dAPC*delta_t,0) - mub*state.get(B);
        double new_B = state.get(B) + dB*delta_t;
        updates.add(new DataStateUpdate(B, new_B));

        double dE = re*state.get(APC) - (delta1*state.get(T) + delta2*state.get(S) + deltaE)*state.get(E);
        double new_E = state.get(E) + dE*delta_t;
        updates.add(new DataStateUpdate(E, new_E));

        double dO = (ro*(1-state.get(O)) - beta1*state.get(APC) - beta2*state.get(E))*state.get(O);
        double new_O = state.get(O) + dO*delta_t;
        updates.add(new DataStateUpdate(O, new_O));

        double dM = rm*state.get(O)*(1-state.get(M)) - gamma*state.get(M)*state.get(E);
        double new_M = state.get(M) + dM*delta_t;
        updates.add(new DataStateUpdate(M, new_M));

        return updates;

    }

    public static double selectTime(RandomGenerator rg, DataState ds){
        return ds.getTimeDelta();
    }


    private static void writeRunsToCSV(RandomGenerator rg, ArrayList<String> labels,
                                       ArrayList<DataStateExpression> F, SystemState s,
                                       int steps, int numRuns, String conditionLabel, String path) throws IOException {
        try (FileWriter fw = new FileWriter(path)) {
            fw.write("run,step");
            for (String label : labels) {
                fw.write("," + label);
            }
            fw.write(",condition\n");
            for (int r = 0; r < numRuns; r++) {
                double[][] data = SystemState.sample(rg, F, s, steps, 1);
                for (int i = 0; i < data.length; i++) {
                    fw.write(r + "," + i);
                    for (int j = 0; j < data[i].length; j++) {
                        fw.write("," + String.format(Locale.US, "%f", data[i][j]));
                    }
                    fw.write("," + conditionLabel + "\n");
                }
            }
        }
    }

    private static void writePRunsToCSV(RandomGenerator rg, ArrayList<String> labels,
                                        ArrayList<DataStateExpression> F, Perturbation perturbation, SystemState s,
                                        int steps, int numRuns, String conditionLabel, String path) throws IOException {
        try (FileWriter fw = new FileWriter(path)) {
            fw.write("run,step");
            for (String label : labels) {
                fw.write("," + label);
            }
            fw.write(",condition\n");
            for (int r = 0; r < numRuns; r++) {
                double[][] data = SystemState.sample(rg, F, perturbation, s, steps, 1);
                for (int i = 0; i < data.length; i++) {
                    fw.write(r + "," + i);
                    for (int j = 0; j < data[i].length; j++) {
                        fw.write("," + String.format(Locale.US, "%f", data[i][j]));
                    }
                    fw.write("," + conditionLabel + "\n");
                }
            }
        }
    }

}