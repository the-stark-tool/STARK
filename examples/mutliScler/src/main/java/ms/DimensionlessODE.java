package ms;

import org.apache.commons.math3.random.RandomGenerator;
import stark.DecoupledTimedSystem;
import stark.DefaultRandomGenerator;
import stark.SystemState;
import stark.controller.NilController;
import stark.ds.DataState;
import stark.ds.DataStateExpression;
import stark.ds.DataStateUpdate;
import stark.perturbation.Perturbation;

import java.io.FileWriter;
import java.io.IOException;
import java.util.*;

public class DimensionlessODE {

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
    public static final double deltaE = 0.2; // E decay
    public static final double re = 0.025; // generation of E from D
    public static final double ro = 0.05; // O generation
    public static final double rm = 1.2; // remyelination from O
    public static final double sigma = 1.0;
    public static final double e0 = 1.0;
    public static final double eb = 0.4;

    public static final double INIT_B = 5;

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


            ArrayList<DataStateExpression> F = new ArrayList<>();
            F.add(ds -> ds.get(APC));
            F.add(ds -> ds.get(T));
            F.add(ds -> ds.get(S));
            F.add(ds -> ds.get(B));
            F.add(ds -> ds.get(E));
            F.add(ds -> ds.get(O));
            F.add(ds -> ds.get(M));


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
        values.put(c, 0.2);
        values.put(d, d0 - alpha*(e0+eb*INIT_B));
        values.put(e, e0+eb*INIT_B);
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

        double new_e = e0+eb*state.get(B);
        updates.add(new DataStateUpdate(e, new_e));

        double new_d = d0-alpha*new_e;
        updates.add(new DataStateUpdate(d, new_d));

        double dAPC = new_e*state.get(APC)/(f*(state.get(T)+ state.get(S)) + g) - ((state.get(T)+ state.get(S))*CONTACT+h)* state.get(APC);
        double new_APC = state.get(APC) + dAPC*delta_t;
        updates.add(new DataStateUpdate(APC, new_APC));

        double dT = Math.pow(state.get(APC), 2)/(state.get(a)*(state.get(T)+state.get(S)) + state.get(b)) - state.get(c)* state.get(T);
        double new_T = state.get(T) + dT*delta_t;
        updates.add(new DataStateUpdate(T, new_T));

        double dS = new_d*state.get(T) - state.get(S);
        double new_S = state.get(S) + dS*delta_t;
        updates.add(new DataStateUpdate(S, new_S));

        double dB = state.get(alphab)*Math.max(dAPC*delta_t,0) - mub*state.get(B);
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
