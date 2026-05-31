package cyberattacks;

import java.io.IOException;

public class Main {

    // COORDINATED SENSOR ATTACK
    public static void main(String[] args) throws IOException {
        try {
            new TwoLanesTwoCarsAttack();
        } catch (RuntimeException e) {
            e.printStackTrace();
        }
    }

}