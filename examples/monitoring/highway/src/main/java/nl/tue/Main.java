package nl.tue;

import java.io.IOException;

public class Main {
    public static void main(String[] args){
        try {
            new TwoLanesTwoCars();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}
