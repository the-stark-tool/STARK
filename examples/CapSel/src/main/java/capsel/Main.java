package capsel;

import java.io.IOException;

public final class Main {

    private Main() {
    }

    public static void main(String[] args) throws IOException {
        CapSelSimulation.RunRequest request = CapSelSimulation.RunRequest.parse(args);
        new CapSelSimulation(request).run();
    }
}
