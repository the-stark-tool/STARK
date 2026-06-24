package two_lanes_merge.constants;

public final class Action {

  private Action() {
    // Utility class
  }

  public static final double FASTER = 1;
  public static final double SLOWER = -1;
  public static final double IDLE = 0;

  public static final double STEER_RIGHT = -1;
  public static final double STEER_LEFT = 1;
  public static final double STEER_STRAIGHT = 0;
}
