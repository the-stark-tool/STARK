package two_lanes_merge.constants;

public final class Encodings {

  public static final double LEFT_LANE = 1.0;
  public static final double RIGHT_LANE = 0.0;
  public static final double BETWEEN_LEFT_AND_RIGHT_LANE = 0.5;
  public static final double LEFT_FROM_LEFT_LANE = 1.5;
  public static final double RIGHT_FROM_RIGHT_LANE = -0.5;

  public static final double DIRECTION_BEHIND = 1.0;
  public static final double DIRECTION_AHEAD = 0.0;
  public static final double DIRECTION_UNDEFINED = -1.0;

  public static final double CAR_FINISHED = 0.0;
  public static final double CAR_NOT_FINISHED = 1.0;
}
