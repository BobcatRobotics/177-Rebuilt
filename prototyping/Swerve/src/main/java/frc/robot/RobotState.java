public class RobotState {
    private static RobotState instance;
      public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }
}
