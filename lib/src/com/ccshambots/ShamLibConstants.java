package frc.robot.ShamLib;

public class ShamLibConstants {

    public static class SMF {
        public static final double transitionTimeout = 2; //seconds
    }

    public static class Motor {
        public static final double KS_INCREMENT_VOLTAGE = 0.05;
        public static final double KS_INCREMENT_TIME = 1; //The time between KS increments (s)
        public static final double KS_THRESHOLD_VELO = 0.1; //The threshold velocity of the motor (r/s)
        public static final double KS_THRESHOLD_TIME = 0.5; //The time the motor must remain above the threshold value
    }

    public static class Swerve {
        //How far the module can be off from the actual position to trigger a correction
        public static final double ALLOWED_MODULE_ERROR = 2; //Deg
        //How close the modules must stay to reset the modules successfully
        public static final double ALLOWED_STOPPED_MODULE_DIFF = .2; //degrees
    }
}
