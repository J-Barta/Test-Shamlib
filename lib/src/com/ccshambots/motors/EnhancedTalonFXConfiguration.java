package frc.robot.ShamLib.motors;


import static frc.robot.ShamLib.motors.EnhancedTalonFXConfiguration.InvertedBehavior.*;
import static frc.robot.ShamLib.motors.EnhancedTalonFXConfiguration.NeutralBehavior.*;
import static frc.robot.ShamLib.motors.EnhancedTalonFXConfiguration.RunMode.*;
import static frc.robot.ShamLib.motors.EnhancedTalonFXConfiguration.TalonType.*;

public class EnhancedTalonFXConfiguration {
    public TalonType type = PRO;
    public RunMode runMode = POWER;

    public PIDSVGains proControlGains = new PIDSVGains();
    public PIDFGains v5ControlGains = new PIDFGains();

    //DO NOT include the ticks to rotations ratio (2048:1) for V5 motors
    public double inputToOutputRatio = 1;

    public String canbus = "";
    public int deviceNumber = 1;

    public double supplyLimit = 0; //amps

    public InvertedBehavior invertedBehavior = CCWP;
    public NeutralBehavior neutralBehavior = COAST;

    public double maxVel = 1; //output/sec
    public double maxAccel = 1; //output/sec^2
    public double jerk = 1000; //output/sec^3

    public EnhancedTalonFXConfiguration() {}


    public enum InvertedBehavior {
        CWP, //Clockwise positive
        CCWP //Counter-clockwise positive
    }

    public enum NeutralBehavior {
        BRAKE,
        COAST
    }

    public enum RunMode {
        POWER,
        VELOCITY,
        MOTION_MAGIC
    }

    public enum TalonType {
        PRO,
        V5
    }

    public EnhancedTalonFXConfiguration setType(TalonType type) {
        this.type = type;

        return this;
    }

    public EnhancedTalonFXConfiguration setRunMode(RunMode runMode) {
        this.runMode = runMode;

        return this;
    }

    public EnhancedTalonFXConfiguration setV5ControlGains(PIDFGains v5ControlGains) {
        this.v5ControlGains = v5ControlGains;

        return this;
    }

    public EnhancedTalonFXConfiguration setProControlGains(PIDSVGains gains) {
        proControlGains = gains;

        return this;
    }

    public EnhancedTalonFXConfiguration setInputToOutputRatio(double inputToOutputRatio) {
        this.inputToOutputRatio = inputToOutputRatio;

        return this;
    }

    public EnhancedTalonFXConfiguration setCanbus(String canbus) {
        this.canbus = canbus;

        return this;
    }

    public EnhancedTalonFXConfiguration setDeviceNumber(int deviceNumber) {
        this.deviceNumber = deviceNumber;

        return this;
    }

    public EnhancedTalonFXConfiguration setInvertedBehavior(InvertedBehavior invertedBehavior) {
        this.invertedBehavior = invertedBehavior;

        return this;
    }

    /**
     * Set the inversion of the motor with true being CWP and false being CCWP
     * @param value the value to set the inversion to
     * @return the configuration object for building
     */
    public EnhancedTalonFXConfiguration setInvertedBehavior(boolean value) {
        this.invertedBehavior = value ? CWP : CCWP;

        return this;
    }

    public EnhancedTalonFXConfiguration setNeutralBehavior(NeutralBehavior neutralBehavior) {
        this.neutralBehavior = neutralBehavior;

        return this;
    }

    public EnhancedTalonFXConfiguration setMaxVel(double maxVel) {
        this.maxVel = maxVel;

        return this;
    }

    public EnhancedTalonFXConfiguration setMaxAccel(double maxAccel) {
        this.maxAccel = maxAccel;

        return this;
    }

    public EnhancedTalonFXConfiguration setJerk(double jerk) {
        this.jerk = jerk;

        return this;
    }

    public EnhancedTalonFXConfiguration setSupplyCurrentLimit(double supplyLimit) {
        this.supplyLimit = supplyLimit;

        return this;
    }

    public EnhancedTalonFX build() {
        return new EnhancedTalonFX(this);
    }
}
