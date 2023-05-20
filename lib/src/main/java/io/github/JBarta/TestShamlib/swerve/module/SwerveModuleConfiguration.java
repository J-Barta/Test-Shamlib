package io.github.JBarta.TestShamlib.swerve.module;

import edu.wpi.first.math.geometry.Translation2d;
import io.github.JBarta.TestShamlib.motors.PIDSVGains;

import static java.lang.Math.PI;

public class SwerveModuleConfiguration {

    public String name = "";
    public String canbus = "";

    public double maxTurnVelo = 1;
    public double maxTurnAccel = 1;
    public double turnJerk = 10000;

    public double currentLimit = 0;

    public boolean extraTelemetry = false;

    public PIDSVGains driveGains = new PIDSVGains(), turnGains = new PIDSVGains();

    public int driveMotorID = 1;
    public int turnMotorID = 1;
    public int encoderID = 1;
    public SDSModuleType moduleType = SDSModuleType.MK4iL3;
    public double encoderOffset = 0;
    public Translation2d offset = new Translation2d();
    public boolean driveInverted = false;
    public boolean turnInverted = false;

    private static final double mk3TurnRatio  =
            (1.0 / 12.8) * //Output revs
            360 //Output degrees
    ;

    //Convert motor revs to output degrees
    private static final double mk4iTurnRatio =
        (7.0 / 150.0) * //Output revs
        360 //Output degrees
    ;

    // in meters
    private static final double mk4iWheelCircumference =
            2 * PI * 0.0508;

    //I believe mk3 wheels are the same as mk4
    private static final double mk3WheelCircumference =
            2 * PI * 0.0508;

    public SwerveModuleConfiguration() {}

    public SwerveModuleConfiguration setDriveMotorID(int driveMotorID) {
        this.driveMotorID = driveMotorID;

        return this;
    }

    public SwerveModuleConfiguration setTurnMotorID(int turnMotorID) {
        this.turnMotorID = turnMotorID;

        return this;
    }

    public SwerveModuleConfiguration setEncoderID(int encoderID) {
        this.encoderID = encoderID;

        return this;
    }

    public SwerveModuleConfiguration setModuleType(SDSModuleType moduleType) {
        this.moduleType = moduleType;

        return this;
    }

    public SwerveModuleConfiguration setEncoderOffset(double encoderOffset) {
        this.encoderOffset = encoderOffset;

        return this;
    }

    public SwerveModuleConfiguration setOffset(Translation2d offset) {
        this.offset = offset;

        return this;
    }

    public SwerveModuleConfiguration setDriveInverted(boolean driveInverted) {
        this.driveInverted = driveInverted;

        return this;
    }

    public SwerveModuleConfiguration setTurnInverted(boolean turnInverted) {
        this.turnInverted = turnInverted;

        return this;
    }

    public SwerveModuleConfiguration setName(String name) {
        this.name = name;

        return this;
    }

    public SwerveModuleConfiguration setCanbus(String canbus) {
        this.canbus = canbus;

        return this;
    }

    public SwerveModuleConfiguration setMaxTurnVelo(double maxTurnVelo) {
        this.maxTurnVelo = maxTurnVelo;

        return this;
    }

    public SwerveModuleConfiguration setMaxTurnAccel(double maxTurnAccel) {
        this.maxTurnAccel = maxTurnAccel;

        return this;
    }

    public SwerveModuleConfiguration setTurnJerk(double turnJerk) {
        this.turnJerk = turnJerk;

        return this;
    }

    public SwerveModuleConfiguration setCurrentLimit(double currentLimit) {
        this.currentLimit = currentLimit;

        return this;
    }

    public SwerveModuleConfiguration setExtraTelemetry(boolean extraTelemetry) {
        this.extraTelemetry = extraTelemetry;

        return this;
    }

    public SwerveModuleConfiguration setDriveGains(PIDSVGains driveGains) {
        this.driveGains = driveGains;

        return this;
    }

    public SwerveModuleConfiguration setTurnGains(PIDSVGains turnGains) {
        this.turnGains = turnGains;

        return this;
    }

    public double getDriveRatio() {
        return moduleType.driveRatio;
    }

    public double getTurnRatio() {
        return moduleType.turnRatio;
    }

    public SwerveModule build() {
        return new SwerveModule(this);
    }

    public SwerveModuleConfiguration copy() {
        return new SwerveModuleConfiguration(
                name, canbus, maxTurnVelo, maxTurnAccel, turnJerk,
                currentLimit, extraTelemetry, driveGains.copy(), turnGains.copy(),
                driveMotorID, turnMotorID, encoderID, moduleType, encoderOffset,
                offset, driveInverted, turnInverted
        );
    }

    public SwerveModuleConfiguration copyToNewModule(int driveMotorID,
                                                     int turnMotorID,
                                                     int encoderID,
                                                     double encoderOffset,
                                                     Translation2d offset) {
        return copy()
                .setDriveMotorID(driveMotorID)
                .setTurnMotorID(turnMotorID)
                .setEncoderID(encoderID)
                .setEncoderOffset(encoderOffset)
                .setOffset(offset);
    }

    //Private constructor for copying to a new object
    private SwerveModuleConfiguration(String name, String canbus, double maxTurnVelo, double maxTurnAccel,
                                      double turnJerk, double currentLimit, boolean extraTelemetry,
                                      PIDSVGains driveGains, PIDSVGains turnGains, int driveMotorID,
                                      int turnMotorID, int encoderID, SDSModuleType moduleType,
                                      double encoderOffset, Translation2d offset, boolean driveInverted,
                                      boolean turnInverted) {
        this.name = name;
        this.canbus = canbus;
        this.maxTurnVelo = maxTurnVelo;
        this.maxTurnAccel = maxTurnAccel;
        this.turnJerk = turnJerk;
        this.currentLimit = currentLimit;
        this.extraTelemetry = extraTelemetry;
        this.driveGains = driveGains;
        this.turnGains = turnGains;
        this.driveMotorID = driveMotorID;
        this.turnMotorID = turnMotorID;
        this.encoderID = encoderID;
        this.moduleType = moduleType;
        this.encoderOffset = encoderOffset;
        this.offset = offset;
        this.driveInverted = driveInverted;
        this.turnInverted = turnInverted;
    }

    public enum SDSModuleType {
        MK4iL1(1/8.14 * mk4iWheelCircumference, mk4iTurnRatio),
        MK4iL2(1/6.75 * mk4iWheelCircumference, mk4iTurnRatio),
        MK4iL3(1/6.12 * mk4iWheelCircumference, mk4iTurnRatio),
        MK3L1(1/8.16 * mk3WheelCircumference, mk3TurnRatio);

        double driveRatio;
        double turnRatio;

        SDSModuleType(double driveRatio, double turnRatio) {
            this.driveRatio = driveRatio;
            this.turnRatio = turnRatio;
        }
    }
}
