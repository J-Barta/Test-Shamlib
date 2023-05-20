package frc.robot.ShamLib.swerve.module;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ShamLib.ShamLibConstants;
import frc.robot.ShamLib.motors.EnhancedTalonFXConfiguration;
import frc.robot.ShamLib.motors.EnhancedTalonFX;

import static frc.robot.ShamLib.motors.EnhancedTalonFXConfiguration.NeutralBehavior.*;
import static frc.robot.ShamLib.motors.EnhancedTalonFXConfiguration.RunMode.*;
import static frc.robot.ShamLib.motors.EnhancedTalonFXConfiguration.TalonType.*;

public class SwerveModule implements Sendable{

    private final String moduleName;

    private final EnhancedTalonFX turnMotor;
    private final EnhancedTalonFX driveMotor;

    private final CANCoder turnEncoder;
    private final double encoderOffset;

    private final boolean extraTelemetry;

    private SwerveModuleState targetState;

    private double targetModuleAngle;

    private final Translation2d moduleOffset;

    public SwerveModule(SwerveModuleConfiguration config
    ) {
        this.moduleOffset = config.offset;
        
        this.moduleName = config.name;

        this.extraTelemetry = config.extraTelemetry;

        this.turnEncoder = new CANCoder(config.encoderID, config.canbus);
        turnEncoder.configFactoryDefault();

        turnEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        turnEncoder.configSensorDirection(false);

        this.encoderOffset = config.encoderOffset;

        turnMotor = new EnhancedTalonFXConfiguration()
                .setDeviceNumber(config.turnMotorID)
                .setCanbus(config.canbus)
                .setType(PRO)
                .setRunMode(MOTION_MAGIC)
                .setProControlGains(config.turnGains)
                .setInputToOutputRatio(config.getTurnRatio())
                .setMaxVel(config.maxTurnVelo)
                .setMaxAccel(config.maxTurnAccel)
                .setJerk(config.turnJerk)
                .setInvertedBehavior(config.turnInverted)
                .setSupplyCurrentLimit(config.currentLimit)
                .build();

        driveMotor = new EnhancedTalonFXConfiguration()
                .setDeviceNumber(config.driveMotorID)
                .setCanbus(config.canbus)
                .setType(PRO)
                .setRunMode(VELOCITY)
                .setProControlGains(config.driveGains)
                .setInputToOutputRatio(config.getDriveRatio())
                .setSupplyCurrentLimit(config.currentLimit)
                .setInvertedBehavior(config.driveInverted)
                .setNeutralBehavior(BRAKE)
                .build();


        pullAbsoluteAngle();

        setDesiredState(
            new SwerveModuleState(0, getTurnAngle())
        );
    }


    private double normalizeDegrees(double degrees) {
        return Math.IEEEremainder(degrees, 360);
    }

    public void setDesiredState(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getTurnAngle());
        targetState = optimizedState;        
        double turnPos = turnMotor.getEncoderPosition();
        targetModuleAngle = turnPos + (normalizeDegrees(optimizedState.angle.getDegrees() - normalizeDegrees(turnPos)));

        turnMotor.setTarget(targetModuleAngle);

        driveMotor.setTarget(targetState.speedMetersPerSecond);
    }

    public double getDriveMotorRate(){
        return driveMotor.getEncoderVelocity();
    } 

    public double getDriveMotorPosition() {
        return driveMotor.getEncoderPosition();
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(getDriveMotorRate(), getTurnAngle());
    }

    public SwerveModuleState getTargetState() {
        return targetState;
    }

    public SwerveModulePosition getCurrentPosition() {
        return new SwerveModulePosition(getDriveMotorPosition(), getTurnAngle());
    }

    public Translation2d getModuleOffset() {return moduleOffset;}

    public void stop() {
        setDesiredState(new SwerveModuleState(0.0, getTurnAngle()));
    }

    public String getModuleName() {
        return moduleName;
    }

    public Command calculateTurnKV(Trigger increment, BooleanSupplier interrupt) {
        return turnMotor.calculateProKV(increment, new Trigger(() -> false), interrupt, false);
    }

    public Command calculateDriveKV(Trigger increment, Trigger invert, BooleanSupplier interrupt, boolean telemetry) {
        return driveMotor.calculateProKV(increment, invert, interrupt, telemetry);
    }

    public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromDegrees(normalizeDegrees(turnEncoder.getAbsolutePosition() - encoderOffset));
    }

    public void pullAbsoluteAngle() {
        turnMotor.resetPosition(normalizeDegrees(turnEncoder.getAbsolutePosition() - encoderOffset));
    }

    public void resetAngle(Rotation2d angle) {
        turnMotor.resetPosition(angle.getDegrees());
    }

    public Rotation2d getTurnAngle(){
        return Rotation2d.fromDegrees(normalizeDegrees(turnMotor.getEncoderPosition()));
    }

    public double getTurnMotorVelo() {
        return turnMotor.getEncoderVelocity();
    }

    /**
     * @return the error of the motor position vs the absolute encoder position (in degrees)
     */
    public double getAbsoluteError() {
        return Math.abs(getAbsoluteAngle().minus(getTurnAngle()).getDegrees());
    }

    public boolean isModuleMisaligned() {
        return getAbsoluteError() > ShamLibConstants.Swerve.ALLOWED_MODULE_ERROR;
    }

    public Command realignModule() {
        return new RealignModuleCommand(this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Swerve Module");

        builder.addDoubleProperty("Angle", () -> getTurnAngle().getDegrees(), null);
        builder.addDoubleProperty("Angle (wrapped - encoder)", () -> normalizeDegrees(turnEncoder.getAbsolutePosition() - encoderOffset), null);
        builder.addDoubleProperty("Raw Setpoint", driveMotor::getTarget, null);
        builder.addDoubleProperty("erorr", () -> Math.abs(turnMotor.getTarget() - turnMotor.getEncoderPosition()), null);
        builder.addDoubleProperty("Target Angle", () -> targetState.angle.getDegrees(), null);
        builder.addDoubleProperty("Velocity", this::getDriveMotorRate, null);
        builder.addDoubleProperty("Target Velocity", () -> targetState.speedMetersPerSecond, null);
        builder.addDoubleProperty("Velo error", () -> Math.abs(driveMotor.getTarget() - getDriveMotorRate()), null);

    }
    
}
