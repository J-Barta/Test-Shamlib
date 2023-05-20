package frc.robot.ShamLib.motors;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenixpro.configs.*;
import com.ctre.phoenixpro.controls.MotionMagicVoltage;
import com.ctre.phoenixpro.controls.VelocityVoltage;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ShamLib.motors.EnhancedTalonFXConfiguration.InvertedBehavior;
import frc.robot.ShamLib.motors.EnhancedTalonFXConfiguration.NeutralBehavior;

import java.util.function.BooleanSupplier;

import static com.ctre.phoenixpro.signals.InvertedValue.*;
import static frc.robot.ShamLib.motors.EnhancedTalonFXConfiguration.NeutralBehavior.*;
import static frc.robot.ShamLib.motors.EnhancedTalonFXConfiguration.RunMode.*;
import static frc.robot.ShamLib.motors.EnhancedTalonFXConfiguration.TalonType.*;

public class EnhancedTalonFX implements Sendable {

    private double inputToOutputRatio;
    private double target; //In output units

    private final boolean pro; //Whether the motor is pro or not
    private final EnhancedTalonFXConfiguration.RunMode runMode;

    private final TalonFX proMotorController;
    private final WPI_TalonFX v5MotorController;

    private final PIDSVGains proControlGains;
    private final PIDFGains v5ControlGains;

    public EnhancedTalonFX(EnhancedTalonFXConfiguration config) {
        this.pro = config.type == PRO;
        this.inputToOutputRatio = config.inputToOutputRatio;

        this.proControlGains = config.proControlGains.copy();
        this.v5ControlGains = config.v5ControlGains.copy();

        runMode = config.runMode;


        if(pro) {
            //Create a phoenix pro talonFX controller
            proMotorController = new TalonFX(config.deviceNumber, config.canbus);
            v5MotorController = null;

            //Configure the neutral power behavior and inversion of the pro controller
            configurePro(config.neutralBehavior, config.invertedBehavior);

            if (config.supplyLimit > 0) applyProCurrentLimit(config.supplyLimit);

            if(config.runMode == MOTION_MAGIC) {
                TalonFXConfiguration motorConfig = new TalonFXConfiguration();

                proMotorController.getConfigurator().refresh(motorConfig);

                motorConfig.Slot0 = getProConfigs(config.proControlGains);

                MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();

                motionMagicConfigs.MotionMagicAcceleration =    outputToDefaultUnits(config.maxAccel);
                motionMagicConfigs.MotionMagicCruiseVelocity =  outputToDefaultUnits(config.maxVel);
                motionMagicConfigs.MotionMagicJerk =            outputToDefaultUnits(config.jerk);

                motorConfig.MotionMagic = motionMagicConfigs;

                proMotorController.getConfigurator().apply(motorConfig);

            } else if(config.runMode == VELOCITY) {
                TalonFXConfiguration motorConfig = new TalonFXConfiguration();

                motorConfig.Slot0 = getProConfigs(config.proControlGains);

                proMotorController.getConfigurator().apply(motorConfig);
            }

        } else {
            //Create a v5 talonFX controller
            proMotorController = null;
            v5MotorController = new WPI_TalonFX(config.deviceNumber, config.canbus);

            //Configure the neutral power behavior and inversion of the v5 controller
            configureV5(config.neutralBehavior, config.invertedBehavior);

            //Account for ticks to rotations with phoenix v5
            inputToOutputRatio /= 2048.0;

            if (config.supplyLimit > 0) applyV5CurrentLimit(config.supplyLimit);

            //30 ms timeout for failure
            int timeout = 30;

            if(config.runMode == MOTION_MAGIC) {
                v5MotorController.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, timeout);

                v5MotorController.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, timeout);
                v5MotorController.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, timeout);

                v5MotorController.configNominalOutputForward(0, timeout);
                v5MotorController.configNominalOutputReverse(0, timeout);
                v5MotorController.configPeakOutputForward(1, timeout);
                v5MotorController.configPeakOutputReverse(-1, timeout);

                applyV5ControlGains(config.v5ControlGains, timeout);

                //Set motion magic velo and accel
                v5MotorController.configMotionCruiseVelocity(
                        outputToDefaultUnits(config.maxVel) / 10.0, timeout);
                v5MotorController.configMotionAcceleration(
                        outputToDefaultUnits(config.maxAccel) / 10.0, timeout);

            } else if(config.runMode == VELOCITY) {
                v5MotorController.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, timeout);

                v5MotorController.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, timeout);

                v5MotorController.configNominalOutputForward(0, timeout);
                v5MotorController.configNominalOutputReverse(0, timeout);
                v5MotorController.configPeakOutputForward(1, timeout);
                v5MotorController.configPeakOutputReverse(-1, timeout);

                applyV5ControlGains(config.v5ControlGains, timeout);
            }
        }
    }

    /*
    GETTERS
     */

    /**
     * Get the position of the encoder (in output units)
     * @return output units
     */
    public double getEncoderPosition() {
        return defaultUnitsToOutput(
                pro ?
                    proMotorController.getPosition().getValue()
                    : v5MotorController.getSelectedSensorPosition()
        );
    }

    public double getRawPosition() {
        return pro ? proMotorController.getRotorPosition().getValue() :
                v5MotorController.getSelectedSensorPosition();
    }


    /**
     * Get the velocity of the motor (in output units / second)
     * @return output units / sec
     */
    public double getEncoderVelocity() {
        return defaultUnitsToOutput(
                pro ?
                    proMotorController.getRotorVelocity().getValue()
                    : v5MotorController.getSelectedSensorVelocity() * 10 //Account for velo / 100ms
        );
    }

    /**
     * Get the velocity of the motor in motor rotations / sec
     * @return the raw velo of the motor
     */
    public double getRawVelo() {
        return pro ? proMotorController.getRotorVelocity().getValue()
            : v5MotorController.getSelectedSensorVelocity() * 10 / 2048;
    }

    public TalonFX getProMotorController() {
        return proMotorController;
    }

    public WPI_TalonFX getV5MotorController() {
        return v5MotorController;
    }


    /**
     * Converts motor default units to output units
     * @param motorDefault encoder ticks
     * @return output units
     */
    public double defaultUnitsToOutput(double motorDefault) {
        return motorDefault * inputToOutputRatio;
    }

    /**
     * Converts some number of output units into default motor units
     * @param output output units
     * @return encoder ticks
     */
    public double outputToDefaultUnits(double output) {
        return output / inputToOutputRatio;
    }

    /**
     * Get the current target of the motor (in output units)
     * @return target
     */
    public double getTarget() {
        return target;
    }

    /**
     * Whether the motor is running as a pro motor controller
     * @return whether the motor is running on phoenix pro
     */
    public boolean isPro() {
        return pro;
    }

    /**
     * Get the current run mode of the motor (velocity, motion magic, manual power
     * @return the current run mode of the motor
     */
    public EnhancedTalonFXConfiguration.RunMode getRunMode() {
        return runMode;
    }

    /*
    ACTIONS
     */

    /**
     * Set the target of the motor
     * This will automatically interpret between velocity (in output units / sec) and motion magic (output units)
     * @param target target (in output units)
     */
    public void setTarget(double target) {
        this.target = target;

        if(pro) {
            if(runMode == VELOCITY) {
                proMotorController.setControl(new VelocityVoltage(outputToDefaultUnits(target)).withSlot(0));
            } else if(runMode == MOTION_MAGIC) {
                proMotorController.setControl(new MotionMagicVoltage(outputToDefaultUnits(target)).withSlot(0));
            }
        } else {
            if(runMode == VELOCITY) {
                //Divided by 10 to account for ticks/100ms on V5
                v5MotorController.set(TalonFXControlMode.Velocity, outputToDefaultUnits(target) / 10.0);
            } else if(runMode == MOTION_MAGIC) {
                v5MotorController.set(TalonFXControlMode.MotionMagic, outputToDefaultUnits(target));
            }
        }

    }

    /**
     * Set a percentage power to the motor
     * @param power decimal from -1 to 1 to set the motor's power to
     */
    public void setManualPower(double power) {
        if(pro) {
            proMotorController.set(power);
        } else {
            v5MotorController.set(power);
        }
    }

    /**
     * Set the raw voltage of the motor
     * @param voltage the voltage to set the motor to
     */
    public void setVoltage(double voltage) {
        if(pro) {
            proMotorController.setVoltage(voltage);
        } else {
            v5MotorController.setVoltage(voltage);
        }
    }

    /**
     * @param pos The position (in output units) to which the motor should be reset
     */
    public void resetPosition(double pos) {
        if(pro) {
            proMotorController.setRotorPosition(outputToDefaultUnits(pos));
        } else {
            v5MotorController.setSelectedSensorPosition(outputToDefaultUnits(pos));
        }
    }

    /**
     * Resets the motor back to zero
     */
    public void resetPosition() {
        resetPosition(0);
    }


    /*
    CONFIGURATION METHODS
     */

    private void configurePro(NeutralBehavior nb, InvertedBehavior ib) {
        MotorOutputConfigs config = new MotorOutputConfigs();
        proMotorController.getConfigurator().refresh(config);
        config.NeutralMode = nb == COAST ? NeutralModeValue.Coast : NeutralModeValue.Brake;
        config.Inverted = ib == InvertedBehavior.CCWP ? CounterClockwise_Positive : Clockwise_Positive;
        proMotorController.getConfigurator().apply(config);
    }

    private void configureV5(NeutralBehavior nb, InvertedBehavior ib) {
        v5MotorController.setNeutralMode(nb == COAST ? NeutralMode.Coast : NeutralMode.Brake);
        v5MotorController.setInverted(ib == InvertedBehavior.CCWP ? TalonFXInvertType.CounterClockwise : TalonFXInvertType.Clockwise);
    }

    public void applyProCurrentLimit(double supplyLimit) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        proMotorController.getConfigurator().refresh(config);

        CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
        limits.SupplyCurrentLimitEnable = true; //Actually enforce the limits
        limits.SupplyCurrentLimit = supplyLimit;

        config.CurrentLimits = limits;
        proMotorController.getConfigurator().apply(config);
    }

    public void applyV5CurrentLimit(double supplyLimit) {
        v5MotorController.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(
                    true, //Actually enforce the limit
                    supplyLimit,
                    supplyLimit, //Trigger threshold
                    0.1 //Threshold time
            )
        );
    }

    /**
     * Update the max velocity, acceleration, and jerk
     * @param vel units/sec
     * @param accel units/sec^2
     * @param jerk units/sec^3
     */
    public void changeSpeed(double vel, double accel, double jerk) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        proMotorController.getConfigurator().refresh(config);

        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();

        motionMagicConfigs.MotionMagicAcceleration =    outputToDefaultUnits(accel);
        motionMagicConfigs.MotionMagicCruiseVelocity =  outputToDefaultUnits(vel);
        motionMagicConfigs.MotionMagicJerk =            outputToDefaultUnits(jerk);

        config.MotionMagic = motionMagicConfigs;

        proMotorController.getConfigurator().apply(config);
    }

    /**
     * Configure the motors PIDF loop for motion magic
     * @param gains PIDF gains
     */
    private Slot0Configs getProConfigs(PIDSVGains gains) {
        //Set the motion magic gains in slot0
        Slot0Configs pidConfigs = new Slot0Configs();
        pidConfigs.kS = gains.getS();
        pidConfigs.kV = gains.getV();
        pidConfigs.kP = gains.getP();
        pidConfigs.kI = gains.getI();
        pidConfigs.kD = gains.getD();

        return pidConfigs;
    }

    public void applyV5ControlGains(PIDFGains gains, int timeout) {
        //Set the motion magic gains in slot0
        v5MotorController.selectProfileSlot(0, 0);
        v5MotorController.config_kF(0, gains.getF(), timeout);
        v5MotorController.config_kP(0, gains.getP(), timeout);
        v5MotorController.config_kI(0, gains.getI(), timeout);
        v5MotorController.config_kD(0, gains.getD(), timeout);

    }

    /*
    CHARACTERIZATION METHODS
     */

    public Command calculateProKS(BooleanSupplier interrupt) {
        return new ConditionalCommand(
                new CalculateKSCommand(this, interrupt),
                new InstantCommand(),
                () -> this.pro
        );
    }

    public Command calculateProKV(Trigger increment, Trigger reverse, BooleanSupplier interrupt, boolean telemetry) {
        return new ConditionalCommand(
                new CalculateKVCommand(this, proControlGains.getS(), increment, reverse, interrupt, telemetry),
                new InstantCommand(),
                () -> this.pro
        );
    }

    public Command calculateProKV(Trigger increment, Trigger reverse, BooleanSupplier interrupt) {
        return calculateProKV(increment, reverse, interrupt, true);
    }

    public Command calculateV5KF(double power, BooleanSupplier interrupt) {
        return new ConditionalCommand(
            new ParallelRaceGroup(
                    new SequentialCommandGroup(
                            new WaitUntilCommand(interrupt),
                            new InstantCommand(() -> this.setManualPower(0)),
                            new InstantCommand(() -> {
                                System.out.println("Interrupted V5 KF calculation");
                            })
                    ),
                    new SequentialCommandGroup(
                            new InstantCommand(() -> this.setManualPower(power)),
                            new WaitCommand(1),
                            new InstantCommand(() -> {
                                System.out.println("Calculated V5 KF: " + (power * 1023) / v5MotorController.getSelectedSensorVelocity());
                                setManualPower(0);
                            })
                    )
            ),
            new InstantCommand(),
            () -> !this.pro
        );

    }

    /*
    Sendable and logging stuff
     */

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("pro", this::isPro, null);
        builder.addStringProperty("run-mode", () -> getRunMode().name(), null);
        builder.addDoubleProperty("target", this::getTarget, null);
        builder.addDoubleProperty("position", this::getEncoderPosition, null);
        builder.addDoubleProperty("velo", this::getEncoderVelocity, null);
    }

}
