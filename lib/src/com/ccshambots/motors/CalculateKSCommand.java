package frc.robot.ShamLib.motors;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;

import static frc.robot.ShamLib.ShamLibConstants.Motor.*;

public class CalculateKSCommand extends CommandBase {

    private final EnhancedTalonFX motor;
    private final DoubleConsumer outputConsumer;
    private final double thresholdVelo;
    private final double thresholdTime;
    private final double incrementVoltage;
    private final double incrementTimeSec;
    private final BooleanSupplier interrupt;

    private final Timer timer = new Timer();

    private boolean hitThresholdVelo; //Whether the sequence has hit the threshold velo at all
    private double hitThresholdTimestamp; //The timer reading when the motor hits the timestamp

    private double currentMultiple;

    /**
     * Command to calculate kS for a TalonFX motor
     * @param motor the motor to calculate kS for
     * @param outputConsumer a consumer for the found kS value
     * @param thresholdVelo the velocity to count as kS
     * @param thresholdTime the time the motor must remain above the threshold to count
     * @param incrementVoltage the voltage increment
     * @param incrementTimeSec the time between voltage increments
     * @param interrupt command interrupt
     */
    public CalculateKSCommand(
            EnhancedTalonFX motor,
            DoubleConsumer outputConsumer,
            double thresholdVelo,
            double thresholdTime,
            double incrementVoltage,
            double incrementTimeSec,
            BooleanSupplier interrupt) {
        this.motor = motor;
        this.outputConsumer = outputConsumer;
        this.thresholdVelo = thresholdVelo;
        this.thresholdTime = thresholdTime;
        this.incrementVoltage = incrementVoltage;
        this.incrementTimeSec = incrementTimeSec;
        this.interrupt = interrupt;
    }

    public CalculateKSCommand(EnhancedTalonFX motor, DoubleConsumer outputConsumer, BooleanSupplier interrupt) {
        this(motor, outputConsumer, KS_THRESHOLD_VELO, KS_THRESHOLD_TIME, KS_INCREMENT_VOLTAGE, KS_INCREMENT_TIME, interrupt);
    }

    public CalculateKSCommand(EnhancedTalonFX motor, BooleanSupplier interrupt) {
        this(motor, null, interrupt);
    }

    @Override
    public void initialize() {
        timer.restart();

        hitThresholdVelo = false;

        currentMultiple = 0;
    }

    @Override
    public void execute() {

        if(timer.get() > currentMultiple * incrementTimeSec) {
            motor.setVoltage(currentMultiple * incrementVoltage);
            currentMultiple++;
        }

        //Logic for hitting and "un-hitting" threshold velo
        if(!hitThresholdVelo) {

            if(motor.getRawVelo() >= thresholdVelo) {
                hitThresholdVelo = true;
                hitThresholdTimestamp = timer.get();
            }

        } else {
            if(motor.getRawVelo() < thresholdVelo) {
                hitThresholdVelo = false;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();

        double currentVoltage = currentMultiple * incrementVoltage;

        if(outputConsumer != null) {
            outputConsumer.accept(currentVoltage);
        }

        System.out.println("Found KS at: " + currentVoltage);

        motor.setVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return interrupt.getAsBoolean() || (hitThresholdVelo && timer.get() - hitThresholdTimestamp >= thresholdTime);
    }
}
