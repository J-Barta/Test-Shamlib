package frc.robot.ShamLib.motors;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

public class CalculateKVCommand extends CommandBase {

    private EnhancedTalonFX motor;
    private double kS;
    private double voltageIncrement;
    private Trigger increment;
    private Trigger reverse;
    private BooleanSupplier interrupt;
    private boolean telemetry;

    private int currentMultiple;
    private boolean invert;

    private List<Double> voltages;
    private List<Double> velos;

    /**
     * Calculates the kV for a phoenix pro TalonFX
     * @param motor
     * @param kS
     * @param voltageIncrement
     * @param increment
     * @param reverse
     * @param interrupt
     * @param telemetry
     */
    public CalculateKVCommand(
            EnhancedTalonFX motor,
            double kS,
            double voltageIncrement,
            Trigger increment,
            Trigger reverse,
            BooleanSupplier interrupt,
            boolean telemetry) {
        this.motor = motor;
        this.kS = kS;
        this.voltageIncrement = voltageIncrement;
        this.increment = increment;
        this.reverse = reverse;
        this.interrupt = interrupt;
        this.telemetry = telemetry;

        increment.and(this::isScheduled).onTrue(new InstantCommand(() -> currentMultiple++));
        reverse.and(this::isScheduled).onTrue(new InstantCommand(() -> invert = !invert));
    }

    public CalculateKVCommand(EnhancedTalonFX motor, double kS, Trigger increment, Trigger reverse, BooleanSupplier interrupt, boolean telemetry) {
        this(motor, kS, 0.05, increment, reverse, interrupt, telemetry);
    }

    @Override
    public void initialize() {
        currentMultiple = 0;
        invert = false;

        voltages = new ArrayList<>();
        velos = new ArrayList<>();

        System.out.println("Beginning kV calculation");
    }

    @Override
    public void execute() {
        double voltage = kS + currentMultiple * voltageIncrement;

        if(invert) voltage *= -1;

        motor.setVoltage(voltage);

        voltages.add(Math.abs(voltage) - kS);
        velos.add(motor.getEncoderVelocity());
    }

    @Override
    public void end(boolean interrupted) {

        double slope = RegressionUtil.getLinearM(voltages, velos);
        double r = RegressionUtil.getRValue(voltages, velos);

        double kV = 1.0 / slope;

        System.out.println("kV calculated as: " + kV);
        System.out.println("kV r-value: " + r);
    }

    @Override
    public boolean isFinished() {
        return interrupt.getAsBoolean();
    }
}
