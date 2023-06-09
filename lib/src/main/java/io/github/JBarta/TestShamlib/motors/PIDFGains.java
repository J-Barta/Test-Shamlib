package io.github.JBarta.TestShamlib.motors;

//💪gains

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class PIDFGains implements Sendable {
	private double P;
	private double I;
	private double D;
	private double F;

	public PIDFGains(double _kP, double _kI, double _kD, double _kF){
		P = _kP;
		I = _kI;
		D = _kD;
		F = _kF;
	}

	public PIDFGains() {
		this(0,0,0,0);
	}

	public double getP() {
		return P;
	}

	public double getI() {
		return I;
	}

	public double getD() {
		return D;
	}

	public double getF() {
		return F;
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty("P", () -> P, (value) -> P = value);
		builder.addDoubleProperty("I", () -> I, (value) -> I = value);
		builder.addDoubleProperty("D", () -> D, (value) -> D = value);
		builder.addDoubleProperty("F", () -> F, (value) -> F = value);
	}

	/**
	 * Create a new copy of the current PIDF gains
	 * @return a new PIDF gains object with the same values
	 */
	public PIDFGains copy() {
		return new PIDFGains(
				P, I, D, F
		);
	}
}
