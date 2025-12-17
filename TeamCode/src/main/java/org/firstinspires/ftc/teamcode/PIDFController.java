package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

public class PIDFController {
	private static final double EPS = 1e-9;

	private double kP, kI, kD;
	private double target = 0.0;
	private boolean enabled = true;

	private double integralSum = 0.0;
	private double lastError = 0.0;
	private double lastMeasurement = 0.0;

	private double filteredDerivative = 0.0;

	private double lastTime = 0.0;
	private boolean initialized = false;

	private double outputMin = -1.0;
	private double outputMax =  1.0;
	private double maxIntegralContribution = 1.0;

	private double derivativeTau = 0.02;

	public PIDFController(double kP, double kI, double kD) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
	}

	public static double time() {
		return System.nanoTime() / 1e9;
	}

	public void setTarget(double target, boolean resetIntegrator) {
		this.target = target;
		if (resetIntegrator) {
			reset();
		}
	}

	public void setTarget(double target) { setTarget(target, false); }


	public void setEnabled(boolean enabled) {
		this.enabled = enabled;
	}
	public boolean getEnabled() {
		return enabled;
	}
	public double getTarget() { return target; }

	public void setOutputLimits(double min, double max) {
		if (max <= min) throw new IllegalArgumentException("max must be > min");
		this.outputMin = min;
		this.outputMax = max;
	}

	public void setMaxIntegralContribution(double maxContribution) {
		this.maxIntegralContribution = Math.abs(maxContribution);
	}

	public void setDerivativeTau(double tauSeconds) {
		this.derivativeTau = Math.max(tauSeconds, 1e-9);
	}

	public void reset() {
		integralSum = 0.0;
		filteredDerivative = 0.0;
		lastError = 0.0;
		lastMeasurement = 0.0;
		initialized = false;
		lastTime = 0.0;
	}

	public double update(double measurement, double feedForward) {
		double now = time();

		if (!initialized) {
			lastTime = now;
			lastError = target - measurement;
			lastMeasurement = measurement;

			filteredDerivative = 0.0;
			initialized = true;

			double p = kP * lastError;
			return Range.clip(p + feedForward, outputMin, outputMax);
		}

		double dt = now - lastTime;

		if (dt < 1e-9) {
			dt = 1e-9;
		}

		double error = target - measurement;

		double pTerm = kP * error;

		double integralProposed = integralSum + 0.5 * (error + lastError) * dt;

		if (kI != 0.0) {
			double maxIntegral = maxIntegralContribution / Math.abs(kI);
			integralProposed = Range.clip(integralProposed, -maxIntegral, maxIntegral);
		}

		double rawDerivative = (lastMeasurement - measurement) / dt;

		double alpha = dt / (derivativeTau + dt);
		filteredDerivative = alpha * rawDerivative + (1.0 - alpha) * filteredDerivative;
		double dTerm = kD * filteredDerivative;

		double iTermProposed = kI * integralProposed;
		double unclipped = pTerm + iTermProposed + dTerm + feedForward;

		double clipped = Range.clip(unclipped, outputMin, outputMax);
		boolean wouldSaturate = Math.abs(unclipped - clipped) > EPS;

		if (!wouldSaturate) {
			integralSum = integralProposed;
		} else {

			double currentITerm = kI * integralSum;
			boolean reducesSaturation =
					(unclipped > outputMax && iTermProposed < currentITerm) ||
							(unclipped < outputMin && iTermProposed > currentITerm);

			if (reducesSaturation) {
				integralSum = integralProposed;
			}
		}

		double iTerm = kI * integralSum;
		double output = pTerm + iTerm + dTerm + feedForward;

		lastError = error;
		lastMeasurement = measurement;

		lastTime = now;

		return Range.clip(output, outputMin, outputMax);
	}
	public void setConstants(double kP, double kI, double kD) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
	}
}