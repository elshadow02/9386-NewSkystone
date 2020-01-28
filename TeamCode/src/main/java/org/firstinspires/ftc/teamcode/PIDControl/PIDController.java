package org.firstinspires.ftc.teamcode.PIDControl;
import com.qualcomm.robotcore.util.Range;

/**
 *
 *    Created by Josh 10-12-2018
 *    Last edited by Ethan L. 1-28-2020
 *    Reference:
 *    Explanation https://www.ijedr.org/papers/IJEDR1402230.pdf
 *    Example of PID in action: https://www.youtube.com/watch?v=fusr9eTceEo
 *
 *        Base PID Class
 *    To Consider: getkX and setkX functions
 *
 */
public class PIDController {

    private double PLACEHOLDER = 100000;

    private double kP;

    private double kI;

    private double kD;

    private double integral;
    private double derivative;

    private double oldTime;
    private double oldError;

    private double minIntegral = -PLACEHOLDER;
    private double maxIntegral = PLACEHOLDER; // placeholder
    private double minOutput   = -1;
    private double maxOutput   = 1;

    public PIDController(double kProportional, double kIntegral, double kDerivative) {
        this.kP = kProportional;
        this.kI = kIntegral;
        this.kD = kDerivative;

        this.integral = 0;
        this.derivative = 0;
        this.oldTime = 0;
    }

    public void setkI(double kI) {
        this.kI = kI;
    }

    public void setkP(double kP) {
        this.kP = kP;
    }

    public void setkD(double kD) {
        this.kD = kD;
    }

    // Main calculation function
    // output = kP * error + kI * (integral + (error * deltaTime) + kD * (error - oldError)/deltaTime
    public double calculate(double error) {
        double time = getTime();
        double deltaT = time - this.oldTime;
        this.integral = Range.clip((this.integral + error) * deltaT * 0.5, this.minIntegral, this.maxIntegral);
        this.derivative = (error - this.oldError)/deltaT; // change in Y / change in X

        this.oldTime = time;
        this.oldError = error;

        double rawOutput = (this.kP * error) + (this.kI * this.integral) + (this.kD * this.derivative);
        return Range.clip(rawOutput, minOutput, maxOutput);
    }

    // GETTERS
    public double getIntegral() { return this.integral; }

    public double getDerivative() { return this.derivative; }

    public double getkP() { return this.kP; }

    public double getkI() { return this.kI; }

    public double getkD() { return this.kD; }

    private double getTime () {
        return System.nanoTime()/1e-9;
    }

    // SETTERS
    public void setMaxIntegral(double minMax) {
        minMax = Math.abs(minMax);
        this.minIntegral = -minMax;
        this.maxIntegral = minMax;
    }

    public void setOutputClip(double minMax) {
        this.minOutput = -minMax;
        this.maxOutput = minMax;
    }

    public void setOutputClip(double min, double max) {
        this.minOutput = min;
        this.maxOutput = max;
    }
}
