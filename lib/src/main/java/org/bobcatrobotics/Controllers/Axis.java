package org.bobcatrobotics.Controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class Axis {
    public double input;
    public double inputDeadband;
    public double inputLimited;
    private SlewRateLimiter limiter = new SlewRateLimiter(3.0);
    public Axis(double input, double stickDeadband){
        this.input = input;
        this.inputDeadband = MathUtil.applyDeadband(this.input,stickDeadband);
        this.limiter = new SlewRateLimiter(3.0);
        this.inputLimited = limiter.calculate(this.input);
    }
    public double getDeadband(){
        return inputDeadband;
    }
    public double getLimited(){
        return inputLimited;
    }
    public double getInput(){
        return input;
    }
}

