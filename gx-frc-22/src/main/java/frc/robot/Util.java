package frc.robot;

public class Util {
    public static double constrain(double val, double min, double max) {
        if(val < min) val = min;
        if(val > max) val = max;
        return val;
    }
}