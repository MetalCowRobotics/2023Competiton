package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Library {
    public static void pushDashboard(String varName, String varValue, boolean debug) {
        if (debug) {
            SmartDashboard.putString(varName, varValue);
        }
    }

    public static void pushDashboard(String varName, double varValue, boolean debug) {
        if (debug) {
            SmartDashboard.putNumber(varName, varValue);
        }
    }


    public static void pushDashboard(String varName, Boolean varValue, boolean debug) {
        if (debug) {
            SmartDashboard.putBoolean(varName, varValue);
        }
    }

    public static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
          return value;
        } else {
          return 0.0;
        }
      }
}
