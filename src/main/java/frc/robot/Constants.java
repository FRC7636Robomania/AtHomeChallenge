// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static double rotationPerPulse = 2048;

    public static double gearRatio = 5.7;
  
    public static double distantsPerPulse =
     Math.PI * 0.1524 / rotationPerPulse;
      //圓周長　／　分辨率（解析度）（一圈的脈衝數）
    public static final double kS = 1.07;
  
    public static final double kV = 0.365;
  
    public static final double kA = 0.008;

    public static final double kP = 1.0;
  
    public static class Motor{
        public static final int leftMaster    = 19;
        public static final int leftFollewer  = 18;
        public static final int rightMaster   = 21;
        public static final int rightFollower = 20;
        public static final double distancePerPulse = 0.1524 * Math.PI / 2048 / 9.7;
        public static final double wheelPitch = 0.7407;

        public static final boolean isRightMotorInvert = false;
        public static final boolean isLeftMotorInvert = true;
        public static final boolean isRightPhaseInvert = false;
        public static final boolean isLeftPhaseInvert = true;
    }
    public static class Path{
        public static final String slalom = "output/slalom.wpilib.json";
        public static final String bounce = "output/bounceRace.wpilib.json";
        public static final String barrel = "output/barrel.wpilib.json";
        public static final String oneMeter = "output/oneMeter.wpilib.json";
        public static final String curve = "output/curve.wpilib.json";
    }
}
