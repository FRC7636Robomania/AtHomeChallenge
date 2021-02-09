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
    public static final String visionName = "RaspberryPi";

    public static final double kS = 0.622;//0.35;
  
    public static final double kV = 0.000526;//0.0054;
  
    public static final double kA = 0.000124;//0.00065;

    public static final double kP = 3.5;

    public static final double kD = 0.000;

    public static final double b = 2;

    public static final double zeta = 0.7;
    public static boolean isBall = false;

    public static class Motor{
        public static final int leftMaster    = 20;
        public static final int leftFollewer  = 21;
        public static final int rightMaster   = 18;
        public static final int rightFollower = 19;
        public static final double distancePerPulse = 0.1524 * Math.PI / 2048 / 9.64;
        public static final double wheelPitch = 0.65;

        public static final boolean isRightMotorInvert = true;
        public static final boolean isLeftMotorInvert = false;
        public static final boolean isRightPhaseInvert = true;
        public static final boolean isLeftPhaseInvert = false;
    }
    public static class Path{
        public static final String[] slalom = {"output/slalom1.wpilib.json"};
        public static final String[] bounce = {"output/bounceRace.wpilib.json"};
        public static final String[] barrel = {"output/barrel.wpilib.json"};
        public static final String[] oneMeter = {"output/oneMeter.wpilib.json"};
        public static final String[] curve = {"output/curve.wpilib.json"};
        public static final String[] A_Blue = {"output/GalacticA_Blue.wpilib.json"};
        public static final String[] A_Red = {"output/GalacticA_Red.wpilib.json"};
        public static final String[] B_Blue = {"output/GalacticB_Blue.wpilib.json"};
        public static final String[] B_Red = {"output/GalacticB_Red.wpilib.json"};
        public static final String[] find = {"output/findBlue.wpilib.json"};
    }
}
