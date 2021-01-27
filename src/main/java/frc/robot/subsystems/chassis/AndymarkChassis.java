// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.chassis;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.chassis.trajectory.TrajectorySystem;

/** Add your docs here. */
public class AndymarkChassis extends SubsystemBase implements TrajectorySystem {
    protected WPI_TalonSRX rightmotor = new WPI_TalonSRX(0);
    protected WPI_VictorSPX rightmotorS = new WPI_VictorSPX(7);
  
    protected WPI_TalonSRX leftmotor = new WPI_TalonSRX(9);
    protected WPI_VictorSPX leftmotorS = new WPI_VictorSPX(1);  
  
    protected static ADXRS450_Gyro ahrs = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
  
    protected DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.7);      // 輪子寬度
    protected DifferentialDriveOdometry odmetry = new DifferentialDriveOdometry(getHeading());
  
    protected static Encoder lencoder = new Encoder(0, 1);
    protected static Encoder rencoder = new Encoder(2, 3, true);
  
    protected Pose2d pose = new Pose2d();
  
    SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA);
  
    PIDController lpidcontroller = new PIDController(
        Constants.kP, 0, 0);
    PIDController rpidcontroller = new PIDController(
        Constants.kP, 0, 0);
  
    public AndymarkChassis(){
      setmotor();
      resetSensor();
      rencoder.reset();
      lencoder.reset();
      rencoder.setDistancePerPulse(Constants.distantsPerPulse);
      lencoder.setDistancePerPulse(Constants.distantsPerPulse);
    }

    public void setmotor(){
      rightmotor.configFactoryDefault();
      rightmotorS.configFactoryDefault();
      leftmotor.configFactoryDefault();
      leftmotorS.configFactoryDefault();
  
      rightmotorS.follow(rightmotor);
      leftmotorS.follow(leftmotor);
    
      rightmotor.setInverted(true);
      leftmotor.setInverted(false);

      rightmotorS.setInverted(InvertType.FollowMaster);
      leftmotorS.setInverted(InvertType.FollowMaster);
  
      rencoder.setReverseDirection(false);
  
    }
    @Override
    public Rotation2d getHeading() {
      return Rotation2d.fromDegrees(-ahrs.getAngle());
    }
    @Override
    public DifferentialDriveWheelSpeeds getSpeed() {
      SmartDashboard.putNumber("leftRate", getLeftVelocity());
      SmartDashboard.putNumber("rightRate", getRightVelocity());
      return new DifferentialDriveWheelSpeeds(
        getLeftVelocity(), 
        getRightVelocity()
        );
    
    }
    @Override
    public SimpleMotorFeedforward getFeedforward() {
      return feedForward;
    }
    @Override 
    public PIDController getLeftPidController() {
      return lpidcontroller;
    }
    @Override
    public PIDController getRightPidController() {
      return rpidcontroller;
    }
    @Override
    public  DifferentialDriveKinematics getKinematics() {
      return kinematics;
    }

    @Override
    public void voltage(double left, double right){
      leftmotor.set(ControlMode.PercentOutput, left / 10);
      rightmotor.set(ControlMode.PercentOutput, right / 10);
      SmartDashboard.putNumber("leftOutput ", left / 10);
      SmartDashboard.putNumber("rightOutput", right / 10);
      // leftmotor.set(ControlMode.PercentOutput, 0);
      // rightmotor.set(ControlMode.PercentOutput, 0);
      // SmartDashboard.putNumber("leftOutput ", 0);
      // SmartDashboard.putNumber("rightOutput", 0);
    }
    @Override
    public double getX(){
      return odmetry.getPoseMeters().getTranslation().getX();
    }
    @Override
    public double getY(){
      return odmetry.getPoseMeters().getTranslation().getY();
    }
    @Override
    public void message(){
      SmartDashboard.putNumber("x", getX());
      SmartDashboard.putNumber("Y", getY());
      //distants
      SmartDashboard.putNumber("leftDistants", getLeftPosition());
      SmartDashboard.putNumber("rightDistants", getRightPosition());
      SmartDashboard.putNumber("Yaw", ahrs.getAngle());
    }
 
    @Override
    public void periodic() {
      pose = odmetry.update(getHeading(), 
                            getLeftPosition(),
                            getRightPosition());
      message();
      
    }

    @Override
    public Pose2d getPose() {
        return pose;
    }

    @Override
    public void setOdmetry(Pose2d pose2d) {
        odmetry.resetPosition(pose2d, pose2d.getRotation());
    }

    @Override
    public double getLeftVelocity() {
        return lencoder.getRate();
    }

    @Override
    public double getLeftPosition() {
        return lencoder.getDistance();
    }

    @Override
    public double getRightVelocity() {
        return rencoder.getRate();
    }

    @Override
    public double getRightPosition() {
        return rencoder.getDistance();
    }

    @Override
    public void resetSensor() {
      ahrs.reset();
      rencoder.reset();
      lencoder.reset();
    }
}
