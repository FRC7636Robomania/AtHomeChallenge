// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulation;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.robot.Constants;
import frc.robot.subsystems.chassis.AndymarkChassis;

/** Add your docs here. */
public class AMSim extends AndymarkChassis {
    Field2d m_field = new Field2d();

    static EncoderSim lSim = new EncoderSim(lencoder);
    static EncoderSim rSim = new EncoderSim(rencoder);
    static ADXRS450_GyroSim gyroSim = new ADXRS450_GyroSim(ahrs);
    static DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
        // Create a linear system from our characterization gains.
        LinearSystemId.identifyDrivetrainSystem(0.296, 0.006, 0.287, 0.005),
        DCMotor.getCIM(2), // The track width is 0.7112 meters.
        0.7112, // 2 NEO motors on each side of the drivetrain.
        5.7,                       // 7.29:1 gearing reduction.
        Units.inchesToMeters(3), // The robot uses 3" radius wheels.

        // The standard deviations for measurement noise:
        // x and y:          0.001 m
        // heading:          0.001 rad
        // l and r velocity: 0.1   m/s
        // l and r position: 0.005 m
        VecBuilder.fill(0.0001, 0.0001, 0.001, 0.001, 0.001, 0.001, 0.001));

    public AMSim(){
        resetSensor();
        rencoder.reset();
        lencoder.reset();
        rencoder.setDistancePerPulse(Constants.distantsPerPulse);
        lencoder.setDistancePerPulse(Constants.distantsPerPulse);
        SmartDashboard.putData("Field", m_field);
    }
    @Override
    public void resetSensor() {
        lSim.resetData();
        rSim.resetData();
        lSim.setDistance(0);
        rSim.setDistance(0);
        gyroSim.setAngle(0);
        m_driveSim.setPose(new Pose2d());
    }
    @Override
    public void periodic() {
        // This will get the simulated sensor readings that we set
        // in the previous article while in simulation, but will use
        // real values on the robot itself.
        pose = odmetry.update(getHeading(),
                          getLeftPosition(),
                          getRightPosition());
        message();
        m_field.setRobotPose(odmetry.getPoseMeters());
      }
    @Override
    public void simulationPeriodic(){
        // Set the inputs to the system. Note that we need to convert
        // the [-1, 1] PWM signal to voltage by multiplying it by the
        // robot controller voltage.
        m_driveSim.setInputs(leftmotor.get() * 10,
                            -rightmotor.get() * 10);

        // Advance the model by 20 ms. Note that if you are running this
        // subsystem in a separate thread or have changed the nominal timestep
        // of TimedRobot, this value needs to match it.
        m_driveSim.update(0.02);

        // Update all of our sensors.
        lSim.setDistance(m_driveSim.getLeftPositionMeters() / Constants.distantsPerPulse);
        lSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond() / Constants.distantsPerPulse);
        rSim.setDistance(m_driveSim.getRightPositionMeters() / Constants.distantsPerPulse);
        rSim.setRate(m_driveSim.getRightVelocityMetersPerSecond() / Constants.distantsPerPulse);
        gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());

        SmartDashboard.putNumber("leftPosition_Sim", m_driveSim.getLeftPositionMeters());
        SmartDashboard.putNumber("rightPosition_Sim", m_driveSim.getRightPositionMeters());
        SmartDashboard.putNumber("gyro_Sim", -m_driveSim.getHeading().getDegrees());

    }

}