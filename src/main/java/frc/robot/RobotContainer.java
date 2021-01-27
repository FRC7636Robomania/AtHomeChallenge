// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Path;
import frc.robot.commands.auto.TrajectoryCommand;
import frc.robot.commands.auto.TrajectoryCommand.OutputMode;
import frc.robot.subsystems.chassis.AndymarkChassis;
import frc.robot.subsystems.chassis.trajectory.TrajectoryDrivetrain;
import frc.robot.subsystems.chassis.trajectory.TrajectoryFactory;
import frc.robot.subsystems.simulation.AMSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static boolean isSimulation = true;
  public static boolean isAndymark   = false;
  AMSim andymarkSim = null;
  AndymarkChassis chassis = null;
  TrajectoryDrivetrain syzygy = null;
  SendableChooser<Trajectory> chooser = null;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if(isAndymark && isSimulation){
      andymarkSim = new AMSim();
    }else if(isAndymark){
      chassis = new AndymarkChassis();
    }else{
      syzygy = new TrajectoryDrivetrain();
    }
    chooser = new SendableChooser<>();
    chooser.addOption("oneMeter", TrajectoryFactory.getTrajectory(Path.oneMeter));
    chooser.addOption("barrel", TrajectoryFactory.getTrajectory(Path.barrel));
    chooser.addOption("bounceRace", TrajectoryFactory.getTrajectory(Path.bounce));
    chooser.addOption("slalom", TrajectoryFactory.getTrajectory(Path.slalom));
    chooser.addOption("curve", TrajectoryFactory.getTrajectory(Path.curve));
    Shuffleboard.getTab("Auto").add(chooser);
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    SmartDashboard.putNumber("TotalTime", chooser.getSelected().getTotalTimeSeconds());
    if(isAndymark && isSimulation){
      return TrajectoryCommand.build(chooser.getSelected(), andymarkSim, OutputMode.VOLTAGE, andymarkSim);

    }else if(isAndymark){
      return TrajectoryCommand.build(chooser.getSelected(), chassis, OutputMode.VOLTAGE, chassis);
    }else{
      return TrajectoryCommand.build(chooser.getSelected(), syzygy, OutputMode.VOLTAGE, syzygy);
    }
  }
}
