// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Path;
import frc.robot.commands.CheckBall;
import frc.robot.commands.auto.TrajectoryCommand;
import frc.robot.commands.auto.TrajectoryCommand.OutputMode;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.chassis.ControlDrivetrain;
import frc.robot.subsystems.chassis.trajectory.TrajectoryDrivetrain;
import frc.robot.subsystems.chassis.trajectory.TrajectoryFactory;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  Joystick joystick = new Joystick(0);
  TrajectoryDrivetrain syzygy = null;
  SendableChooser<String> chooser = null;
  private static String trajectoryName = "";
  private static String[] path;
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  ControlDrivetrain controlDrivetrain = new ControlDrivetrain();
  Arm arm = new Arm();
  Intake intake = new Intake();
  static boolean isG = false;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    syzygy = new TrajectoryDrivetrain();

    inst.getTable(Constants.visionName).getEntry("area").forceSetNumber(0.0);
    inst.getTable(Constants.visionName).getEntry("x").forceSetNumber(0.0);

    chooser = new SendableChooser<>();
    chooser.setDefaultOption("oneMeter", "one");
    chooser.addOption("barrel", "barrel");
    chooser.addOption("bounceRace", "bounce");
    chooser.addOption("slalom", "slalom");
    chooser.addOption("curve", "curve");
    chooser.addOption("Galactic", "G");
    chooser.addOption("A_R", "A_R");
    chooser.addOption("A_B", "A_B");
    chooser.addOption("B_R", "B_R");
    chooser.addOption("B_B", "B_B");
    chooser.addOption("find", "find");
    SmartDashboard.putData(chooser);
    configureButtonBindings();
    controlDrivetrain.setDefaultCommand(new RunCommand(
        () -> controlDrivetrain.curvatureDrive(joystick.getY() * 0.3, joystick.getZ() * 0.3, joystick.getTrigger()),
        controlDrivetrain));
  }

  public void out(){
    arm.out();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(joystick, 2).whenHeld(new InstantCommand(()->arm.in(), arm));
  }

  public static String getChoose() {
    return trajectoryName;
  }

  public void intakeSpin(){
    intake.intakeForward();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    trajectoryName = chooser.getSelected();
    process();
    Trajectory[] t;
    SmartDashboard.putString("PathName", trajectoryName);
    Command command = null;
    if (trajectoryName == Path.find[0]) {
      // SmartDashboard.putBoolean("getAuto", true);
      t = new Trajectory[1];
      t[0] = TrajectoryFactory.getTrajectory(Path.find[0]);
      command = TrajectoryCommand.build(syzygy, OutputMode.VOLTAGE, controlDrivetrain, t)
                  .andThen(new CheckBall(syzygy, intake));
    } else {
      t = new Trajectory[path.length];
      for (int i = 0; i < path.length; i++) {
        t[i] = TrajectoryFactory.getTrajectory(path[i]);
      }
      command = TrajectoryCommand.build(syzygy, OutputMode.VOLTAGE, controlDrivetrain, t);
    }
    if(isG){
      command = new InstantCommand(()->arm.out(), arm).andThen(command);
    }
    if(isG && trajectoryName != Path.find[0]){
      command = new InstantCommand(()->intake.forward())
                  .andThen(command)
                  .andThen(new InstantCommand(()->intake.stop()));
      isG = false;
    }
    return command;
  }

  public static void process() {
    switch (trajectoryName) {
      case "G":
        isG = true;
        if(Vision.getBall()){ // red
          SmartDashboard.putBoolean("ball?", true);
          if(Vision.isRight()){
            trajectoryName = Path.A_Red[0];
            path = Path.A_Red;
          }else{
            trajectoryName = Path.B_Red[0];
            path = Path.B_Red;
          }
        }else{ //blue
          SmartDashboard.putBoolean("ball?", false);
          trajectoryName = Path.find[0];
          path = Path.find;
        }
        SmartDashboard.putString("path", trajectoryName);
        return;
      case "slalom":
        path = Path.slalom;
        return;
      case "bounce":
        path = Path.bounce;
        return;
      case "barrel":
        path = Path.barrel;
        return;
      case "one" :
        path = Path.oneMeter;
        return;
      case "A_R" :
        path = Path.A_Red;
        return;
      case "A_B" :
        path = Path.A_Blue;
        return;
      case "B_R" :
        path = Path.B_Red;
        return;
      case "B_B" :
        path = Path.B_Blue;
        return;
      case "find" :
        path = Path.find;
        return;
      default : 
        return;
    }
  }
}
