/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
// import frc.robot.commands.Drive;
import frc.robot.commands.DrivePID;
// import frc.robot.subsystems.DriveTrain;
//import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.subsystems.DriveTrainPID;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // private final DriveTrain driveSubsystem = new DriveTrain();
  private final DriveTrainPID drivePIDSubsystem = new DriveTrainPID();

  public final Joystick rightStick = new Joystick(Constants.RightJoystick);
  public final Joystick leftStick = new Joystick(Constants.LeftJoystick);
  public final JoystickButton button1 = new JoystickButton(rightStick, 1);

  // private final Drive drive = new Drive(driveSubsystem);
  private final DrivePID drivePID = new DrivePID(drivePIDSubsystem);

  public Joystick getLeft(){
    return leftStick;
  }
  public Joystick getRight(){
    return rightStick;
  }
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    //driveSubsystem.setDefaultCommand(drive);
    drivePIDSubsystem.setDefaultCommand(drivePID);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
