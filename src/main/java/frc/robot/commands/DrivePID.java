/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
//package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
// import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrainPID;

/**
 * An example command that uses an example subsystem.
 */
public class DrivePID extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public DriveTrainPID dt;

  public double fwd;
  public double rot;
  //private final Drive m_subsystem;
  // haha noob
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DrivePID(DriveTrainPID d) {
    dt = d;
    addRequirements(dt);
    requires(dt);
    //requires(Robot.DriveTrain)
  }

  private void requires(DriveTrainPID dt2) {
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //dt.useOutput();
    dt.DriveRob(Robot.m_robotContainer.getLeft(), Robot.m_robotContainer.getRight());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
