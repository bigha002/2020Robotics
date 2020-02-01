/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.PIDSubsystem;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
// import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

public class DriveTrainPID extends PIDSubsystem {
  /**
   * Creates a new ExampleSubsystem.
   */
  private int[] cans = Constants.cans;

  public Timer timer = new Timer();

  public CANSparkMax can = new CANSparkMax(cans[0], MotorType.kBrushless);
  public CANSparkMax can2 = new CANSparkMax(cans[1], MotorType.kBrushless);
  public CANSparkMax can3 = new CANSparkMax(cans[2], MotorType.kBrushless);
  public CANSparkMax can4 = new CANSparkMax(cans[3], MotorType.kBrushless);

  // public TalonSRX talon = new TalonSRX(4);
  public PigeonIMU gyro = new PigeonIMU(Constants.gyroID);

  //public final CANEncoder leftEncoder = new CANEncoder(Constants.leftEncoders[0], Constants.leftEncoders[1]);
  public final CANEncoder leftEncoder = new CANEncoder(can);
  public final CANEncoder rightEncoder = new CANEncoder(can3);

  public SpeedControllerGroup leftControllerGroup = new SpeedControllerGroup(can, can2);;
  public SpeedControllerGroup rightControllerGroup = new SpeedControllerGroup(can3, can4);;

  public DifferentialDrive differentialDrive = new DifferentialDrive(leftControllerGroup, rightControllerGroup);

  public Joystick left;

  public double rightStickTime = 0;

  public DriveTrainPID() {
    super(new PIDController(Constants.P, Constants.I, Constants.D));
    getController().setTolerance(Constants.driveTolerance);
    //leftEncoder.setDistancePerPulse(Constants.EncoderDistancePP);
    //leftEncoder.
    //rightEncoder.setDistancePerPulse(Constants.EncoderDistancePP);
    setSetpoint(Constants.DriveTargetPID);

    timer.start();
  }

  public void DriveRob(Joystick leftStick, Joystick rightStick) {
    SmartDashboard.putNumber("Left Stick", leftStick.getY());
    SmartDashboard.putNumber("Right Stick", rightStick.getX());
    SmartDashboard.putNumber("Right Encoder", rightEncoder.getVelocity());
    SmartDashboard.putNumber("Left Encoder", leftEncoder.getVelocity());
    SmartDashboard.putNumber("RightMotor", rightControllerGroup.get());
    SmartDashboard.putNumber("LeftMotor", leftControllerGroup.get());

    double[] ypr = new double[3];
    gyro.getYawPitchRoll(ypr);
    SmartDashboard.putNumber("Gyro Yaw", ypr[0]);
    SmartDashboard.putNumber("Gyro Pitch", ypr[1]);
    SmartDashboard.putNumber("Gyro Roll", ypr[2]);
    // PigeonIMU.GeneralStatus genStatus = new PigeonIMU.GeneralStatus();
    // gyro.GetGeneralStatus(genStatus);

    SmartDashboard.putNumber("Time", timer.get());
    SmartDashboard.putNumber("Cooldown", timer.get() - rightStickTime);

    differentialDrive.arcadeDrive(-rightStick.getX(), -leftStick.getY());

    if((Math.abs(rightStick.getX()) <= .1) == false) {
      rightStickTime = timer.get();
    }

    if((Math.abs(rightStick.getX()) <= .1) == true && timer.get() - rightStickTime >= .5 && Math.abs(leftEncoder.getVelocity() - rightEncoder.getVelocity()) >= 20) {
      //rightStickTime = timer.get();
      final double leftVel = Math.abs(leftEncoder.getVelocity());
      final double rightVel = Math.abs(rightEncoder.getVelocity());
      final double difference = rightVel - leftVel;

      SmartDashboard.putNumber("PID Output", getController().calculate(difference));

      if(difference <= 0) {
        //Left side is moving faster
        SmartDashboard.putNumber("Right Encoder", rightEncoder.getVelocity());
        setSetpoint(leftEncoder.getVelocity());
        SmartDashboard.putNumber("PID Output", getController().calculate(rightEncoder.getVelocity()));
        //leftControllerGroup.set(leftControllerGroup.get() * .98f);
      } else{
        //Right side is moving faster
        SmartDashboard.putNumber("Left Encoder", leftEncoder.getVelocity());
        setSetpoint(rightEncoder.getVelocity());
        SmartDashboard.putNumber("PID Output", getController().calculate(leftEncoder.getVelocity()));
        //rightControllerGroup.set(rightControllerGroup.get() * .98f);
      }
    } else {
      differentialDrive.arcadeDrive(-rightStick.getX(), -leftStick.getY());
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    // TODO Auto-generated method stub
    SmartDashboard.putNumber("PID Output", output);
    SmartDashboard.putNumber("PID Setpoint", setpoint);
    //differentialDrive.arcadeDrive(.5 + 1 * setpoint, .5 - 1 * setpoint);
  }

  @Override
  protected double getMeasurement() {
    // TODO Auto-generated method stub
    return 0;
  }
}
