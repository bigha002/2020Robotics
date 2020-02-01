/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //52 53 are right, 54 55 left
    public static int[] cans = {52, 53, 54, 55};
    public static int[] leftEncoders = {0, 1};
    public static int[] rightEncoders = {2, 3};

    public static int gyroID = 21;

    public static double EncoderDistanceP = 7;
    public static double DriveTargetPID = 1;

    public static int LeftJoystick = 0;
    public static int RightJoystick = 1;

    public static double P = 1;
    public static double I = 0;
    public static double D = 0;

    public static double driveTolerance = 1;
}
