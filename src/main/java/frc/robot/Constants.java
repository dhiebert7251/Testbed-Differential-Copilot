// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriverConstants {
    public static final int kFrontLeftMotorPort = 1;
    public static final int kFrontRightMotorPort = 2;
    public static final int kBackLeftMotorPort = 3;
    public static final int kBackRightMotorPort = 4;

    public static final int kFrontLeftEncoderChannelA = 0;
    public static final int kFrontLeftEncoderChannelB = 1;
    public static final int kFrontRightEncoderChannelA = 2;
    public static final int kFrontRightEncoderChannelB = 3;
    public static final int kBackLeftEncoderChannelA = 4;
    public static final int kBackLeftEncoderChannelB = 5;
    public static final int kBackRightEncoderChannelA = 6;
    public static final int kBackRightEncoderChannelB = 7;

    public static final boolean kFrontLeftEncoderReversed = false;
    public static final boolean kFrontRightEncoderReversed = false;
    public static final boolean kBackLeftEncoderReversed = false;
    public static final boolean kBackRightEncoderReversed = false;



  }
}
