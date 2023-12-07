// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/* Testbed notes
 * Front motor controllers:  Talon SRX
 * Rear motor controllers:   Victor SPX
 * 
 * Gearboxes:  MicroToughbox 12.75:1
 * 
 * Encoders:  CimCoder
 *            20 pulses per revolution/channel
 * 
 * 
 *Wheel diameter:
  * Pneumatic: 8"
  */


public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private WPI_TalonSRX m_FrontLeftMotor = new WPI_TalonSRX(Constants.DriverConstants.kFrontLeftMotorPort);
  private WPI_TalonSRX m_FrontRightMotor = new WPI_TalonSRX(Constants.DriverConstants.kFrontRightMotorPort);

  private WPI_VictorSPX m_BackLeftMotor = new WPI_VictorSPX(Constants.DriverConstants.kBackLeftMotorPort);
  private WPI_VictorSPX m_BackRightMotor = new WPI_VictorSPX(Constants.DriverConstants.kBackRightMotorPort);

  private Encoder m_FrontLeftEncoder = new Encoder(Constants.DriverConstants.kFrontLeftEncoderChannelA, 
                                                 Constants.DriverConstants.kFrontLeftEncoderChannelB, 
                                                 Constants.DriverConstants.kFrontLeftEncoderReversed);
  private Encoder m_FrontRightEncoder = new Encoder(Constants.DriverConstants.kFrontRightEncoderChannelA, 
                                                  Constants.DriverConstants.kFrontRightEncoderChannelB, 
                                                  Constants.DriverConstants.kFrontRightEncoderReversed);
  private Encoder m_BackLeftEncoder = new Encoder(Constants.DriverConstants.kBackLeftEncoderChannelA, 
                                                Constants.DriverConstants.kBackLeftEncoderChannelB, 
                                                Constants.DriverConstants.kBackLeftEncoderReversed);
  private Encoder m_BackRightEncoder = new Encoder(Constants.DriverConstants.kBackRightEncoderChannelA, 
                                                 Constants.DriverConstants.kBackRightEncoderChannelB, 
                                                 Constants.DriverConstants.kBackRightEncoderReversed);

  private final MotorControllerGroup m_LeftMotors = new MotorControllerGroup(m_FrontLeftMotor, m_BackLeftMotor);
  private final MotorControllerGroup m_RightMotors = new MotorControllerGroup(m_FrontRightMotor, m_BackRightMotor);

  private final DifferentialDrive m_Drive = new DifferentialDrive(m_LeftMotors, m_RightMotors);
  
  private final AHRS m_Gyro = new AHRS();
                                                 
  public double getLeftDistance() {
    return (m_FrontLeftEncoder.getDistance() + m_BackLeftEncoder.getDistance()) / 2;
  }

  public double getRightDistance() {
    return (m_FrontRightEncoder.getDistance() + m_BackRightEncoder.getDistance()) / 2;
  }

  public double getDistance() {
    return (getLeftDistance() + getRightDistance()) / 2;
  }
  

  public Drivetrain() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
