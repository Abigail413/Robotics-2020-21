/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;
import static frc.robot.Gains.*;

public class Drivetrain extends SubsystemBase {
  private CANSparkMax LFrontWheel = new CANSparkMax(kLFrontWheelPort, MotorType.kBrushless);
  private CANSparkMax RFrontWheel = new CANSparkMax(kRFrontWheelPort, MotorType.kBrushless);
  private CANSparkMax LBackWheel = new CANSparkMax(kLBackWheelPort, MotorType.kBrushless);
  private CANSparkMax RBackWheel = new CANSparkMax(kRBackWheelPort, MotorType.kBrushless);

  private DifferentialDrive roboDrive = new DifferentialDrive(LFrontWheel, RFrontWheel);

  //drive PIDs
  private PIDController leftDrivePID
    = new PIDController(leftDrive.kP, leftDrive.kI, leftDrive.kD);

  private PIDController rightDrivePID = 
    new PIDController(rightDrive.kP, rightDrive.kI, rightDrive.kD);

  //gyro setup
  private AHRS gyro = new AHRS(SPI.Port.kMXP);
  private double gyroPosition = -gyro.getAngle();
  private PIDController gyroController 
    = new PIDController(0.1, 0, 0);

  //autonomous tracking
  private DifferentialDriveKinematics kinematics = 
    new DifferentialDriveKinematics(kTrackWidthMeters);

  private DifferentialDriveOdometry odometry = 
    new DifferentialDriveOdometry(getHeading());

  private SimpleMotorFeedforward feedforwardDrive
    = new SimpleMotorFeedforward(feedForwardDriveLeft.kS, feedForwardDriveLeft.kV, feedForwardDriveLeft.kA);

  private Pose2d pose = new Pose2d();
  

  public Drivetrain() {
    LBackWheel.follow(LFrontWheel);
    RBackWheel.follow(RFrontWheel);

    LFrontWheel.getEncoder().setPosition(0);
    RFrontWheel.getEncoder().setPosition(0);

    gyro.reset();
  }

  public DifferentialDrive getDifferentialDrive() {
    return roboDrive;
  }

  public double deadband(double JoystickValue, double DeadbandCutOff) {
    double deadbandReturn;
    if (Math.abs(JoystickValue) < DeadbandCutOff) {
      deadbandReturn = 0;

    } else {
      deadbandReturn = JoystickValue;
    }
    
    return deadbandReturn;
  }

public void setOutputVolts(double leftVolts, double rightVolts) {
  LFrontWheel.setVoltage(leftVolts);
  RFrontWheel.setVoltage(rightVolts);
}

  public double getAngle() {
    return -gyro.getAngle();
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(getAngle());
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public Pose2d getPose() {
    return pose;
  }

  public SimpleMotorFeedforward getFeedForwardDrive() {
    return feedforwardDrive;
  }

  public PIDController getLeftDrivePID() {
    return leftDrivePID;
  }

  public PIDController getRightDrivePID() {
    return rightDrivePID;
  }

  public double getLeftDistanceMeters() {
    return LFrontWheel.getEncoder().getPosition() /
      RFrontWheel.getEncoder().getCountsPerRevolution() * 2 * Math.PI * kDriveWheelRadiusMeters;
  }

  public double getRightDistanceMeters() {
    return RFrontWheel.getEncoder().getPosition() /
      LFrontWheel.getEncoder().getCountsPerRevolution() * 2 * Math.PI * kDriveWheelRadiusMeters;
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      LFrontWheel.getEncoder().getVelocity() / kGearRatio * 2 * Math.PI * kDriveWheelRadiusMeters / 60,
      RFrontWheel.getEncoder().getVelocity() / kGearRatio * 2 * Math.PI * kDriveWheelRadiusMeters / 60);
  }  

   /*ChassisSpeeds leftDriveSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
     LFrontWheel.getEncoder().getVelocity(), 0, getAngle(), getHeading());
  
   ChassisSpeeds rightDriveSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
     RFrontWheel.getEncoder().getVelocity(), 0, getAngle(), getHeading());*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pose = odometry.update(getHeading(), getLeftDistanceMeters(), getRightDistanceMeters());
  }
}
