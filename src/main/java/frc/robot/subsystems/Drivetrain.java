/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Drivetrain extends SubsystemBase {
  /**
   * Creates a new Drivetrain.
   */
  private CANSparkMax LFrontWheel = new CANSparkMax(kLFrontWheelPort, MotorType.kBrushless);
  private CANSparkMax RFrontWheel = new CANSparkMax(kRFrontWheelPort, MotorType.kBrushless);
  private CANSparkMax LBackWheel = new CANSparkMax(kLBackWheelPort, MotorType.kBrushless);
  private CANSparkMax RBackWheel = new CANSparkMax(kRBackWheelPort, MotorType.kBrushless);

  private DifferentialDrive roboDrive = new DifferentialDrive(LFrontWheel, RFrontWheel);

  public Drivetrain() {
    LBackWheel.follow(LFrontWheel);
    RBackWheel.follow(RFrontWheel);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
