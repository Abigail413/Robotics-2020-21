// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Plucker extends SubsystemBase {
  /** Creates a new Plucker. */
  private WPI_TalonSRX plucker = new WPI_TalonSRX(kPluckerPort);

  private boolean engaged = false;

  private Shooter m_shooter;
  private ChangePosition goalMover; 

  public Plucker(ChangePosition changePosition, Shooter shooter) {
    m_shooter = shooter;
    goalMover = changePosition;
  }

  public void stop() {
    setSpeed(0);
    engaged = false;
  }

  public void setSpeed(double pluckerVolts) {
    if (goalMover.isPosOut()) {
      plucker.setVoltage(-pluckerVolts);

    } else {
      plucker.setVoltage(pluckerVolts);
      engaged = true;
    }
  }

  public void toggleSpeed(double pluckerVolts) {
    if (engaged) {
      stop();

    } else {
      setSpeed(pluckerVolts);
    }
  }

  public boolean getEngaged() {
    return engaged;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
