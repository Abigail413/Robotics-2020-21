// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private CANSparkMax launcher = new CANSparkMax(kShooterPort, MotorType.kBrushless);

  private boolean engaged = false;

  private ChangePosition goalMover;


  public Shooter(ChangePosition changePos) {
    goalMover = changePos;
  }

public void collect(double intakeVolts) {
      launcher.setVoltage(intakeVolts);
      engaged = true;
  }

  public void shoot(double shooterVolts) {
    launcher.setVoltage(shooterVolts);
    engaged = true;
  }

  public void stop() {
    launcher.setVoltage(0);

    engaged = false;
  }

  public void setSpeedVolts(double intakeVolts, double shooterVolts) {
    if (goalMover.isPosOut()) {
      collect(intakeVolts);

    } else {
      shoot(shooterVolts);
    }
  }

  public void toggleSpeedVolts() {
    if (engaged) {
      stop();

    } else {
      setSpeedVolts(intakeVolts, shooterVolts);
    }
  }

  public boolean isEngaged() {
    return engaged;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
