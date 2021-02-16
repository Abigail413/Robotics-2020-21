// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.vision.Limelight;


import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private CANSparkMax launcher = new CANSparkMax(kShooterPort, MotorType.kBrushless);

  private boolean engaged = false;

  private ChangePosition changePos;

  public Shooter(ChangePosition changePos, Limelight limelight) {
    
  }

public void collect() {
      launcher.setVoltage(intakeVolts);
      engaged = true;
  }

  public void shoot() {
    launcher.setVoltage(shooterVolts);
    engaged = true;
  }

  public void stop() {
    launcher.setVoltage(0);
    engaged = false;
  }

  public void setSpeedVolts() {
    if (changePos.isPosOut()) {
      collect();

    } else {
      shoot();
    }
  }

  public void toggleSpeedVolts() {
    if (engaged) {
      stop();

    } else {
      setSpeedVolts();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
