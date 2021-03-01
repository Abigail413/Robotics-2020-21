// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Conveyor extends SubsystemBase {
  /** Creates a new Conveyor. */
  private WPI_TalonSRX conveyor = new WPI_TalonSRX(kConveyorPort);

  private ChangePosition goalMover;
  private Shooter m_shooter;

  public Conveyor(ChangePosition changePosition, Shooter shooter) {
    goalMover = changePosition;
    m_shooter = shooter;
  }

  public void shoot(double conveyorVolts) {
    conveyor.setVoltage(conveyorVolts);
  }

  public void collect(double conveyorVolts) {
    conveyor.setVoltage(-conveyorVolts);
  }

  public void stop() {
    setSpeed(0);
  }
  public void setSpeed(double conveyorVolts) {
    if (goalMover.isPosOut()) {
      collect(conveyorVolts);

    } else {
      shoot(conveyorVolts);
    }
  }

  public void toggleSpeed(double conveyorVolts) {
    if (m_shooter.isEngaged()) {
      setSpeed(conveyorVolts);

    } else {
      stop();
    }
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
