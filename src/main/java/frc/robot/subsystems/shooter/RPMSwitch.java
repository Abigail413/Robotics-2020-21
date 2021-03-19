// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;
import frc.robot.subsystems.shooter.Shooter;


public class RPMSwitch extends SubsystemBase {
  /** Creates a new RPMSwitch. */
  private Shooter shooter;
  public RPMSwitch(Shooter m_shooter) {
    shooter = m_shooter;
  }
  public static zoneSelector selector = zoneSelector.far;

  public enum zoneSelector {
    near, mid, far;
}
/**
 * switches through the zones and switches RPM of shooter
 * @return new zone
 */
  public void zoneSwitch() {
    switch(selector) {
      case far:
        selector = zoneSelector.mid;
        shooter.shootingRPM = midShootingRPM;

      case mid:
        selector = zoneSelector.near;
        shooter.shootingRPM = nearShootingRPM;
        
      case near:
        selector = zoneSelector.far;
        shooter.shootingRPM = farShootingRPM;

      default:
        selector = zoneSelector.mid;
        shooter.shootingRPM = midShootingRPM;

    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
