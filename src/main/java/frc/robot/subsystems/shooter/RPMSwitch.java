// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;


public class RPMSwitch extends SubsystemBase {
  /** Creates a new RPMSwitch. */
  private Shooter shooter;
  public RPMSwitch(Shooter m_shooter) {
    shooter = m_shooter;
  }
  public zoneSelector selector = zoneSelector.far;

  public enum zoneSelector {
    near("Near"), 
    mid("Mid"), 
    far("Far");

    public final String positionName;

    zoneSelector (final String positionName) {
      this.positionName = positionName;
    }
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
        //SmartDashboard.putString("Shooting Postion:", zoneSelector.mid.positionName);

      case mid:
        selector = zoneSelector.near;
        shooter.shootingRPM = nearShootingRPM;
        //SmartDashboard.putString("Shooting Postion:", zoneSelector.near.positionName);
        
      case near:
        selector = zoneSelector.far;
        shooter.shootingRPM = farShootingRPM;
        //SmartDashboard.putString("Shooting Postion:", zoneSelector.far.positionName);

      //default: 
        /*selector = zoneSelector.far;
        shooter.shootingRPM = farShootingRPM;
        SmartDashboard.putString("Shooting Postion:", zoneSelector.far.positionName);*/

    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
