// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.*;
import static frc.robot.Gains.shooterPID.*;

import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private CANSparkMax launcher = new CANSparkMax(kShooterPort, MotorType.kBrushless);

  private CANPIDController launcherController = launcher.getPIDController();

  private boolean engaged = false;

  private ChangePosition goalMover;

  public int shootingRPM = midShootingRPM;

  public Shooter(ChangePosition changePos) {
    goalMover = changePos;

    launcherController.setP(kP);
    launcherController.setI(kI);
    launcherController.setD(kD);
    launcherController.setFF(kFF);
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

  public void setSpeedSpark() {
    if (goalMover.isPosOut()) {
      launcherController.setReference(intakeRPM, ControlType.kVelocity);

    } else {
      launcherController.setReference(shootingRPM, ControlType.kVelocity);
    }

    engaged = true;
  }

  public void toggleSpeedSpark() {
    if (engaged) {
      stop();

    } else {
      setSpeedSpark();
    }
  }

  public boolean isEngaged() {
    return engaged;
  }

  public zoneSelector selector = zoneSelector.mid;

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
        shootingRPM = midShootingRPM;
        break;

      case mid:
        selector = zoneSelector.near;
        shootingRPM = nearShootingRPM;
        break;

      case near:
        selector = zoneSelector.far;
        shootingRPM = farShootingRPM;
        break;

      default: 
        selector = zoneSelector.mid;
        shootingRPM = midShootingRPM;

    }

  }
  /**
   * This calculates the rotations per minute necessary to shoot a ball into the target from a given distance. It chooses between two calculated RPMs and chooses the lesser positive one.
   * @param distanceMeters from target
   * @return RPM appropriate for the distance
   */
  private double calculateRPM(double distanceMeters) {
    double bValue = ((kTargetToCameraHeight * Math.cos(kShooterAngle)) / Math.pow(distanceMeters, 2)); 
    double aValue = -((Math.sin(kShooterAngle)) / distanceMeters);

    double shooterVelocityPos = (-(bValue) + Math.sqrt(Math.pow(bValue, 2) - (4 * aValue * kAccelerationGravity))) / (2 * Math.sin(aValue));
    double shooterVelocityNeg = (-(bValue) - Math.sqrt(Math.pow(bValue, 2) - (4 * aValue * kAccelerationGravity))) / (2 * Math.sin(aValue));

    double radPerSecondPos = shooterVelocityPos / kShooterWheelRadiusMeters;
    double radPerSecondNeg = shooterVelocityNeg / kShooterWheelRadiusMeters;

    double RPMPos = Units.radiansPerSecondToRotationsPerMinute(radPerSecondPos);
    double RPMNeg = Units.radiansPerSecondToRotationsPerMinute(radPerSecondNeg);

    if (RPMPos < RPMNeg && RPMPos > 0) { // If the RPM + is less than - and positive, use it for a lower arc
      return RPMPos;

    } else if (RPMNeg < RPMPos && RPMNeg > 0) { // If the RPM - is less than + and positive, use it for a lower arc
      return RPMNeg;

    } else if (RPMPos == RPMNeg && RPMNeg > 0) { // If they are the same value (and positive), return that value
      return RPMNeg;

    } else if (RPMNeg > 0) { // Return RPMNeg if positive, this is here in case RPMPos is negative
      return RPMNeg;

    } else if (RPMPos > 0) { // Return RPMPos if positive, this is here in case RPMNeg is negative
      return RPMPos;

    } else {
      return 0; // something went wrong, let us not break things. It could be that RPMNeg and RPMPos are both negative
    }

  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
