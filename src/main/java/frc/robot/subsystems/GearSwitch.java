// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class GearSwitch extends SubsystemBase {
  /** Creates a new GearSwitch. */
  private Solenoid leftGear = new Solenoid(kCompressorModule, kSwitchLeftPort);
  private Solenoid rightGear = new Solenoid(kCompressorModule, kSwitchRightPort);

  private boolean fastOn = false;

  private void fast() {
    leftGear.set(true);
    rightGear.set(true);

    fastOn = true;
  }

  private void slow() {
    leftGear.set(false);
    rightGear.set(false);

    fastOn = false;
  }

  public void switchGears() {
    if (fastOn) {
      slow();

    } else {
      fast();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
