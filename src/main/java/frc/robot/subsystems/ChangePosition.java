/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;

import static frc.robot.Constants.*;
public class ChangePosition extends SubsystemBase {
  /**
   * Creates a new ChangePosition.
   */
  private Solenoid leftPiston = new Solenoid(kCompressorModule, kLeftPistonPort);
  private Solenoid rightPiston = new Solenoid(kCompressorModule, kRightPistonPort);


  private Compressor airow = new Compressor(0);

  private boolean posOut = false;

  public void posSwitch() {
    if (posOut) {
      leftPiston.set(false);
      rightPiston.set(false);
      posOut = false;

    } else {
      leftPiston.set(true);
      rightPiston.set(true);
      posOut = true;
    }
  }

  public ChangePosition() {
    airow.start();
  }

  public boolean isPosOut() {
    return posOut;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
