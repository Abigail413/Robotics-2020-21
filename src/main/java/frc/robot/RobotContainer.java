/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;

import edu.wpi.first.wpilibj.XboxController;
import static edu.wpi.first.wpilibj.XboxController.Axis.*;
import static edu.wpi.first.wpilibj.XboxController.Button.*;

import frc.robot.vision.Limelight;
import frc.robot.vision.AimTarget;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Plucker;
import frc.robot.subsystems.ChangePosition;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;

import static frc.robot.Constants.*;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController xbox = new XboxController(kControllerPort);

  private final Drivetrain drivetrain = new Drivetrain();

  private final ChangePosition changePosition = new ChangePosition();

  private final Limelight limelight = new Limelight();

  private final Shooter shooter = new Shooter(changePosition);

  private final Conveyor conveyor = new Conveyor(changePosition, shooter);

  private final Plucker plucker = new Plucker(changePosition);

  private final Lift lift = new Lift();

  private Command manualDrive = new RunCommand(
    () -> drivetrain.getDifferentialDrive().tankDrive(
      kDrivePctLimit * drivetrain.deadband(xbox.getRawAxis(kLeftY.value), 0.05),
      kDrivePctLimit * drivetrain.deadband(xbox.getRawAxis(kRightY.value), 0.05),
      false
    ),
    drivetrain
  );

  private Command moveLift = new RunCommand(
    () -> lift.move(xbox.getRawAxis(kRightTrigger.value - kLeftTrigger.value)), lift);

  private SequentialCommandGroup shooterStartup = new SequentialCommandGroup(
    new WaitCommand(shooterStartupTime).withInterrupt(changePosition::isPosOut),
    new InstantCommand(() -> conveyor.setSpeed(conveyorVolts), conveyor),
    new InstantCommand(() -> plucker.setSpeed(pluckerVolts), plucker)
    );

  private SequentialCommandGroup stopFeeders = new SequentialCommandGroup(
    new InstantCommand(() -> conveyor.stop()),
    new InstantCommand(() -> plucker.stop())
  );
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    drivetrain.setDefaultCommand(manualDrive);
    lift.setDefaultCommand(moveLift);
  }

  public void init(){
    limelight.driverMode();
    limelight.lightOff();
    limelight.PiPSecondaryStream();

    shooter.stop();
    conveyor.stop();
    plucker.stop();
  }
  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //switch shooter positions
    new JoystickButton(xbox, kY.value)
      .whenPressed(new InstantCommand(() -> changePosition.posSwitch(), changePosition));

    //toggle limelight
    new JoystickButton(xbox, kX.value)
      .whenPressed(new AimTarget(limelight, drivetrain));

    //shooting
    new JoystickButton(xbox, kBumperLeft.value)
      .whenPressed(new InstantCommand(() -> shooter.toggleSpeedVolts()))
      .whenPressed(new ConditionalCommand(shooterStartup, stopFeeders, shooter::isEngaged));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
