/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

import static edu.wpi.first.wpilibj.XboxController.Axis.*;
import static edu.wpi.first.wpilibj.XboxController.Button.*;

import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.AimTarget;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.drive.GearSwitch;
import frc.robot.subsystems.lift.Lift;
import frc.robot.subsystems.shooter.Plucker;
import frc.robot.subsystems.shooter.ChangePosition;
import frc.robot.subsystems.shooter.Conveyor;
import frc.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;

import static frc.robot.Constants.*;

import java.util.Arrays;

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

  private final Shooter shooter = new Shooter(changePosition, limelight);

  private final Conveyor conveyor = new Conveyor(changePosition, shooter);

  private final Plucker plucker = new Plucker(changePosition);

  private final Lift lift = new Lift();

  private final GearSwitch driveGears = new GearSwitch();


  private Command manualDrive = new RunCommand(
    () -> drivetrain.getDifferentialDrive().tankDrive(
      kDrivePctLimit * drivetrain.deadband(xbox.getRawAxis(kLeftY.value), kPctDeadband),
      kDrivePctLimit * drivetrain.deadband(xbox.getRawAxis(kRightY.value), kPctDeadband),
      false
    ),
    drivetrain
  );

  private Command moveLift = new RunCommand(
    () -> lift.move(xbox.getRawAxis(kRightTrigger.value) - xbox.getRawAxis(kLeftTrigger.value)), lift);

  private SequentialCommandGroup shooterStartup = new SequentialCommandGroup(
    new WaitCommand(shooterStartupTime).withInterrupt(changePosition::isPosOut),
    new InstantCommand(() -> conveyor.setSpeed(conveyorVolts), conveyor),
    new WaitCommand(0.5),
    new InstantCommand(() -> plucker.setSpeed(pluckerVolts), plucker)
    );

  private SequentialCommandGroup stopFeeders = new SequentialCommandGroup(
    new InstantCommand(() -> conveyor.stop()),
    new InstantCommand(() -> plucker.stop())
  );



  
  /*private SequentialCommandGroup onAndOff = new SequentialCommandGroup(
    new InstantCommand(() -> conveyor.setSpeed(conveyorVolts), conveyor),
    new InstantCommand(() -> plucker.setSpeed(pluckerVolts), plucker),
    new WaitCommand(0.2),
    new InstantCommand(() -> conveyor.stop()),
    new InstantCommand(() -> plucker.stop()),
    new WaitCommand(0.2)
  );*/

  private RamseteCommand rBase = new RamseteCommand(
    getMovingTrajectory(), 
    drivetrain::getPose,
    new RamseteController(), 
    drivetrain.getFeedForwardDrive(), 
    drivetrain.getKinematics(), 
    drivetrain::getSpeeds, 
    drivetrain.getLeftDrivePID(), 
    drivetrain.getRightDrivePID(), 
    drivetrain::setOutputVolts, 
    drivetrain);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    drivetrain.setDefaultCommand(manualDrive);
    lift.setDefaultCommand(moveLift);
  }
  /**
   * settings for the robot on startup
   */
  public void init(){
    limelight.driverMode();
    limelight.lightOff();
    limelight.PiPSecondaryStream();

    shooter.stop();
    conveyor.stop();
    plucker.stop();
  }

  public void periodic() {
    /*SmartDashboard.putString("Shooting Postion:", shooter.selector.positionName);
    SmartDashboard.putNumber("Shooting RPM", shooter.shootingRPM);*/
  }
  /**
   * calculates the trajectory of the robot as it moves
   * @return trajectory of the robot
   */
  private Trajectory getMovingTrajectory() {
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      Arrays.asList(Update.getStartingPose(), new Pose2d(1.0, 0, new Rotation2d()),  
        new Pose2d(2.3, 1.2, Rotation2d.fromDegrees(90.0))),
      new TrajectoryConfig(kMaxSafeVelocityMeters, kMaxSafeAccelerationMeters)
    );

    return trajectory;
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
      .whenHeld(new AimTarget(limelight, drivetrain));

    //toggle shooter
    new JoystickButton(xbox, kBumperLeft.value)
      .whenPressed(new InstantCommand(() -> shooter.toggleSpeedSpark()))
      .whenPressed(new ConditionalCommand(shooterStartup, stopFeeders, shooter::isEngaged));

    //toggle feeders
    new JoystickButton(xbox, kBumperRight.value)
      .whenPressed(new InstantCommand(() -> plucker.toggleSpeed(pluckerVolts), plucker))
      .whenPressed(new InstantCommand(() -> conveyor.toggleSpeed(conveyorVolts), conveyor));

    //switch drivetrain speeds
    new JoystickButton(xbox, kA.value)
      .whenPressed(new InstantCommand(() -> driveGears.switchGears(), driveGears));

    //toggle feeders
    /*new JoystickButton(xbox, kBumperRight.value)
      .whenHeld(onAndOff);*/
      
    //switch RPM of shooter
    /*new JoystickButton(xbox, kStart.value)
      .whenPressed(new InstantCommand(() -> shooter.zoneSwitch(), shooter))
      .whenPressed(new PrintCommand("Pressed"));*/
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
