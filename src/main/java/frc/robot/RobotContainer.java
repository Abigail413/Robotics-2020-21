/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
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
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.drive.GearSwitch;
import frc.robot.subsystems.lift.Lift;
import frc.robot.subsystems.shooter.ChangePosition;
import frc.robot.subsystems.shooter.Conveyor;
import frc.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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

  private final Lift lift = new Lift();

  private final GearSwitch driveGears = new GearSwitch();

  private final Controller xboxController = new Controller(xbox);


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

  /*private SequentialCommandGroup shooterStartup = new SequentialCommandGroup(
    new WaitCommand(shooterStartupTime).withInterrupt(changePosition::isPosOut),
    new InstantCommand(() -> conveyor.setSpeed(conveyorVolts), conveyor),
    new WaitCommand(0.5),
    new InstantCommand(() -> plucker.setSpeed(pluckerVolts), plucker)
    );*/

  private ParallelCommandGroup stopFeeders = new ParallelCommandGroup(
    new InstantCommand(() -> conveyor.stop()),
    new InstantCommand(() -> limelight.lightOff()),
    new InstantCommand(() -> limelight.driverMode())

  );

  private SequentialCommandGroup waitUntilVelocity = new SequentialCommandGroup(
    new WaitUntilCommand(() -> shooter.atSpeed(100)),
    new InstantCommand(() -> conveyor.setSpeed(conveyorVolts), conveyor)
  );
/**
 * Tests each robot component individually. This command group runs in test mode.
 */
  public SequentialCommandGroup testRobot = new SequentialCommandGroup(
    new RunCommand(() -> drivetrain.getDifferentialDrive().tankDrive(0.4, 0.4), drivetrain).withTimeout(2), //checks drivetrain
    new WaitCommand(.25),

    new InstantCommand(() -> driveGears.switchGears()), //checks gear switching solenoids
    new WaitCommand(.25),
    new InstantCommand(() -> driveGears.switchGears()),
    new WaitCommand(.25),

    new InstantCommand(() -> lift.move(.5)), //checks lift
    new WaitCommand(.5),
    new InstantCommand(() -> lift.stop()),
    new WaitCommand(.25),

    new InstantCommand(() -> shooter.toggleSpeedSpark()), //checks shooter
    new WaitCommand(.5), 
    new InstantCommand(() -> shooter.stop()),
    new WaitCommand(.25),

    new InstantCommand(() -> conveyor.toggleSpeed(conveyorVolts)), //checks conveyor
    new WaitCommand(.5), 
    new InstantCommand(() -> conveyor.stop()),
    new WaitCommand(.25),

    new InstantCommand(() -> changePosition.posSwitch()), //checks shooter solenoids
    new WaitCommand(.5),
    new InstantCommand(() -> changePosition.posSwitch())
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
   * Shakes the robot back and forth to dislodge balls
   */
  private void robotShaker() {
    int i;
    for (i = 0; i < 5; i++) {
      new RunCommand(() -> drivetrain.getDifferentialDrive().tankDrive(0.2, -0.2), drivetrain).withTimeout(1);
      new RunCommand(() -> drivetrain.getDifferentialDrive().tankDrive(-0.2, 0.2), drivetrain).withTimeout(1);
    }
  }

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
  }

  public void periodic() {
    /*SmartDashboard.putString("Shooting Postion:", shooter.selector.positionName);
    SmartDashboard.putNumber("Shooting RPM", shooter.shootingRPM);*/
    if (Timer.getMatchTime() < 30 && Timer.getMatchTime() > 28) {
      xboxController.startRumble();

    } else {
      xboxController.stopRumble();
    }
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

    //toggle shooter with RPM calculated by limelight
    new JoystickButton(xbox, kBumperLeft.value)
      .whenPressed(new InstantCommand(() -> limelight.visionMode(), limelight))
      .whenPressed(new InstantCommand(() -> limelight.lightOn(), limelight))
      .whenPressed(new InstantCommand(() -> shooter.toggleSpeedSpark()))
      .whenPressed(new ConditionalCommand(waitUntilVelocity, stopFeeders, shooter::isEngaged));

    //toggle feeders
    new JoystickButton(xbox, kBumperRight.value)
      .whenPressed(new InstantCommand(() -> conveyor.toggleSpeed(conveyorVolts), conveyor));

    //switch drivetrain speeds
    new JoystickButton(xbox, kA.value)
      .whenPressed(new InstantCommand(() -> driveGears.switchGears(), driveGears));

    //shoot at consistent speed (in case vision is messed up)
    new JoystickButton(xbox, kB.value)
      .whenPressed(new InstantCommand(() -> shooter.toggleStaticSpeedSpark()))
      .whenPressed(new ConditionalCommand(waitUntilVelocity, stopFeeders, shooter::isEngaged));

    //toggle feeders
    /*new JoystickButton(xbox, kBumperRight.value)
      .whenHeld(onAndOff);*/
      
    //switch RPM of shooter
    /*new JoystickButton(xbox, kStart.value)
      .whenPressed(new InstantCommand(() -> shooter.zoneSwitch(), shooter))
      .whenPressed(new PrintCommand("Pressed"));*/

    //shakes the robot to dislodge balls
    new JoystickButton(xbox, kStart.value)
      .whileHeld(new InstantCommand(() -> robotShaker()))
      .whileHeld(new InstantCommand(() -> xboxController.startRumbleCalm()))
      .whenReleased(new InstantCommand(() -> xboxController.stopRumble()));
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
