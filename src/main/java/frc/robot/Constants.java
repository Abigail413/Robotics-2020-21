/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //Xbox controller port
    public static final int kControllerPort = 0;

    //drivetrain ports
    public static final int kLFrontWheelPort = 1;
    public static final int kRFrontWheelPort = 3;
    public static final int kLBackWheelPort = 2;
    public static final int kRBackWheelPort = 4;

    //limits on drivetrain
    public static final double kDrivePctLimit = 1;
    public static final double kPctDeadband = 0.05;

    //gear switch ports
    public static final int kSwitchLeftPort = 2;
    public static final int kSwitchRightPort = 3;

    //shooter port
    public static final int kShooterPort = 5;

    //shooter voltages
    public static final int intakeVolts = -4;
    public static final int shooterVolts = 12;

    //Shooter RPM
    public static final int intakeRPM = -1000;
    public static final int nearShootingRPM = 1500;
    public static final int midShootingRPM = 4000;
    public static final int farShootingRPM = 7000;

    //compressor port
    public static final int kCompressorModule = 20;

    //conveyor port
    public static final int kConveyorPort = 8;

    //conveyor voltage
    public static final int conveyorVolts = 10;

    //plucker port
    public static final int kPluckerPort = 6;

    //plucker voltage
    public static final int pluckerVolts = 10;

    //solenoid ports
    public static final int kLeftPistonPort = 0;
    public static final int kRightPistonPort = 1;

    //lift port
    public static final int kLiftPort = 7;

    //shooter startup wait time
    public static final int shooterStartupTime = 6;

    //robot measurements
    public static final double kTicksPerRev = 4096;
    public static final double kTrackWidthMeters = Units.inchesToMeters(10);
    public static final double kGearRatio = 7.29;

    public static final double kDriveWheelRadiusMeters = Units.inchesToMeters(3);
    public static final double kShooterWheelRadiusMeters = Units.inchesToMeters(3);
   
    public static final double kMaxSafeVelocityMeters = Units.feetToMeters(2);
    public static final double kMaxSafeAccelerationMeters = Units.feetToMeters(2);

    public static final double kCameraAngle = 0;
    public static final double kShooterAngle = 57.5;

    public static final double kTargetToCameraHeight = Units.inchesToMeters(80.25);
    public static final double kHighGoalHeight = Units.inchesToMeters(96);
    public static final double kShooterHeight = Units.inchesToMeters(18);

    public static final double kCameraToCenterOfBall = Units.inchesToMeters(18.5);

    //acceleration due to gravity
    public static final double kAccelerationGravity = 9.80665;
}
