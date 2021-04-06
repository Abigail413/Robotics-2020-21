// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.shooter.Shooter;


/** Add your docs here. */
public class Update {
    //starting positions
    //private final Pose2d left = new Pose2d(-1, 0, Rotation2d.fromDegrees(0));
    private static final Pose2d center = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    //private final Pose2d right = new Pose2d(1, 0, Rotation2d.fromDegrees(0));

    //private static final SendableChooser choosePosition = new SendableChooser<Pose2d>();

    private Shooter shooter;

    public Update(Shooter m_shooter) {
        shooter = m_shooter;
        /*choosePosition.setDefaultOption("Center", center);
        choosePosition.addOption("Left", left);
        choosePosition.addOption("Right", right);
        SmartDashboard.putData("Starting Position", choosePosition);*/
        //SmartDashboard.putNumber("Calculated RPM", shooter.calculateRPM());
    }
    /**
     * shows position of the robot at the beginning of a match
     * @return starting position of the robot
     */
    public static Pose2d getStartingPose() {
        final Pose2d position = center;
        return position;
    }
}
