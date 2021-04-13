// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ChangePosition;



/** Add your docs here. */
public class Update {
    private Shooter m_shooter;
    private ChangePosition positionChange;

    //starting positions
    //private final Pose2d left = new Pose2d(-1, 0, Rotation2d.fromDegrees(0));
    private static final Pose2d center = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    //private final Pose2d right = new Pose2d(1, 0, Rotation2d.fromDegrees(0));

    //private static final SendableChooser choosePosition = new SendableChooser<Pose2d>();

    public Update(Shooter shooter, ChangePosition changePos) {
        m_shooter = shooter;
        positionChange = changePos;

        //displays the starting position of the robot on a competition field
        /*choosePosition.setDefaultOption("Center", center);
        choosePosition.addOption("Left", left);
        choosePosition.addOption("Right", right);
        SmartDashboard.putData("Starting Position", choosePosition);*/

        //display whether the shooter is in shooting position
        SmartDashboard.putBoolean("Shooter Up", positionChange.isPosOut());

        //display the current shooter RPM
        SmartDashboard.putNumber("Shooter RPM", m_shooter.getVelocity());

        //displays the calculated RPM for the shooter
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

    public void periodic() {
        //display whether the shooter is in shooting position
        SmartDashboard.putBoolean("Shooter Up", positionChange.isPosOut());

        //display the current shooter RPM
        SmartDashboard.putNumber("Shooter RPM", m_shooter.getVelocity());

        //displays the calculated RPM for the shooter
        //SmartDashboard.putNumber("Calculated RPM", shooter.calculateRPM());
    }
}
