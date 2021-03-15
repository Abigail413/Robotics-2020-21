// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Gains {
    public static class angleCorrection {
		public static double kP = 0.01;
		public static double kI = 0;
		public static double kD = 0;
	}

	public static class shooterPID {
		public static double kP = 0.0015;
		public static double kI = 0;
		public static double kD = 0.3;
		public static double kFF = 0.0005;
	}

	public static class leftDrive {
		public static double kP = 0.1;
		public static double kI = 0;
		public static double kD = 0;
	}

	public static class rightDrive {
		public static double kP = 0.1;
		public static double kI = 0;
		public static double kD = 0;
	}

	public static class feedForwardDriveRight {
		public static double kS = 0;
		public static double kV = 0;
		public static double kA = 0;
	}

	public static class feedForwardDriveLeft {
		public static double kS = 0;
		public static double kV = 0;
		public static double kA = 0;
	}

	public static class Ramsete {
		public static final double kBeta = 2.0;
		public static final double kZeta = 0.7;
	}
}
