// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static class DriverConstants {

		public static final int frontLeftSwervePort = 1;
		public static final int frontRightSwervePort = 3;
		public static final int backLeftSwervePort = 5;
		public static final int backRightSwervePort = 7;
	
		public static final int[] swerveMotorPorts = {
			frontLeftSwervePort,
			frontRightSwervePort,
			backLeftSwervePort,
			backRightSwervePort
		};

    	public static final int frontLeftDrivePort = 2;
    	public static final int frontRightDrivePort = 4;
		public static final int backLeftDrivePort = 6;
		public static final int backRightDrivePort = 8;

		public static final int[] driveMotorPorts = {
			frontLeftDrivePort,
			frontRightDrivePort,
			backLeftDrivePort,
			backRightDrivePort
		};

		public static final int frontLeftEncoder = 0;
		public static final int frontRightEncoder = 1;
		public static final int backLeftEncoder = 4;
		public static final int backRightEncoder = 3;

		public static final int[] encoders = {
			frontLeftEncoder,
			frontRightEncoder,
			backLeftEncoder,
			backRightEncoder
		};

		public static final double[] absoluteOffsets = {
			//51,
			//158,
			//32,
			//346
			321.06, 360, 350.59, 14.87
		};

		// TODO: change to meters possibly??
		public static final Translation2d frontLeft = new Translation2d(0.27305, -0.27305);
		public static final Translation2d frontRight= new Translation2d(0.27305, 0.27305);
		public static final Translation2d backLeft = new Translation2d(-0.27305, -0.27305);
		public static final Translation2d backRight = new Translation2d(-0.27305, 0.27305);

		public static final double swerveP = 0.010;
		public static final double swerveI = 0.00;
		public static final double swerveD = 0.00;
		public static final double swerveFF = 0.0;

		public static final double highDriveSpeed = 0.5;// 7.26;
		public static final double standardDriveSpeed = (14/50)/12.8;
		public static final double speedModifier = 1.0;

		public static final double firstStageRatio = (14/50) / 12.8;
		public static final double inchesPerRotation = Math.PI;
		public static final double metersPerRotation = Units.inchesToMeters(inchesPerRotation);

    	public static final SimpleMotorFeedforward[] driveFeedForward = {
            new SimpleMotorFeedforward(0.153, 1.6, 0.18),
            new SimpleMotorFeedforward(0.153, 1.6, 0.18),
            new SimpleMotorFeedforward(0.153, 1.6, 0.18),
            new SimpleMotorFeedforward(0.153, 1.6, 0.18)};

  	}

	public static class OperatorConstants {
		public static final int driverControllerPort = 0;
	}

	public static class VisionConstants {
		public static final String ipAddress = "ws://10.54.80.201";
		public static final int CameraRotations[] = {0};
		public static HashMap<String, Integer> apriltagAngles = new HashMap<>();
		
		public VisionConstants() {
			apriltagAngles.put("13", 0);
			apriltagAngles.put("tag2", 45);
			apriltagAngles.put("tag3", 60);
			apriltagAngles.put("tag4", 90);
		}
	}
}