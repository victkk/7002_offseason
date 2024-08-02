// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.config.SwerveModuleConfig;
import frc.robot.config.SwerveModuleConfig.ModuleCorner;

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
  public static final double kPeriodicDt = TimedRobot.kDefaultPeriod; // seconds
  public static final NeutralOut NEUTRAL = new NeutralOut();
  public static final double kLongCANTimeoutSec = 0.1; // use for constructors
  public static final int kLongCANTimeoutMs = 100; // use for constructors

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class SwerveModuleConstants {
    public static final SwerveModuleConfig FL, FR, BL, BR;

    static {
      FL = new SwerveModuleConfig();
      FL.driveID = 6;
      FL.azimuthID = 7;
      FL.azimuthBlockCenterDegree = 105.0;
      FL.azimuthEncoderOffsetRotation = -0.038574;
      FL.lightGateID = 1;
      FL.corner = ModuleCorner.Front_Left;
      FL.invertDrive = InvertedValue.CounterClockwise_Positive;
      FL.invertAzimuth = InvertedValue.CounterClockwise_Positive;
      FL.DRIVE_KP = 3.0;
      FL.DRIVE_KI = 0.0;
      FL.DRIVE_KD = 0.0;
      FL.DRIVE_KF = 1.087;
      FL.AZIMUTH_KP = 60.0;
      FL.AZIMUTH_KI = 0.0;
      FL.AZIMUTH_KD = 0.0;
      FL.AZIMUTH_KF = 0.677;

      FR = new SwerveModuleConfig();
      FR.driveID = 4;
      FR.azimuthID = 5;
      FR.azimuthBlockCenterDegree = 0.0;
      FR.azimuthEncoderOffsetRotation = -0.035400;
      FR.lightGateID = 2;
      FR.corner = ModuleCorner.Front_Right;
      FR.invertDrive = InvertedValue.CounterClockwise_Positive;
      FR.invertAzimuth = InvertedValue.CounterClockwise_Positive;
      FR.DRIVE_KP = 3.0;
      FR.DRIVE_KI = 0.0;
      FR.DRIVE_KD = 0.0;
      FR.DRIVE_KF = 1.087;
      FR.AZIMUTH_KP = 60.0;
      FR.AZIMUTH_KI = 0.0;
      FR.AZIMUTH_KD = 0.0;
      FR.AZIMUTH_KF = 0.677;

      BL = new SwerveModuleConfig();
      BL.driveID = 2;
      BL.azimuthID = 3;
      BL.azimuthBlockCenterDegree = 0.0;
      BL.azimuthEncoderOffsetRotation = 0.666504;
      BL.lightGateID = 0;
      BL.corner = ModuleCorner.Back_Left;
      BL.invertDrive = InvertedValue.Clockwise_Positive;
      BL.invertAzimuth = InvertedValue.CounterClockwise_Positive;
      BL.DRIVE_KP = 3.0;
      BL.DRIVE_KI = 0.0;
      BL.DRIVE_KD = 0.0;
      BL.DRIVE_KF = 1.087;
      BL.AZIMUTH_KP = 60.0;
      BL.AZIMUTH_KI = 0.0;
      BL.AZIMUTH_KD = 0.0;
      BL.AZIMUTH_KF = 0.677;

      BR = new SwerveModuleConfig();
      BR.driveID = 0;
      BR.azimuthID = 1;
      BR.azimuthBlockCenterDegree = 105.0;
      BR.azimuthEncoderOffsetRotation = 0.075684;
      BR.lightGateID = 3;//4;
      BR.corner = ModuleCorner.Back_Right;
      BR.invertDrive = InvertedValue.Clockwise_Positive;
      BR.invertAzimuth = InvertedValue.CounterClockwise_Positive;
      BR.DRIVE_KP = 3.0;
      BR.DRIVE_KI = 0.0;
      BR.DRIVE_KD = 0.0;
      BR.DRIVE_KF = 1.087;
      BR.AZIMUTH_KP = 60.0;
      BR.AZIMUTH_KI = 0.0;
      BR.AZIMUTH_KD = 0.0;
      BR.AZIMUTH_KF = 0.677;
    }
  }

  public static final class DriveConstants {

    // driver constants
    public static final double kDeadband = 0.1;

    // chassis dimensions
    public static final double kTrackWidth = 0.565;
    public static final double kWheelBase = 0.565;
    public static final double kChassisWheelDiameterMeters = 0.13; // meters
    public static final double kChassisWheelCircumferenceMeters = kChassisWheelDiameterMeters * Math.PI; // meters

    // Pigeon constants
    public static final int kPigeonPort = 20;

    // physical robot constants
    public static final double kPhysicalMaxSpeedMetersPerSecond = 4.2;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 5.0;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond
        * 0.8;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
  }

  public static final class OdometryConstants {
    public static final double kLongCANTimeoutSec = 0.1;
    // Update Freq (Hz): minimum 4Hz, maximum 1000Hz
    public static final int kOdomUpdateFreq = 100; // signal for odometry
    public static final int kDefaultUpdateFreq = 50;
  }

  public static final class IntakeConstants {
    // INTAKER
    public static final int INTAKER_ID = 11;
    public static final int INTAKER_CENTER_ID = 13;
    // used for LED to tell note entered state
    public static final int INTAKER_ENTER_OMRON_ID = 9;
    public static final int INTAKE_EXTERIOR_ID = 30;
  }

  public static final class TransferConstants {
    // TRANSFER
    public static final int TRANSFER_ID = 27;
    public static final int TRANSFER_OMRON_PORT = 5;
  }

  public static final class ArmConstants {
    // ARM
    public static final int ARM_ID = 10;
    // arm pitch when resting in degrees
    public static final double ARM_REST_ANGLE = 22.0;
    // arm pitch when resting in rotations
    public static final double ARM_REST_POSITION = ARM_REST_ANGLE / 360.0;
    // arm pitch when in max pitch in degrees
    public static final double ARM_MAX_ANGLE = 120.0;
    // arm pitch when in max pitch in rotations
    public static final double ARM_MAX_POSITION = ARM_MAX_ANGLE / 360.0;

    public static final double STABILIZE_TIME = 0.1;
  }

  public static final class ClimberConstants{
    public static final int LEFT_CLIMBER_ID =10; // TODO 
    public static final int RIGHT_CLIMBER_ID =11;
    public static final double MAX_TRAVEL_DIST =15;
    public static final double DIAMETER=0.10;
    public static final double CLIMBER_GEAR_RATIO=6;

  }

  public static final class IntakerConstants{
    public static final int ROLLER_ID =10; // TODO 
    public static final int ARM_ID =14;
        // arm pitch when resting in degrees
    public static final double REST_ANGLE = 13.0;
    // arm pitch when resting in rotations
    public static final double REST_POSITION = REST_ANGLE / 360.0;
    // arm pitch when in max pitch in degrees
    public static final double MAX_ANGLE = 193.0;
    // arm pitch when in max pitch in rotations
    public static final double MAX_POSITION = MAX_ANGLE / 360.0;
    public static final double ARM_GEAR_RATIO=24;

    public static final double FEED_ANGLE=30.0;
    public static final double FEED_POSITION = FEED_ANGLE / 360.0;
  }
  public static final class ShooterConstants {
    public static final int SHOOTER_ID = 12;
    public static final int SHOOTER_FOLLOWER = 15;
    public static final double GEAR_RATIO = 1.6;
  }

  public static final class VisionShootConstants {
    public static final InterpolatingDoubleTreeMap kSpeakerAngleMap = new InterpolatingDoubleTreeMap();

    static {
      // TODO : TUNE
      // distance meters <-> arm angle degrees
      kSpeakerAngleMap.put(1.00, 59.0);
      kSpeakerAngleMap.put(1.30, 56.5);
      kSpeakerAngleMap.put(1.40, 52.0);
      kSpeakerAngleMap.put(1.65, 48.5);
      kSpeakerAngleMap.put(2.00, 45.0);
      kSpeakerAngleMap.put(2.25, 45.0);
      kSpeakerAngleMap.put(2.50, 46.0);
      // kSpeakerAngleMap.put(3.00, 49.0);
      // kSpeakerAngleMap.put(4.00, 38.0);
    }

    public static final Translation2d kRedSpeaker = new Translation2d(16.32, 5.55);
    public static final Translation2d kBlueSpeaker = new Translation2d(0.26, 5.54);
  }

  public static final class FiendConstants {
    public static final Pose2d BELOW_SPEAKER_POSITION_BLUE = new Pose2d(new Translation2d(1.42, 5.54),
        new Rotation2d(0.0));
    public static final Pose2d BELOW_SPEAKER_POSITION_RED = new Pose2d(new Translation2d(15.289, 5.54),
        new Rotation2d(Units.degreesToRadians(180)));
    public static final Pose2d IN_FRONT_AMP_POSITION_BLUE = new Pose2d(new Translation2d(1.79, 7.84),
        new Rotation2d(Units.degreesToRadians(90.0)));
    public static final Pose2d IN_FRONT_AMP_POSITION_RED = new Pose2d(new Translation2d(14.742, 7.84),
        new Rotation2d(Units.degreesToRadians(90.0)));
    public static final Pose2d NOTE_51_POSITION = new Pose2d(new Translation2d(8.30, 7.45),
        new Rotation2d(0.0));
    public static final Pose2d NOTE_52_POSITION = new Pose2d(new Translation2d(8.30, 5.77), new Rotation2d(0.0));
    public static final Pose2d NOTE_53_POSITION = new Pose2d(new Translation2d(8.30, 4.11), new Rotation2d(0.0));
    public static final Pose2d NOTE_54_POSITION = new Pose2d(new Translation2d(8.30, 2.43), new Rotation2d(0.0));
    public static final Pose2d NOTE_55_POSITION = new Pose2d(new Translation2d(8.30, 0.78), new Rotation2d(0.0));
  }
}
