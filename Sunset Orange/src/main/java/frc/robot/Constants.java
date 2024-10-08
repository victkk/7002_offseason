// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.path.PathConstraints;

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
      FL.azimuthEncoderOffsetRotation = -0.047607;
      FL.lightGateID = 0;
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
      FR.azimuthEncoderOffsetRotation = 0.131836;
      FR.lightGateID = 1;
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
      BL.azimuthEncoderOffsetRotation = -0.479980;
      BL.lightGateID = 2;
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
      BR.azimuthEncoderOffsetRotation = 0.054443;
      BR.lightGateID = 3;
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




  public static final class ClimberConstants{
    public static final int LEFT_CLIMBER_ID =15; 
    public static final int RIGHT_CLIMBER_ID =16;
    public static final double MAX_TRAVEL_DIST =15;
    public static final double DIAMETER=0.10;
    public static final double CLIMBER_GEAR_RATIO=84.0/18.0*40.0/16.0;

  }

  public static final class IntakerConstants{
    // motor id for Intaker arm and roller
    public static final int ROLLER_ID =10; 
    public static final int ARM_ID =14;
     
    //arm angle when the intaker rests on shooter
    public static final double REST_ANGLE = 13.0;
    public static final double REST_POSITION = REST_ANGLE / 360.0;

    //arm angle when the intaker rests on ground
    public static final double MAX_ANGLE = 193.0;
    public static final double MAX_POSITION = MAX_ANGLE / 360.0;

    //arm angle when the intaker feed to shooter
    public static final double FEED_ANGLE=25.0;
    public static final double FEED_POSITION = FEED_ANGLE / 360.0;
  
    //arm angle when the intaker intake
    public static final double INTAKE_ANGLE=175.0;
    public static final double INTAKE_POSITION = INTAKE_ANGLE/360.0;                                                   

    public static final double AMP_ANGLE=78.0;
    public static final double AMP_POSITION = AMP_ANGLE / 360.0;

      public static final double ADJUST_ANGLE=60.0;
    public static final double ADJUST_POSITION = ADJUST_ANGLE / 360.0;
    //arm gear ratio
    public static final double ARM_GEAR_RATIO=24;

    // used for stop intaking
    public static final int INTAKER_ENTER_OMRON_ID = 9;

    //roller
    public static final double ROLLER_FEED_VOLTAGE=-12;
    public static final double ROLLER_INTAKE_VOLTAGE=10;
    public static final double ROLLER_AMP_VOLTAGE=-3.8;
  }
  public static final class ShooterConstants {
    public static final int SHOOTER_ID = 11;
    public static final int SHOOTER_FOLLOWER = 13;
    public static final double GEAR_RATIO = 20.0/32.0;
    public static final double SHOOT_RPS =40;
    public static final double VISION_RPS = 40;

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

    public static final Translation2d kRedSpeaker = new Translation2d(16.58, 5.55);
    public static final Translation2d kBlueSpeaker = new Translation2d(0.00, 5.54);
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
  
  public static final class PathfindConstants {
    public static final PathConstraints constraints =
        new PathConstraints(0.5, 5.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
  }
}
