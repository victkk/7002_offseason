package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GamePieceProcessor;

public class FollowNoteCommand extends Command {

  private final DrivetrainSubsystem drivetrainSubsystem;
  private final GamePieceProcessor gamePieceProcessor;
  private Translation2d targetPosition;
  private Rotation2d targetAngle;
  private final double positionTolerance = 0.1; // 目标位置的容忍度，单位为米
  private final double angleTolerance = 5; // 目标角度的容忍度，单位为度

  private final PIDController xPIDController = new PIDController(1.0, 0.0, 0.0);
  private final PIDController yPIDController = new PIDController(1.0, 0.0, 0.0);
  private final PIDController rotationPIDController = new PIDController(1.0, 0.0, 0.0);

  private final double maxTranslationSpeed = 1.0; // 最大平移速度
  private final double maxRotationSpeed = 1.0; // 最大旋转速度

  private final double xFeedforward = 0.0; // X轴前馈参数
  private final double yFeedforward = 0.0; // Y轴前馈参数
  private final double rotationFeedforward = 0.0; // 旋转前馈参数

  public FollowNoteCommand(DrivetrainSubsystem drivetrainSubsystem, GamePieceProcessor gamePieceProcessor) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.gamePieceProcessor = gamePieceProcessor;
    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    // 获取目标信息
    var targetOptional = gamePieceProcessor.getClosestGamePieceInfo();
    if (targetOptional.isPresent()) {
      var target = targetOptional.get();
      targetPosition = gamePieceProcessor.robotToPiece(target);
      targetAngle = new Rotation2d(targetPosition.getX(), targetPosition.getY()).minus(drivetrainSubsystem.getHeading());
    } else {
      targetPosition = new Translation2d(0, 0); // 如果没有检测到目标，默认位置为原点
      targetAngle = drivetrainSubsystem.getHeading(); // 如果没有检测到目标，默认角度为当前角度
    }

    // 设置PID控制器的容忍度
    xPIDController.setTolerance(positionTolerance);
    yPIDController.setTolerance(positionTolerance);
    rotationPIDController.setTolerance(angleTolerance);
  }

  @Override
  public void execute() {
    // 获取当前底盘位置和角度
    var currentPosition = drivetrainSubsystem.getPose().getTranslation();
    var currentAngle = drivetrainSubsystem.getHeading();

    // 计算旋转速度
    double rotationSpeed = rotationPIDController.calculate(currentAngle.getDegrees(), targetAngle.getDegrees()) + rotationFeedforward;
    rotationSpeed = normalizeSpeed(rotationSpeed, maxRotationSpeed);

    // 如果旋转速度在容忍度范围内，开始计算平移速度
    if (Math.abs(rotationSpeed) < 0.1) {
      double xSpeed = xPIDController.calculate(currentPosition.getX(), targetPosition.getX()) + xFeedforward;
      double ySpeed = yPIDController.calculate(currentPosition.getY(), targetPosition.getY()) + yFeedforward;
      xSpeed = normalizeSpeed(xSpeed, maxTranslationSpeed);
      ySpeed = normalizeSpeed(ySpeed, maxTranslationSpeed);

      // 使用场中心模式（field-centric mode）
      boolean fieldCentric = true;

      // 驱动底盘
      drivetrainSubsystem.drive(new Translation2d(xSpeed, ySpeed), rotationSpeed, fieldCentric);
    } else {
      // 如果旋转速度不在容忍度范围内，只进行旋转
      drivetrainSubsystem.drive(new Translation2d(0, 0), rotationSpeed, true);
    }
  }

  // 归一化速度到合理的范围内
  private double normalizeSpeed(double speed, double maxSpeed) {
    return Math.max(-maxSpeed, Math.min(maxSpeed, speed));
  }

  @Override
  public boolean isFinished() {
    // 获取当前底盘位置和角度
    var currentPosition = drivetrainSubsystem.getPose().getTranslation();
    var currentAngle = drivetrainSubsystem.getHeading();

    // 如果当前位置在目标位置的容忍度范围内且当前角度在目标角度的容忍度范围内，命令结束
    boolean positionReached = currentPosition.getDistance(targetPosition) < positionTolerance;
    boolean angleReached = Math.abs(targetAngle.minus(currentAngle).getDegrees()) < angleTolerance;

    return positionReached && angleReached;
  }

  @Override
  public void end(boolean interrupted) {
    // 停止驱动底盘
    drivetrainSubsystem.drive(new Translation2d(0, 0), 0, true);
  }
}