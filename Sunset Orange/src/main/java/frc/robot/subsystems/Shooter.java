package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private final TalonFX mLeftTalon;
    private final TalonFX mRightTalon;
    
  private final VelocityVoltage shooterTargetVelocity = new VelocityVoltage(0);

    public Shooter() {
        mLeftTalon = new TalonFX(ShooterConstants.SHOOTER_ID);
        mRightTalon = new TalonFX(ShooterConstants.SHOOTER_FOLLOWER);

        
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        shooterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        shooterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        shooterConfig.Voltage.PeakForwardVoltage = 12.0;
        shooterConfig.Voltage.PeakReverseVoltage = -12.0;
        shooterConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
        shooterConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
        shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        shooterConfig.CurrentLimits.SupplyCurrentThreshold = 60.0;
        shooterConfig.CurrentLimits.SupplyTimeThreshold = 0.5;
        shooterConfig.Slot0.kV = 0.213;
        shooterConfig.Slot0.kP = 0.25;
        shooterConfig.Slot0.kI = 3.0;
        shooterConfig.Slot0.kD = 0.0;
        shooterConfig.Feedback.SensorToMechanismRatio = ShooterConstants.GEAR_RATIO;        
        mLeftTalon.getConfigurator().apply(shooterConfig);
        mLeftTalon.setControl(Constants.NEUTRAL);
        mLeftTalon.optimizeBusUtilization();

        shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        mRightTalon.getConfigurator().apply(shooterConfig);
        mRightTalon.setControl(Constants.NEUTRAL);
        mRightTalon.optimizeBusUtilization();
        
        mLeftTalon.getVelocity().setUpdateFrequency(100, Constants.kLongCANTimeoutSec);
        mRightTalon.getVelocity().setUpdateFrequency(100, Constants.kLongCANTimeoutSec);
    }
    public void setSpeed(double target_rps){
        shooterTargetVelocity.Velocity = target_rps;
        mLeftTalon.setControl(shooterTargetVelocity);
        mRightTalon.setControl(shooterTargetVelocity);
    }
    public void stop() {
        mLeftTalon.setControl(Constants.NEUTRAL);
        mRightTalon.setControl(Constants.NEUTRAL);
    }


    public double getMainMotorVelocity() {

        return mLeftTalon.getVelocity().getValueAsDouble();
      }
    
      public double getFollowerVelocity() {

      
        return mRightTalon.getVelocity().getValueAsDouble();
      }
    

    @Override
    public void periodic() {
    }

    @Override
    public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty(getName()+"Main RPS", ()->getMainMotorVelocity(), null);
      
      builder.addDoubleProperty(getName()+"Follower RPS", ()->getFollowerVelocity(), null);

 
  }
}
