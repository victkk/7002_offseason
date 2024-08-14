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
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakerConstants;

public class Climber extends SubsystemBase {
    private final TalonFX leftClimberMotor;
    private final TalonFX rightClimberMotor;
    private final MotionMagicVoltage leftMotionMagic = new MotionMagicVoltage(0.0);
    private final MotionMagicVoltage rightMotionMagic = new MotionMagicVoltage(0.0);
    
    private final DutyCycleOut leftRequest = new DutyCycleOut(0.0);
    private final DutyCycleOut rightRequest = new DutyCycleOut(0.0);  
    public Climber() {
        leftClimberMotor = new TalonFX(ClimberConstants.LEFT_CLIMBER_ID);
        rightClimberMotor = new TalonFX(ClimberConstants.RIGHT_CLIMBER_ID);


            

        var leftClimberConfig = new TalonFXConfiguration();
        var leftSoftLimitConf  = new SoftwareLimitSwitchConfigs();
        leftClimberConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftClimberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftClimberConfig.SoftwareLimitSwitch = leftSoftLimitConf;
        leftClimberConfig.Voltage.PeakForwardVoltage = 12.0;
        leftClimberConfig.Voltage.PeakReverseVoltage = -12.0;
        leftClimberConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
        leftClimberConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
        leftClimberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leftClimberConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
        leftClimberConfig.CurrentLimits.SupplyCurrentThreshold = 40.0;
        leftClimberConfig.CurrentLimits.SupplyTimeThreshold = 0.0;
        leftClimberConfig.MotionMagic.MotionMagicAcceleration=5.0;
        leftClimberConfig.MotionMagic.MotionMagicCruiseVelocity=5.0;
        leftClimberConfig.Slot0.kP = 0.5;
        leftClimberConfig.Slot0.kI = 0.0;
        leftClimberConfig.Slot0.kD = 2.0;
        leftClimberConfig.Feedback.SensorToMechanismRatio = ClimberConstants.CLIMBER_GEAR_RATIO;
        
        var rightClimberConfig = new TalonFXConfiguration();
        var rightSoftLimitConf  = new SoftwareLimitSwitchConfigs();
        rightClimberConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightClimberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightClimberConfig.SoftwareLimitSwitch = rightSoftLimitConf;
        rightClimberConfig.Voltage.PeakForwardVoltage = 12.0;
        rightClimberConfig.Voltage.PeakReverseVoltage = -12.0;
        rightClimberConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
        rightClimberConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
        rightClimberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rightClimberConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
        rightClimberConfig.CurrentLimits.SupplyCurrentThreshold = 40.0;
        rightClimberConfig.CurrentLimits.SupplyTimeThreshold = 0.0;
        rightClimberConfig.MotionMagic.MotionMagicAcceleration=5.0;
        rightClimberConfig.MotionMagic.MotionMagicCruiseVelocity=5.0;
        rightClimberConfig.Slot0.kP = 0.5;
        rightClimberConfig.Slot0.kI = 0.0;
        rightClimberConfig.Slot0.kD = 2.0;
        rightClimberConfig.Feedback.SensorToMechanismRatio = ClimberConstants.CLIMBER_GEAR_RATIO;

        leftClimberMotor.getConfigurator().apply(leftClimberConfig);
        rightClimberMotor.getConfigurator().apply(rightClimberConfig);
        leftClimberMotor.setPosition(0,Constants.kLongCANTimeoutSec);   
        rightClimberMotor.setPosition(0,Constants.kLongCANTimeoutSec);
        leftClimberMotor.optimizeBusUtilization();
        rightClimberMotor.optimizeBusUtilization();
    }
    
    public void setClimberDuty(double duty1,double duty2) {
        leftClimberMotor.setControl(leftRequest.withOutput(duty1));
        rightClimberMotor.setControl(rightRequest.withOutput(duty2));
    }

    public void stop() {
        leftClimberMotor.setControl(leftRequest.withOutput(0));
        leftClimberMotor.setControl(leftRequest.withOutput(0));
    }


    public void setPosition(double rotation) { // reuse previously created
        leftClimberMotor.setControl(leftMotionMagic.withPosition(rotation));
        rightClimberMotor.setControl(rightMotionMagic.withPosition(rotation));
      }
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
