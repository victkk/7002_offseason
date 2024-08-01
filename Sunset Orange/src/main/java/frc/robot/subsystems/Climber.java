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
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
    private final TalonFX leftClimberMotor;
    private final TalonFX rightClimberMotor;
    private final DutyCycleOut leftRequest = new DutyCycleOut(0.0);
    private final DutyCycleOut rightRequest = new DutyCycleOut(0.0);  
    public Climber() {
        leftClimberMotor = new TalonFX(ClimberConstants.LEFT_CLIMBER_ID);
        rightClimberMotor = new TalonFX(ClimberConstants.RIGHT_CLIMBER_ID);

        // Set the right motor to follow the left motor
        rightClimberMotor.setControl(new Follower(leftClimberMotor.getDeviceID(), false));

        // Configure current limits
        var leftClimberConfigs = new TalonFXConfiguration();
        var rightClimberConfigs = new TalonFXConfiguration();
            

        var leftClimberConfig = new TalonFXConfiguration();
        var mSoftLimitConf  = new SoftwareLimitSwitchConfigs();
        leftClimberConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftClimberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        mSoftLimitConf.ForwardSoftLimitThreshold = ClimberConstants.MAX_TRAVEL_DIST/ClimberConstants.DIAMETER/Math.PI;
        mSoftLimitConf.ForwardSoftLimitEnable = true;
        mSoftLimitConf.ReverseSoftLimitThreshold = 0;
        mSoftLimitConf.ReverseSoftLimitEnable = true;
        leftClimberConfig.SoftwareLimitSwitch = mSoftLimitConf;
        leftClimberConfig.Voltage.PeakForwardVoltage = 12.0;
        leftClimberConfig.Voltage.PeakReverseVoltage = -12.0;
        leftClimberConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
        leftClimberConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
        leftClimberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leftClimberConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        leftClimberConfig.CurrentLimits.SupplyCurrentThreshold = 60.0;
        leftClimberConfig.CurrentLimits.SupplyTimeThreshold = 0.5;
        leftClimberConfig.Feedback.SensorToMechanismRatio = ClimberConstants.CLIMBER_GEAR_RATIO;

        leftClimberMotor.getConfigurator().apply(leftClimberConfigs);
        rightClimberMotor.getConfigurator().apply(rightClimberConfigs);
        leftClimberMotor.setPosition(0,Constants.kLongCANTimeoutSec);   
        rightClimberMotor.setPosition(0,Constants.kLongCANTimeoutSec);
    }

    public void setClimberDuty(double duty) {
        leftClimberMotor.setControl(leftRequest.withOutput(duty));
        rightClimberMotor.setControl(rightRequest.withOutput(duty));
    }

    public void stop() {
        leftClimberMotor.setControl(leftRequest.withOutput(0));
        leftClimberMotor.setControl(leftRequest.withOutput(0));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
