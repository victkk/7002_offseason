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

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakerConstants;

public class Intaker extends SubsystemBase {
    private final TalonFX mRollerTalon;
    private final TalonFX mArmTalon;
 
    private final VoltageOut IntakeVoltage = new VoltageOut(0);
    private final MotionMagicVoltage ArmMotionMagic = new MotionMagicVoltage(ArmConstants.ARM_REST_POSITION);

    public Intaker() {
        mRollerTalon = new TalonFX(IntakerConstants.ROLLER_ID);
        mArmTalon = new TalonFX(IntakerConstants.ARM_ID);

        //arm configs       
        var armConfig = new TalonFXConfiguration();
        var armSoftLimitConf  = new SoftwareLimitSwitchConfigs();
        armConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        armSoftLimitConf.ForwardSoftLimitThreshold = IntakerConstants.REST_POSITION-0.05;
        armSoftLimitConf.ForwardSoftLimitEnable = true;
        armSoftLimitConf.ReverseSoftLimitThreshold = IntakerConstants.MAX_POSITION+0.05;
        armSoftLimitConf.ReverseSoftLimitEnable = true;
        armConfig.SoftwareLimitSwitch = armSoftLimitConf;
        armConfig.Voltage.PeakForwardVoltage = 12.0;
        armConfig.Voltage.PeakReverseVoltage = -12.0;
        armConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
        armConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
        armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        armConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        armConfig.CurrentLimits.SupplyCurrentThreshold = 60.0;
        armConfig.CurrentLimits.SupplyTimeThreshold = 0.5;
  
        armConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        armConfig.Slot0.kG = 0.1;
        armConfig.Slot0.kV = 3.028;
        armConfig.Slot0.kP = 10.0;
        armConfig.Slot0.kI = 0.0;
        armConfig.Slot0.kD = 0.0;
        armConfig.MotionMagic.MotionMagicJerk = 0.0;
        // seems mechanism rotation not rotor position
        armConfig.MotionMagic.MotionMagicCruiseVelocity = 2.5;
        armConfig.MotionMagic.MotionMagicAcceleration = 0.5;
        armConfig.Feedback.SensorToMechanismRatio = IntakerConstants.ARM_GEAR_RATIO;
        
        mArmTalon.getConfigurator().apply(armConfig);
        mArmTalon.setPosition(IntakerConstants.REST_POSITION,Constants.kLongCANTimeoutSec);   
        mArmTalon.optimizeBusUtilization();
        
        //roller configs
        TalonFXConfiguration rollerConfigs = new TalonFXConfiguration();
        rollerConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rollerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        rollerConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        rollerConfigs.Voltage.PeakForwardVoltage = 12.0;
        rollerConfigs.Voltage.PeakReverseVoltage = -12.0;
        rollerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollerConfigs.CurrentLimits.SupplyCurrentLimit = 20.0;
        rollerConfigs.CurrentLimits.SupplyCurrentThreshold = 20.0;
        rollerConfigs.CurrentLimits.SupplyTimeThreshold = 0.0;
        mRollerTalon.getConfigurator().apply(rollerConfigs);
        mRollerTalon.setControl(Constants.NEUTRAL);
        mRollerTalon.optimizeBusUtilization();
    }

    public void setAngle(double angle_deg) {

        if (angle_deg < IntakerConstants.REST_ANGLE || angle_deg > IntakerConstants.MAX_ANGLE){
          return;}

        double angle_rotation = angle_deg / 360.0;
        mArmTalon.setControl(ArmMotionMagic.withPosition(angle_rotation)); // reuse previously created
        
        // ControlRequest Instance
      }
    
      
      public void setRollerIntake() {
        mRollerTalon.setControl(IntakeVoltage.withOutput(6));
        // ControlRequest Instance
      }
      public void setRollerFeed() {
        mRollerTalon.setControl(IntakeVoltage.withOutput(-3));
        // ControlRequest Instance
      }
      public double getRotation() {
        return mArmTalon.getPosition().getValueAsDouble();
      }
    
      public double getStatorCurrent() {
        return mArmTalon.getStatorCurrent().getValueAsDouble();
      }
    
      public double getSupplyCurrent() {
        return mArmTalon.getSupplyCurrent().getValueAsDouble();
      }
    
      public double getTargetAngleDeg() {
        if (mArmTalon.getAppliedControl().getClass() == MotionMagicVoltage.class) {
          return ArmMotionMagic.Position * 360.0; // last applied motion magic value
        } else {
          return Double.NaN;
        }
      }
    
      public double getAngleDeg() {
        return getRotation() * 360.0;
      }
      
    public void stop() {
        mArmTalon.setPosition(IntakerConstants.FEED_POSITION);
        mRollerTalon.setControl(IntakeVoltage.withOutput(0.0));
    }
    public void stopRoller() {
        mRollerTalon.setControl(IntakeVoltage.withOutput(0.0));
    }
    @Override
    public void periodic() {
      SmartDashboard.putNumber("Current Position", getRotation());
      SmartDashboard.putNumber("target Position", getTargetAngleDeg());
      SmartDashboard.putNumber("Stator Current", getStatorCurrent());
      SmartDashboard.putNumber("Supply Current", getSupplyCurrent());
    }
      @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addStringProperty(
        getName() + "Intake Motor Control Request",
        () -> mArmTalon.getAppliedControl().toString(),
        null);


  }
}
