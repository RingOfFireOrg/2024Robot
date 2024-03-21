package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class KrakenShooterSubsystem extends SubsystemBase {

  private final TalonFX shooterMotorTop;
  private final TalonFX shooterMotorBottom;
  private final MotionMagicVelocityVoltage mmvv = new MotionMagicVelocityVoltage(0);
  private final double maxRPM = 6000; 
  private final double maxRPMTeleOp = 3200;
  public enum KrakenShooterSubsystemStatus {
    READY,
    REVING,
    REVERSE,
    IDLE
  }

  KrakenShooterSubsystemStatus krakenShooterSubsystemStatus = KrakenShooterSubsystemStatus.IDLE;
  public KrakenShooterSubsystem() {
    shooterMotorTop = new TalonFX(30);
    shooterMotorBottom = new TalonFX(31);


    var shooterConfig = new TalonFXConfiguration();
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    var slot0Configs = shooterConfig.Slot0;
    slot0Configs.kS = 0.2;
    slot0Configs.kV = 0.12; 
    slot0Configs.kA = 0.01; 
    slot0Configs.kP = 0.3; 
    slot0Configs.kI = 0; 
    slot0Configs.kD = 0.0001; 

    var leftMotionMagicConfig = shooterConfig.MotionMagic;
    leftMotionMagicConfig.MotionMagicAcceleration = 200; // ?
    leftMotionMagicConfig.MotionMagicJerk = 4000; // ?????

    shooterMotorTop.getConfigurator().apply(shooterConfig);


    /* Follow the Top Shooter */
    shooterMotorBottom.setControl(new Follower(30, false));  
  }


  @Override
  public void periodic() {
    
    double rotorvelocity  = shooterMotorTop.getRotorVelocity().getValueAsDouble()*60;
    SmartDashboard.putNumber("krshooter_rpm Kraken Rotor Top shooter", rotorvelocity);
    SmartDashboard.putNumber("krshooter_rps Kraken Rotor Top shooter", rotorvelocity/60);
    SmartDashboard.putString("kr_Shooter Status", krakenShooterSubsystemStatus.toString());


    if (rotorvelocity >= 3200 ) {
      krakenShooterSubsystemStatus = KrakenShooterSubsystemStatus.READY;
    }
    else if (rotorvelocity >= 100) {
      krakenShooterSubsystemStatus = KrakenShooterSubsystemStatus.REVING;
    }
    else if (rotorvelocity <= -100) {
      krakenShooterSubsystemStatus = KrakenShooterSubsystemStatus.REVERSE;
    }
    else {  
      krakenShooterSubsystemStatus = KrakenShooterSubsystemStatus.IDLE;
    }

  }


  public KrakenShooterSubsystemStatus getStatus() {
    return krakenShooterSubsystemStatus;
  }

  public void setVelocity(double velocity){
    shooterMotorTop.setControl(mmvv.withVelocity(velocity*(maxRPMTeleOp/60)));
  }

  public void setRPM(double rpm){
    shooterMotorTop.setControl(mmvv.withVelocity(rpm/60));
  }

  public void setRPS(double rps){
    shooterMotorTop.setControl(mmvv.withVelocity(rps));
  }

  public void setMotor(double speed){
    shooterMotorTop.set(speed);
  }

  public void stopMotors(){
    shooterMotorTop.stopMotor();
  }

}
