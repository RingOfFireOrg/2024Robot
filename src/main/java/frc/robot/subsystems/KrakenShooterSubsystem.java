package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


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


  /* ---------- */
  //Distance(meters), TopMotorSpeed, BottomMotorSpeed

  //Bottom inc -> farther angle
  InterpolatingDoubleTreeMap topShooterRPMTreeMap = new InterpolatingDoubleTreeMap();
  InterpolatingDoubleTreeMap bottomShooterRPMTreeMap = new InterpolatingDoubleTreeMap();
  private final double[][] speedTable = {
    {1.0, 3200, 3200},
    {1.0, 3200, 3200},
    {1.0, 3200, 3200}
  };

  
  

  
  /* ---------- */



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

    var MotionMagicConfig = shooterConfig.MotionMagic;
    MotionMagicConfig.MotionMagicAcceleration = 200; 
    MotionMagicConfig.MotionMagicJerk = 4000; 

    shooterMotorTop.getConfigurator().apply(shooterConfig);
    shooterMotorBottom.getConfigurator().apply(shooterConfig);

    SmartDashboard.getNumber("kr_Amp RPM Top", 500);
    SmartDashboard.getNumber("kr_Amp RPM Bottom", 500);

    SmartDashboard.getNumber("kr_Top Table Test", 500);
    SmartDashboard.getNumber("kr_Bottom Table Test", 500);

    for (double[] pair : speedTable) {
      topShooterRPMTreeMap.put(pair[0], pair[1]);
    }

    for (double[] pair : speedTable) {
      bottomShooterRPMTreeMap.put(pair[0], pair[2]);
    }
    /* Follow the Top Shooter */
    //shooterMotorBottom.setControl(new Follower(30, false));  
  }


  @Override
  public void periodic() {
    
    double rotorvelocityTOP = -shooterMotorTop.getRotorVelocity().getValueAsDouble()*60;
    SmartDashboard.putNumber("kr_Top rpm", rotorvelocityTOP);
    SmartDashboard.putNumber("kr_Top rps", rotorvelocityTOP/60);
    double rotorVelocityBOTTOM  = shooterMotorBottom.getRotorVelocity().getValueAsDouble()*60;
    SmartDashboard.putNumber("kr_Bottom rpm", rotorVelocityBOTTOM);
    SmartDashboard.putNumber("kr_Bottom rps", rotorVelocityBOTTOM/60);
    
    
    SmartDashboard.putString("kr_Shooter Status", krakenShooterSubsystemStatus.toString());





    //SmartDashboard.putNumber("kr_Amp RPM", 900);

    if (rotorvelocityTOP >= 3000 && rotorVelocityBOTTOM >= 3000) {
      krakenShooterSubsystemStatus = KrakenShooterSubsystemStatus.READY;
    }
    else if (rotorvelocityTOP >= 100 && rotorVelocityBOTTOM >= 100) {
      krakenShooterSubsystemStatus = KrakenShooterSubsystemStatus.REVING;
    }
    else if (rotorvelocityTOP <= -100 && rotorVelocityBOTTOM <= -100) {
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
    shooterMotorBottom.setControl(mmvv.withVelocity(velocity*(maxRPMTeleOp/60)));
  }

  public void setRPM(double rpm){
    shooterMotorTop.setControl(mmvv.withVelocity(rpm/60));
    shooterMotorBottom.setControl(mmvv.withVelocity(rpm/60));
  }

  public void setRPM(double rpmTop, double rpmBottom){
    shooterMotorTop.setControl(mmvv.withVelocity(rpmTop/60));
    shooterMotorBottom.setControl(mmvv.withVelocity(rpmBottom/60));
  }

  public void setRPS(double rps){
    shooterMotorTop.setControl(mmvv.withVelocity(rps));
    shooterMotorBottom.setControl(mmvv.withVelocity(rps));

  }

  public void setMotor(double speed){
    shooterMotorTop.set(speed);
    shooterMotorBottom.set(speed);
  }

  public void stopMotors(){
    shooterMotorTop.stopMotor();
    shooterMotorBottom.stopMotor();
  }

  public double getRPMfromDistance(InterpolatingDoubleTreeMap treeMap, double distance) {
    return treeMap.get(distance);
  }

  public Command ampSpeed() {
    return this.run(() -> setRPM(
      (-SmartDashboard.getNumber("kr_Amp RPM Top", 500)),
      (-SmartDashboard.getNumber("kr_Amp RPM Bottom", 500))
      ));
  }

  public Command changeSpeed() {
    return this.run(() -> setRPM(
      (-SmartDashboard.getNumber("kr_Top Table Test", 500)),
      (-SmartDashboard.getNumber("kr_Bottom Table Test", 500))));
  }

  public Command sendableSpeed() {
    return this.run(() -> setRPM(
      (-SmartDashboard.getNumber("kr_Top Table Test", 500)),
      (-SmartDashboard.getNumber("kr_Bottom Table Test", 500))));
  }

  public Command distanceShot(double distance) {
    return this.run(() -> setRPM(
      (getRPMfromDistance(topShooterRPMTreeMap, distance)),
      (getRPMfromDistance(bottomShooterRPMTreeMap, distance))
    ));
    //return new InstantCommand();
  }

  public Command rpmCMD(double rpm) {
    return this.run(() -> setRPM(rpm));
  }

  public Command intakeSpeed() {
    return this.run(() -> setRPM(1300));
  }

  public Command stopMotorsCMD() {
    return this.runOnce(() -> stopMotors());
  }
  

}
