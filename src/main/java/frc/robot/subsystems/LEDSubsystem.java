// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ShooterSubsystem.ShooterSubsystemStatus;

public class LEDSubsystem extends SubsystemBase {


  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  // Store what the last hue of the first pixel is
  private int m_rainbowFirstPixelHue;
  private int variable1 = 0;
  private boolean variable2 = true;  
  private int variable3 = 0;
  private int hue;
  private int hue2;



  public LEDSubsystem() {

    m_led = new AddressableLED(8);

    m_ledBuffer = new AddressableLEDBuffer(60);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_led.setData(m_ledBuffer);
  }


  

  public void setLed(String pattern) {  //TODO: Replace this with switch & case
    if (pattern == "rainbow") {
      rainbow();
    }
    else if (pattern == "redGradient") {
      redGradient();
    }
    else if (pattern == "redChase") {
      redChase();
    }
    else if (pattern == "blueGradient" ) {
      blueGradient();
    }
  }

  public void ledMasterset(ShooterSubsystemStatus shooterSubsystemStatus) {
    if (shooterSubsystemStatus == ShooterSubsystem.ShooterSubsystemStatus.IDLE) {
      rainbow();
    }
    if (shooterSubsystemStatus == ShooterSubsystem.ShooterSubsystemStatus.REVING) {
      redGradient();
    }
  }

  public enum ledModes {
    rainbow,
    redGradient,
    redChase,
    blueGradient
  }

  public void setLedCase(ledModes ledMode) {
    switch (ledMode) {
      case rainbow:
        rainbow();
        break;
      case redGradient:
        redGradient();
        break;
      case redChase:
        redChase();
        break;
      case blueGradient:
        blueGradient();
        break;
    }
  }



  public void setLEDRGB(int red, int blue, int green) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, red, blue, green);
    }    
    m_led.setData(m_ledBuffer);
  }


  public void setLEDHSV(int h, int s, int v) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setHSV(i, h, s, v);
    }    
    m_led.setData(m_ledBuffer);
  }







  public void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    m_rainbowFirstPixelHue += 3;
    m_rainbowFirstPixelHue %= 180;
    m_led.setData(m_ledBuffer);
  }
  
  

  public void gradientTest() {
    // For every pixel
    var range = 255;

    if (variable2 == true) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {

      final var hue = (m_rainbowFirstPixelHue + (i * range / m_ledBuffer.getLength()/2 )) % range;

      m_ledBuffer.setRGB(i, 252, variable3 - hue, 3);
      //m_ledBuffer.setHSV(i, m_rainbowFirstPixelHue, m_rainbowFirstPixelHue, variable1);(i, 252, hue, 3);

      }
      variable1 += 1;
      if (variable1 == m_ledBuffer.getLength()) {
        variable2 = false;
        variable3 = hue;
      }
    }
    else {

      for (var i = 0; i < m_ledBuffer.getLength(); i++) {

      final var hue = (m_rainbowFirstPixelHue + (i * range / m_ledBuffer.getLength()/2 )) % range;

      m_ledBuffer.setRGB(i, 252, variable3-hue, 3);
      //m_ledBuffer.setHSV(i, m_rainbowFirstPixelHue, m_rainbowFirstPixelHue, variable1);(i, 252, hue, 3);

      }
      variable1 -= 1;
      if (variable1 == 0) {
        variable2 = true;
        variable3 = hue;
      }
    }






    m_rainbowFirstPixelHue += 1;
    m_rainbowFirstPixelHue %= range;

    m_led.setData(m_ledBuffer);
  }
 
  
  
  public void redGradient() {
    // For every pixel
    var step = 1;
    var range = m_ledBuffer.getLength()*step;

    for (var i = 0; i < m_ledBuffer.getLength(); i++) {

      final var hue = (m_rainbowFirstPixelHue + (i * range / m_ledBuffer.getLength()/2 )) % range;

      m_ledBuffer.setRGB(i, 252, hue, 3);
      //m_ledBuffer.setHSV(i, m_rainbowFirstPixelHue, m_rainbowFirstPixelHue, variable1);(i, 252, hue, 3);

    }
    if (variable1 == m_ledBuffer.getLength()) {
      variable2 = false;
    }
    else if (variable1 == 0) {
      variable2 = true;
    }

    if (variable2 == true) {
      m_rainbowFirstPixelHue += step;
      variable1 += step;
      m_rainbowFirstPixelHue %= range;
      variable1 %= range;
    }
    else {
      m_rainbowFirstPixelHue -= step;
      variable1 -= step;
      m_rainbowFirstPixelHue %= range;
      variable1 %= range;
    }
    m_rainbowFirstPixelHue %= range;

    m_led.setData(m_ledBuffer);
  }




  public void blueGradient() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      m_ledBuffer.setHSV(i, hue, 255, 128);
      m_ledBuffer.setRGB(i, 20, hue, 255);
    }
    m_rainbowFirstPixelHue += 3;
    m_rainbowFirstPixelHue %= 180;
    m_led.setData(m_ledBuffer);

  }


  public void random1() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    m_rainbowFirstPixelHue += 3;
    m_rainbowFirstPixelHue %= 180;
    m_led.setData(m_ledBuffer);
  }  

  public void whiteChase() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      m_ledBuffer.setRGB(i, 255-hue, 255-hue, 255-hue);
    }
    m_rainbowFirstPixelHue += 3;
    m_rainbowFirstPixelHue %= 180;
    m_led.setData(m_ledBuffer);
  }  

  public void redChase() {
    // red chase with dimming black
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      final var hue = (m_rainbowFirstPixelHue + (i * 255 / m_ledBuffer.getLength())) % 255;
      m_ledBuffer.setRGB(i, 255-hue, 0, 0);
    }
    m_rainbowFirstPixelHue += 3;
    m_rainbowFirstPixelHue %= 255;
    m_led.setData(m_ledBuffer);
  }   
  


}
