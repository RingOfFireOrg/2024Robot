// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

  private int ledSegment1 = 31;
  private int ledSegment2 = 41;
  private int ledSegment3 = 31;


  int ledLengthBar = 31+41+31;
  int ledSegment1Start = 0; // Left bar
  int ledSegment2Start = 31; // top Bar
  int ledSegment3Start = 31+41; // Right Bar
  int ledSegmentTotal = 31+41+31;
  int ledLegnthBoard = 0;

  public LEDSubsystem() {

    m_led = new AddressableLED(8);

    m_ledBuffer = new AddressableLEDBuffer(ledLengthBar);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  @Override
  public void periodic() {
    m_led.setData(m_ledBuffer);
  }


  

  public void setLed(String pattern) {  //TODO: Replace this with switch & case ??
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


  // --------------------------------- SET RGB FUNCTIONS ------------------------------------- \\

  public void setLEDRGB(int red, int green, int blue) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, red, green, blue);
    }    
    m_led.setData(m_ledBuffer);
  }

  public void setLEDRGB_BAR(int red, int green, int blue) {
    for (var i = 0; i < ledSegment2Start; i++) {
      m_ledBuffer.setRGB(ledSegment2Start-i-1, red, green, blue);
      m_ledBuffer.setRGB(i + ledSegment3Start, red, green, blue);
    }       
    m_led.setData(m_ledBuffer);
  }

  public void setLEDRGB_TOP(int red, int green, int blue) {
    for (var i = 0; i < ledSegment2; i++) {
      m_ledBuffer.setRGB(ledSegment2Start+i, red, green, blue);
    }    
    m_led.setData(m_ledBuffer);
  }


  public void setLEDHSV(int h, int s, int v) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setHSV(i, h, s, v);
    }    
    m_led.setData(m_ledBuffer);
  }


  // ------------------------------------ Red chase ------------------------------------------------- \\
  //                                  

  public void redMoveSplit() {
    for (var i = 0; i < ledSegment2Start; i++) {
      final var hue = (m_rainbowFirstPixelHue + (i * 255 / 30)) % 255;
      m_ledBuffer.setRGB(ledSegment2Start-i-1, 255-hue, 0, 0);
      m_ledBuffer.setRGB(i + ledSegment3Start, 255-hue, 0, 0);
    }
    m_rainbowFirstPixelHue += 6;
    m_rainbowFirstPixelHue %= 255;
    m_led.setData(m_ledBuffer);
  } 


  public void redMoveSplit_REVERSE() {
    for (var i = 0; i < ledSegment2Start; i++) {
      final var hue = (m_rainbowFirstPixelHue + (i * 255 / 30)) % 255;
      m_ledBuffer.setRGB(ledSegment1Start +i, 255-hue, 0, 0);
      m_ledBuffer.setRGB(ledSegmentTotal - i - 1, 255-hue, 0, 0);
    }
    m_rainbowFirstPixelHue += 6;
    m_rainbowFirstPixelHue %= 255;
    m_led.setData(m_ledBuffer);
  } 





/*  --------------------------------Shifting orange Pattern for Red Alliance--------------------------------------- */   
  int firstOrange = 2;
  boolean flip = false;

  public void shiftingOrange() {
    if (flip == false) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, firstOrange, 0);
      }
      firstOrange += 1;
      m_led.setData(m_ledBuffer);
      if (firstOrange >= 140) {
        flip = true;
      }
    }
    if (flip == true) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, firstOrange, 0);
      }
      firstOrange -= 1;
      m_led.setData(m_ledBuffer);
      if (firstOrange <= 5) {
        flip = false;
      }
    }
  }


  // int firstOrange_Bar = 2;
  // boolean flip_Bar = false;

  public void shiftingOrange_BAR() {
    if (flip == false) {
      for (var i = 0; i < ledSegment2Start; i++) {
        m_ledBuffer.setRGB(ledSegment1Start + i, 255, firstOrange, 0);
        m_ledBuffer.setRGB(ledSegment3Start + i, 255, firstOrange, 0);
      }
      firstOrange += 1;
      m_led.setData(m_ledBuffer);
      if (firstOrange >= 140) {
        flip = true;
      }
    }
    if (flip == true) {
      for (var i = 0; i < ledSegment2Start; i++) {
        m_ledBuffer.setRGB(ledSegment1Start + i, 255, firstOrange, 0);
        m_ledBuffer.setRGB(ledSegment3Start + i, 255, firstOrange, 0);;

      }
      firstOrange -= 1;
      m_led.setData(m_ledBuffer);
      if (firstOrange <= 5) {
        flip = false;
      }
    }
  }



  /*  ------------------------------------------------------------------------------------ */   



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
