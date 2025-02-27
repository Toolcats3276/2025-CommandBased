
package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase{

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;
  private int m_rainbowFirstPixelHue;
  private Color LED_COLOR;
  
    public LED() {
      m_ledBuffer = new AddressableLEDBuffer(20);
      m_led = new AddressableLED(9);
      m_led.setLength(m_ledBuffer.getLength());
      m_led.start();
  
      m_rainbowFirstPixelHue = 0;
    }
  
    public enum Mode{
      off, 
      on,
      rainbow,
      Infeeding,
      Infeed_Done,
      Coral, 
      Algae
    }
  
    Mode LED_Mode = Mode.off;
  
    @Override
    public void periodic() {
        
      switch(LED_Mode){
  
        case off:{
          for(var i = 0; i<m_ledBuffer.getLength(); i++) {
            //sets rgb value
            m_ledBuffer.setRGB(i, 0, 0, 0);
          }
          break;
        }
  
        case on:{
          for(var i = 0; i<m_ledBuffer.getLength(); i++){
            //sets rgb value
            m_ledBuffer.setLED(i, LED_COLOR);
          }
          break;
        }
  
        case Infeeding:{
          for(var i = 0; i<m_ledBuffer.getLength(); i++){
            //sets rgb value
            m_ledBuffer.setLED(i, Color.kRed);
          }
          break;
        }
  
        case Infeed_Done:{
          for(var i = 0; i<m_ledBuffer.getLength(); i++){
            //sets rgb value
            m_ledBuffer.setLED(i, Color.kGreen);
          }
          break;
        }
  
        case rainbow:{
          for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
            // Set the value
            m_ledBuffer.setHSV(i, hue, 255, 128);
            }
            // Increase by to make the rainbow "move"
             m_rainbowFirstPixelHue += 3;
            // Check bounds
            m_rainbowFirstPixelHue %= 180;
            break;
        }

        case Coral:{
          for(var i = 0; i<m_ledBuffer.getLength(); i++){
            //sets rgb value
            m_ledBuffer.setLED(i, Color.kBlanchedAlmond);
          }
          break;
        }

        case Algae:{
          for(var i = 0; i<m_ledBuffer.getLength(); i++){
            //sets rgb value
            m_ledBuffer.setLED(i, Color.kDarkGreen);
          }
          break;
        }
  
      }
  
      m_led.setData(m_ledBuffer);
  
    }
  
    public void off(){
      LED_Mode = Mode.off;
      System.out.println("Off");
      m_led.setData(m_ledBuffer);
    }
  
    public void on(Color LED_COLOR){
      LED_Mode = Mode.on;
      this.LED_COLOR = LED_COLOR;
    m_led.setData(m_ledBuffer);
  }

  public void Infeeding(){
    LED_Mode = Mode.Infeeding;
    m_led.setData(m_ledBuffer);
  }

  public void Infeed_Done(){
    LED_Mode = Mode.Infeed_Done;
    m_led.setData(m_ledBuffer);
  }

  public void rainbow() {
    LED_Mode = Mode.rainbow;
    System.out.println("Rainbow");
    m_led.setData(m_ledBuffer);
  }


}