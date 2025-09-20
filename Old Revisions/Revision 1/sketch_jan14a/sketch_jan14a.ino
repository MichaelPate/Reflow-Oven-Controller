#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

// Software SPI (slower updates, more flexible pin options):
// pin 7 - Serial clock out (SCLK)
// pin 6 - Serial data out (DIN)
// pin 5 - Data/Command select (D/C)
// pin 4 - LCD chip select (CS)
// pin 3 - LCD reset (RST)
Adafruit_PCD8544 display = Adafruit_PCD8544(7,6,5,4,3);

void setup() {
  // put your setup code here, to run once:
  display.begin();
  display.clearDisplay();
  delay(1000);
  display.setContrast(60);
  display.clearDisplay();
  delay(500);

  // Display cycle stage
  display.setTextSize(2);
  display.setCursor(0,0);
  display.print("Idle"); //Preheat, Soak, Reflow, Cool, Idle
  display.drawLine(0, 17, 84, 17, BLACK);
  //display.display();

  // Stage time, cycle time, set temp, PCB temp
  // maybe some symbols for what heaters are activated
  // maybe curve name or number
  display.setTextSize(1);
  display.setCursor(0, 19+1);
  display.print("95.7^F");
  display.setCursor(55, 19+1);
  display.print("P:01");
  //display.setTextColor(WHITE,BLACK);
  //display.print("1");
  //display.setCursor(55, 19+1);
  //display.print("2");
  //display.setTextColor(BLACK);
  display.setCursor(0, 19+8+4);
  display.print("Stage: --:--");
  display.setCursor(0, 19+8+8+5);
  display.print("Cycle: --:--");
  display.display();
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
