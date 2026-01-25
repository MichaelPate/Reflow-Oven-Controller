#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptSettings.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterrupt.h>

#define BUZZER 11

#define C0 2
#define C1 3
#define C2 4

#define R0 5
#define R1 6
#define R2 7
#define R3 8

// simple ring buffer
#define BUFFER_SIZE 64
volatile byte buffer[BUFFER_SIZE];
volatile byte head = 0;
volatile byte tail = 0;

void bufferPush(byte data)
{
  byte nextHead = (head + 1) % BUFFER_SIZE;
  if (nextHead == tail) return;  
  buffer[head] = data;
  head = nextHead;
}

void bufferPop(byte *data)
{
  if (head == tail) return false;
  *data = buffer[tail];
  tail = (tail + 1) % BUFFER_SIZE;
  return true;
}

bool bufferIsEmpty()
{
  return head == tail;
}

bool bufferIsFull()
{
  return ((head + 1) % BUFFER_SIZE) == tail;
}

uint8_t bufferCount()
{
  if (head >= tail) return head - tail;
  return BUFFER_SIZE - tail + head;
}


void row0_ISR(void)
{
  // disable interrupt for now
  disablePCINT(digitalPinToPCINT(R0));
  disablePCINT(digitalPinToPCINT(R1));
  disablePCINT(digitalPinToPCINT(R2));
  disablePCINT(digitalPinToPCINT(R3));

  // turn off the columns so we can switch them one at a time
  digitalWrite(C0, LOW);
  digitalWrite(C1, LOW);
  digitalWrite(C2, LOW);
  bool foundKey = false;
  while (foundKey == false)
  {
    // C0 is high, anything on the rows?
    digitalWrite(C0, HIGH);
    if (digitalRead(R0) == HIGH) { bufferPush(7); foundKey = true; }
    else if (digitalRead(R1) == HIGH) { bufferPush(4); foundKey = true; }
    else if (digitalRead(R2) == HIGH) { bufferPush(1); foundKey = true; }
    else if (digitalRead(R3) == HIGH) { bufferPush(10); foundKey = true; }  // 10 is start key
    digitalWrite(C0, LOW);

    digitalWrite(C1, HIGH);
    if (digitalRead(R0) == HIGH) { bufferPush(8); foundKey = true; }
    else if (digitalRead(R1) == HIGH) { bufferPush(5); foundKey = true; }
    else if (digitalRead(R2) == HIGH) { bufferPush(2); foundKey = true; }
    else if (digitalRead(R3) == HIGH) { bufferPush(0); foundKey = true; }
    digitalWrite(C1, LOW);

    digitalWrite(C2, HIGH);
    if (digitalRead(R0) == HIGH) { bufferPush(9); foundKey = true; }
    else if (digitalRead(R1) == HIGH) { bufferPush(6); foundKey = true; }
    else if (digitalRead(R2) == HIGH) { bufferPush(3); foundKey = true; }
    else if (digitalRead(R3) == HIGH) { bufferPush(11); foundKey = true; }  // 11 is stop key
    digitalWrite(C2, LOW);
  }

  // turn the columns back on
  digitalWrite(C0, LOW);
  digitalWrite(C1, LOW);
  digitalWrite(C2, LOW);
  // re-enable interrupts after we are done
  enablePCINT(digitalPinToPCINT(R0));
  enablePCINT(digitalPinToPCINT(R1));
  enablePCINT(digitalPinToPCINT(R2));
  enablePCINT(digitalPinToPCINT(R3));
}

void setup() {
  pinMode(BUZZER, OUTPUT);
  
  pinMode(C0, OUTPUT);
  pinMode(C1, OUTPUT);
  pinMode(C2, OUTPUT);
  pinMode(R0, INPUT);
  pinMode(R1, INPUT);
  pinMode(R2, INPUT);
  pinMode(R3, INPUT);

  Serial.begin(115200);


  // Prepare columns
  digitalWrite(C0, HIGH);
  digitalWrite(C1, HIGH);
  digitalWrite(C2, HIGH);
  attachPCINT(digitalPinToPCINT(R0), row0_ISR, RISING);
  attachPCINT(digitalPinToPCINT(R1), row0_ISR, RISING);
  attachPCINT(digitalPinToPCINT(R2), row0_ISR, RISING);
  attachPCINT(digitalPinToPCINT(R3), row0_ISR, RISING);

  //enable interrupts
}

void loop() {
  // put your main code here, to run repeatedly:

  if (bufferIsEmpty() == false)
  {
    byte d = 0;
    bufferPop(d);
    Serial.println(d);
    delay(100);
  }
  
}
