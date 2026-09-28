#include <Arduino.h>
#include <DFRobotDFPlayerMini.h>


HardwareSerial mySerial(1);
DFRobotDFPlayerMini myDFPlayer;

void printDetail(uint8_t type, int value);

void setup()
{
  Serial.begin(115200);

  // RX = GPIO20
  // TX = GPIO21
  mySerial.begin(9600, SERIAL_8N1, 20, 21);

  if (!myDFPlayer.begin(mySerial)) {
    Serial.println("Unable to begin:");
    Serial.println("1. Please recheck the connection!");
    Serial.println("2. Please insert the SD card!");

    while (true);
  }

  Serial.println("DFPlayer Mini online.");

  myDFPlayer.setTimeOut(500);

  // Volume 0-30
  myDFPlayer.volume(30);

  // Loop Song no 1 continuously
  myDFPlayer.play(1);
}

void loop()
{
  if (myDFPlayer.available()) {

    uint8_t type = myDFPlayer.readType();
    int value = myDFPlayer.read();

    if (type == DFPlayerPlayFinished) {

      Serial.print("Lagu selesai: ");
      Serial.println(value);

      delay(200);

      // Repeat Song no 1
      myDFPlayer.play(1);
    }
  }
}

void printDetail(uint8_t type, int value)
{
  switch (type) {

    case TimeOut:
      Serial.println("Time Out!");
      break;

    case WrongStack:
      Serial.println("Stack Wrong!");
      break;

    case DFPlayerCardInserted:
      Serial.println("Card Inserted!");
      break;

    case DFPlayerCardRemoved:
      Serial.println("Card Removed!");
      break;

    case DFPlayerPlayFinished:
      Serial.print("Number:");
      Serial.print(value);
      Serial.println(" Play Finished!");
      break;

    case DFPlayerError:

      Serial.print("DFPlayerError:");

      switch (value) {

        case Busy:
          Serial.println("Card not found");
          break;

        case Sleeping:
          Serial.println("Sleeping");
          break;

        case SerialWrongStack:
          Serial.println("Get Wrong Stack");
          break;

        case CheckSumNotMatch:
          Serial.println("Check Sum Not Match");
          break;

        case FileIndexOut:
          Serial.println("File Index Out of Bound");
          break;

        case FileMismatch:
          Serial.println("Cannot Find File");
          break;

        case Advertise:
          Serial.println("In Advertise");
          break;

        default:
          break;
      }
      break;

    default:
      break;
  }
}