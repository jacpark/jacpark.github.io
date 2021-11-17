
/*
 Based on
 WiFi Web Server LED Blink
created for arduino 25 Nov 2012
 by Tom Igoe
ported for sparkfun esp32 
31.01.2017 by Jan Hendrik Berlin
*/

#include <WiFi.h>

const char* ssid     = "Scytale";
const char* password = "";
#define up 5
#define down 33
#define left 13
#define right 19
#define forward 16
#define backward 32
#define clockw 4
#define cclockw 27


WiFiServer server(80);

void setup()
{
    Serial.begin(115200);
    pinMode(up, OUTPUT);      // set the LED pin mode
    pinMode(down, OUTPUT);
    pinMode(left, OUTPUT); 
    pinMode(right, OUTPUT); 
    pinMode(forward, OUTPUT); 
    pinMode(backward, OUTPUT); 
    pinMode(clockw, OUTPUT); 
    pinMode(cclockw, OUTPUT);  
    delay(10);

    // We start by connecting to a WiFi network

    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    
    server.begin();

}

int value = 0;

void loop(){
  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("New Client.");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // the content of the HTTP response follows the header:
            // PUT YOUR NAME HERE
            client.print("Jacobs Drone Command<br>"); // PUT YOUR NAME HERE
            // PUT YOUR NAME HERE
            client.print("<button><a href=\"/U\">Up</a></button><br>");
            client.print("<button><a href=\"/D\">Down</a></button><br>");
            client.print("<button><a href=\"/R\">Right</a></button> <br>");
            client.print("<button><a href=\"/L\">Left</a></button> <br>");
            client.print("<button><a href=\"/F\">Forward</a></button> <br>");
            client.print("<button><a href=\"/B\">Backward</a></button> <br>");
            client.print("<button><a href=\"/CW\">Clockwise</a></button> <br>");
            client.print("<button><a href=\"/CCW\">Counter Clockwise</a></button><br>");

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          } else {    // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /H" or "GET /L":
        if (currentLine.endsWith("GET /U")) {
            digitalWrite(up,HIGH);
            digitalWrite(down, LOW);
            digitalWrite(left,LOW);
            digitalWrite(right,LOW);
            digitalWrite(forward,LOW);
            digitalWrite(backward,LOW);
            digitalWrite(clockw,LOW);
            digitalWrite(cclockw,LOW);
        }
        if (currentLine.endsWith("GET /D")) {
          digitalWrite(up,LOW);
          digitalWrite(down, HIGH);
          digitalWrite(left,LOW);
          digitalWrite(right,LOW);
          digitalWrite(forward,LOW);
          digitalWrite(backward,LOW);
          digitalWrite(clockw,LOW);
          digitalWrite(cclockw,LOW);          // GET /H turns the LED on
        }
        if (currentLine.endsWith("GET /L")) {
          digitalWrite(up,LOW);
          digitalWrite(down, LOW);
          digitalWrite(left,HIGH);
          digitalWrite(right,LOW);
          digitalWrite(forward,LOW);
          digitalWrite(backward,LOW);
          digitalWrite(clockw,LOW);
          digitalWrite(cclockw,LOW);
        }
        if (currentLine.endsWith("GET /R")) {
          digitalWrite(up,LOW);
          digitalWrite(down, LOW);
          digitalWrite(left,LOW);
          digitalWrite(right,HIGH);
          digitalWrite(forward,LOW);
          digitalWrite(backward,LOW);
          digitalWrite(clockw,LOW);
          digitalWrite(cclockw,LOW);
        }
        if (currentLine.endsWith("GET /F")) {
          digitalWrite(up,LOW);
          digitalWrite(down, LOW);
          digitalWrite(left,LOW);
          digitalWrite(right,LOW);
          digitalWrite(forward,HIGH);
          digitalWrite(backward,LOW);
          digitalWrite(clockw,LOW);
          digitalWrite(cclockw,LOW);
        }
        if (currentLine.endsWith("GET /B")) {
          digitalWrite(up,LOW);
          digitalWrite(down, LOW);
          digitalWrite(left,LOW);
          digitalWrite(right,LOW);
          digitalWrite(forward,LOW);
          digitalWrite(backward,HIGH);
          digitalWrite(clockw,LOW);
          digitalWrite(cclockw,LOW);
        }
        if (currentLine.endsWith("GET /CW")) {
          digitalWrite(up,LOW);
          digitalWrite(down, LOW);
          digitalWrite(left,LOW);
          digitalWrite(right,LOW);
          digitalWrite(forward,LOW);
          digitalWrite(backward,LOW);
          digitalWrite(clockw,HIGH);
          digitalWrite(cclockw,LOW);
        }
        if (currentLine.endsWith("GET /CCW")) {
          digitalWrite(up,LOW);
          digitalWrite(down, LOW);
          digitalWrite(left,LOW);
          digitalWrite(right,LOW);
          digitalWrite(forward,LOW);
          digitalWrite(backward,LOW);
          digitalWrite(clockw,LOW);
          digitalWrite(cclockw,HIGH);
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("Client Disconnected.");
  }
}
