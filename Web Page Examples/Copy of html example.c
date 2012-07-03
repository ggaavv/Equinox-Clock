// Servo Control Parameters ------------------------------------------------

Servo servoX;   // create servo object to control a servo
Servo servoY;   // a maximum of eight servo objects can be created

// Limits for servo positions
int minRange = 20;
int maxRange = 160;
int median = 90;
int posX = 90;         // Start tail in center position

int incomingByte = 90;   // value inbetween wags and curls
int wagStep = 1;      // Increment stepper value
boolean positive = true;// Flag for managing wag direction

int wagPattern = 90;   // Stores byte value for wagging tail
int curlPattern = 90;   // Stores byte value for curling tail

// Methods ------------------------------------------------------------------

void setup() {
   // Initialize WiServer and have it use the sendPage function to serve pages
   Serial.begin(115200);
   Serial.println("setup() ... ");
   servoX.attach(2);  // attaches the servo on pin 2 to the servo object
   servoY.attach(4);  // attaches the servo on pin 4 to the servo object
   servoX.write(median);
   servoY.write(median);
   WiServer.init(sendPage);
}

boolean sendPage(char* URL) {
   if (URL[1] == '?') {
      // Set wagPattern or curlPattern based on URL
      incomingByte = URL[2];
      if(incomingByte < 91) {
         wagPattern = incomingByte;
      }else if(incomingByte > 96) {
         curlPattern = incomingByte;
      }
   }

   Serial.print("serving ... ");
   // Serial.println(counter, DEC);
   WiServer.print("<html>"
	"<head>"
     "<meta name='viewport' content='width=300'>"
	  "<style type='text/css'>html{width:300px;font-family:sans-serif;text-align:center}body{margin:0px}a{display:block;margin:3px;padding:3px;text-decoration:none;border:solid 1px black;}</style>"
	"</head>"
	"<body>Wags<a href='?A'>no swing</a>"
		   "<a href='?B'>light swing</a>"
		   "<a href='?C'>medium swing</a>"
		   "<a href='?D'>heavy swing</a>"
		   "<a href='?E'>jittery swing</a>Curls<a href='?a'>straight down</a>"
		   "<a href='?b'>slight curl back</a><a href='?c'>full curl back</a>"
		   "<a href='?d'>slight curl front</a><a href='?e'>full curl front</a>"
	 "</body>"
	"</html>");
   Serial.print("served ... ");
   Serial.println(incomingByte, DEC);

   return true;
}

void stepServoX(int delayVal,int max,int min) {
   if(posX >= max){ positive = false; }
   if(posX <= min){ positive = true; }
   if(positive){
      posX += wagStep;
   }else{
      posX -= wagStep;
   }
   servoX.write(posX);
   delay(delayVal);
}

void wag() {
   switch (wagPattern) {
      case 66:
         // B. light wag
         stepServoX(50,maxRange,minRange);
         break;

      case 67:
         // C. medium wag
         stepServoX(25,maxRange,minRange);
         break;

      case 68:
         // D. heavy wag
         stepServoX(5,maxRange,minRange);
         break;

      case 69:
         // E. shiver
         stepServoX(5,median+10,median-10);
         break;

      default:
         // A. no wag
         servoX.write(median);
         positive = true;
   }
}

void curl(){
   switch (curlPattern) {
      case 98:
         // b. light curl back
         servoY.write(median+30);
         break;

      case 99:
         // c. heavy curl back
         servoY.write(maxRange);
         break;

      case 100:
         // d. light curl front
         servoY.write(median-30);
         break;

      case 101:
         // e. heavy curl front
         servoY.write(minRange);
         break;

      default:
         // a. straight down
         servoY.write(median);
   }
}

void loop(){
   WiServer.server_task();
   wag();
   curl();
}
