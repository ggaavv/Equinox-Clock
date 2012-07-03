#include <WiServer.h>
#include <LCD4Bit.h>


LCD4Bit lcd = LCD4Bit(4);

// Wireless configuration parameters ----------------------------------------
unsigned char local_ip[] = {
  192,168,2,111};	// IP address of WiShield
unsigned char gateway_ip[] = {
  192,168,2,1};	// router or gateway IP address
unsigned char subnet_mask[] = {
  255,255,255,0};	// subnet mask for the local network
const prog_char ssid[] PROGMEM = {
  "xxxxxxxxx"};		// max 32 bytes

unsigned char security_type = 3;	// 0 - open; 1 - WEP; 2 - WPA; 3 - WPA2

// WPA/WPA2 passphrase
const prog_char security_passphrase[] PROGMEM = {
  "xxxxxxxxx"};	// max 64 characters

// WEP 128-bit keys
// sample HEX keys
prog_uchar wep_keys[] PROGMEM = {
};

// setup the wireless mode
// infrastructure - connect to AP
// adhoc - connect to another WiFi device
unsigned char wireless_mode = 1;

unsigned char ssid_len;
unsigned char security_passphrase_len;
// End of wireless configuration parameters ----------------------------------------


/*
 * We'll have the WiServer call this function whenever a resource needs to be sent
 * @param URL null-terminated URL of the resource to send
 */
boolean sendPage(char* URL) {
  Serial.println(URL);
  // Request for the main page (checked last because other URL's also start with '/')
  if (strcmp(URL, "/") == 0) {
    WiServer.print("<!DOCTYPE html PUBLIC \"-//W3C//DTD XHTML 1.0 Strict//EN\" \"http://www.w3.org/TR/xhtml1/DTD/xhtml1-strict.dtd\">");
    WiServer.print("<html xmlns=\"http://www.w3.org/1999/xhtml\" dir=\"ltr\" lang=\"en-gb\" xml:lang=\"en-gb\">");
    WiServer.print("<head>");
    WiServer.print("</head><body>"); 
    WiServer.print("<script src=\"http://download.milesburton.com/Arduino/WiServer/htmlScript.js\" type=\"text/javascript\"></script>");
    WiServer.print("</body></html>");
    return true;   
  }
  else{
    // Consider a URL to be a request to output to a URL
    // Each forward slash represents a new line (4 lines max)
    
    int j = 1; // Start at char 1 of the URL (ignore the first /)
    lcd.clear(); // Clear LCD
    Serial.println("Writing to LCD");
    
    int len = strlen(URL);
    // Loop through each line
    for(int i=1;i<5;i++)
    {
      Serial.print("Writing to line");
      Serial.print(i);
      Serial.print('\n');


      lcd.cursorTo(i,0); // Move to line
      
      // Read next page of URL
      
      for(j=j;j<len;j++)
      {
        if(((j+2)<=len) && URL[j]=='%' && URL[j+1]=='2' && URL[j+2]=='0')
        {
          lcd.print(' ');
          j+=2;
          continue;
        }  
        
         
        if(URL[j]=='/')
        {
          j++;
          break; // If the current char is a /, break out read for the next line
        }
        lcd.print(URL[j]); // Write out char in URL to LCD
        Serial.print(URL[j]); // Send to Serial also
      }
       Serial.print('\n'); // Write a new line for easy reading
    }
    
    Serial.println("...Waiting for next command...");
    return true; // Return 200 to page
  }
}

void setup() {
  // Init the WiServer with our sendPage function that we defined above
  Serial.begin(9200);
  Serial.println("Loading..");
  
  // Output some bits and peices to screen
  lcd.init();
  lcd.clear();
  lcd.cursorTo(1,0);
  lcd.printIn("Wireless LCD Shield");
  lcd.cursorTo(2,0);
  lcd.printIn("MilesBurton.com 2009");
  lcd.cursorTo(3,0);
  lcd.printIn("Powered by WiShield");
  lcd.cursorTo(4,0);
  lcd.printIn("Loading...");
  
  // Initalise and connect Wifi
  WiServer.init(sendPage);  

  // Confirm we're finished loading
  Serial.println("Finished loading...");
  // Update the LCD
  lcd.cursorTo(4,0); 
  lcd.printIn("Ready...");
}


// Main loop
void loop(){ 
  // Handle web page requests
  WiServer.server_task();

  delay(10);
}


