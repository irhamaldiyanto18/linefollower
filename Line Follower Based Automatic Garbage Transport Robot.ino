// --------------------------------------------------- DECLARATION RELAY WITH SENSOR IR ----------------------------------
const int RELAY = 32;
const int trashsensor = 35;

// --------------------------------------------------- IF THERE IS AN OBSTACLE STOP USING THE ULTRASONIC SENSOR ----------
int TRIG = 26; // membuat varibel trig yang di set ke-pin D6
int ECHO = 25; // membuat variabel echo yang di set ke-pin D7
long duration, distance;     // membuat variabel durasi dan jarak

// --------------------------------------------------- DECLARATION FRONT REAR MOTOR DC LEFT ------------------------------------
int frontleftmotorPin1 = 21;
int rearleftmotorPin2 = 22;
int enablePinA = 23;

// --------------------------------------------------- DECLARATION FRONT REAR MOTOR DC RIGHT -----------------------------------
int frontrightmotorPin1 = 18;
int rearrightmotorPin2 = 19;
int enablePinB = 5;

// --------------------------------------------------- DECLARATION SENSOR IR ---------------------------------------------
int leftsensor1 = 27;
int leftsensor2 = 14;
int rightsensor1 = 13;
int rightsensor2 = 12;

// --------------------------------------------------- SETTING PWM FRONT REAR LEFT DC MOTOR ------------------------------
const int freqA = 30000;
const int pwmChannelA = 0;
const int resolutionA = 8;

// --------------------------------------------------- SETTING PWM FRONT REAR RIGHT DC MOTOR -----------------------------
const int freqB = 30000;
const int pwmChannelB = 4;
const int resolutionB = 8;

// --------------------------------------------------- VOID SETUP --------------------------------------------------------
void setup() {
    // setup pin relay with sensor IR
    pinMode(trashsensor, INPUT);
    pinMode(RELAY, OUTPUT);
    digitalWrite(RELAY, HIGH);

    // setup pin sensor ultrasonik
    pinMode(TRIG, OUTPUT);    // set pin trig menjadi OUTPUT
    pinMode(ECHO, INPUT);     // set pin echo menjadi 

    //set pin sebagai output
    pinMode(enablePinA, OUTPUT);
    pinMode(enablePinB, OUTPUT);
    
    pinMode(frontleftmotorPin1, OUTPUT);
    pinMode(rearleftmotorPin2, OUTPUT);
    
    pinMode(frontrightmotorPin1, OUTPUT);
    pinMode(rearrightmotorPin2, OUTPUT);

    pinMode(leftsensor1, INPUT);
    pinMode(leftsensor2, INPUT);
    pinMode(rightsensor1, INPUT);
    pinMode(rightsensor2, INPUT);

// --------------------------------------------------- CONFIGURE LED PWM FUNCTIONALITITES --------------------------------
ledcSetup(pwmChannelA, freqA, resolutionA);
ledcSetup(pwmChannelB, freqB, resolutionB);

// --------------------------------------------------- ATTACH THE CHANNEL TO THE GPIO TO BE CONTROLLED -------------------
ledcAttachPin(enablePinA, pwmChannelA);
ledcAttachPin(enablePinB, pwmChannelB);

    Serial.begin(115200);

  }

// --------------------------------------------------- VOID FUNCTION -----------------------------------------------------

// MOVE FORWARD //
void maju (void) {
    //run left motors clockwise
    digitalWrite (frontleftmotorPin1,HIGH);
    digitalWrite (rearleftmotorPin2,LOW);
    //run right motors clockwise
    digitalWrite (frontrightmotorPin1,LOW);
    digitalWrite (rearrightmotorPin2,HIGH);
    for(int dutyCycle = 0; dutyCycle <= 180; dutyCycle++){   
    // changing the LED brightness with PWM
    ledcWrite(pwmChannelA, dutyCycle);
    ledcWrite(pwmChannelB, dutyCycle);
  }
}

// MOVE RIGHT //
void belokkanan (void) {
    //run left motors clockwise
    digitalWrite (frontleftmotorPin1,LOW);
    digitalWrite (rearleftmotorPin2,HIGH);
    //run right motors clockwise
    digitalWrite (frontrightmotorPin1,LOW);
    digitalWrite (rearrightmotorPin2,HIGH);
    for(int dutyCycle = 0; dutyCycle <= 180; dutyCycle++){   
    // changing the LED brightness with PWM
    ledcWrite(pwmChannelA, dutyCycle);
    ledcWrite(pwmChannelB, dutyCycle);
  }
}

// MOVE LEFT //
void belokkiri (void) {
    //run left motors clockwise
    digitalWrite (frontleftmotorPin1,HIGH);
    digitalWrite (rearleftmotorPin2,LOW);
    //run right motors clockwise
    digitalWrite (frontrightmotorPin1,HIGH);
    digitalWrite (rearrightmotorPin2,LOW);
    for(int dutyCycle = 0; dutyCycle <= 180; dutyCycle++){   
    // changing the LED brightness with PWM
    ledcWrite(pwmChannelA, dutyCycle);
    ledcWrite(pwmChannelB, dutyCycle);
  }
}

// STOP //
void berhenti (void) {
    //run left motors clockwise
    digitalWrite (frontleftmotorPin1,LOW);
    digitalWrite (rearleftmotorPin2,LOW);
    //run right motors clockwise
    digitalWrite (frontrightmotorPin1,LOW);
    digitalWrite (rearrightmotorPin2,LOW);
    for(int dutyCycle = 0; dutyCycle <= 0; dutyCycle++){   
    // changing the LED brightness with PWM
    ledcWrite(pwmChannelA, dutyCycle);
    ledcWrite(pwmChannelB, dutyCycle);
  }
  
  }


void linefollow(void)
  {
    int leftsensor1 = digitalRead(27); 
    int leftsensor2 = digitalRead(14);
    int rightsensor1 = digitalRead(13);
    int rightsensor2 = digitalRead(12);

    if (leftsensor1 == LOW && leftsensor2 == LOW && rightsensor1 == LOW && rightsensor2 == LOW) {
        maju ();
    }
    else if (leftsensor1 == HIGH && leftsensor2 == LOW && rightsensor1 == LOW && rightsensor2 == LOW) {
        belokkiri ();
    }
    else if (leftsensor1 == LOW && leftsensor2 == LOW && rightsensor1 == LOW && rightsensor2 == HIGH) {
        belokkanan ();
    }
    else if (leftsensor1 == LOW && leftsensor2 == HIGH && rightsensor1 == LOW && rightsensor2 == LOW) {
        belokkiri ();
    }
    else if (leftsensor1 == LOW && leftsensor2 == LOW && rightsensor1 == HIGH && rightsensor2 == LOW) {
        belokkanan ();
    }
    else if (leftsensor1 == HIGH && leftsensor2 == HIGH && rightsensor1 == LOW && rightsensor2 == LOW) {
        belokkiri ();
    }
    else if (leftsensor1 == LOW && leftsensor2 == LOW && rightsensor1 == HIGH && rightsensor2 == HIGH) {
        belokkanan ();
    }
    else {
        berhenti ();

    }
  }

// --------------------------------------------------- VOID LOOP --------------------------------------------------------
void loop() {
  //line follow
  linefollow();

  //trashsensor
  int sensorState = digitalRead(trashsensor);
  if (sensorState == LOW) {
      digitalWrite(RELAY, LOW);
      /*Serial.println("Full Trash");
      delay(500);*/
    } else {
         digitalWrite(RELAY, HIGH);
         /*Serial.println("Trash Not Full");
         delay(500);*/
      }

  //obstacle ultrasonik
  digitalWrite(TRIG, LOW);
  delayMicroseconds(10);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  delayMicroseconds(10);
  duration = pulseIn(ECHO, HIGH); // menerima suara ultrasonic
  distance = (duration / 2) / 29.1;  // mengubah durasi menjadi jarak (cm)
  if (distance < 8){
      berhenti ();
      /*Serial.println("There Are Obstacle Ahead");*/
      delay(60);
    }
    /*else if (distance > 8) {
      Serial.println("No Obstacle Ahead");
      delay(500);
    }*/

   /*Serial.print("Jarak Benda: ");
   Serial.print(distance);          // menampilkan jarak pada serial monitor
   Serial.println(" cm");
   delay(500);*/
}
