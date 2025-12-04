//--------------------------------------------Mockery
//All MOCK values, should be replaced by actual ones
//All delays in the code are solely for the purpose of seeing LEDs lit
//now do i somehow need to mock the camera and its outputs for python?

//inputs to be verified 
  const int portion_time = 5000;                  //how long it takes to deliver one portion of food in ms
  const int timeout = 13000;                      //timeout for idle state; currently 13 seconds
  unsigned long int begin_timeout = 0;            //controls when a timeout began
  unsigned long int begin_feeding = 0;            //controls when feeding began



  //state names enum-like
  const int IDLE = 2;
  const int DETECTING = 3;
  const int FEEDING = 4;
  const int EMPTY = 5;
  
  const int BIRD = 6; //extra and in parallel: show if a bird was spotted!

  int state = IDLE;                               //initializing the state of our machine

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(IDLE, OUTPUT);
  pinMode(DETECTING, OUTPUT);
  pinMode(FEEDING, OUTPUT);
  pinMode(EMPTY, OUTPUT);
  pinMode(BIRD, OUTPUT);
}

void loop() {
  
  bool bird_found = Serial.read();          //Python MUST say if a bird was detected or not
  unsigned long int food = Serial.read();   //Python MUST say how much food there is in the container


  //to show a bird was detectable at said point
  if(bird_found) digitalWrite(BIRD, HIGH);
  else digitalWrite(BIRD, LOW);

  
  switch(state){
    case IDLE:                                              //wait for timeout and switch to detection state
      digitalWrite(IDLE, HIGH);
      digitalWrite(DETECTING, LOW);
      digitalWrite(FEEDING, LOW);
      digitalWrite(EMPTY, LOW);
      if(millis() - begin_timeout >= timeout) {
        state = DETECTING;
      }
      break; 
      
    case DETECTING:                                         //if a bird is found, switches to feeding state;
      digitalWrite(IDLE, LOW);
      digitalWrite(DETECTING, HIGH);
      digitalWrite(FEEDING, LOW);
      digitalWrite(EMPTY, LOW);
      
      delay(1000);
      
      if(bird_found) {
        begin_feeding = millis();
        state = FEEDING; 
      }
      break;  
      
    case FEEDING:                                          //While food is being released 
      digitalWrite(IDLE, LOW);
      digitalWrite(DETECTING, LOW);
      digitalWrite(FEEDING, HIGH);
      digitalWrite(EMPTY, LOW);

      delay(1000);
      
      if(millis() - begin_feeding >= portion_time) {
        state = (food ? IDLE : EMPTY);
      }
      
      //do i need to mock feeding too?
      if(state == IDLE) begin_timeout = millis();
      else Serial.println("NO FOOD DUMBASS");
      
      break;
    case EMPTY:                                            //When it's empty
      digitalWrite(IDLE, LOW);
      digitalWrite(DETECTING, LOW);
      digitalWrite(FEEDING, LOW);
      digitalWrite(EMPTY, HIGH);

      delay(1000);
      
      if(food) {
        state = IDLE;
        begin_timeout = millis();
      }
      break;
      
  }


  

} 
