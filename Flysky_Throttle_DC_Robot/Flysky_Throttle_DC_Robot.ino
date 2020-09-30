#define SRC_NEUTRAL 1500
#define SRC_MAX 2000
#define SRC_MIN 1000
#define TRC_NEUTRAL 1500
#define TRC_MAX 2000

#define TRC_MIN 1000
#define RC_DEADBAND 50
#define ERROR_center 50
#define pERROR 100  

uint16_t unSteeringMin = SRC_MIN + pERROR;
uint16_t unSteeringMax = SRC_MAX - pERROR;
uint16_t unSteeringCenter = SRC_NEUTRAL;

uint16_t unThrottleMin = TRC_MIN + pERROR;
uint16_t unThrottleMax = TRC_MAX - pERROR;
uint16_t unThrottleCenter = TRC_NEUTRAL;

#define PWM_MIN 0
#define PWM_MAX 255

#define GEAR_NONE 1
#define GEAR_IDLE 1
#define GEAR_FULL 2

#define PWM_SPEED_LEFT 10
#define PWM_SPEED_RIGHT 11
#define LEFT1 4
#define LEFT2 5
#define RIGHT1 6
#define RIGHT2 7

#define PROGRAM_PIN 9

#define THROTTLE_IN_PIN 2
#define STEERING_IN_PIN 3

#define THROTTLE_FLAG 1
#define STEERING_FLAG 2

// holds the update flags defined above
volatile uint8_t bUpdateFlagsShared;

volatile uint16_t unThrottleInShared;
volatile uint16_t unSteeringInShared;

// to refer to these in loop and the ISR then they would need to be declared volatile
uint32_t ulThrottleStart;
uint32_t ulSteeringStart;

uint8_t gThrottle = 0;
uint8_t gGear = GEAR_NONE;
uint8_t gOldGear = GEAR_NONE;

#define DIRECTION_STOP 0
#define DIRECTION_FORWARD 1
#define DIRECTION_REVERSE 2
#define DIRECTION_ROTATE_RIGHT 3
#define DIRECTION_ROTATE_LEFT 4

uint8_t gThrottleDirection = DIRECTION_STOP;
uint8_t gDirection = DIRECTION_STOP;
uint8_t gOldDirection = DIRECTION_STOP;

#define IDLE_MAX 50

#define MODE_RUN 0

uint8_t gMode = MODE_RUN;

unsigned long pulse_time  ;

void setup()
{
Serial.begin(9600);
Serial.println("hello");

attachInterrupt(0 /* INT0 = THROTTLE_IN_PIN */,calcThrottle,CHANGE);
attachInterrupt(1 /* INT1 = STEERING_IN_PIN */,calcSteering,CHANGE);

pinMode(PWM_SPEED_LEFT,OUTPUT);
pinMode(PWM_SPEED_RIGHT,OUTPUT);
pinMode(LEFT1,OUTPUT);
pinMode(LEFT2,OUTPUT);
pinMode(RIGHT1,OUTPUT);
pinMode(RIGHT2,OUTPUT);
pinMode(12,OUTPUT);
pulse_time =millis() ;
pinMode(PROGRAM_PIN,INPUT);
}

void loop()
{

 static uint16_t unThrottleIn;
 static uint16_t unSteeringIn;
 // local copy of update flags
 static uint8_t bUpdateFlags;
// fail_safe();

 if(bUpdateFlagsShared)
 {
  noInterrupts();
   pulse_time =millis() ;

  bUpdateFlags = bUpdateFlagsShared;

  if(bUpdateFlags & THROTTLE_FLAG)
  {
   unThrottleIn = unThrottleInShared;
  }

  if(bUpdateFlags & STEERING_FLAG)
  {
   unSteeringIn = unSteeringInShared;
  }

  bUpdateFlagsShared = 0;

  interrupts();
 }

  
  if(gMode == MODE_RUN)
  {
    // we are checking to see if the channel value has changed, this is indicated 
    // by the flags.
    if(bUpdateFlags & THROTTLE_FLAG)
    {
      unThrottleIn = constrain(unThrottleIn,unThrottleMin,unThrottleMax);
      
      if(unThrottleIn > (unThrottleCenter + ERROR_center))
      {
        gThrottle = map(unThrottleIn,(unThrottleCenter + ERROR_center),unThrottleMax,PWM_MIN,PWM_MAX);
        gThrottleDirection = DIRECTION_FORWARD;
        Serial.println(unThrottleIn);
      }
      else if (unThrottleIn < (unThrottleCenter - ERROR_center))
      {
        gThrottle = map(unThrottleIn,unThrottleMin,(unThrottleCenter- ERROR_center),PWM_MAX,PWM_MIN);
        gThrottleDirection = DIRECTION_REVERSE;
      }
      else
      {
      gThrottleDirection =DIRECTION_STOP;
      gThrottle=0;
      }
  
      if(gThrottle < IDLE_MAX)
      {
        gGear = GEAR_IDLE;
      }
      else
      {
        gGear = GEAR_FULL;
      }
    }
    if(bUpdateFlags & STEERING_FLAG)
    {
      uint8_t throttleLeft = gThrottle;
      uint8_t throttleRight = gThrottle;
  
      gDirection = gThrottleDirection;
      
      // see previous comments regarding trapping out of range errors
      // this is left for the user to decide how to handle and flag
      unSteeringIn = constrain(unSteeringIn,unSteeringMin,unSteeringMax);
  
      // if idle spin on spot
      switch(gGear)
      {
      case GEAR_IDLE:
        if(unSteeringIn > (unSteeringCenter + RC_DEADBAND))
        {
          gDirection = DIRECTION_ROTATE_RIGHT;
          // use steering to set throttle
          throttleRight = throttleLeft = map(unSteeringIn,unSteeringCenter,unSteeringMax,PWM_MIN,PWM_MAX);
        }
        else if(unSteeringIn < (unSteeringCenter - RC_DEADBAND))
        {
          gDirection = DIRECTION_ROTATE_LEFT;
          // use steering to set throttle
          throttleRight = throttleLeft = map(unSteeringIn,unSteeringMin,unSteeringCenter,PWM_MAX,PWM_MIN);
        }
        break;
      // if not idle proportionally restrain inside track to turn vehicle around it
      case GEAR_FULL:
        if(unSteeringIn > (unSteeringCenter + RC_DEADBAND))
        {
          throttleLeft = map(unSteeringIn,unSteeringCenter,unSteeringMax,gThrottle,PWM_MIN);
        }
        else if(unSteeringIn < (unSteeringCenter - RC_DEADBAND))
        {
          throttleRight = map(unSteeringIn,unSteeringMin,unSteeringCenter,PWM_MIN,gThrottle);
        }
        break;
      }
      analogWrite(PWM_SPEED_LEFT,throttleLeft);
      analogWrite(PWM_SPEED_RIGHT,throttleRight);
    }
  }
  if((gDirection != gOldDirection) || (gGear != gOldGear))
  {
    gOldDirection = gDirection;
    gOldGear = gGear;

 digitalWrite(LEFT1,LOW);
 digitalWrite(LEFT2,LOW);
 digitalWrite(RIGHT1,LOW);
 digitalWrite(RIGHT2,LOW);

switch(gDirection)
{
case DIRECTION_FORWARD:
 digitalWrite(LEFT1,LOW);
 digitalWrite(LEFT2,HIGH);
 digitalWrite(RIGHT1,LOW);
 digitalWrite(RIGHT2,HIGH);
 Serial.write("for");
 break;
case DIRECTION_REVERSE:
 digitalWrite(LEFT1,HIGH);
 digitalWrite(LEFT2,LOW);
 digitalWrite(RIGHT1,HIGH);
 digitalWrite(RIGHT2,LOW);
 Serial.write("rev");
 break;
case DIRECTION_ROTATE_RIGHT:
 digitalWrite(LEFT1,HIGH);
 digitalWrite(LEFT2,LOW);
 digitalWrite(RIGHT1,LOW);
 digitalWrite(RIGHT2,HIGH);
 Serial.write("right");
 break;
 
case DIRECTION_ROTATE_LEFT:
 digitalWrite(LEFT1,LOW);
 digitalWrite(LEFT2,HIGH);
 digitalWrite(RIGHT1,HIGH);
 digitalWrite(RIGHT2,LOW);
 Serial.write("left");
 break;
case DIRECTION_STOP:
 digitalWrite(LEFT1,LOW);
 digitalWrite(LEFT2,LOW);
 digitalWrite(RIGHT1,LOW);
 digitalWrite(RIGHT2,LOW);
 break;
}
  }
  bUpdateFlags = 0;
}

void calcThrottle()
{
  if(digitalRead(THROTTLE_IN_PIN) == HIGH)
  {
    ulThrottleStart = micros();
  }
  else
  {
    unThrottleInShared = (uint16_t)(micros() - ulThrottleStart);
    bUpdateFlagsShared |= THROTTLE_FLAG;
  }
}

void calcSteering()
{
  if(digitalRead(STEERING_IN_PIN) == HIGH)
  {
    ulSteeringStart = micros();
  }
  else
  {
    unSteeringInShared = (uint16_t)(micros() - ulSteeringStart);
    bUpdateFlagsShared |= STEERING_FLAG;
  }
}
