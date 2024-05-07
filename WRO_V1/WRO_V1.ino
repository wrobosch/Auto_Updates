#include <RPLidar.h>
#include <Servo.h>

#define RPLIDAR_MOTOR 3 // The PWM pin for control the speed of RPLIDAR's motor (MOTOCTRL).
#define motor 5         // Motor Pin
#define l_motor 12      // Laufrichtung Pin linker Motor
#define r_motor 13      // Laufrichtung Pin rechter Motor
#define lenk_servo 10   // Pin des Servos für die Lenkung

Servo lenkung;
RPLidar lidar;

int winkel(int servo=0);  // ini der Funktion
int speed(int speed=0);
                      
void setup() 
{
  Serial.begin(115200);   // zum Debuggen
  Serial1.begin(115200);  // Für den RPLidar
  lidar.begin(Serial1);   // Start der Seriellen Übertragung
  pinMode(RPLIDAR_MOTOR, OUTPUT);  // set pin modes
  pinMode(motor, OUTPUT); // Geschwindigkeit beider Motoren
  pinMode(l_motor, OUTPUT);    // linker Motor Laufrichtung
  pinMode(r_motor, OUTPUT);    // Laufrichtung rechter Motor
  digitalWrite(l_motor, HIGH); // Laurichtung vorwärts
  digitalWrite(r_motor, HIGH); // Laurichtung vorwärts
  lenkung.attach(lenk_servo);     // PWM Pin vom Lenkungsservo
  analogWrite(motor, 0);  // Motorgeschwindigkeit = 0
}

float maxDistance = 0;
float angleAtMaxDist = 0;
float minDistance = 10000;
float angleAtMinDist = 0;
float backDistance = 10000;
float angleAtBackDist = 0;
int mSpeed = 0;             // Geschwindigkeit des Motors
                            // in 1s -> 50=10,2cm; 100=23,7cm; 150=37cm; 200=49,5cm; 250=60cm

void loop() {
  if (IS_OK(lidar.waitPoint())) {
    //perform data processing here... 
    float distance = lidar.getCurrentPoint().distance;
    float angle = lidar.getCurrentPoint().angle - 180;  // -180 bis +180 deg für besseren 0 Punkt
    
    if (lidar.getCurrentPoint().startBit) 
    {
       // a new scan, display the previous data...
       printData(angleAtMaxDist, maxDistance);

      if(angleAtBackDist < 0 && angleAtMaxDist < 0)
        lenkung.write(winkel(map(backDistance, 50, 400, 40, 0)));
      else if(angleAtBackDist > 0 && angleAtMaxDist > 0)
        lenkung.write(winkel(-map(backDistance, 50, 400, 40, 0)));
      else
        lenkung.write(winkel(map(angleAtMaxDist, -60, 60, -25, 25)));

      // falls der minimale oder maximaler Abstand unterschreitet wird folgt daraus,
      // dass das Auto direkt vor einer Wand steht und nur ein Rückwärtsfahren Sinn macht.
      if(maxDistance < 250 || minDistance < 120)
      {
        // beide Motoren werden ausgeschalten, umgepolt und fahren 1,5s rückwärts gefahren.
        mSpeed = 0;
        delay(100);
        digitalWrite(l_motor, LOW);
        digitalWrite(r_motor, LOW);
        delay(100);
        lenkung.write(winkel(angleAtMinDist)); //Lenkung zum Minimalen Abstand
        mSpeed = 80;
        delay(1500);
        mSpeed = 0;
        delay(100);
        digitalWrite(l_motor, HIGH);
        digitalWrite(r_motor, HIGH);
        delay(100);
      }
      //sonst 
      else if(maxDistance < 600 || minDistance < 300)
      {
        mSpeed = 80;
      }
      else if(maxDistance < 1000)
      {
        mSpeed = 115;
      }
      else if(maxDistance < 1400)
      {
        mSpeed = 150;
      }
      else if(maxDistance < 2000)
      {
        mSpeed = 200;
      }
      else
      {
        mSpeed = 250;
      }

      analogWrite(motor, mSpeed);

      maxDistance = 0;
      angleAtMaxDist = 0;
      minDistance = 10000;
      angleAtMinDist = 0;
      backDistance = 10000;
      angleAtBackDist = 0;
    } 
    else {
       if (-60 < angle && angle < 60 &&  distance > maxDistance) 
       {
          maxDistance = distance;
          angleAtMaxDist = angle;
       }
       if (-50 < angle && angle < 50 && distance < minDistance && distance > 50)
       {
          minDistance = distance;
          angleAtMinDist = angle;
       }
       if ((-135 < angle && angle < 0 || 0 < angle && angle < 135) && distance < 400 && distance > 50)
       {
          backDistance = distance;
          angleAtBackDist = angle;
       }
    }
  } 
  else {
    analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor
    // Try to detect RPLIDAR
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
       // Detected
       lidar.startScan();
       analogWrite(RPLIDAR_MOTOR, 255);
       delay(1000);
    }
  }
}

void printData(float angle, float distance)
{
  Serial.print("dist: ");
  Serial.print(distance);
  Serial.print("    angle: ");
  Serial.println(angle);
}

int winkel(int servo=0)
{
  if(servo >= 45)
    servo = 45;
  else if(servo <= -45)
    servo = -45;
  return servo+97;
}

int speed(int speed=0)
{
  return map(speed, 0, 60, 0, 250);
}