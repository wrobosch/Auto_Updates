#include <RPLidar.h>
#include <Servo.h>
#include <Pixy2.h>

#define RPLIDAR_MOTOR 3 // The PWM pin for control the speed of RPLIDAR's motor (MOTOCTRL).
#define motor 5         // Motor Pin
#define l_motor 8      // Laufrichtung Pin linker Motor
#define r_motor 9      // Laufrichtung Pin rechter Motor
#define lenk_servo 10   // Pin des Servos für die Lenkung

Pixy2 pixy;
Servo lenkung;
RPLidar lidar;

int winkel(int servo=0);  // ini der Funktion
int speed(int speed=0);
int taster = 4;
                      
void setup() 
{
  Serial.begin(115200);   // zum Debuggen
  Serial1.begin(115200);  // Für den RPLidar
  lidar.begin(Serial1);   // Start der Seriellen Übertragung
  pinMode(4, INPUT_PULLUP);
  pinMode(RPLIDAR_MOTOR, OUTPUT);  // set pin modes
  pinMode(motor, OUTPUT); // Geschwindigkeit beider Motoren
  pinMode(l_motor, OUTPUT);    // linker Motor Laufrichtung
  pinMode(r_motor, OUTPUT);    // Laufrichtung rechter Motor
  digitalWrite(l_motor, 0); // Laufrichtung vorwärts
  digitalWrite(r_motor, 0); // Laufrichtung vorwärts
  lenkung.attach(lenk_servo);     // PWM Pin vom Lenkungsservo
  analogWrite(motor, 0);  // Motorgeschwindigkeit = 0
  pixy.init();
  pixy.setLamp(0, 0);
}           

float maxDistance = 0;
float angleAtMaxDist = 0;
float minDistance = 10000;
float angleAtMinDist = 0;
float safeDistance = 10000;
float angleAtSafeDist = 0;
float speedDistance = 10000;
float angleAtSpeedDist = 0;
float angleAtCamDist = 0;
float camDistance = 10000;
int zaehler = 0;
int ecke = 0; // 1/4 Einer Runde
//bool vor = true;
int block_color = 0;
int block_location = 0;
bool orange_line = 0;
bool voillet_line = 0;
int mSpeed = 0;             // Geschwindigkeit des Motors
                            // in 1s -> 50=10,2cm; 100=23,7cm; 150=37cm; 200=49,5cm; 250=60cm
bool status = true;

void loop() {
  if(!digitalRead(taster))
  {
    while(!digitalRead(taster));
    status = !status;
  }
  if(!status)
  {
    if(ecke == 12){            //Wenn er 12-mal bei den Linien Orange und Voillet 
      analogWrite(motor, 0);   //vorbei färht hält der Motor an
      lidar.end();
      while(true){}       
    }


    boden();

    if (IS_OK(lidar.waitPoint()))  //
    {
            //perform data processing here... 
      float distance = lidar.getCurrentPoint().distance;
      float angle = lidar.getCurrentPoint().angle - 180;  // -180 bis +180 deg für besseren 0 Punkt
      
      if (lidar.getCurrentPoint().startBit) 
      {
        // a new scan, display the previous data...
        //printData(angleAtSpeedDist, speedDistance, mSpeed);
        //Serial.println(block_color);
        //Serial.println(block_location);
        // ausweichen wenn Hindernis und Zielrichtung auf der gleichen seite sind
        if(angleAtSafeDist < 0 && angleAtMaxDist < 0)
          lenkung.write(winkel(map(safeDistance, 50, 400, 40, 0)));
        else if(angleAtSafeDist > 0 && angleAtMaxDist > 0)
          lenkung.write(winkel(-map(safeDistance, 50, 400, 40, 0)));
        else
        {
          boden();
          if(block_color == 4) //4=grün
          {
            if(block_location<0)
            {
              lenkung.write(winkel(-45));
            }
            else
            {
              lenkung.write(winkel(-45+block_location));
            }
          }
          else if(block_color == 1) //1=rot
          {
            if(block_location>0)
            {
              lenkung.write(winkel(45));
            }
            else
            {
              lenkung.write(winkel(45+block_location));
            }
          }
          else
          {
            lenkung.write(winkel(map(angleAtMaxDist, -90, 90, -45, 45)));
          }
          //block_color = 0;
          
        }

        

        // falls der minimale oder maximaler Abstand unterschreitet wird folgt daraus,
        // dass das Auto direkt vor einer Wand steht und nur ein Rückwärtsfahren Sinn macht.
        if(speedDistance < 170 )
        {
          // beide Motoren werden ausgeschalten, umgepolt und fahren 1,5s rückwärts gefahren.
          analogWrite(motor, 0);
          /*if(block_color == 4) //4=grün
          {
            lenkung.write(winkel(45));
          }
          else if(block_color == 1) //1=rot
          {
            lenkung.write(winkel(-45));
          }
          else
          {
            lenkung.write(winkel(angleAtMinDist)); //Lenkung zum Minimalen Abstand
          }
          //block_color = 0;
        */

          
          back(80);
          delay(1500);
          //lenkung.write(winkel(angleAtMaxDist));
          analogWrite(motor, 0);
          mSpeed = 80;
        }
        else if(speedDistance < 280 || minDistance < 50 )
        {
          mSpeed = 60;
        }
        else if(speedDistance < 320 || minDistance < 60 )
        {
          mSpeed = 80;
        }
        else if(speedDistance < 380)
        {
          mSpeed = 120;
        }
        else if(speedDistance < 450)
        {
          mSpeed = 150;
        }
        else
        {
          //mSpeed = 200;
        }

        //block_color = 0;

        if(camDistance < 20000)
        {
          pixy.ccc.getBlocks();
          // If there are detect blocks, print them!
          if (pixy.ccc.numBlocks)
          {
            //Serial.print("Detected ");
            //Serial.println(pixy.ccc.numBlocks);
            int old_age = 0;
            block_color = 0;
            for (int i=0; i<pixy.ccc.numBlocks; i++)
            {
              //Serial.print("  block ");
              //Serial.print(i);
              //Serial.print(": ");
              //pixy.ccc.blocks[i].print();
              //Serial.println(pixy.ccc.blocks[i].m_signature);
              if(pixy.ccc.blocks[i].m_age > old_age )// && 130 > (pixy.ccc.blocks[i].m_height + pixy.ccc.blocks[i].m_y))
              {
                old_age = pixy.ccc.blocks[i].m_age;
                block_color = pixy.ccc.blocks[i].m_signature;
                //Serial.print("Color ");
                //Serial.print(i);
                //Serial.print(" : ");
                //Serial.println(block_color);
                //x=0 ist links und x=315 ist rechts --> ab 158 ist rechts --> x + width/2 > 158 dann rechts
                block_location = map(( pixy.ccc.blocks[i].m_x + pixy.ccc.blocks[i].m_width/2 ), 0, 315, -27, 27); //somit ist 0 = -27° und 315 = +27°
                //block_hight = pixy.ccc.blocks[i].m_height + pixi.ccc.blocks[i].m_y;
                //Serial.println(block_location);
                
              }
            }
          } 
          else
            {
              
            }
          //zaehler = 0;
        }

        vor(mSpeed);

        // Variablen Reset
        maxDistance = 0;
        angleAtMaxDist = 0;
        speedDistance = 10000;
        angleAtSpeedDist = 0;
        minDistance = 10000;
        angleAtMinDist = 0;
        safeDistance = 10000;
        angleAtSafeDist = 0;
        angleAtCamDist = 0;
        camDistance = 10000;

      } 
      else {
        boden();
        if (-90 < angle && angle < 90 &&  distance > maxDistance) 
        {
            maxDistance = distance;
            angleAtMaxDist = angle;
        }
        if (-28 < angle && angle < 28 &&  distance < camDistance && distance > 50) 
        {
            camDistance = distance;
            angleAtCamDist = angle;
        }
        int currentLenkung = (lenkung.read()-97)/2;
        //Serial.println(currentLenkung);
        if (-45+currentLenkung < angle && angle < 45+currentLenkung &&  distance < speedDistance && distance > 50 && distance < 140 ||
            -28+currentLenkung < angle && angle < 28+currentLenkung &&  distance < speedDistance && distance > 50 && distance < 180 || 
            -22+currentLenkung < angle && angle < 22+currentLenkung &&  distance < speedDistance && distance > 50 && distance < 220 ||
            -18+currentLenkung < angle && angle < 18+currentLenkung &&  distance < speedDistance && distance > 50 && distance < 270 ||
            -16+currentLenkung < angle && angle < 16+currentLenkung &&  distance < speedDistance && distance > 50 && distance < 320 ||
            -14+currentLenkung < angle && angle < 14+currentLenkung &&  distance < speedDistance && distance > 50 && distance < 370 ||
            -12+currentLenkung < angle && angle < 12+currentLenkung &&  distance < speedDistance && distance > 50 && distance < 420 ||  
            -10+currentLenkung < angle && angle < 10+currentLenkung &&  distance < speedDistance && distance > 50 && distance < 470    
            )
        {
            speedDistance = distance;
            angleAtSpeedDist = angle;
        }
        if ((-90 < angle && angle < -35 || 35 < angle && angle < 90) && distance < minDistance && distance > 50)
        {
            minDistance = distance;
            angleAtMinDist = angle;
        }
        if ((-55 < angle && angle < -26 || 26 < angle && angle < 55) && distance < 700 && distance > 50)
        {
            safeDistance = distance;  //füherkennung
            angleAtSafeDist = angle;
        }
      }
    } 
    else 
      {
        analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor
        // Try to detect RPLIDAR
        rplidar_response_device_info_t info;
        if (IS_OK(lidar.getDeviceInfo(info, 100))) 
        {
          // Detected
          lidar.startScan();
          analogWrite(RPLIDAR_MOTOR, 255);
          delay(1000);
        }
      }
  }
}

void printData(float angle, float distance, int speed)
{
  Serial.print("angle: ");
  Serial.print(angle);
  Serial.print("    distance: ");
  Serial.print(distance);
  Serial.print("    speed: ");
  Serial.println(speed);
}

int winkel(int servo=0)
{
  if(servo >= 45)
    servo = 45;
  else if(servo <= -45)
    servo = -45;
  else if(servo < 0)
    servo = map(servo, -52, 0, -45, 0);
  return servo+97;
}

int speed(int speed=0)
{
  return map(speed, 0, 60, 0, 250);
}

void vor(int speed)
{
  if(digitalRead(l_motor))
  {
    analogWrite(motor, 0);
    delay(100);
    digitalWrite(l_motor, 0);
    digitalWrite(r_motor, 0);
  }
  analogWrite(motor, speed);
}

void back(int speed)
{
  if(!digitalRead(l_motor))
  {
    analogWrite(motor, 0);
    delay(100);
    digitalWrite(l_motor, 1);
    digitalWrite(r_motor, 1);
  }
  analogWrite(motor, speed);
}

void boden()
{
   if(block_color == 2 && pixy.ccc.blocks[7].m_age < pixy.ccc.blocks[2].m_age) //Orange = 2
   {
      //pixy.setLamp(1,0);
      //mSpeed = 150;
      orange_line=1;
      Serial.println(block_color);
      block_color=0;
   }
   else if(block_color == 7 && pixy.ccc.blocks[7].m_age > pixy.ccc.blocks[2].m_age) //Voillet = 7
   {
      //pixy.setLamp(1,0);
      //mSpeed = 150;
      voillet_line=1;
      Serial.println(block_color);
      block_color=0;
   }
   if(orange_line == 1)
   {
      if(block_color == 7) //Violett = 7
      {
        orange_line = 0;
        //mSpeed = 0;
        Serial.println(block_color);
        //pixy.setLamp(0,0);
        ecke++;
        Serial.print("Ecke: ");
        Serial.print(ecke);
      }
   }
   if(voillet_line == 1)
   {
      if(block_color == 2) //Orange = 2
      {
        voillet_line = 0;
        //mSpeed = 0;
        Serial.println(block_color);
        //pixy.setLamp(0,0);
        ecke++;
        Serial.print("Ecke: ");
        Serial.print(ecke);
      }
   }
   
}
