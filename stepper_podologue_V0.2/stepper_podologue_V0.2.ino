// control moteur pas à pas pour projet podologue

#include <AccelStepper.h>
#include <PID_v1.h>

#define dirPin 4
#define stepPin 2
#define motorInterfaceType 1

float vitesse_memo = 0;
int vitesse_max = 2000;
int period = 100;
unsigned long time_memo = 0;

// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

//PID pour le controle du moteur pas à pas
double Kp = 0.35, Ki = 0.001, Kd = 0;
double Position_Setpoint, Stepper_Position, Commande_moteur;
//PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction)
PID myPID(&Stepper_Position, &Commande_moteur, &Position_Setpoint, Kp, Ki, Kd, DIRECT);

void setup()
{
  stepper.setMaxSpeed(vitesse_max); // 6000 step/sec = 15tr/s = 900 tr/min

  time_memo = millis(); //millis a un overflow après 50 jours
  //vitesse en step/s (1 tr = 400 step)

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-vitesse_max, vitesse_max); //vitesse max = 2000

  Serial.begin(115200);
  delay(3000);
}

void loop()
{
  //il faut essayer de ne pas utiliser la fct runToPosition(), c'est une fct bloquante
  // Set the current position to 0:
  float commande_vitesse;


  stepper.setCurrentPosition(0);
  while (stepper.currentPosition() < 800) //800 = 2 tours
  {
    stepper.setSpeed(400); // 400 = 1tr/s
    stepper.runSpeed();
  }
  delay(4000);

  while (stepper.currentPosition() < 100000)
  {
    Position_Setpoint = 30000;
    Stepper_Position = stepper.currentPosition();
    myPID.Compute();

    //Serial.println(Position_Setpoint - Stepper_Position);

    if (millis() > time_memo + period) { //tache cyclicle 100ms
      //wait approx. [period] ms
      commande_vitesse = Vitesse_rampee(0.1, Commande_moteur);
      time_memo = millis();

      Serial.println(commande_vitesse);
      //Serial.println(Stepper_Position);
    }

    //si l'erreur = 0 on force l'arrêt du moteur
    //on évite l'oscillation du moteur
    if  (Position_Setpoint - Stepper_Position == 0) 
    {

      commande_vitesse = 0;
    }
    else
    {

    }
    stepper.setSpeed(commande_vitesse);
    stepper.runSpeed();

  }

  //  while (stepper.currentPosition() < 1000000)
  //  {
  //    if (millis() > time_memo + period) {
  //      //wait approx. [period] ms
  //      commande_vitesse = Vitesse_rampee(0.01, 6000);
  //      time_memo = millis();
  //
  //      Serial.println(commande_vitesse);
  //    }
  //    stepper.setSpeed(commande_vitesse);
  //    stepper.runSpeed();
  //  }

  //delay(1);

}
float Vitesse_rampee (float cst_tps, float consigne_vitesse)
{
  float vitesse;
  vitesse = cst_tps * consigne_vitesse + (1 - cst_tps) * vitesse_memo;
  vitesse_memo = vitesse; // mémorise la vitesse précédente
  return vitesse;
}
