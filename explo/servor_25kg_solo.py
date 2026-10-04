import RPi.GPIO as GPIO
import time
import math

### Pour conneter le tout a la raspberry pi et au external power, il faut que 
# le + du power supply aille au + du moteur, 
# le signal a la PIN du raspberry,
# le ground du raspberry pie vers le powersupply et du moteur au powersupply


#On peut envoyer des cycle de 2 a 12 dans ce cervo moteur, qui lui a un angle de 0 a 270 degre.
#Set function to calculate percent from angle
def angle_to_percent (angle) :
    # On restreint les mouvements a un range afin de ne pas abimer l'installation (fils trop court de la camera) 
    if angle > 270 or angle < 0 :
        return False

    start = 2
    end = 12
    ratio = (end - start)/270 #Calcul ratio from angle to percent

    angle_as_percent = angle * ratio

    return start + angle_as_percent

#Fonction pour plus orienter le mecanisme en x et y
def move_it(x):
    p1.ChangeDutyCycle(angle_to_percent(x))
    

#chiken way out mais enleve le probleme si end
#GPIO.setwarnings(False)

servoPIN1 = 2 # X

GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN1, GPIO.OUT)


p1 = GPIO.PWM(servoPIN1, 50) # GPIO 17 for PWM with 50Hz

p1.start(angle_to_percent(0)) # Initialization

#En X si x augmente, va vers la gauche
#En Y si y augmente, va vers le haut

f_x = 33

move_it(f_x)
time.sleep(2)

f_x = f_x + 72
move_it(f_x)
time.sleep(2)

p1.stop()
#p2.stop()

GPIO.cleanup()
