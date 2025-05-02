#pragma region definitions
#include <LCD_I2C.h>
#include <AccelStepper.h>
#include <HCSR04.h>
#include <U8g2lib.h>
#include "Alarm.h"

#define MOTOR_INTERFACE_TYPE 4

#define BUZZER_PIN 2

#define LED_PIN_RED 3
#define LED_PIN_GREEN 4
#define LED_PIN_BLUE 5

#define IN_1 8
#define IN_2 9
#define IN_3 10
#define IN_4 11

#define TRIGGER_PIN 6
#define ECHO_PIN 7

#define CLK_PIN 30
#define DIN_PIN 34
#define CS_PIN 32

float distance;
Alarm alarm(LED_PIN_RED, LED_PIN_GREEN, LED_PIN_BLUE, BUZZER_PIN, distance);

LCD_I2C lcd(0x27, 16, 2);
AccelStepper motor(MOTOR_INTERFACE_TYPE, IN_1, IN_3, IN_2, IN_4);
HCSR04 hc(TRIGGER_PIN, ECHO_PIN);
U8G2_MAX7219_8X8_F_4W_SW_SPI u8g2(
  U8G2_R0,             // rotation
  /* clock=*/CLK_PIN,  // pin Arduino reliée à CLK (horloge)
  /* data=*/DIN_PIN,   // pin Arduino reliée à DIN (données)
  /* cs=*/CS_PIN,      // pin Arduino reliée à CS (chip select)
  /* dc=*/U8X8_PIN_NONE,
  /* reset=*/U8X8_PIN_NONE);

enum StepperAppState { NORMAL,
                       TOO_CLOSE,
                       TOO_FAR };

StepperAppState stepperAppState = NORMAL;

// enum AlertAppState { ALERT_OFF,
//                      ALERT_ON };

// AlertAppState alertAppState = ALERT_OFF;

enum MatrixAppState {
  EMPTY,
  ERROR,
  BAD_COMMAND,
  CHECKMARK
};

MatrixAppState matrixAppState = EMPTY;

char lcdBuff[2][16] = {
  "                ",
  "                "
};

unsigned long current_time = 0;

int frequency = 1;

int current_color = 0;

// float distance;
int min_distance = 30;
int max_distance = 60;
int alert_distance = 15;

int angle;
int min_angle = 10;
int max_angle = 170;
float lap = 2038;

float max_step = (lap * max_angle) / 360;
float min_step = (lap * min_angle) / 360;

String tampon = "";

unsigned long start_timer_matrix = 0;
bool timer_started_matrix = false;
const long timer_interval_matrix = 3000;
#pragma endregion

#pragma region states
int normal_state() {
  float current_position = map(distance, min_distance, max_distance, min_step, max_step);
  current_position = constrain(current_position, min_step, max_step);

  angle = map(current_position, min_step, max_step, min_angle, max_angle);

  motor.moveTo(current_position);
  motor.run();

  bool transition_T_C = distance < min_distance && (motor.distanceToGo() == 0);
  bool transition_T_F = distance > max_distance && (motor.distanceToGo() == 0);

  if (transition_T_C) {
    stepperAppState = TOO_CLOSE;
  } else if (transition_T_F) {
    stepperAppState = TOO_FAR;
  }

  return angle;
}

void too_close_state() {
  motor.disableOutputs();

  bool transition_N = distance > min_distance;

  if (transition_N) {
    stepperAppState = NORMAL;
  }
}

void too_far_state() {
  motor.disableOutputs();

  bool transition = distance < max_distance;

  if (transition) {
    stepperAppState = NORMAL;
  }
}

// void alert_off_state() {
//   bool transition = distance < alert_distance;

//   noTone(BUZZER_PIN);
//   set_color_task(0, 0, 0);

//   if (transition) {
//     alertAppState = ALERT_ON;
//   }
// }

// void alert_on_state(unsigned long ct) {
//   static unsigned long start_timer = 0;
//   static bool timer_started = false;
//   const long timer_interval = 3000;

//   bool transition = distance > alert_distance;

//   tone(BUZZER_PIN, frequency);
//   led_blink_task(ct);

//   if (transition) {        // detecte condition sortie etat
//     if (!timer_started) {  // demarre le timer si pas deja demarre
//       start_timer = ct;
//       timer_started = true;
//     } else if (ct - start_timer >= timer_interval) {  // change detat si le timer est supperieur a 3 secondes
//       timer_started = false;
//       alertAppState = ALERT_OFF;
//     }
//   } else {  // reinitialise le timer si la transition tombe a false
//     timer_started = false;
//   }
// }

void stepper_state_manager() {
  switch (stepperAppState) {
    case NORMAL:
      normal_state();
      break;

    case TOO_CLOSE:
      too_close_state();
      break;

    case TOO_FAR:
      too_far_state();
      break;
  }
}

// void alert_state_manager(unsigned long ct) {
//   switch (alertAppState) {
//     case ALERT_OFF:
//       alert_off_state();
//       break;

//     case ALERT_ON:
//       alert_on_state(ct);
//       break;
//   }
// }

void matrix_state_manager(unsigned long ct) {
  u8g2.clearBuffer();
  switch (matrixAppState) {
    case EMPTY:
      u8g2.clearBuffer();
      u8g2.sendBuffer();
      break;
    case ERROR:
      matrix_error(ct);
      break;

    case BAD_COMMAND:
      matrix_bad_command(ct);
      break;

    case CHECKMARK:
      matrix_checkmark(ct);
      break;
  }
}
#pragma endregion

#pragma region tasks
int distance_task(unsigned long ct) {
  static unsigned long previous_time = 0;
  const long interval = 50;
  static int last_distance;

  if (ct - previous_time >= interval) {
    distance = hc.dist();

    previous_time = ct;

    if (distance > 0) {
      last_distance = distance;
    } else {
      distance = last_distance;
    }
  }

  return distance;
}

void print_task(unsigned long ct) {
  static unsigned long previous_time = 0;
  const long interval = 100;

  if (ct - previous_time >= interval) {
    previous_time = ct;

    // Serial.print("etd:2255309,dist:");
    // Serial.print(distance);
    // Serial.print(",deg:");

    lcd.clear();

    char distStr[10];
    dtostrf(distance, 6, 2, distStr);
    snprintf(lcdBuff[0], sizeof(lcdBuff[0]), "Dist :%s cm", distStr);
    lcd.setCursor(0, 0);
    lcd.print(lcdBuff[0]);

    lcd.setCursor(0, 1);
    lcd.print("Obj  : ");

    if (distance < min_distance) {
      if (distance < alert_distance) {
        snprintf(lcdBuff[1], sizeof(lcdBuff[1]), "ALERTE");
        // Serial.println("ALERTE");
      } else {
        snprintf(lcdBuff[1], sizeof(lcdBuff[1]), "Trop pres");
        // Serial.println("Trop pres");
      }
    } else if (distance > max_distance) {
      snprintf(lcdBuff[1], sizeof(lcdBuff[1]), "Trop loin");
      // Serial.println("Trop loin");
    } else {
      snprintf(lcdBuff[1], sizeof(lcdBuff[1]), "%d deg", angle);
      // Serial.println(angle);
    }

    lcd.print(lcdBuff[1]);
  }
}

void set_color_task(int R, int G, int B) {
  analogWrite(LED_PIN_RED, R);
  analogWrite(LED_PIN_GREEN, G);
  analogWrite(LED_PIN_BLUE, B);
}

void led_blink_task(unsigned long ct) {
  static unsigned long previous_time = 0;
  const long led_interval = 200;

  if (ct - previous_time >= led_interval) {
    previous_time = ct;

    switch (current_color) {
      case 0:
        set_color_task(255, 0, 0);  // rouge
        current_color++;
        break;
      case 1:
        set_color_task(0, 0, 255);  // bleu
        current_color = 0;
        break;
    }
  }
}


void serial_event_default_msg(String tampon) {
  Serial.print("PC : " + tampon + "\n" + "Arduino : ");
}

bool matrix_timer(unsigned long ct) {
  if (!timer_started_matrix) {  // demarre le timer si pas deja demarre
    start_timer_matrix = ct;
    timer_started_matrix = true;
  } else if (ct - start_timer_matrix >= timer_interval_matrix) {  // change detat si le timer est supperieur a 3 secondes
    start_timer_matrix = 0;
    timer_started_matrix = false;
    matrixAppState = EMPTY;
    return true;
  }
  return false;
}

void matrix_error(unsigned long ct) {
  u8g2.drawCircle(3, 3, 3);
  u8g2.drawLine(4, 2, 2, 4);  // pour signe interdit
  u8g2.sendBuffer();
  matrix_timer(ct);
}

void matrix_bad_command(unsigned long ct) {
  u8g2.drawLine(1, 1, 6, 6);
  u8g2.drawLine(1, 6, 6, 1);  // pour X
  u8g2.sendBuffer();
  matrix_timer(ct);
}

void matrix_checkmark(unsigned long ct) {
  u8g2.drawLine(3, 1, 1, 3);
  u8g2.drawLine(1, 3, 5, 7);  // pour crochet
  u8g2.sendBuffer();
  matrix_timer(ct);
}

void parsing_command(const String& tampon, String& command, String& arg1, String& arg2) {
  command = "";
  arg1 = "";
  arg2 = "";

  int first_sep = tampon.indexOf(';');
  int second_sep = tampon.indexOf(';', first_sep + 1);

  if (first_sep == -1) {
    command = tampon;
    return;
  }

  command = tampon.substring(0, first_sep);

  if (second_sep != -1) {
    arg1 = tampon.substring(first_sep + 1, second_sep);

    arg2 = tampon.substring(second_sep + 1);
  } else {
    arg1 = tampon.substring(first_sep + 1);
  }
}

void serial_event_task(unsigned long ct) {
  while (Serial.available()) {
    char input_char = Serial.read();
    if (input_char == '\n') {
      tampon.trim();

      String command;
      String arg1, arg2;

      parsing_command(tampon, command, arg1, arg2);

      if (command == "g_dist" || command == "gDist") {
        serial_event_default_msg(tampon);
        Serial.println(distance);
        matrixAppState = CHECKMARK;

      } else if (command == "cfg" && arg1 == "alm") {
        serial_event_default_msg(tampon);

        int new_alert_distance = arg2.toInt();

        if (new_alert_distance > 0 && new_alert_distance < min_distance) {
          alert_distance = new_alert_distance;
          Serial.print("Nouvelle distance de détection de l'alarme = ");
          Serial.println(alert_distance);
          matrixAppState = CHECKMARK;
        } else {
          Serial.println("🚫");
          matrixAppState = ERROR;
        }

      } else if (command == "cfg" && (arg1 == "lim_inf" || arg1 == "lim_sup")) {
        serial_event_default_msg(tampon);

        int new_limit = arg2.toInt();

        if (new_limit > 0) {
          if (arg1 == "lim_inf" && new_limit < max_distance) {
            min_distance = new_limit;
            Serial.print("Nouvelle distance minimum du moteur pas-à-pas = ");
            Serial.println(min_distance);
            matrixAppState = CHECKMARK;
          } else if (arg1 == "lim_sup" && new_limit > min_distance) {
            max_distance = new_limit;
            Serial.print("Nouvelle distance maximum du moteur pas-à-pas = ");
            Serial.println(max_distance);
            matrixAppState = CHECKMARK;
          } else {
            Serial.println("🚫");
            matrixAppState = ERROR;
          }
        }

      } else if (command == "turn_on" || command == "turnOn") {
        serial_event_default_msg(tampon);
        Serial.println("Système d'alarme allumé");

        alarm.turnOn();

      } else if (command == "turn_off" || command == "turnOff") {
        serial_event_default_msg(tampon);
        Serial.println("Système d'alarme éteint");

        alarm.turnOff();

      } else if (command == "test") {
        serial_event_default_msg(tampon);
        Serial.println("Test du système d'alarme");

        alarm.test();

      } else {
        Serial.println("❌");
        matrixAppState = BAD_COMMAND;
      }
      tampon = "";

    } else {
      tampon += input_char;
    }
  }
}
#pragma endregion

#pragma region setup - loop
void setup() {
  Serial.begin(115200);

  pinMode(BUZZER_PIN, OUTPUT);

  pinMode(LED_PIN_RED, OUTPUT);
  pinMode(LED_PIN_GREEN, OUTPUT);
  pinMode(LED_PIN_BLUE, OUTPUT);

  lcd.begin();
  lcd.backlight();

  motor.setMaxSpeed(500);
  motor.setAcceleration(100);
  motor.setSpeed(500);
  motor.setCurrentPosition(0);

  u8g2.begin();
  u8g2.setContrast(5);
  u8g2.setFont(u8g2_font_4x6_tr);
  u8g2.clearBuffer();
  u8g2.sendBuffer();

  start_task();
}

void loop() {
  current_time = millis();

  distance_task(current_time);
  stepper_state_manager();
  // alert_state_manager(current_time);
  alarm.update();
  print_task(current_time);
  matrix_state_manager(current_time);
  serial_event_task(current_time);
}
#pragma endregion

#pragma region start_task
void start_task() {
  lcd.setCursor(0, 0);
  lcd.print("2255309");

  lcd.setCursor(0, 1);
  lcd.print("Labo 5");

  delay(2000);
  lcd.clear();
}
#pragma endregion