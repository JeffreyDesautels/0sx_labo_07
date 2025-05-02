#pragma region definitions
#include <LCD_I2C.h>
#include <HCSR04.h>
#include <U8g2lib.h>
#include "Alarm.h"
#include "ViseurAutomatique.h"

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

LCD_I2C lcd(0x27, 16, 2);
HCSR04 hc(TRIGGER_PIN, ECHO_PIN);
U8G2_MAX7219_8X8_F_4W_SW_SPI u8g2(
  U8G2_R0,
  CLK_PIN,
  DIN_PIN,
  CS_PIN,
  U8X8_PIN_NONE,
  U8X8_PIN_NONE);

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

float distance;

unsigned long current_time = 0;

String tampon = "";

unsigned long start_timer_matrix = 0;
bool timer_started_matrix = false;
const long timer_interval_matrix = 3000;

Alarm alarm(LED_PIN_RED, LED_PIN_GREEN, LED_PIN_BLUE, BUZZER_PIN, distance);
ViseurAutomatique viseurAutomatique(IN_1, IN_2, IN_3, IN_4, distance);
#pragma endregion

#pragma region states
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

    lcd.clear();

    char distStr[10];
    dtostrf(distance, 6, 2, distStr);
    snprintf(lcdBuff[0], sizeof(lcdBuff[0]), "Dist :%s cm", distStr);
    lcd.setCursor(0, 0);
    lcd.print(lcdBuff[0]);

    lcd.setCursor(0, 1);
    lcd.print("Obj  : ");
    lcd.print(viseurAutomatique.getEtatTexte());
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

        float new_alert_distance = arg2.toFloat();

        if (new_alert_distance > 0) {
          alarm.setDistance(new_alert_distance);
          Serial.print("Nouvelle distance de détection de l'alarme = ");
          Serial.println(alarm.getDistance());
          matrixAppState = CHECKMARK;

        } else {
          Serial.println("🚫");
          matrixAppState = ERROR;
        }

      } else if (command == "cfg" && (arg1 == "lim_inf" || arg1 == "lim_sup")) {
        serial_event_default_msg(tampon);

        float new_limit = arg2.toFloat();

        if (new_limit > 0) {

          if (arg1 == "lim_inf" && new_limit < viseurAutomatique.getDistanceMaxSuivi()) {
            viseurAutomatique.setDistanceMinSuivi(new_limit);
            Serial.print("Nouvelle distance minimum du moteur pas-à-pas = ");
            Serial.println(viseurAutomatique.getDistanceMinSuivi());
            matrixAppState = CHECKMARK;

          } else if (arg1 == "lim_sup" && new_limit > viseurAutomatique.getDistanceMinSuivi()) {
            viseurAutomatique.setDistanceMaxSuivi(new_limit);
            Serial.print("Nouvelle distance maximum du moteur pas-à-pas = ");
            Serial.println(viseurAutomatique.getDistanceMaxSuivi());
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

      } else if (command == "activer") {
        serial_event_default_msg(tampon);
        Serial.println("Moteur activé");

        viseurAutomatique.activer();

      } else if (command == "desactiver") {
        serial_event_default_msg(tampon);
        Serial.println("Moteur désactivé");

        viseurAutomatique.desactiver();

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

  lcd.begin();
  lcd.backlight();

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
  viseurAutomatique.update();
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