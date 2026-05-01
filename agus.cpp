#include <ESP32Servo.h>
#include <SPI.h>
#include <MFRC522.h>

// =============================== PINES (segun diagram.json) ===============================
// Luz
#define LED 25
#define FOTORESISTOR 35
// Puerta
#define BUZZER 4
#define ULTRASONIC_TRIG 5
#define SERVO_PIN 12
#define BUTTON_APP 14
#define RFID_SCK 18
#define RFID_MISO 19
#define RFID_SDA 21
#define RFID_RST 22
#define RFID_MOSI 23
#define ULTRASONIC_ECHO 34

// =============================== PARAMETROS ===============================
#define UMBRAL_LUZ 2048
#define UMBRAL_PROXIMIDAD_CM 30
#define TIMEOUT_PUERTA_MS 5000
#define ANGULO_CERRADA 90
#define ANGULO_ABIERTA_AFUERA 0
#define ANGULO_ABIERTA_ADENTRO 180
#define BEEP_MS 80

// Comentar/descomentar para arrancar bloqueada por defecto
// #define INICIO_BLOQUEADO

// =============================== SENSORES ===============================
#define MAX_CANT_SENSORES 4
#define IDX_SENSOR_LUZ 0
#define IDX_SENSOR_PROXIMIDAD 1
#define IDX_SENSOR_RFID 2
#define IDX_SENSOR_APP 3

struct stSensor
{
  int pin;
  int estado;
  long valor_actual;
  long valor_previo;
};
stSensor sensores[MAX_CANT_SENSORES];

// =============================== LUZ - FSM ===============================
enum events_luz
{
  EV_CONT,
  EV_DIA_DETECTADO,
  EV_NOCHE_DETECTADA
};
#define CANT_MAX_EVENTOS_LUZ 3

enum states_luz
{
  ST_LUZ_APAGADA,
  ST_LUZ_ENCENDIDA
} current_state_luz;
#define CANT_MAX_ESTADOS_LUZ 2

enum actions_luz
{
  ACC_ENCENDER_LUZ,
  ACC_APAGAR_LUZ
};
#define CANT_MAX_ACCIONES_LUZ 2

#define TAM_EV_COLA_LUZ 10
#define TAM_ACC_COLA_LUZ 10
QueueHandle_t queueEventos_luz;
QueueHandle_t queueAcciones_luz;

// =============================== PUERTA - FSM ===============================
enum events_puerta
{
  EV_INIT_NO_BLOQUEADA,
  EV_INIT_BLOQUEADA,
  EV_DESBLOQUEO_POR_APP,
  EV_BLOQUEO_POR_APP,
  EV_ANIMAL_DETECTADO_ADENTRO,
  EV_ANIMAL_DETECTADO_AFUERA,
  EV_TIMEOUT
};
#define CANT_MAX_EVENTOS_PUERTA 7

enum states_puerta
{
  ST_ARRANQUE,
  ST_CERRADA_NO_BLOQUEADA,
  ST_CERRADA_BLOQUEADA,
  ST_ABIERTA_DESDE_AFUERA,
  ST_ABIERTA_DESDE_ADENTRO
} current_state_puerta;
#define CANT_MAX_ESTADOS_PUERTA 5

enum actions_puerta
{
  ACC_ABRIR_DESDE_AFUERA,
  ACC_ABRIR_DESDE_ADENTRO,
  ACC_CERRAR
};
#define CANT_MAX_ACCIONES_PUERTA 3

#define TAM_EV_COLA_PUERTA 10
#define TAM_ACC_COLA_PUERTA 10
QueueHandle_t queueEventos_puerta;
QueueHandle_t queueAcciones_puerta;

TimerHandle_t timer_puerta;

// =============================== ACTUADORES ===============================
Servo servo_puerta;
MFRC522 mfrc522(RFID_SDA, RFID_RST);

// =============================== FIRMAS ===============================
typedef void (*transition)();

// Comun
void none();

// Luz - transiciones
void encender_luz();
void apagar_luz();
// Luz - config
void crear_colas_luz();
void configuracion_sensores_luz();
void configuracion_estado_inicial_luz();
void crear_tareas_luz();
// Luz - tareas
void luz_deteccion(void *pvParameters);
void luz_controlador(void *pvParameters);
void luz_accion(void *pvParameters);

// Puerta - transiciones
void init_no_bloqueada();
void init_bloqueada();
void bloquear_puerta();
void desbloquear_puerta();
void abrir_desde_afuera();
void abrir_desde_adentro();
void cerrar_puerta();
// Puerta - config
void crear_colas_puerta();
void configuracion_sensores_puerta();
void configuracion_estado_inicial_puerta();
void crear_timer_puerta();
void crear_tareas_puerta();
void timer_puerta_callback(TimerHandle_t xTimer);
// Puerta - tareas
void puerta_deteccion(void *pvParameters);
void puerta_controlador(void *pvParameters);
void puerta_accion(void *pvParameters);
long medir_distancia_cm();
void buzzer_beep(int freq_hz, int duration_ms);

// Generales
void configuracion_debbug_esp32();
void configuracion_pines_esp32();
void configuracion_servo();
void configuracion_rfid();

// =============================== TABLAS DE TRANSICION ===============================
transition luz_state_table[CANT_MAX_ESTADOS_LUZ][CANT_MAX_EVENTOS_LUZ] =
    {
        {none, none, encender_luz}, // state ST_LUZ_APAGADA
        {none, apagar_luz, none}    // state ST_LUZ_ENCENDIDA

        // EV_CONT  , EV_DIA_DETECTADO  , EV_NOCHE_DETECTADA
};

transition puerta_state_table[CANT_MAX_ESTADOS_PUERTA][CANT_MAX_EVENTOS_PUERTA] =
    {
        {init_no_bloqueada, init_bloqueada, none,               none,            none,                none,               none         }, // state ST_ARRANQUE
        {none,              none,           none,               bloquear_puerta, abrir_desde_adentro, abrir_desde_afuera, none         }, // state ST_CERRADA_NO_BLOQUEADA
        {none,              none,           desbloquear_puerta, none,            none,                none,               none         }, // state ST_CERRADA_BLOQUEADA
        {none,              none,           none,               none,            none,                none,               cerrar_puerta}, // state ST_ABIERTA_DESDE_AFUERA
        {none,              none,           none,               none,            none,                none,               cerrar_puerta}  // state ST_ABIERTA_DESDE_ADENTRO

        // EV_INIT_NO_BLOQUEADA, EV_INIT_BLOQUEADA, EV_DESBLOQUEO_POR_APP, EV_BLOQUEO_POR_APP, EV_ANIMAL_DETECTADO_ADENTRO, EV_ANIMAL_DETECTADO_AFUERA, EV_TIMEOUT
};

// =============================== TRANSICIONES - LUZ ===============================
void none()
{
  return;
}

void encender_luz()
{
  actions_luz accion = ACC_ENCENDER_LUZ;
  if (xQueueSend(queueAcciones_luz, &accion, 0) != pdPASS)
  {
    Serial.println("[encender_luz] Cola de acciones LLENA");
    return;
  }
  current_state_luz = ST_LUZ_ENCENDIDA;
  Serial.println(">> Transicion luz: APAGADA -> ENCENDIDA");
}

void apagar_luz()
{
  actions_luz accion = ACC_APAGAR_LUZ;
  if (xQueueSend(queueAcciones_luz, &accion, 0) != pdPASS)
  {
    Serial.println("[apagar_luz] Cola de acciones LLENA");
    return;
  }
  current_state_luz = ST_LUZ_APAGADA;
  Serial.println(">> Transicion luz: ENCENDIDA -> APAGADA");
}

// =============================== TRANSICIONES - PUERTA ===============================
void init_no_bloqueada()
{
  current_state_puerta = ST_CERRADA_NO_BLOQUEADA;
  Serial.println(">> Init puerta -> ST_CERRADA_NO_BLOQUEADA");
}

void init_bloqueada()
{
  current_state_puerta = ST_CERRADA_BLOQUEADA;
  Serial.println(">> Init puerta -> ST_CERRADA_BLOQUEADA");
}

void bloquear_puerta()
{
  current_state_puerta = ST_CERRADA_BLOQUEADA;
  Serial.println(">> Transicion puerta: NO_BLOQUEADA -> BLOQUEADA (app)");
}

void desbloquear_puerta()
{
  current_state_puerta = ST_CERRADA_NO_BLOQUEADA;
  Serial.println(">> Transicion puerta: BLOQUEADA -> NO_BLOQUEADA (app)");
}

void abrir_desde_afuera()
{
  actions_puerta accion = ACC_ABRIR_DESDE_AFUERA;
  if (xQueueSend(queueAcciones_puerta, &accion, 0) != pdPASS)
  {
    Serial.println("[abrir_desde_afuera] Cola de acciones LLENA");
    return;
  }
  current_state_puerta = ST_ABIERTA_DESDE_AFUERA;
  xTimerStart(timer_puerta, 0);
  Serial.println(">> Transicion puerta: NO_BLOQUEADA -> ABIERTA_DESDE_AFUERA");
}

void abrir_desde_adentro()
{
  actions_puerta accion = ACC_ABRIR_DESDE_ADENTRO;
  if (xQueueSend(queueAcciones_puerta, &accion, 0) != pdPASS)
  {
    Serial.println("[abrir_desde_adentro] Cola de acciones LLENA");
    return;
  }
  current_state_puerta = ST_ABIERTA_DESDE_ADENTRO;
  xTimerStart(timer_puerta, 0);
  Serial.println(">> Transicion puerta: NO_BLOQUEADA -> ABIERTA_DESDE_ADENTRO");
}

void cerrar_puerta()
{
  actions_puerta accion = ACC_CERRAR;
  if (xQueueSend(queueAcciones_puerta, &accion, 0) != pdPASS)
  {
    Serial.println("[cerrar_puerta] Cola de acciones LLENA");
    return;
  }
  current_state_puerta = ST_CERRADA_NO_BLOQUEADA;
  Serial.println(">> Transicion puerta: ABIERTA -> NO_BLOQUEADA (timeout)");
}

// =============================== SETUP / LOOP ===============================
void setup()
{
  configuracion_debbug_esp32();
  crear_colas_luz();
  crear_colas_puerta();
  configuracion_pines_esp32();
  configuracion_sensores_luz();
  configuracion_sensores_puerta();
  configuracion_servo();
  configuracion_rfid();
  configuracion_estado_inicial_luz();
  configuracion_estado_inicial_puerta();
  crear_timer_puerta();
  crear_tareas_luz();
  crear_tareas_puerta();
}

void loop()
{
}

// =============================== CONFIG ===============================
void configuracion_debbug_esp32()
{
  Serial.begin(115200);
}

void configuracion_pines_esp32()
{
  // Luz
  pinMode(LED, OUTPUT);
  pinMode(FOTORESISTOR, INPUT);
  // Puerta
  pinMode(BUZZER, OUTPUT);
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);
  pinMode(BUTTON_APP, INPUT); // pull-down externo en el diagrama
  digitalWrite(BUZZER, LOW);
  digitalWrite(ULTRASONIC_TRIG, LOW);
}

void configuracion_servo()
{
  servo_puerta.attach(SERVO_PIN);
  servo_puerta.write(ANGULO_CERRADA);
}

void configuracion_rfid()
{
  SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_SDA);
  mfrc522.PCD_Init();
}

void configuracion_sensores_luz()
{
  sensores[IDX_SENSOR_LUZ].pin = FOTORESISTOR;
  sensores[IDX_SENSOR_LUZ].estado = 1; // 1 = dia, 0 = noche
  sensores[IDX_SENSOR_LUZ].valor_actual = 0;
  sensores[IDX_SENSOR_LUZ].valor_previo = 0;
}

void configuracion_sensores_puerta()
{
  sensores[IDX_SENSOR_PROXIMIDAD].pin = ULTRASONIC_ECHO;
  sensores[IDX_SENSOR_PROXIMIDAD].estado = 1; // 1 = libre, 0 = animal cerca
  sensores[IDX_SENSOR_PROXIMIDAD].valor_actual = 0;
  sensores[IDX_SENSOR_PROXIMIDAD].valor_previo = 0;

  sensores[IDX_SENSOR_RFID].pin = RFID_SDA;
  sensores[IDX_SENSOR_RFID].estado = 0;
  sensores[IDX_SENSOR_RFID].valor_actual = 0;
  sensores[IDX_SENSOR_RFID].valor_previo = 0;

  sensores[IDX_SENSOR_APP].pin = BUTTON_APP;
  sensores[IDX_SENSOR_APP].estado = LOW;
  sensores[IDX_SENSOR_APP].valor_actual = 0;
  sensores[IDX_SENSOR_APP].valor_previo = 0;
}

void configuracion_estado_inicial_luz()
{
  current_state_luz = ST_LUZ_APAGADA;
}

void configuracion_estado_inicial_puerta()
{
  current_state_puerta = ST_ARRANQUE;
#ifdef INICIO_BLOQUEADO
  events_puerta evento_init = EV_INIT_BLOQUEADA;
#else
  events_puerta evento_init = EV_INIT_NO_BLOQUEADA;
#endif
  xQueueSend(queueEventos_puerta, &evento_init, 0);
}

void crear_colas_luz()
{
  queueEventos_luz = xQueueCreate(TAM_EV_COLA_LUZ, sizeof(events_luz));
  queueAcciones_luz = xQueueCreate(TAM_ACC_COLA_LUZ, sizeof(actions_luz));
}

void crear_colas_puerta()
{
  queueEventos_puerta = xQueueCreate(TAM_EV_COLA_PUERTA, sizeof(events_puerta));
  queueAcciones_puerta = xQueueCreate(TAM_ACC_COLA_PUERTA, sizeof(actions_puerta));
}

void timer_puerta_callback(TimerHandle_t xTimer)
{
  events_puerta evento = EV_TIMEOUT;
  xQueueSend(queueEventos_puerta, &evento, 0);
}

void crear_timer_puerta()
{
  timer_puerta = xTimerCreate(
      "TimerPuerta",
      pdMS_TO_TICKS(TIMEOUT_PUERTA_MS),
      pdFALSE, // one-shot
      NULL,
      timer_puerta_callback);
}

// =============================== TAREAS - LUZ ===============================
void luz_deteccion(void *pvParameters)
{
  while (1)
  {
    sensores[IDX_SENSOR_LUZ].valor_actual = analogRead(FOTORESISTOR);

    Serial.print("[luz_deteccion] ADC=");
    Serial.print(sensores[IDX_SENSOR_LUZ].valor_actual);
    Serial.print(" pin25=");
    Serial.println(digitalRead(LED));

    int condicion_actual = (sensores[IDX_SENSOR_LUZ].valor_actual >= UMBRAL_LUZ) ? 1 : 0;

    if (condicion_actual != sensores[IDX_SENSOR_LUZ].estado)
    {
      events_luz evento = (condicion_actual == 0) ? EV_NOCHE_DETECTADA : EV_DIA_DETECTADO;
      if (xQueueSend(queueEventos_luz, &evento, 0) != pdPASS)
      {
        Serial.println("[luz_deteccion] Cola de eventos LLENA");
      }
      else
      {
        sensores[IDX_SENSOR_LUZ].estado = condicion_actual;
        sensores[IDX_SENSOR_LUZ].valor_previo = sensores[IDX_SENSOR_LUZ].valor_actual;
        Serial.print(">> Evento luz: ");
        Serial.println(evento == EV_DIA_DETECTADO ? "EV_DIA_DETECTADO" : "EV_NOCHE_DETECTADA");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void luz_controlador(void *pvParameters)
{
  events_luz evento_recibido;
  while (1)
  {
    if (xQueueReceive(queueEventos_luz, &evento_recibido, 0) == pdPASS)
    {
      Serial.print("[luz_controlador] Evento=");
      Serial.print(evento_recibido);
      Serial.print(" estado_previo=");
      Serial.println(current_state_luz);

      luz_state_table[current_state_luz][evento_recibido]();
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void luz_accion(void *pvParameters)
{
  actions_luz accion_recibida;
  while (1)
  {
    if (xQueueReceive(queueAcciones_luz, &accion_recibida, 0) == pdPASS)
    {
      switch (accion_recibida)
      {
      case ACC_ENCENDER_LUZ:
        digitalWrite(LED, HIGH);
        Serial.println("[luz_accion] LED ENCENDIDO");
        break;
      case ACC_APAGAR_LUZ:
        digitalWrite(LED, LOW);
        Serial.println("[luz_accion] LED APAGADO");
        break;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void crear_tareas_luz()
{
  int tam_stack_bytes = 1024 * 8;
  xTaskCreate(luz_deteccion, "Luz detección", tam_stack_bytes, NULL, 1, NULL);
  xTaskCreate(luz_controlador, "Luz controlador", tam_stack_bytes, NULL, 1, NULL);
  xTaskCreate(luz_accion, "Luz accion", tam_stack_bytes, NULL, 1, NULL);
}

// =============================== TAREAS - PUERTA ===============================
long medir_distancia_cm()
{
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);

  long duracion = pulseIn(ULTRASONIC_ECHO, HIGH, 30000UL); // 30 ms timeout
  if (duracion == 0)
  {
    return -1;
  }
  return (long)((duracion * 343UL) / 20000UL); // cm
}

// Genera onda cuadrada en BUZZER sin usar LEDC (evita conflicto con ESP32Servo)
void buzzer_beep(int freq_hz, int duration_ms)
{
  if (freq_hz <= 0 || duration_ms <= 0)
    return;
  unsigned long period_us = 1000000UL / (unsigned long)freq_hz;
  unsigned long half_us = period_us / 2;
  unsigned long cycles = ((unsigned long)duration_ms * 1000UL) / period_us;
  for (unsigned long i = 0; i < cycles; i++)
  {
    digitalWrite(BUZZER, HIGH);
    delayMicroseconds(half_us);
    digitalWrite(BUZZER, LOW);
    delayMicroseconds(half_us);
  }
}

void puerta_deteccion(void *pvParameters)
{
#ifdef INICIO_BLOQUEADO
  static bool app_supuesto_bloqueado = true;
#else
  static bool app_supuesto_bloqueado = false;
#endif

  while (1)
  {
    // ---- Proximidad (animal detectado adentro) ----
    long distancia = medir_distancia_cm();
    sensores[IDX_SENSOR_PROXIMIDAD].valor_actual = distancia;
    int condicion_prox = (distancia > 0 && distancia < UMBRAL_PROXIMIDAD_CM) ? 0 : 1;
    // 0 = animal cerca, 1 = libre

    if (condicion_prox != sensores[IDX_SENSOR_PROXIMIDAD].estado)
    {
      sensores[IDX_SENSOR_PROXIMIDAD].estado = condicion_prox;
      if (condicion_prox == 0)
      {
        events_puerta evento = EV_ANIMAL_DETECTADO_ADENTRO;
        if (xQueueSend(queueEventos_puerta, &evento, 0) == pdPASS)
        {
          Serial.print(">> Evento puerta: EV_ANIMAL_DETECTADO_ADENTRO (cm=");
          Serial.print(distancia);
          Serial.println(")");
        }
      }
    }

    // ---- RFID (animal detectado afuera) ----
    if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial())
    {
      events_puerta evento = EV_ANIMAL_DETECTADO_AFUERA;
      if (xQueueSend(queueEventos_puerta, &evento, 0) == pdPASS)
      {
        Serial.println(">> Evento puerta: EV_ANIMAL_DETECTADO_AFUERA (RFID)");
      }
      mfrc522.PICC_HaltA();
      mfrc522.PCD_StopCrypto1();
    }

    // ---- Boton app (toggle bloqueo/desbloqueo, no leemos la FSM) ----
    int btn_actual = digitalRead(BUTTON_APP);
    if (btn_actual == HIGH && sensores[IDX_SENSOR_APP].estado == LOW)
    {
      app_supuesto_bloqueado = !app_supuesto_bloqueado;
      events_puerta evento = app_supuesto_bloqueado ? EV_BLOQUEO_POR_APP : EV_DESBLOQUEO_POR_APP;
      if (xQueueSend(queueEventos_puerta, &evento, 0) == pdPASS)
      {
        Serial.print(">> Evento puerta: ");
        Serial.println(app_supuesto_bloqueado ? "EV_BLOQUEO_POR_APP" : "EV_DESBLOQUEO_POR_APP");
      }
    }
    sensores[IDX_SENSOR_APP].estado = btn_actual;

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void puerta_controlador(void *pvParameters)
{
  events_puerta evento_recibido;
  while (1)
  {
    if (xQueueReceive(queueEventos_puerta, &evento_recibido, 0) == pdPASS)
    {
      Serial.print("[puerta_controlador] Evento=");
      Serial.print(evento_recibido);
      Serial.print(" estado_previo=");
      Serial.println(current_state_puerta);

      puerta_state_table[current_state_puerta][evento_recibido]();
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void puerta_accion(void *pvParameters)
{
  actions_puerta accion_recibida;
  while (1)
  {
    if (xQueueReceive(queueAcciones_puerta, &accion_recibida, 0) == pdPASS)
    {
      switch (accion_recibida)
      {
      case ACC_ABRIR_DESDE_AFUERA:
        servo_puerta.write(ANGULO_ABIERTA_AFUERA);
        buzzer_beep(1000, 300);
        Serial.println("[puerta_accion] ABIERTA DESDE AFUERA (buzzer 1000Hz)");
        break;
      case ACC_ABRIR_DESDE_ADENTRO:
        servo_puerta.write(ANGULO_ABIERTA_ADENTRO);
        buzzer_beep(1500, 300);
        Serial.println("[puerta_accion] ABIERTA DESDE ADENTRO (buzzer 1500Hz)");
        break;
      case ACC_CERRAR:
        servo_puerta.write(ANGULO_CERRADA);
        buzzer_beep(600, 150);
        vTaskDelay(pdMS_TO_TICKS(100));
        buzzer_beep(600, 150);
        Serial.println("[puerta_accion] CERRADA (buzzer 600Hz x2)");
        break;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void crear_tareas_puerta()
{
  int tam_stack_bytes = 1024 * 8;
  xTaskCreate(puerta_deteccion, "Puerta detección", tam_stack_bytes, NULL, 1, NULL);
  xTaskCreate(puerta_controlador, "Puerta controlador", tam_stack_bytes, NULL, 1, NULL);
  xTaskCreate(puerta_accion, "Puerta accion", tam_stack_bytes, NULL, 1, NULL);
}

/*
 BaseType_t xTaskCreate( TaskFunction_t pvTaskCode,
                         const char * const pcName,
                         const configSTACK_DEPTH_TYPE uxStackDepth,
                         void *pvParameters,
                         UBaseType_t uxPriority,
                         TaskHandle_t *pxCreatedTask
                       );

*/
