#include <ESP32Servo.h>

// Revisar los PINS y agregar el RFID
#define LED 25
#define FOTORESISTOR 35

// Cantidad máxima de sensores
#define MAX_CANT_SENSORES 1
#define IDX_SENSOR_LUZ 0

// Eventos
enum events_luz
{
  EV_CONT,
  EV_DIA_DETECTADO,
  EV_NOCHE_DETECTADA
};
#define CANT_MAX_EVENTOS_LUZ 3

// Estados
enum states_luz
{
  ST_LUZ_APAGADA,
  ST_LUZ_ENCENDIDA
  // STATE_LUZ_COUNT
} current_state_luz; // Declaro el estado global de la luz
#define CANT_MAX_ESTADOS_LUZ 2

// Acciones
enum actions_luz
{
  ACC_ENCENDER_LUZ,
  ACC_APAGAR_LUZ
};
#define CANT_MAX_ACCIONES_LUZ 2

// Tamaños de las colas
#define TAM_EV_COLA_LUZ 10
#define TAM_ACC_COLA_LUZ 10
QueueHandle_t queueEventos_luz;
QueueHandle_t queueAcciones_luz;

// Umbrales
#define UMBRAL_LUZ 2048 // Probar en wokwi y ajustar

struct stSensor
{
  int pin;
  int estado;
  long valor_actual;
  long valor_previo;
};
stSensor sensores[MAX_CANT_SENSORES];

typedef void (*transition)();

// Firmas de las funciones
void none();
void encender_luz();
void apagar_luz();
void crear_colas_luz();
void configuracion_debbug_esp32();
void configuracion_pines_esp32();
void configuracion_sensores_luz();
void configuracion_estado_inicial_luz();
void crear_tareas_luz();
void luz_deteccion(void *pvParameters);
void luz_controlador(void *pvParameters);
void luz_accion(void *pvParameters);

transition luz_state_table[CANT_MAX_ESTADOS_LUZ][CANT_MAX_EVENTOS_LUZ] =
    {
        {none, none, encender_luz}, // state ST_LUZ_APAGADA
        {none, apagar_luz, none}    // state ST_LUZ_ENCENDIDA

        // EV_CONT  , EV_DIA_DETECTADO  , EV_NOCHE_DETECTADA
};

void none()
{
  // No hacer nada
  return;
}

void encender_luz()
{
  // Emitir la acción a la cola de acciones
  // Transicionar a ST_LUZ_ENCENDIDA
  return;
}

void apagar_luz()
{
  // Emitir la acción a la cola de acciones
  // Transicionar a ST_LUZ_APAGADA
  return;
}

// --------------- SETUP Y LOOP ---------------
void setup()
{
  configuracion_debbug_esp32();
  crear_colas_luz();
  configuracion_pines_esp32();
  configuracion_sensores_luz();
  configuracion_estado_inicial_luz();
  crear_tareas_luz();
}

void configuracion_sensores_luz()
{
  sensores[IDX_SENSOR_LUZ].pin = FOTORESISTOR;
  sensores[IDX_SENSOR_LUZ].estado = 0; // Esto lo vamos a usar?
  sensores[IDX_SENSOR_LUZ].valor_actual = 0;
  sensores[IDX_SENSOR_LUZ].valor_previo = 0; // Esto lo vamos a usar?
}

void crear_colas_luz()
{
  queueEventos_luz = xQueueCreate(TAM_EV_COLA_LUZ, sizeof(events_luz));
  queueAcciones_luz = xQueueCreate(TAM_ACC_COLA_LUZ, sizeof(actions_luz));
}

void configuracion_debbug_esp32()
{
  // Configurar el puerto serial para debugguear
  Serial.begin(115200); // default de wokwi?
}

void configuracion_estado_inicial_luz()
{
  current_state_luz = ST_LUZ_APAGADA;
}

void configuracion_pines_esp32()
{
  pinMode(LED, OUTPUT);
  pinMode(FOTORESISTOR, INPUT);
}

// --------------- TAREAS ---------------
void luz_deteccion(void *pvParameters)
{
  while (1)
  {
    // Leer el valor del fotoresistor
    // Comparar con el umbral
    // Emitir evento correspondiente a la cola de eventos
    sensores[IDX_SENSOR_LUZ].valor_actual = analogRead(FOTORESISTOR);

    Serial.print("[luz_deteccion] ADC=");
    Serial.print(sensores[IDX_SENSOR_LUZ].valor_actual);

    events_luz evento;
    bool hay_evento = false;

    if (current_state_luz == ST_LUZ_APAGADA &&
        sensores[IDX_SENSOR_LUZ].valor_actual < UMBRAL_LUZ)
    {
      evento = EV_NOCHE_DETECTADA;
      hay_evento = true;
    }
    else if (current_state_luz == ST_LUZ_ENCENDIDA &&
             sensores[IDX_SENSOR_LUZ].valor_actual >= UMBRAL_LUZ)
    {
      evento = EV_DIA_DETECTADO;
      hay_evento = true;
    }
    if (hay_evento)
    {
      TickType_t timeOut = 0;
      if (xQueueSend(queueEventos_luz, &evento, timeOut) != pdPASS)
      {
        Serial.println("[luz_deteccion] Cola de eventos LLENA");
      }
      else
      {
        Serial.print(">> Evento emitido: ");
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
    // Esperar eventos en la cola de eventos
    // Ejecutar la transición correspondiente de la tabla de estados
    TickType_t timeOut = 0; // hace falta ponerle un valor? creo que no porque usamos vTaskDelay(pdMS_TO_TICKS(200));
    if (xQueueReceive(queueEventos_luz, &evento_recibido, timeOut) == pdPASS)
    {
      Serial.print("[luz_controlador] Evento recibido=");
      Serial.print(evento_recibido);
      Serial.print(" | estado_previo=");
      Serial.println(current_state_luz);

      // Usar la tabla de estados para ejecutar la transición correspondiente
    }
    vTaskDelay(pdMS_TO_TICKS(200)); // Falta pdMS_TO_TICKS si queremos pasar de segundos a ticks
                                    // Para nosotros es un await
  }
}

void luz_accion(void *pvParameters)
{
  while (1)
  {
    // Esperar acciones en la cola de acciones
    // Ejecutar la acción correspondiente (encender o apagar el LED)

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

void loop()
{
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
