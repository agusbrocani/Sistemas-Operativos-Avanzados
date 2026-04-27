#include <ESP32Servo.h>

// Revisar los PINS y agregar el RFID
#define LED 25
#define FOTORESISTOR 35

// Cantidad máxima de sensores
#define MAX_CANT_SENSORES 1

// Tamaños de las colas
#define TAM_EV_COLA_LUZ 10
#define TAM_ACC_COLA_LUZ 10

// Eventos
enum events_luz {
    EV_CONT,
    EV_DIA_DETECTADO,
    EV_NOCHE_DETECTADA
};
#define CANT_MAX_EVENTOS_LUZ 3

// Estados
enum states_luz {
    ST_LUZ_APAGADA,
    ST_LUZ_ENCENDIDA
    // STATE_LUZ_COUNT
} current_state_luz; //Declaro el estado global de la luz
#define CANT_MAX_ESTADOS_LUZ 2

// Umbrales
#define UMBRAL_LUZ 2048 // Probar en wokwi y ajustar

struct stSensor
{
  int  pin;
  int  estado;
  long valor_actual;
  long valor_previo;
};
stSensor sensores[MAX_CANT_SENSORES];


typedef void (*transition)();

void none();
void encender_luz();
void apagar_luz();

transition luz_state_table[CANT_MAX_ESTADOS_LUZ][CANT_MAX_EVENTOS_LUZ] =
{
      {none     , none          , encender_luz  } , // state ST_LUZ_APAGADA
      {none     , apagar_luz    , none          } // state ST_LUZ_ENCENDIDA
      
     //EV_CONT  , EV_DIA_DETECTADO  , EV_NOCHE_DETECTADA
};


void none(){
    // No hacer nada
    return;
}

void encender_luz(){
    // Emitir la acción a la cola de acciones
    // Transicionar a ST_LUZ_ENCENDIDA
    return;
}

void apagar_luz(){
    // Emitir la acción a la cola de acciones
    // Transicionar a ST_LUZ_APAGADA
    return;
}


// --------------- SETUP Y LOOP ---------------
void setup() {
    // configuracion_debbug_esp32();
    configuracion_pines_esp32();
    configuracion_estado_inicial_luz();
    crear_tareas_luz();
}

void configuracion_debbug_esp32() {
    // Configurar el puerto serial para debugguear
    Serial.begin(9600);
}

void configuracion_estado_inicial_luz() {
    current_state = ST_LUZ_APAGADA;
}


void configuracion_pines_esp32() {
    pinMode(LED, OUTPUT);
    pinMode(FOTORESISTOR, INPUT);
}


// --------------- TAREAS ---------------
void luz_deteccion(void *pvParameters) {
    while(1) {
        // Leer el valor del fotoresistor
        // Comparar con el umbral
        // Emitir evento correspondiente a la cola de eventos

        vTaskDelay(200 );
    }
}


void luz_controlador(void *pvParameters) {
    while(1) {
        // Esperar eventos en la cola de eventos
        // Ejecutar la transición correspondiente de la tabla de estados

        vTaskDelay(200);
    }
}


void luz_accion(void *pvParameters) {
    while(1) {
        // Esperar acciones en la cola de acciones
        // Ejecutar la acción correspondiente (encender o apagar el LED)

        vTaskDelay(200);
    }
}


void crear_tareas_luz() {
    int tam_stack_bytes = 1024 * 8;
    xTaskCreate(luz_deteccion, "Luz detección", tam_stack_bytes, NULL, 1, NULL);
    xTaskCreate(luz_controlador, "Luz controlador", tam_stack_bytes, NULL, 1, NULL);
    xTaskCreate(luz_accion, "Luz accion", tam_stack_bytes, NULL, 1, NULL);
}


void loop() {

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