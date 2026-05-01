#include <ESP32Servo.h>
#include <MFRC522.h>

// Revisar los PINS y agregar el RFID
#define LED 25
#define FOTORESISTOR 35

// RFID MFRC522
#define RFID_SS    21   // SDA en el módulo, SS en SPI
#define RFID_SCK   18
#define RFID_MOSI  23
#define RFID_MISO  19
#define RFID_RST   22

// Proximidad HC-SR04
#define SENSOR_PROXIMIDAD_ECHO 34
#define SENSOR_PROXIMIDAD_TRIGGER 5 

#define BUZZER 4
#define SERVO 12

// // Cantidad máxima de sensores
#define MAX_CANT_SENSORES 1
#define IDX_SENSOR_LUZ 0
// #define IDX_SENSOR_PROXIMIDAD 1
// #define IDX_SENSOR_RFID 2


// ---------------------- Luz ----------------------
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

// Acciones
enum actions_luz {
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


// ---------------------- Puerta ----------------------
// Eventos
enum events_puerta {
  EV_INIT_NO_BLOQUEADA,
  EV_INIT_BLOQUEADA,
  EV_DESBLOQUEO_POR_APP,
  EV_BLOQUEO_POR_APP,
  EV_ANIMAL_DETECTADO_ADENTRO,
  EV_ANIMAL_DETECTADO_AFUERA,
  EV_TIMEOUT
};
#define CANT_MAX_EVENTOS_PUERTA 7 

// Estados
enum states_puerta {
  ST_ARRANQUE,
  ST_CERRADA_NO_BLOQUEADA,
  ST_CERRADA_BLOQUEADA,
  ST_ABIERTA_DESDE_AFUERA,
  ST_ABIERTA_DESDE_ADENTRO
} current_state_puerta; //Declaro el estado global de la puerta
#define CANT_MAX_ESTADOS_PUERTA 5

// Acciones
enum actions_puerta {
  ACC_ABRIR_DESDE_AFUERA,
  ACC_ABRIR_DESDE_ADENTRO,
  ACC_CERRAR
};
#define CANT_MAX_ACCIONES_PUERTA 3

// Tamaños de las colas
#define TAM_EV_COLA_PUERTA 10
#define TAM_ACC_COLA_PUERTA 10
QueueHandle_t queueEventos_puerta;
QueueHandle_t queueAcciones_puerta;


// ---------------------- Estructura de datos ----------------------
// Array de sensores (CAMBIAR, ANDA SOLO PARA LUZ)
enum estado_sensor {
    ESTADO_HABILITADO,
    ESTADO_DESHABILITADO
};


struct stSensor
{
  int  pin;
  int  estado; // arreglar estado
  long valor_actual;
  long valor_previo;
};
stSensor sensores[MAX_CANT_SENSORES]; // sacar array

#define TIME_OUT_SENSOR_PROXIMIDAD 30000
struct stSensorProximidad {
    int pin_echo;
    int pin_trigger;
    estado_sensor estado;
    float distancia_actual_cm;
    int tiempo_transcurrido_ms;
    const float velocidad_sonido = 0.0343;
    const float distancia_minima_cm = 30;
}; stSensorProximidad sensor_proximidad; //Sensor global de proximidad

// RFID (crea el objeto que ocupa el lector)
MFRC522 rfid(RFID_SS, RFID_RST);

struct stSensorRFID {
    int pin_ss;
    int pin_reset;
    estado_sensor estado;
    int id_tag;
    bool acceso_permitido;
}; stSensorRFID sensor_rfid; //Sensor global de RFID


// Servomotor
Servo servo;

// Timer
TimerHandle_t timer_puerta;
#define TIEMPO_TIMEOUT_PUERTA 4500


// ---------------------- Firmas de las funciones ----------------------
// Firmas de las funciones
void none();
void encender_luz();
void apagar_luz();
void configuracion_debbug_esp32();
void configuracion_pines_esp32();
void setup_luz();

void configuracion_sensores_luz();
void configuracion_estado_inicial_luz();
void crear_colas_luz();

void crear_tareas_luz();
void luz_deteccion(void *pvParameters);
void luz_controlador(void *pvParameters);
void luz_accion(void *pvParameters);

void setup_puerta();
void crear_colas_puerta();
void configuracion_sensores_puerta();
void configuracion_estado_inicial_puerta();

void crear_tareas_puerta();
void puerta_deteccion(void *pvParameters);
void puerta_controlador(void *pvParameters);
void puerta_accion(void *pvParameters);

typedef void (*transition)();
transition luz_state_table[CANT_MAX_ESTADOS_LUZ][CANT_MAX_EVENTOS_LUZ] =
{
      {none     , none          , encender_luz  } , // state ST_LUZ_APAGADA
      {none     , apagar_luz    , none          } // state ST_LUZ_ENCENDIDA
      
     //EV_CONT  , EV_DIA_DETECTADO  , EV_NOCHE_DETECTADA
};


void none(){
    return;
}

void encender_luz(){
    // Emitir la acción a la cola de acciones
    // Transicionar a ST_LUZ_ENCENDIDA
    Serial.print("Transición iniciada: Luz encendida\n");
    current_state_luz = ST_LUZ_ENCENDIDA;
    actions_luz action = ACC_ENCENDER_LUZ;
    if (xQueueSend(queueAcciones_luz, &action, 0) != pdPASS) {
        Serial.println("[luz_controlador] Cola de acciones LLENA");
    }
    else {
        Serial.print(">> Acción emitida: ACC_ENCENDER_LUZ");
    }
    return;
}

void apagar_luz(){
    // Emitir la acción a la cola de acciones
    // Transicionar a ST_LUZ_APAGADA
    Serial.print("Transición iniciada: Luz apagada\n");
    current_state_luz = ST_LUZ_APAGADA;
    actions_luz action = ACC_APAGAR_LUZ;
    if (xQueueSend(queueAcciones_luz, &action, 0) != pdPASS) {
        Serial.println("[luz_controlador] Cola de acciones LLENA");
    }
    else {
        Serial.print(">> Acción emitida: ACC_APAGAR_LUZ");
    }
    return;
}


// --------------- SETUP Y LOOP ---------------
void setup() {
    configuracion_debbug_esp32();
    configuracion_pines_esp32();
    setup_luz();
    setup_puerta();
}

void setup_luz() {
    crear_colas_luz();
    configuracion_sensores_luz();
    configuracion_estado_inicial_luz();
    crear_tareas_luz();
}


void configuracion_sensores_luz() {
    sensores[IDX_SENSOR_LUZ].pin = FOTORESISTOR;
    sensores[IDX_SENSOR_LUZ].estado = 1; // Esto lo vamos a usar?
    sensores[IDX_SENSOR_LUZ].valor_actual = 0;
    sensores[IDX_SENSOR_LUZ].valor_previo = 0; // Esto lo vamos a usar?
}

void crear_colas_luz() {
    queueEventos_luz = xQueueCreate(TAM_EV_COLA_LUZ, sizeof(events_luz));
    queueAcciones_luz = xQueueCreate(TAM_ACC_COLA_LUZ, sizeof(actions_luz));
}


void configuracion_debbug_esp32() {
    // Configurar el puerto serial para debugguear
    Serial.begin(115200); // default de wokwi?
}

void configuracion_estado_inicial_luz() {
    current_state_luz = ST_LUZ_APAGADA;
}


void configuracion_pines_esp32() {
    pinMode(LED, OUTPUT);
    pinMode(FOTORESISTOR, INPUT);
    pinMode(BUZZER, OUTPUT);
    servo.attach(SERVO);
    pinMode(SENSOR_PROXIMIDAD_ECHO, INPUT);
    pinMode(SENSOR_PROXIMIDAD_TRIGGER, OUTPUT);
}


// --------------- TAREAS ---------------
void luz_deteccion(void *pvParameters) {
    while(1) {
        // Leer el valor del fotoresistor
        // Comparar con el umbral
        // Emitir evento correspondiente a la cola de eventos
        sensores[IDX_SENSOR_LUZ].valor_actual = analogRead(FOTORESISTOR);

        Serial.print("[luz_deteccion] ADC=");
        Serial.print(sensores[IDX_SENSOR_LUZ].valor_actual);

        events_luz evento;
        bool hay_evento = false;

        if (current_state_luz == ST_LUZ_APAGADA &&
            sensores[IDX_SENSOR_LUZ].valor_actual > UMBRAL_LUZ) {
            evento = EV_NOCHE_DETECTADA;
            hay_evento = true;
        }
        else if (current_state_luz == ST_LUZ_ENCENDIDA &&
                 sensores[IDX_SENSOR_LUZ].valor_actual <= UMBRAL_LUZ) {
            evento = EV_DIA_DETECTADO;
            hay_evento = true;
        }
        if (hay_evento) {
            TickType_t timeOut = 0;
            if (xQueueSend(queueEventos_luz, &evento, timeOut) != pdPASS) {
                Serial.println("[luz_deteccion] Cola de eventos LLENA");
            }
            else {
                Serial.print(">> Evento emitido: ");
                Serial.println(evento == EV_DIA_DETECTADO ? "EV_DIA_DETECTADO" : "EV_NOCHE_DETECTADA");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}


void luz_controlador(void *pvParameters) {
    events_luz evento_recibido;
    while(1) {
        // Esperar eventos en la cola de eventos
        // Ejecutar la transición correspondiente de la tabla de estados
        TickType_t timeOut = 0; // hace falta ponerle un valor? creo que no porque usamos vTaskDelay(pdMS_TO_TICKS(200));
        if (xQueueReceive(queueEventos_luz, &evento_recibido, timeOut) == pdPASS) {
            Serial.print("[luz_controlador] Evento recibido=");
            Serial.print(evento_recibido == EV_NOCHE_DETECTADA ? "EV_NOCHE_DETECTADA" : "EV_DIA_DETECTADO");
            Serial.print(" | estado_previo=");
            Serial.println(current_state_luz == ST_LUZ_APAGADA ? "ST_LUZ_APAGADA" : "ST_LUZ_ENCENDIDA");

            if (evento_recibido < CANT_MAX_EVENTOS_LUZ)
            {
                transition transition_function = luz_state_table[current_state_luz][evento_recibido];
                transition_function();
            }
            else
            {
                Serial.println("[luz_controlador] Evento fuera de rango");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2000)); // Falta pdMS_TO_TICKS si queremos pasar de segundos a ticks
        // Para nosotros es un await
    }
}


void luz_accion(void *pvParameters) {
    while(1) {
        actions_luz action_recibido;
        // Esperar acciones en la cola de acciones
        // Ejecutar la acción correspondiente (encender o apagar el LED)
        TickType_t timeOut = 0; // hace falta ponerle un valor? creo que no porque usamos vTaskDelay(pdMS_TO_TICKS(200));
        if (xQueueReceive(queueAcciones_luz, &action_recibido, timeOut) == pdPASS) {
            Serial.print("[luz_accion] Accion recibida=");
            Serial.print(action_recibido == ACC_ENCENDER_LUZ ? "ACC_ENCENDER_LUZ" : "ACC_APAGAR_LUZ");
            if (action_recibido == ACC_ENCENDER_LUZ) {
                digitalWrite(LED, HIGH);
            }
            else if (action_recibido == ACC_APAGAR_LUZ) {
                digitalWrite(LED, LOW);
            }
            else {
                Serial.println("[luz_accion] Accion fuera de rango");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
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


/* ---------------------- Setup puerta ---------------------- */
void init_no_bloqueada();
void init_bloqueada();
void bloquear_puerta();
void desbloquear_puerta();
void abrir_desde_adentro();
void abrir_desde_afuera();
void cerrar_puerta();

transition puerta_state_table[CANT_MAX_ESTADOS_PUERTA][CANT_MAX_EVENTOS_PUERTA] =
{
    {init_no_bloqueada,      init_bloqueada,    none,                  none,               none,                        none,                       none         }, // state ST_ARRANQUE
    {none,                   none,              none,                  bloquear_puerta,    abrir_desde_adentro,         abrir_desde_afuera,         none         }, // state ST_CERRADA_NO_BLOQUEADA
    {none,                   none,              desbloquear_puerta,    none,               none,                        none,                       none         }, // state ST_CERRADA_BLOQUEADA
    {none,                   none,              none,                  none,               none,                        none,                       cerrar_puerta}, // state ST_ABIERTA_DESDE_AFUERA
    {none,                   none,              none,                  none,               none,                        none,                       cerrar_puerta}  // state ST_ABIERTA_DESDE_ADENTRO

    // EV_INIT_NO_BLOQUEADA, EV_INIT_BLOQUEADA, EV_DESBLOQUEO_POR_APP, EV_BLOQUEO_POR_APP, EV_ANIMAL_DETECTADO_ADENTRO, EV_ANIMAL_DETECTADO_AFUERA, EV_TIMEOUT
};

// Init
void init_no_bloqueada() {
    current_state_puerta = ST_CERRADA_NO_BLOQUEADA;
}
void init_bloqueada() {
    current_state_puerta = ST_CERRADA_BLOQUEADA;
}

// APP
void bloquear_puerta() {
    current_state_puerta = ST_CERRADA_BLOQUEADA;
}
void desbloquear_puerta() {
    current_state_puerta = ST_CERRADA_NO_BLOQUEADA;
}

// Aperturas de la puerta
void abrir_desde_adentro() {
    current_state_puerta = ST_ABIERTA_DESDE_ADENTRO;
    actions_puerta action = ACC_ABRIR_DESDE_ADENTRO;
    if (xQueueSend(queueAcciones_puerta, &action, 0) != pdPASS) {
        Serial.println("[puerta_accion] Cola de acciones LLENA");
    }
    else {
        Serial.print(">> Acción emitida: ACC_ABRIR_DESDE_ADENTRO");
    }
}

void abrir_desde_afuera() {
    current_state_puerta = ST_ABIERTA_DESDE_AFUERA;
    actions_puerta action = ACC_ABRIR_DESDE_AFUERA;
    if (xQueueSend(queueAcciones_puerta, &action, 0) != pdPASS) {
        Serial.println("[puerta_accion] Cola de acciones LLENA");
    }
    else {
        Serial.print(">> Acción emitida: ACC_ABRIR_DESDE_AFUERA");
    }
}

// Cierre de la puerta
void cerrar_puerta() {
    current_state_puerta = ST_CERRADA_NO_BLOQUEADA;
    actions_puerta action = ACC_CERRAR;
    if (xQueueSend(queueAcciones_puerta, &action, 0) != pdPASS) {
        Serial.println("[puerta_accion] Cola de acciones LLENA");
    }
    else {
        Serial.print(">> Acción emitida: ACC_CERRAR");
    }
}


void timer_callback_puerta(TimerHandle_t xTimer) {
    Serial.println("[timer_callback_puerta] Timeout de la puerta");
    events_puerta evento = EV_TIMEOUT;
    if (xQueueSend(queueEventos_puerta, &evento, 0) != pdPASS) {
        Serial.println("[timer_callback_puerta] Cola de eventos LLENA");
    }
}


void setup_puerta() {
    crear_colas_puerta();
    configuracion_sensores_puerta();
    configuracion_estado_inicial_puerta();
    timer_puerta = xTimerCreate("Timer_Puerta", pdMS_TO_TICKS(TIEMPO_TIMEOUT_PUERTA), pdFALSE, NULL, timer_callback_puerta);
    crear_tareas_puerta();
}

void crear_colas_puerta() {
    queueEventos_puerta = xQueueCreate(TAM_EV_COLA_PUERTA, sizeof(events_puerta));
    queueAcciones_puerta = xQueueCreate(TAM_ACC_COLA_PUERTA, sizeof(actions_puerta));
}

void configuracion_sensores_puerta() {
    // Sensor de proximidad
    sensor_proximidad.pin_echo = SENSOR_PROXIMIDAD_ECHO;
    sensor_proximidad.pin_trigger = SENSOR_PROXIMIDAD_TRIGGER;
    sensor_proximidad.estado = ESTADO_HABILITADO;
    sensor_proximidad.distancia_actual_cm = 0;
    sensor_proximidad.tiempo_transcurrido_ms = 0;

    // Sensor RFID
    sensor_rfid.pin_ss = RFID_SS;
    sensor_rfid.pin_reset = RFID_RST;
    sensor_rfid.estado = ESTADO_HABILITADO;
    sensor_rfid.id_tag = 0;

    SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_SS);
    rfid.PCD_Init();
}


void configuracion_estado_inicial_puerta() {
    current_state_puerta = ST_CERRADA_NO_BLOQUEADA; // Agregar arranque!
    servo.write(90); // Posición inicial del servomotor
}


void crear_tareas_puerta() {
    int tam_stack_bytes = 1024 * 8;
    xTaskCreate(puerta_deteccion, "Puerta detección", tam_stack_bytes, NULL, 1, NULL);
    xTaskCreate(puerta_controlador, "Puerta controlador", tam_stack_bytes, NULL, 1, NULL);
    xTaskCreate(puerta_accion, "Puerta accion", tam_stack_bytes, NULL, 1, NULL);
}


void puerta_controlador(void *pvParameters) {
    while(1) {
        events_puerta evento_recibido;
        if (xQueueReceive(queueEventos_puerta, &evento_recibido, 0) == pdPASS) {
            Serial.print("[puerta_controlador] Evento recibido");
            if (evento_recibido < CANT_MAX_EVENTOS_PUERTA) {
                transition transition_function = puerta_state_table[current_state_puerta][evento_recibido];
                transition_function();
            }
            else {
                Serial.println("[puerta_controlador] Evento fuera de rango");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}


void puerta_accion(void *pvParameters) {
    while(1) {
        actions_puerta action_recibido;
        if (xQueueReceive(queueAcciones_puerta, &action_recibido, 0) == pdPASS) {
            Serial.print("[puerta_accion] Accion recibida=");            
            if (action_recibido == ACC_ABRIR_DESDE_AFUERA) {
                Serial.println("ACC_ABRIR_DESDE_AFUERA");
                servo.write(0);
                xTimerStart(timer_puerta, 0);
            }
            else if (action_recibido == ACC_ABRIR_DESDE_ADENTRO) {
                Serial.println("ACC_ABRIR_DESDE_ADENTRO 180 grados ACA");
                servo.write(180);
                xTimerStart(timer_puerta, 0);
            }
            else if (action_recibido == ACC_CERRAR) {
                Serial.println("ACC_CERRAR");
                servo.write(90);
                sensor_proximidad.estado = ESTADO_HABILITADO;
                sensor_rfid.estado = ESTADO_HABILITADO;
            }
            else {
                Serial.println("[puerta_accion] Accion fuera de rango");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}


void puerta_deteccion(void *pvParameters) {
    while(1) {
        leer_sensor_proximidad();

        if (current_state_puerta == ST_CERRADA_NO_BLOQUEADA) {
            if (sensor_proximidad_detectar_animal()) {
                events_puerta evento = EV_ANIMAL_DETECTADO_ADENTRO;
                if (xQueueSend(queueEventos_puerta, &evento, 0) != pdPASS) {
                    Serial.println("[puerta_deteccion] Cola de eventos LLENA");
                }
                sensor_rfid.estado = ESTADO_DESHABILITADO;
            }

            leer_sensor_rfid();
           
            if (sensor_rfid_detectar_animal()) {
                events_puerta evento = EV_ANIMAL_DETECTADO_AFUERA;
                if (xQueueSend(queueEventos_puerta, &evento, 0) != pdPASS) {
                    Serial.println("[puerta_deteccion] Cola de eventos LLENA");
                }
                sensor_proximidad.estado = ESTADO_DESHABILITADO;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}


void leer_sensor_rfid() {
    if (!rfid.PICC_IsNewCardPresent() && !rfid.PICC_ReadCardSerial())
        return;

    Serial.println(F("RFID detectado"));

    // Magia negra del RFID, no tocar
    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
    sensor_rfid.acceso_permitido = true;
}

bool sensor_rfid_detectar_animal() {
    if (sensor_rfid.acceso_permitido &&
        sensor_rfid.estado == ESTADO_HABILITADO &&
        current_state_puerta == ST_CERRADA_NO_BLOQUEADA
    ) {
        Serial.println("[sensor_rfid_detectar_animal] Animal detectado desde afuera");
        sensor_rfid.acceso_permitido = false;
        return true;
    }
    else {
        Serial.println("[sensor_rfid_detectar_animal] Animal no detectado desde afuera");
        return false;
    }
}

void leer_sensor_proximidad() {
    digitalWrite(sensor_proximidad.pin_trigger, LOW);
    delayMicroseconds(2);
    digitalWrite(sensor_proximidad.pin_trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(sensor_proximidad.pin_trigger, LOW);
    // Leer el tiempo de la señal
    sensor_proximidad.tiempo_transcurrido_ms = pulseIn(sensor_proximidad.pin_echo, HIGH, TIME_OUT_SENSOR_PROXIMIDAD);
    float distanciaCm = sensor_proximidad.tiempo_transcurrido_ms * sensor_proximidad.velocidad_sonido / 2;
    sensor_proximidad.distancia_actual_cm = distanciaCm;
}

bool sensor_proximidad_detectar_animal() {
    if (sensor_proximidad.distancia_actual_cm < sensor_proximidad.distancia_minima_cm
        && sensor_proximidad.estado == ESTADO_HABILITADO
        && current_state_puerta == ST_CERRADA_NO_BLOQUEADA // Evitamos enviar eventos useless
    ) {
        Serial.println("[sensor_proximidad_detectar_animal] Animal detectado desde adentro");
        return true;
    }
    else {
        Serial.println("[sensor_proximidad_detectar_animal] Animal no detectado desde adentro");
        return false;
    }
}


