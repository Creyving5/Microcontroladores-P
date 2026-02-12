// Portón Automático (FSM) Maquina de Estados

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"
#include "driver/gpio.h"

// Enumeración de estados (mismos valores) 
#define ST_INICIO        0
#define ST_DESCONOCIDO   1
#define ST_CERRANDO      2
#define ST_ABRIENDO      3
#define ST_CERRADO       4
#define ST_ABIERTO       5
#define ST_STOP_OBJ      6
#define ST_STOP_USER     7
#define ST_ERROR         8

// Bloque de Entradas/Salidas 
typedef struct
{
    unsigned int fca;     // FC abierto (sensor de límite al abrir)
    unsigned int fcc;     // FC cerrado (sensor de límite al cerrar)
    unsigned int ftc;     // Fotocelda / obstáculo
    unsigned int bc;      // Botón remoto: cerrar
    unsigned int ba;      // Botón remoto: abrir
    unsigned int bs;      // Botón remoto: stop
    unsigned int be;      // Botón remoto: emergencia / reset de fallo
    unsigned int pp;      // Push-push (toggle)
    unsigned int mc;      // Salida motor cerrar
    unsigned int ma;      // Salida motor abrir
    unsigned int lamp;    // Salida lámpara
    unsigned int buzzer;  // Salida buzzer
} IO_t;

static IO_t io;

// Constantes del sistema (idénticas en valor) 
#define FCT_CONST   7
#define RUN_TIME_US 180000000

#define MOTOR_ON    1
#define MOTOR_OFF   0
#define LAMP_ON     1
#define LAMP_OFF    0
#define BUZ_ON      1
#define BUZ_OFF     0

// Variables de control de la FSM 
static int pp_prev = 0;                 // valor anterior del botón pp
static int pp_rise = 0;                 // flanco ascendente detectado
static int estado_actual = ST_INICIO;
static int estado_siguiente = ST_INICIO;
static int estado_anterior = ST_STOP_USER;

static int64_t t_motor_ini = 0;         // marca de tiempo para timeout

// Prototipos de funciones de estado 
static void st_inicio(void);
static void st_desconocido(void);
static void st_cerrando(void);
static void st_abriendo(void);
static void st_cerrado(void);
static void st_abierto(void);
static void st_stop_obj(void);
static void st_stop_user(void);
static void st_error(void);

// “Tick” de la FSM: calcula flanco y ejecuta estado 
void TimerIO(void)
{
    estado_siguiente = estado_actual;

    // Detecta transición 0->1 del pulsador pp 
    pp_rise = (io.pp == 1 && pp_prev == 0);
    pp_prev = io.pp;

    switch (estado_actual)
    {
        case ST_INICIO:       st_inicio();       break;
        case ST_DESCONOCIDO:  st_desconocido();  break;
        case ST_CERRANDO:     st_cerrando();     break;
        case ST_ABRIENDO:     st_abriendo();     break;
        case ST_CERRADO:      st_cerrado();      break;
        case ST_ABIERTO:      st_abierto();      break;
        case ST_STOP_OBJ:     st_stop_obj();     break;
        case ST_STOP_USER:    st_stop_user();    break;
        case ST_ERROR:        st_error();        break;
        default:              estado_siguiente = ST_ERROR; break;
    }

    estado_anterior = estado_actual;
    estado_actual   = estado_siguiente;
}

// Estados 

// Estado: Inicio
 // Pone todas las salidas en reposo y pasa a “desconocido”.
 
static void st_inicio(void)
{
    io.mc     = MOTOR_OFF;
    io.ma     = MOTOR_OFF;
    io.lamp   = LAMP_OFF;
    io.buzzer = BUZ_OFF;

    t_motor_ini = 0;
    estado_siguiente = ST_DESCONOCIDO;
}

// Estado: Desconocido
 //Intenta cerrar hasta encontrar el final de carrera de cerrado; si excede el tiempo -> Error.

static void st_desconocido(void)
{
    io.mc     = MOTOR_ON;
    io.ma     = MOTOR_OFF;
    io.lamp   = LAMP_OFF;
    io.buzzer = BUZ_ON;

    if (t_motor_ini == 0)
    {
        t_motor_ini = esp_timer_get_time();
    }
    else if (io.fcc)
    {
        estado_siguiente = ST_CERRADO;
        t_motor_ini = 0;
    }
    else if ((esp_timer_get_time() - t_motor_ini) > RUN_TIME_US)
    {
        estado_siguiente = ST_ERROR;
    }
}

// Estado: Cerrando
// Cierra con monitoreo de: fin de carrera, fotocelda, stop por usuario y timeout.
 
static void st_cerrando(void)
{
    io.mc     = MOTOR_ON;
    io.ma     = MOTOR_OFF;
    io.lamp   = LAMP_OFF;
    io.buzzer = BUZ_ON;

    if (t_motor_ini == 0)
    {
        t_motor_ini = esp_timer_get_time();
    }
    else if (io.fcc)
    {
        estado_siguiente = ST_CERRADO;
        t_motor_ini = 0;
    }
    else if (io.ftc)
    {
        estado_siguiente = ST_STOP_OBJ;
        t_motor_ini = 0;
    }
    else if (pp_rise)
    {
        estado_siguiente = ST_STOP_USER;
        t_motor_ini = 0;
    }
    else if ((esp_timer_get_time() - t_motor_ini) > RUN_TIME_US)
    {
        estado_siguiente = ST_ERROR;
    }
}

// Estado: Abriendo
 // Abre con monitoreo de: fin de carrera, stop por usuario y timeout.
 
static void st_abriendo(void)
{
    io.mc     = MOTOR_OFF;
    io.ma     = MOTOR_ON;
    io.lamp   = LAMP_ON;
    io.buzzer = BUZ_ON;

    if (t_motor_ini == 0)
    {
        t_motor_ini = esp_timer_get_time();
    }
    else if (io.fca)
    {
        estado_siguiente = ST_ABIERTO;
        t_motor_ini = 0;
    }
    else if (pp_rise)
    {
        estado_siguiente = ST_STOP_USER;
        t_motor_ini = 0;
    }
    else if ((esp_timer_get_time() - t_motor_ini) > RUN_TIME_US)
    {
        estado_siguiente = ST_ERROR;
    }
}

// Estado: Cerrado
 //Motor apagado; con pp abre.
 
static void st_cerrado(void)
{
    io.mc     = MOTOR_OFF;
    io.ma     = MOTOR_OFF;
    io.lamp   = LAMP_OFF;
    io.buzzer = BUZ_OFF;

    if (pp_rise)
    {
        estado_siguiente = ST_ABRIENDO;
    }
}

// Estado: Abierto
 // Motor apagado; con pp cierra.
 
static void st_abierto(void)
{
    io.mc     = MOTOR_OFF;
    io.ma     = MOTOR_OFF;
    io.lamp   = LAMP_OFF;
    io.buzzer = BUZ_OFF;

    if (pp_rise)
    {
        estado_siguiente = ST_CERRANDO;
    }
}

// Estado: Parado por Objeto
 // Paro total por obstáculo; con pp vuelve a abrir.
 
static void st_stop_obj(void)
{
    io.mc     = MOTOR_OFF;
    io.ma     = MOTOR_OFF;
    io.lamp   = LAMP_OFF;
    io.buzzer = BUZ_OFF;

    if (pp_rise)
    {
        estado_siguiente = ST_ABRIENDO;
    }
}

// Estado: Parado por Usuario
// Paro total; con pp invierte según el estado previo (si venía cerrando -> abre; si no -> cierra).
 
static void st_stop_user(void)
{
    io.mc     = MOTOR_OFF;
    io.ma     = MOTOR_OFF;
    io.lamp   = LAMP_OFF;
    io.buzzer = BUZ_OFF;

    if (pp_rise)
    {
        if (estado_anterior == ST_CERRANDO)
            estado_siguiente = ST_ABRIENDO;
        else
            estado_siguiente = ST_CERRANDO;
    }
}

// Estado: Error
 // Apaga todo y espera el botón de emergencia/reset para reiniciar.
 
static void st_error(void)
{
    io.mc     = MOTOR_OFF;
    io.ma     = MOTOR_OFF;
    io.lamp   = LAMP_OFF;
    io.buzzer = BUZ_OFF;

    if (io.be)
    {
        estado_siguiente = ST_INICIO;
    }
}

void app_main(void)
{
    
}
