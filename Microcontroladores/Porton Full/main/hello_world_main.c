// Porton Automatico (FSM) - ESP-IDF + FreeRTOS Software Timer

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#include "esp_timer.h"
#include "esp_config.h"
#include "driver/gpio.h"

// FreeRTOS tick rate fallback (standard ESP32 config: 100 Hz)
#ifndef CONFIG_FREERTOS_HZ
#define CONFIG_FREERTOS_HZ 100
#endif

//  GPIO 
#define GPIO_FCA        GPIO_NUM_34   // Input: Limit switch ABIERTO (solo input)
#define GPIO_FCC        GPIO_NUM_35   // Input: Limit switch CERRADO (solo input)
#define GPIO_FTC        GPIO_NUM_32   // Input: Fotocelda / obstaculo
#define GPIO_PP         GPIO_NUM_33   // Input: Push-Push (opcional)
#define GPIO_BE         GPIO_NUM_25   // Input: Boton emergencia / reset falla

#define GPIO_MC         GPIO_NUM_26   // Output: Motor cerrar
#define GPIO_MA         GPIO_NUM_27   // Output: Motor abrir
#define GPIO_LAMP       GPIO_NUM_14   // Output: Lampara
#define GPIO_BUZZER     GPIO_NUM_12   // Output: Buzzer

// Niveles activos 
#define IN_ACTIVE_LEVEL   1
#define OUT_ACTIVE_LEVEL  1

// ESTADOS
#define Inicio          0
#define desconocido     1
#define cerrando        2
#define abriendo        3
#define cerrado         4
#define abierto         5
#define Espera_Abierto  6
#define Falla           7

// IO STRUCT
struct IO {
    unsigned int fca;     // Final de carrera abierto (entrada)
    unsigned int fcc;     // Final de carrera cerrado (entrada)
    unsigned int ftc;     // Fotocelda (entrada)
    unsigned int be;      // Emergencia / reset falla (entrada)
    unsigned int pp;      // Push-Push (entrada - opcional)

    unsigned int mc;      // Motor cerrar (salida)
    unsigned int ma;      // Motor abrir (salida)
    unsigned int lamp;    // Lampara (salida)
    unsigned int buzzer;  // Buzzer (salida)
} io;

// TIEMPO
#define MOVE_TIMEOUT_US   (20LL * 1000000LL)   // 20 s
#define WAIT_OPEN_US      (5LL  * 1000000LL)   // 5 s

#define Motor_ON   1
#define Motor_OFF  0
#define Lamp_ON    1
#define Lamp_OFF   0
#define Buzzer_ON  1
#define Buzzer_OFF 0

// VARIABLES FSM
static int pp_prev = 0;
static int flanco_pp = 0;

static int Estado_Actual = Inicio;
static int Estado_Siguiente = Inicio;

static int64_t motor_start_time = 0;
static int64_t wait_start_time  = 0;

// “Falla diferida”: cuando hay timeout, se invierte sentido para ir a un extremo seguro
// y al llegar al extremo se detiene en FALLA.
static int fault_pending = 0;
static int fault_target  = cerrado; // cerrado o abierto

// PROTOTIPOS
static void TimerIO(void);

static void Func_inicio(void);
static void Func_desconocido(void);
static void Func_cerrando(void);
static void Func_abriendo(void);
static void Func_cerrado(void);
static void Func_abierto(void);
static void Func_espera_abierto(void);
static void Func_falla(void);

static void read_inputs(void);
static void write_outputs(void);
static void gpio_init_all(void);

// LECTURA/ESCRITURA GPIO 
static inline int read_in(gpio_num_t pin)
{
    return (gpio_get_level(pin) == IN_ACTIVE_LEVEL) ? 1 : 0;
}

static inline void write_out(gpio_num_t pin, int val)
{
    gpio_set_level(pin, (val ? OUT_ACTIVE_LEVEL : !OUT_ACTIVE_LEVEL));
}

static void read_inputs(void)
{
    io.fca = read_in(GPIO_FCA);
    io.fcc = read_in(GPIO_FCC);
    io.ftc = read_in(GPIO_FTC);
    io.be  = read_in(GPIO_BE);
    io.pp  = read_in(GPIO_PP);
}

static void write_outputs(void)
{
    write_out(GPIO_MC,     io.mc);
    write_out(GPIO_MA,     io.ma);
    write_out(GPIO_LAMP,   io.lamp);
    write_out(GPIO_BUZZER, io.buzzer);
}

static void gpio_init_all(void)
{
    // Inputs
    gpio_config_t in_cfg = {
        .pin_bit_mask = (1ULL<<GPIO_FCA) | (1ULL<<GPIO_FCC) | (1ULL<<GPIO_FTC) |
                        (1ULL<<GPIO_PP)  | (1ULL<<GPIO_BE),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,    
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&in_cfg);

    // Outputs
    gpio_config_t out_cfg = {
        .pin_bit_mask = (1ULL<<GPIO_MC) | (1ULL<<GPIO_MA) | (1ULL<<GPIO_LAMP) | (1ULL<<GPIO_BUZZER),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&out_cfg);

    // Estado seguro
    io.mc = Motor_OFF;
    io.ma = Motor_OFF;
    io.lamp = Lamp_OFF;
    io.buzzer = Buzzer_OFF;
    write_outputs();
}

// FSM CORE 
static void TimerIO(void)
{
    Estado_Siguiente = Estado_Actual;

    flanco_pp = (io.pp == 1 && pp_prev == 0);
    pp_prev = io.pp;

    switch (Estado_Actual)
    {
        case Inicio:          Func_inicio();          break;
        case desconocido:     Func_desconocido();     break;
        case cerrando:        Func_cerrando();        break;
        case abriendo:        Func_abriendo();        break;
        case cerrado:         Func_cerrado();         break;
        case abierto:         Func_abierto();         break;
        case Espera_Abierto:  Func_espera_abierto();  break;
        case Falla:           Func_falla();           break;
        default:              Estado_Siguiente = Falla; break;
    }

    Estado_Actual = Estado_Siguiente;
}

// ESTADOS

static void Func_inicio(void)
{
    io.mc = Motor_OFF;
    io.ma = Motor_OFF;
    io.lamp = Lamp_OFF;
    io.buzzer = Buzzer_OFF;

    motor_start_time = 0;
    wait_start_time  = 0;

    fault_pending = 0;
    fault_target  = cerrado;

    // Calibrar: llevar a CERRADO para conocer referencia
    Estado_Siguiente = desconocido;
}

static void Func_desconocido(void)
{
    // Forzar cierre hasta tocar FCC (o timeout -> FALLA)
    io.mc = Motor_ON;
    io.ma = Motor_OFF;
    io.lamp = Lamp_OFF;
    io.buzzer = Buzzer_ON;

    if (motor_start_time == 0) motor_start_time = esp_timer_get_time();

    if (io.fcc)
    {
        motor_start_time = 0;
        Estado_Siguiente = cerrado;
        return;
    }

    if ((esp_timer_get_time() - motor_start_time) > MOVE_TIMEOUT_US)
    {
        // No pudo calibrar a cerrado -> falla directa
        motor_start_time = 0;
        fault_pending = 0;
        Estado_Siguiente = Falla;
        return;
    }
}

static void Func_cerrado(void)
{
    io.mc = Motor_OFF;
    io.ma = Motor_OFF;
    io.lamp = Lamp_OFF;
    io.buzzer = Buzzer_OFF;

    // Ciclo automático: al estar cerrado pasa a abrir
    // (si quieres dispararlo por pp: cambia esto por if(flanco_pp) )
    Estado_Siguiente = abriendo;
}

static void Func_abriendo(void)
{
    io.mc = Motor_OFF;
    io.ma = Motor_ON;
    io.lamp = Lamp_ON;
    io.buzzer = Buzzer_ON;

    if (motor_start_time == 0) motor_start_time = esp_timer_get_time();

    // Llegó al abierto
    if (io.fca)
    {
        motor_start_time = 0;

        // Si veníamos en “modo rescate por timeout”, al llegar a extremo seguro => FALLA
        if (fault_pending && fault_target == abierto)
        {
            fault_pending = 0;
            Estado_Siguiente = Falla;
        }
        else
        {
            Estado_Siguiente = abierto;
        }
        return;
    }

    // Timeout de apertura: invertir a cerrar y “detener en falla” cuando llegue a FCC
    if ((esp_timer_get_time() - motor_start_time) > MOVE_TIMEOUT_US)
    {
        motor_start_time = 0;
        fault_pending = 1;
        fault_target  = cerrado;      // meta segura
        Estado_Siguiente = cerrando;  // invertir
        return;
    }
}

static void Func_abierto(void)
{
    io.mc = Motor_OFF;
    io.ma = Motor_OFF;
    io.lamp = Lamp_OFF;
    io.buzzer = Buzzer_OFF;

    // Pasar a espera 5 s antes de cerrar
    wait_start_time = 0;
    Estado_Siguiente = Espera_Abierto;
}

static void Func_espera_abierto(void)
{
    io.mc = Motor_OFF;
    io.ma = Motor_OFF;
    io.lamp = Lamp_OFF;
    io.buzzer = Buzzer_OFF;

    if (wait_start_time == 0) wait_start_time = esp_timer_get_time();

    if ((esp_timer_get_time() - wait_start_time) > WAIT_OPEN_US)
    {
        wait_start_time = 0;
        Estado_Siguiente = cerrando;
    }
}

static void Func_cerrando(void)
{
    io.mc = Motor_ON;
    io.ma = Motor_OFF;
    io.lamp = Lamp_OFF;
    io.buzzer = Buzzer_ON;

    if (motor_start_time == 0) motor_start_time = esp_timer_get_time();

    // Llegó al cerrado
    if (io.fcc)
    {
        motor_start_time = 0;

        // Si veníamos en “modo rescate por timeout”, al llegar a extremo seguro => FALLA
        if (fault_pending && fault_target == cerrado)
        {
            fault_pending = 0;
            Estado_Siguiente = Falla;
        }
        else
        {
            Estado_Siguiente = cerrado;
        }
        return;
    }

    // Timeout de cierre: invertir a abrir y “detener en falla” cuando llegue a FCA
    if ((esp_timer_get_time() - motor_start_time) > MOVE_TIMEOUT_US)
    {
        motor_start_time = 0;
        fault_pending = 1;
        fault_target  = abierto;      // meta segura
        Estado_Siguiente = abriendo;  // invertir
        return;
    }
}

static void Func_falla(void)
{
    // Estado de falla: todo apagado, se sale solo con BE
    io.mc = Motor_OFF;
    io.ma = Motor_OFF;
    io.lamp = Lamp_OFF;
    io.buzzer = Buzzer_OFF;

    if (io.be)
    {
        Estado_Siguiente = Inicio;
    }
}

// FREE RTOS SOFTWARE TIMER
static TimerHandle_t fsm_timer = NULL;

static void fsm_timer_cb(TimerHandle_t xTimer)
{
    (void)xTimer;
    read_inputs();
    TimerIO();
    write_outputs();
}

void app_main(void)
{
    gpio_init_all();

    // Tick de la FSM: 10 ms (100 Hz). Ajustable.
    fsm_timer = xTimerCreate(
        "gate_fsm",
        (10 / portTICK_PERIOD_MS),
        pdTRUE,
        NULL,
        fsm_timer_cb
    );

    if (fsm_timer != NULL)
    {
        xTimerStart(fsm_timer, 0);
    }
}
