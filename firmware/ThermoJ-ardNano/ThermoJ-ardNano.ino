/*
   Author: Juan Carlos Aguero Flores, ][af.
   telf. 929 498 433
   email: firwar21@gmail.com
   www.firwar.com
*/
extern "C"
{
#include "src/system.h"
#include "src/types.h"
#include "src/main.h"
#include "src/lcdan/lcdan.h"
#include "src/lcdan/lcdan_aux.h"
#include "src/ikey/ikey.h"
#include "src/ikey/kb.h"
#include "src/num/num_textedit.h"
#include "src/num/num_textedit_hwspecific.h"
#include "src/PID/PID.h"
#include "src/usart/usart.h"
}

//Ktes calibracion de Termocupla Tipo J
//INA129
float thermoJ_seedback = 52e-6;
float gain_amp = 99.998;  //500 ohm medidos (2 resis de 100k en paralelo)
int TMAX = TEMPER_MAX;//250;         //C
int T25 = 25;           //C
float diff_v = thermoJ_seedback * gain_amp * (TMAX - T25); //Volts
float m = (TMAX - T25) / diff_v; //C/Volts
float x_at_25C = 2.556; //volts @25 C
float b = T25 - (m * x_at_25C);// b=-466.548292504312
float x_at_250C = x_at_25C + diff_v;

volatile struct _isr_flag
{
    unsigned f20ms: 1;
    unsigned newpiece: 1;
    unsigned reset: 1;
    unsigned __a: 5;

} isr_flag = {0};
volatile uint8_t PID_out_as_dutycycle = 0; //using in ISR

volatile struct _main_flag main_flag = {0};

struct _eep_param
{
    int Temp_sp;
    int Tminutes_max;

};
struct _eep_param sram_param;
struct _eep_param EEMEM eep_param;


int8_t  temper_actual_get_new(void);
void temper_display(float temper_actual);
void reset_all(void);

uint16_t min_ticks;
uint16_t min_counter;
int8_t timer_counter_enable = 0;
int8_t newpiece_counter;
void timer_display(void);
float temper_actual;//temp actual

int8_t timer_1min(void);
void timer_1min_reset(void);

void pulse_reset(void);
void pulse_newpice(void);

int8_t set_temper_sp_num_textedit(void);
void set_temper_sp(void);
int8_t set_param(void);
void set_min(void);

void process_set_texts(void);
uint16_t PID_access_delay;

#define DEBUG_PROCESS
void setup()
{
    USART_Init ( MYUBRR );//@9600
    lcdan_init();
    //
    key_initialization();
    PinTo1(PORTWxKBCOL_1, PINxKBCOL_1);
    PinTo1(PORTWxKBCOL_2, PINxKBCOL_2);
    PinTo1(PORTWxKBCOL_3, PINxKBCOL_3);
    PinTo1(PORTWxKBCOL_4, PINxKBCOL_4);
    //
    PinTo0(PORTWxTIMER_ACTV, PINxTIMER_ACTV);
    ConfigOutputPin(CONFIGIOxTIMER_ACTV, PINxTIMER_ACTV);

    RELAY1_OFF();
    RELAY2_OFF();
    ConfigOutputPin(CONFIGIOxRELAY1, PINxRELAY1);
    ConfigOutputPin(CONFIGIOxRELAY2, PINxRELAY2);
    //
    main_flag.process_disp_enable = 1;
    eeprom_read_block(&sram_param, &eep_param, sizeof(struct _eep_param));
    //
#define CTC_SET_OCRnA(CTC_FREQ, CTC_PRESCALER) ( (uint8_t)( (F_CPU/ (2.0*CTC_PRESCALER*CTC_FREQ)) -1) )//q la division sea entre decimals
    TCNT1 = 0x0000;//if no zero, -> t= 1/(16e6/1024) -> t*(65535-77) = 4.18s
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS12) | (0 << CS11) | (0 << CS10); //CTC PRES=256
    OCR1A = 1249;//CTC_SET_OCRnA( 25, 64);// 25Hz-> cada 20ms , prees=256 --> 1249
    TIMSK1 |= (1 << OCIE1A); //TIMSK1 = (1<<OCIE1A);
    sei();
    //
    reset_all();
    while (!temper_actual_get_new())
    {;}
    temper_display(temper_actual);

    PID_init();
    PID_set_setpoint(sram_param.Temp_sp);
}

void loop()
{
    static int8_t sm0, sm1;


    if (isr_flag.f20ms)//sync para toda la pasada
    {
        isr_flag.f20ms = 0;
        //
        main_flag.f20ms = 1;
    }
    // if (isr_flag.newpiece)
    // {
    //     isr_flag.newpiece  = 0;
    //     main_flag.newpiece = 1;
    // }
    // if (isr_flag.reset)
    // {
    //     isr_flag.reset  = 0;
    //     main_flag.reset = 1;
    // }
    //----------------------------
    if (main_flag.f20ms)
    {
        if (temper_actual_get_new())
        {
            if (main_flag.process_disp_enable)
            {temper_display(temper_actual);}
        }
        //
        pulse_newpice();
        pulse_reset();
        kb_job();
    }
    //----------------------------
    if ( sm1 == 0)
    {
        if (kb_key_is_ready2read(KB_LYOUT_KEY_MENU))
        {
            //kb_key_was_read(KB_LYOUT_KEY_MENU);
            main_flag.process_disp_enable = 0;
            lcdan_clear();
            set_min();
            set_temper_sp();
            sm1++;
        }
    }
    else if ( sm1 == 1)
    {
        if (set_param())
        {
            sm1 = 0;
            main_flag.process_disp_enable = 1;
            process_set_texts();
        }
    }

    //----------------------------
    if (main_flag.reset)
    {
        main_flag.reset = 0;
        reset_all();
        sm0 = 0;

        #ifdef DEBUG_PROCESS
        usart_print_PSTRstring(PSTR("reset\n"));
        #endif
    }
    if (sm0 == 0)
    {
        /*if (main_flag.newpiece)
        {
            main_flag.newpiece = 0;

            #ifdef DEBUG_PROCESS
            usart_print_PSTRstring(PSTR("newpiece\n"));
            #endif
            PinTo1(PORTWxTIMER_ACTV, PINxTIMER_ACTV);

            newpiece_counter++;

            if (newpiece_counter == 1)
            {
                timer_counter_enable = 1;
            }

            if (newpiece_counter == 2)
            {
                timer_1min_reset();
                if (main_flag.process_disp_enable)
                    {timer_display();}

                newpiece_counter = 0;
            }
        }*/
        if (main_flag.newpiece)
        {
            main_flag.newpiece = 0;

            #ifdef DEBUG_PROCESS
            usart_print_PSTRstring(PSTR("newpiece\n"));
            #endif
            PinTo1(PORTWxTIMER_ACTV, PINxTIMER_ACTV);

            timer_counter_enable = 1;
            timer_1min_reset();
            if (main_flag.process_disp_enable)
            {timer_display();}
        }

        if (timer_counter_enable)//counter begin...
        {
            if (timer_1min())
            {
                if (main_flag.process_disp_enable)
                {timer_display();}

            }
            if (min_counter >= sram_param.Tminutes_max)
            {
                timer_1min_reset();
                if (main_flag.process_disp_enable)
                {
                    timer_display();
                    lcdan_set_cursor_in_row0(0x0D);
                    lcdan_print_PSTRstring(PSTR("ON "));
                }
                //
                #ifdef DEBUG_PROCESS
                usart_print_PSTRstring(PSTR("Temp.Control\n"));
                #endif
                PinTo0(PORTWxTIMER_ACTV, PINxTIMER_ACTV);
                RELAY1_ON();
                main_flag.temp_control = 1;
                //
                PID_out_as_dutycycle = (uint8_t) PID_control(temper_actual);
                //
                sm0++;
            }
        }
    }
    else
    {
        if (main_flag.f20ms )
        {
            //if (++PID_access_delay >= 100)  //2s
            if (++PID_access_delay >= 500)  //10000ms = 10s
            {
                PID_access_delay = 0;
                PID_out_as_dutycycle = (uint8_t) PID_control(temper_actual);
                //PID_out_as_dutycycle = 95;
                //
            }
        }
    }

    //-------------------
    main_flag.f20ms = 0;
    kb_flush(); //no debe llamarse 2 veces seguidas a un handler de keyboard
    //sino,, la tecla q todavia no se ha limpiado, puede "ingresar"al sgte. codigo
}

ISR(TIMER1_COMPA_vect)//cada 20ms
{
    isr_flag.f20ms = 1;
    if (main_flag.temp_control)
        {PID_control_output(PID_out_as_dutycycle);}
}

int8_t  temper_actual_get_new(void)
{
    static uint32_t analog_acc;
    static uint16_t analog_samples;
    float analog_mean;
    float volt;
    int8_t cod_ret = 0;

#define ANALOG_SAMPLES 50
    analog_acc += analogRead(A6);
    if (++analog_samples >= ANALOG_SAMPLES)
    {
        analog_samples = 0x00;
        analog_mean =  (analog_acc / ANALOG_SAMPLES);
        analog_acc = 0x00;
        //
        volt = (analog_mean * 5.05) / 1023;
        temper_actual = (m * volt) + b;
        //
        cod_ret = 1;
    }
    return cod_ret;
}
int8_t timer_1min(void)
{
    if (main_flag.f20ms )
    {
        if (++min_ticks >= (50 * 60) ) //1 minuto
        {
            min_ticks = 0x0000;
            if (++min_counter > TMINUTES_MAX)
            {min_counter = 0;}
            return 1;
        }
    }
    return 0;
}

void timer_1min_reset(void)
{
    min_ticks = 0x0000;
    min_counter = 0x0000;
}
void temper_format_print(int16_t temper, char *str_out)
{
    char buff[10];
    itoa(temper, buff, 10); // convierte
    // 3 positions to display: 999
    strcpy(str_out, "   ");
    if (temper < 10)
    {
        strncpy(&str_out[2], buff, 1);
    }
    else if (temper < 100)
    {
        strncpy(&str_out[1], buff, 2);
    }
    else
    {
        strncpy(&str_out[0], buff, 3);
    }
}
void temper_display(float temper)
{
    char str[10];
    temper_format_print(temper, str);
    lcdan_set_cursor_in_row1(0x03);
    lcdan_print_string(str);
}

//T=999:12s Tc=OFF
void process_set_texts(void)
{
    if (main_flag.process_disp_enable)
    {
        lcdan_clear();
        lcdan_set_cursor_in_row0(0x00);
        lcdan_print_PSTRstring(PSTR("t=   m Tctrl=OFF"));

        timer_display();

        //
        lcdan_set_cursor_in_row0(0x0D);
        if (main_flag.temp_control)
            lcdan_print_PSTRstring(PSTR("ON "));
        else
            lcdan_print_PSTRstring(PSTR("OFF"));
        //
        timer_1min_reset();
        timer_display();
        lcdan_set_cursor_in_row1(0x00);
        lcdan_print_PSTRstring(PSTR("Ta=   C Tsp=   C"));


        temper_display(temper_actual);

        char str[10];
        temper_format_print(sram_param.Temp_sp, str);
        lcdan_set_cursor_in_row1(0x0C);
        lcdan_print_string(str);
    }

}
void reset_all(void)
{
    newpiece_counter = 0;
    timer_counter_enable = 0;
    main_flag.temp_control = 0;
    //
    PID_out_as_dutycycle = 0;
    PID_access_delay = 0;
    //
    process_set_texts();

    RELAY1_OFF();
    RELAY2_OFF();
    PinTo0(PORTWxTIMER_ACTV, PINxTIMER_ACTV);
}

void minutes_format_print(int16_t min, char *str_out)
{
    char buff[10];
    itoa(min, buff, 10); // convierte
    // 3 positions to display: 999
    strcpy(str_out, "   ");
    if (min < 10)
    {
        strncpy(&str_out[2], buff, 1);
    }
    else if (min < 100)
    {
        strncpy(&str_out[1], buff, 2);
    }
    else
    {
        strncpy(&str_out[0], buff, 3);
    }
}
void timer_display(void)
{
    char str[10];
    minutes_format_print(min_counter, str);
    lcdan_set_cursor_in_row0(0x02);
    lcdan_print_string(str);
}
void pulse_newpice(void)
{
    static int8_t sm0;
    int8_t p = ReadPin(PORTRxNEWPIECE, PINxNEWPIECE);

    if (sm0 == 0)
    {
        if (p == 1)
            sm0++;
    }
    else if (sm0 == 1)
    {
        if (p != 1)
            sm0--;
        else
        {
            main_flag.newpiece = 1;
            sm0++;
        }
    }
    else if (sm0 == 2)
    {
        if (p == 0)
            sm0++;
    }
    else if (sm0 == 3)
    {
        if (p != 0)
            sm0--;
        else
            sm0 = 0x00;
    }
}
void pulse_reset(void)
{
    static int8_t sm0;
    int8_t p = ReadPin(PORTRxRESET, PINxRESET);

    if (sm0 == 0)
    {
        if (p == 1)
            sm0++;
    }
    else if (sm0 == 1)
    {
        if (p != 1)
            sm0--;
        else
        {
            main_flag.reset = 1;
            sm0++;
        }
    }
    else if (sm0 == 2)
    {
        if (p == 0)
            sm0++;
    }
    else if (sm0 == 3)
    {
        if (p != 0)
            sm0--;
        else
            sm0 = 0x00;
    }
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void set_temper_sp(void)
{
    int8_t idx_ascursor;

    lcdan_set_cursor_in_row0(0x00);
    lcdan_print_PSTRstring(PSTR("Temp[sp]=    C  "));
    //

    num_textedit_disp_set_x(0x09);
    num_textedit_disp_set_y(0);
    num_textedit_valuelimits_set_max(TEMPER_MAX);
    num_textedit_valuelimits_set_min(TEMPER_MIN);//por ahora num_textedit manejo enteros positivos
    num_textedit_set_num_digits(3);
    //
    idx_ascursor = num_textedit_set_value_as_char(sram_param.Temp_sp);
    num_textedit_hwspecific_print_disp(idx_ascursor);
}

void set_min(void)
{
    int8_t idx_ascursor;

    lcdan_set_cursor_in_row1(0x00);
    lcdan_print_PSTRstring(PSTR("Tiempo=    min  "));
    //

    num_textedit_disp_set_x(0x07);
    num_textedit_disp_set_y(1);
    num_textedit_valuelimits_set_max(TMINUTES_MAX);
    num_textedit_valuelimits_set_min(TMINUTES_MIN);//por ahora num_textedit manejo enteros positivos
    num_textedit_set_num_digits(3);
    //
    idx_ascursor = num_textedit_set_value_as_char(sram_param.Tminutes_max);
    num_textedit_hwspecific_print_disp(idx_ascursor);
}

int8_t set_param(void)
{
    static int8_t sm0, c;
    int8_t idx_ascursor;
    int8_t cod_ret = 0;

    struct _kb_num_textedit_actionhandler actionhandler;
    kb_num_textedit_actionhandler(&actionhandler);//recoge la accion especifica

    if (actionhandler.char_dig == 'E')//Enter
    {

        if (c == 0)
        {
            //graba directamente el valor de temperatura
            sram_param.Temp_sp = num_textedit_get_value_as_int();

            //cambia de layout
            num_textedit_disp_set_x(0x07);
            num_textedit_disp_set_y(1);
            num_textedit_valuelimits_set_max(TMINUTES_MAX);
            num_textedit_valuelimits_set_min(TMINUTES_MIN);//por ahora num_textedit manejo enteros positivos
            num_textedit_set_num_digits(3);
            //
            idx_ascursor = num_textedit_set_value_as_char(sram_param.Tminutes_max);
            num_textedit_hwspecific_print_disp(idx_ascursor);
        }
        else
        {
            //graba directamente el valor de minutos
            sram_param.Tminutes_max = num_textedit_get_value_as_int();
            //cambia de layout
            num_textedit_disp_set_x(0x09);
            num_textedit_disp_set_y(0);
            num_textedit_valuelimits_set_max(TEMPER_MAX);
            num_textedit_valuelimits_set_min(TEMPER_MIN);//por ahora num_textedit manejo enteros positivos
            num_textedit_set_num_digits(3);
            //
            idx_ascursor = num_textedit_set_value_as_char(sram_param.Temp_sp);
            num_textedit_hwspecific_print_disp(idx_ascursor);
        }

        if (++c > 1)
        {
            c = 0;
        }
        //cod_ret = 1;
    }
    if (kb_key_is_ready2read(KB_LYOUT_KEY_2ND))//cancela la edicion
    {
        //kb_key_was_read(KB_LYOUT_KEY_2ND);
        //
        lcdan_write_cmd(LCDAN_DISP_ON_CURSOR_OFF_BLINK_OFF);

        if (c == 0)
            sram_param.Temp_sp = num_textedit_get_value_as_int();
        else
            sram_param.Tminutes_max = num_textedit_get_value_as_int();

        c = 0x00;//bug fix!

        eeprom_update_block(&sram_param, &eep_param, sizeof(struct _eep_param));

        PID_set_setpoint(sram_param.Temp_sp);

        cod_ret = 1;
    }
    return cod_ret;
}





