#include "Peripheral_Setup.h"
#include "pid_reg3.h"

//#include <Peripheral_Setup.h>

/**
 * main.c
 */
//Declara��o de vari�veis
uint32_t count = 0;
uint32_t index= 0;
float va, vb, vc, iLa, iLb, iLc, vDC, Vref;
float plot1[512];
float plot2[512];

float *padc1 =  &va;
//float *padc2 =  &iLa;

//variavel para o sensor de temperatura
unsigned char ERRO_TEMP = 0;         // Desliga o conversor por eleva��o de temperatura

//variavel para ligar/desligar o boost
unsigned char LigDesL_PWM = 0;         // Liga ou desliga o conversor boost

//Variavel para o trip zone
struct {
    unsigned int pre_carga;
    unsigned int geral;
    unsigned int index;
    unsigned int trip_event;
} contadores;

//variaveis para controle PI
const float delta_theta = 6.28318531/600.0;
float theta = 0;
float Ampl = 0.5;
float iLa_ref = 0;
float duty_inv_a = 0;
float iLb_ref = 0;
float duty_inv_b = 0;
float iLc_ref = 0;
float duty_inv_c = 0;

//Compensadores das correntes c.a.
PIDREG3 C_iLa = PIDREG3_DEFAULTS;
PIDREG3 C_iLb = PIDREG3_DEFAULTS;
PIDREG3 C_iLc = PIDREG3_DEFAULTS;

// Function Prototypes
interrupt void isr_cpu_timer0(void);  //fun��o de interrup��o de tempo
__interrupt void isr_adc(void);     //fun�ao de interrup�ao do ADC
__interrupt void isr_epwm_trip(void);  //fun��o do trip zone

//teste entra em fun��o
float teste = 0;

//Sobrecorrente
int sobrecorrente = 0;

//Maquina de Estados//
//Variaveis globais
void (*PonteiroDeFuncao)(); //ponteiro de fun��o da maquina de estados

//prototypes:
void Init(void);       //fun��o que representa o estado inicial da m�quina de estados.
void On(void);   //fun��o que representa o estado ligado.
void Off(void);   //fun��o que representa o estado desligado.
void Error(void);   //fun��o que representa o estado de erro.

//Declara��es da Maquina de Estados
typedef enum
{
    Init,
    On,
    Off,
    Error,
}init_state;

typedef enum
{
    overcurrent,
    overtemperature,
    off_state,
    on_state,
    none,
}events;

////////////Declara��o das fun��es///////////
init_state ini(void)
{
    return Init;
}

init_state goto_ON(void)
{
    EINT;
    ERTM;

  //c�digo da estado


    return On;
}

init_state goto_OFF(void)
{
    DINT;

  //c�digo da estado



    return Off;
}

init_state goto_ERROR(void)
{
    DINT;




    return Error;
}

init_state readevents(void)
{
    if(ia > Isat || ib > Isat || ic < -Isat){
            return overcurrent;
        }
    if(temp ==0){    //GPIO-14 ==0   sensor de temperatura
            return overtemperature;
        }
        if(turn_off_command == 1){
            PieCtrlRegs.PIEIER1.bit.INTx7 = 0;
            return off_state;
        }
        if(turn_on_command == 1){
            PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
            return on_state;
        }
        return none;
    }


int main(void)
    {

    init_state next_state = on;
    events new_event;

    //loop infinito
    while(1){
   events new_event = readevents();

switch(next_state){

    case on:{


            //Fun��es b�sicas definidas pela Texas Instrument

                InitSysCtrl();       //Initialize System Control

                DINT;                //Disable CPU interrupts  (Chave global das interrup��es, desliga tudo)
                InitPieCtrl();        //Initialize the PIE control registers to their default state
                IER = 0x0000;        //Disable CPU interrupts (Chave )
                IFR = 0x0000;        //Clear all CPU interrupt flags
                InitPieVectTable();  //Initialize the PIE vector table  (preenche a matriz de interrup��o)


                Setup_GPIO();          // Fun��o que configura o GPIO
                Setup_ePWM();    //Fun��o que configura o ePWM
                Setup_ADC_A(); //Configura o ADC A
                Setup_ADC_B(); //Configura o ADC B
                Setup_ADC_C(); //Configura o ADC C

                //Fun��es que tratam das interrup��es,
                EALLOW;
                PieVectTable.TIMER0_INT = &isr_cpu_timer0;  //endere�o da fun��o que vai ser executada quando ocorrer a interrup��o do timer
                PieVectTable.ADCA1_INT =  &isr_adc;  //endere�o da fun��o que vai ser executada quando ocorrer a interrup��o do adc
                PieVectTable.EPWM4_TZ_INT = &isr_epwm_trip;  // habilita a fun��o que � chamada quando der o erro, ocorrer o trip
                EDIS;

                //interrup��o dos perif�ricos - pag. 96 PIE channel Mapping - Technical reference
                PieCtrlRegs.PIEIER1.bit.INTx7 = 1; //Timer 0 - habilita a coluna 7 da linha 1 que corresponde a interrup�ao do timer 0
                PieCtrlRegs.PIEIER1.bit.INTx1 = 1;  // ADC A1 (interrup��o do ADC A) (linha 1 da coluna 1)
                PieCtrlRegs.PIEIER2.bit.INTx4 = 1;  //Enable PieVector to TZ4 TRIP1 interrupt (linha 2, coluna 4)
                IER |= (M_INT1|M_INT2); // Enable lines of interrupt ,  habilita a linha 1 e a linha 2 nas interrup��es globais

                //timer
                InitCpuTimers(); //inicializa o timer
                ConfigCpuTimer(&CpuTimer0, 200, 100000); //configura a fun��o timer (timer, freq do dsp, per�odo[micro segundos]) t= 0.1seg
                CpuTimer0Regs.TCR.all = 0x4001; //habilita a interrup��o dentro do timer


                EINT;                 //Enable Global interrupt INTM
                ERTM;                 //Enable Global realtime interrupt DBGM

                next_state = goto_ON();
           }
        break;

    case On:{
        if(overcurrent == new_event){
            next_state = goto_ERROR();
        }

        if(overtemperature == new_event){
                  next_state = goto_ERROR();
        }

        if(off_state == new_event){
            next_state = goto_OFF();
        }
    }
    break;
    case Off:{
        if(on_state == new_event){
            next_state = goto_ON();
        }
    }
    break;

    case Error:{
        if(on_state == new_event){
            next_state = end_init_goto_ON();
        }
    }
    break;
    default:
        break;
    }
}

                //mantem os leds desligados inicialmente
                GpioDataRegs.GPBDAT.bit.GPIO34 = 1;
                GpioDataRegs.GPADAT.bit.GPIO31 = 0;

                //inicializa as variaveis do contador
                  contadores.geral = 0;
                  contadores.index = 0;
                  contadores.pre_carga = 0;
                  contadores.trip_event = 0;

                //inicializa as variavies de tens�o e corrente
                  va = 0;
                  vb = 0;
                  vc = 0;
                  iLa = 0;
                  iLb = 0;
                  iLc = 0;
                  vDC = 0;
                  Vref = 0;

             //inicializa o GPIO 26 que liga e desliga o pwm do boost
              GpioDataRegs.GPADAT.bit.GPIO26 = 0;



              //Ganhos dos controladores PI
              C_iLa.Kp = 1.126*0.5;
              C_iLa.Ki = 2844*0.5/36000.0/C_iLa.Kp;
              C_iLa.Kc = 1;
              C_iLa.OutMax = 0.98;
              C_iLa.OutMin = -0.98;

              C_iLb.Kp = 1.126*0.5;
              C_iLb.Ki = 2844*0.5/36000.0/C_iLb.Kp;
              C_iLb.Kc = 1;
              C_iLb.OutMax = 0.98;
              C_iLb.OutMin = -0.98;

              C_iLc.Kp = 1.126*0.5;
              C_iLc.Ki = 2844*0.5/36000.0/C_iLc.Kp;
              C_iLc.Kc = 1;
              C_iLc.OutMax = 0.98;
              C_iLc.OutMin = -0.98;

//Inicializa a m�quina de estados//
 //aponta para o estado inicial. Nunca esquecer de informar um estado


                //loop infinito
       // while(1){
           //  for(count = 0; count < 0x00FFFFF; count++){

             //    }
             //Faz piscar o led azul ligado na GPIO 31
             // GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1; // ou exclusivo (XOR), alterna o valor entre 0 e 1

             //Faz piscar o led vermelho ligado na GPIO 34  - observar que ele usa o GPB toggle enquanto que o led azul usa o GPA toggle   (pag. 13 pdf overview)
            GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Or exclusivo (XOR), alterna o valor entre 0 e 1

            //verifica a temperatura do conversor boost
            ERRO_TEMP = GpioDataRegs.GPADAT.bit.GPIO14;

           // if(ERRO_TEMP != 0){
            //Liga ou desliga o conversor boost
            LigDesL_PWM = GpioDataRegs.GPADAT.bit.GPIO26;
           // }

            //Maquina de estados//


               // }


               return 0;

}

interrupt void isr_cpu_timer0(void){
GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1; //faz piscar o led azul dentro da interrup��o

PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // serve para resetar o flag da interrup��o da linha
}

__interrupt void isr_adc(void){

 // Over temperature  - Prote��o contra eleva��o de temperatura (l� o GPIO 14 de temp. e desliga o GPIO 26 pwm boost)
 GpioDataRegs.GPADAT.bit.GPIO26 = (!GpioDataRegs.GPADAT.bit.GPIO14) ? 1 : GpioDataRegs.GPADAT.bit.GPIO26;

 //Liga novamente o pwm do conversor boost caso a temperatura tenha reduzido (quando o GPIO 14 volta pra 1)
  if(GpioDataRegs.GPADAT.bit.GPIO14 != 0){
            GpioDataRegs.GPADAT.bit.GPIO26 = 0;
            }

  //Teste sobrecorrente (quando o sobrecorrente vai pra 1)
   if(sobrecorrente != 0){
             //GpioDataRegs.GPADAT.bit.GPIO22 = 0;  //habilita o trip zone

         //habilitar o trip zone via software no gpio 41  n�o precisa usar um gpio externo ligado em jumper no gpio41
       // sets the TZFLG[OST] bit     (deve ter que limpar o flag)

       EALLOW;

       EPwm4Regs.TZFRC.bit.OST = 1;   //pag.1907 - Force a One-Shot Trip Event via Software
       EPwm5Regs.TZFRC.bit.OST = 1;
       EPwm6Regs.TZFRC.bit.OST = 1;


          EDIS;



          }

    while(!AdcbRegs.ADCINTFLG.bit.ADCINT1);     // Wait ADCB finished  quando o ADCINT1 =1 , o pulso de interrup��o ja foi gerado
// Le os 4 valores do ADC A, 2 valores do ADC B e 2 valores do ADC C
    va =  0.001*((int)AdccResultRegs.ADCRESULT0 - 0x7FF); //soc 0 ADC C
    vb =  0.001*((int)AdcbResultRegs.ADCRESULT0 - 0x7FF); //soc 0 ADC B
    vc =  0.001*((int)AdcaResultRegs.ADCRESULT1 - 0x7FF); //soc 1 ADC A
    vDC = 0.001*((int)AdcaResultRegs.ADCRESULT0 - 0x7FF); //soc 0 ADC A
    iLa =  0.001*((int)AdccResultRegs.ADCRESULT1 - 0x7FF); //soc 1 ADC C
    iLb =  0.001*((int)AdcbResultRegs.ADCRESULT1 - 0x7FF); //soc 1 ADC B
    iLc =  0.001*((int)AdcaResultRegs.ADCRESULT2 - 0x7FF); //soc 2 ADC A
    Vref =  0.001*((int)AdcaResultRegs.ADCRESULT3 - 0x7FF); //soc 3 ADC A


// Over voltage - Prote��o de sobreten��o no barramento CC, caso a tens�o fique maior que 30 V desliga o PWM
GpioDataRegs.GPADAT.bit.GPIO26 = (vDC > 30.0) ? 1 : GpioDataRegs.GPADAT.bit.GPIO26;

//para gerar o angulo theta limitado em 2 pi
theta = theta + delta_theta;
if(theta > 6.2831853072)
    theta -= 6.2831853072;
else if(theta < -6.2831853072)
    theta += 6.2831853072;


//verifica se o pwm est� habilitado, sen�o desliga o PI
if(GpioDataRegs.GPADAT.bit.GPIO26){

    //teste se entra na fun��o
     teste = 1;

    C_iLa.reset(&C_iLa); //Ia
    duty_inv_a = 0;

    //utilizar um evento de trip zone para isso

    C_iLb.reset(&C_iLb); //Ib
    duty_inv_b = 0;

    C_iLc.reset(&C_iLc); //Ic
    duty_inv_c = 0;


}else{
    iLa_ref = (__sin(theta)*Ampl);   //corrente de referencia
    C_iLa.Err = (iLa_ref - iLa*0.1) ;  //valor do erro (referencia - corrente realimentada)
    C_iLa.calc(&C_iLa);
    duty_inv_a = C_iLa.Out;    //duty cycle fase A criado

    iLb_ref = (__sin(theta)*Ampl);   //corrente de referencia
    C_iLb.Err = (iLb_ref - iLb*0.1) ;  //valor do erro (referencia - corrente realimentada)
    C_iLb.calc(&C_iLb);
    duty_inv_b = C_iLb.Out;    //duty cycle fase B criado

    iLc_ref = (__sin(theta)*Ampl);   //corrente de referencia
    C_iLc.Err = (iLc_ref - iLc*0.1) ;  //valor do erro (referencia - corrente realimentada)
    C_iLc.calc(&C_iLc);
    duty_inv_c = C_iLc.Out;    //duty cycle fase C criado

    //zera o teste se entra na fun��o
       teste = 0;

}

//novo valor para o duty cyclo do pwm 4
EPwm4Regs.CMPA.bit.CMPA =  (uint16_t)(( duty_inv_a + 1.0) * ((float)(EPwm4Regs.TBPRD>>1)) );
//novo valor para o duty cyclo do pwm 5
EPwm5Regs.CMPA.bit.CMPA =  (uint16_t)(( duty_inv_b + 1.0) * ((float)(EPwm5Regs.TBPRD>>1)) );
//novo valor para o duty cyclo do pwm 6
EPwm6Regs.CMPA.bit.CMPA =  (uint16_t)(( duty_inv_c + 1.0) * ((float)(EPwm6Regs.TBPRD>>1)) );

//Quando retira o 1 somado ao duty cycle o pwm zera, da forma acima (correto) o pwm n�o zera

//novo valor para o duty cyclo do pwm 4
//EPwm4Regs.CMPA.bit.CMPA =  (uint16_t)(( duty_inv_a) * ((float)(EPwm4Regs.TBPRD>>1)) );
//novo valor para o duty cyclo do pwm 5
//EPwm5Regs.CMPA.bit.CMPA =  (uint16_t)(( duty_inv_b) * ((float)(EPwm5Regs.TBPRD>>1)) );
//novo valor para o duty cyclo do pwm 6
//EPwm6Regs.CMPA.bit.CMPA =  (uint16_t)(( duty_inv_c) * ((float)(EPwm6Regs.TBPRD>>1)) );

    plot1[index] = *padc1;//envia os valores para o vetor plot para verifica��o via debug
   // plot2[index] = *padc2;

    index = (index == 512) ? 0 : (index+1); //incrementador

    //limpa os flags dos ADCs
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;                      //clear INT1 flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


__interrupt void isr_epwm_trip(void){

    //dispara o contador quando ocorrer o trip
       //contadores.trip_event = contadores.trip_event + 1;
      contadores.trip_event++;    //dispara o contador quando ocorrer o trip

     // To Re-enable the OST Interrupt, do the following:

      EALLOW;
      //pwm 4
      EPwm4Regs.TZCLR.bit.OST = 1;
      EPwm4Regs.TZCLR.bit.INT = 1;

      //pwm 5
      EPwm5Regs.TZCLR.bit.OST = 1;
      EPwm5Regs.TZCLR.bit.INT = 1;

      //pwm 6
      EPwm6Regs.TZCLR.bit.OST = 1;
      EPwm6Regs.TZCLR.bit.INT = 1;
      EDIS;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;

}

