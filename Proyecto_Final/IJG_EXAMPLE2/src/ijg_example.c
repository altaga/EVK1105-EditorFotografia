#include "string.h"
#include "adc.h"}
#include "tc.h"
#include "conf_clock.h"
#include "board.h"
#include "navigation.h"
#include "sdramc.h"
#include "intc.h"
#include "gpio.h"
#include "print_funcs.h"
#include "sd_mmc_spi.h"
#include "fat.h"
#include "file.h"
#include "ctrl_access.h"
#include "usart.h"
#include "et024006dhu.h"
#include "power_clocks_lib.h"
#include "delay.h"
#include "jpeg_decoder.h"
#include "jpegfile.h"
#include "pwm.h"

#define FALSE			false
#define Arriba 			AVR32_PIN_PB22 // Up
#define Abajo 			AVR32_PIN_PB23 // Down
#define Derecha 		AVR32_PIN_PB24 // Right
#define Izquierda 		AVR32_PIN_PB25 // Left
#define Enter			AVR32_PIN_PB26 // Enter
#define LED0_GPIO 		AVR32_PIN_PB27
#define LED1_GPIO 		AVR32_PIN_PB28
#define LED2_GPIO 		AVR32_PIN_PA05
#define LED3_GPIO 		AVR32_PIN_PA06
#define LineTouch			1
#define EICtouchirq			33
#define EICtouchPIN			22
#define EICtouchFunction	1
#define filtro 	        3
#define AVR32_TC_IRQ0                      448

//Pines del modulo touchscreen de la pantalla
#define T_r 63  //62
#define T_t 62
#define T_b 23
#define T_l 21

//Coordenadas de la touchscreen y variables para filtro
int ye1=0;
int X=0,Y=0,x1=0,x2=0,x3=0,xprom=0,x32=0,x21=0,xtot=0,x4=0,x5=0,x54=0,x43=0,y43=0,y2=0,y3=0,y4=0,xytot=0,y32=0,y21=0,ytot=0;
static bool first_ls;
bool busy=false;
unsigned int Filessize[10];
static char str_buff[MAX_FILE_PATH_LENGTH];
static char filenames[4][MAX_FILE_PATH_LENGTH];
const U8 * stream_jpeg_src_ptr;		// JPEG source pointer
U16 stream_src_size;				// JPEG source size
et024006_color_t const *RawImg;
bool BrilloACT=false;
bool NegACT=false;
bool BNACT=false;
bool AutoFiltroACT=false;
bool dilatar=false;
bool reset=false;
bool blur =false;
bool eros=false;
char ADCc=0;
char step=0;
const char LEDS [4]={LED0_GPIO,LED1_GPIO,LED2_GPIO,LED3_GPIO};
int forzada = 0;
bool entrada = false;

pcl_freq_param_t   pcl_freq_param=
{
	.cpu_f  =       32000000//Hz         // Put here the wanted CPU freq
	,  .pba_f    =     32000000//Hz          // Put here the wanted PBA freq
	,  .osc0_f     =   12000000//Hz           // Oscillator 0 frequency
	,  .osc0_startup = OSC0_STARTUP     // Oscillator 0 startup time
};

static const gpio_map_t ADC_GPIO_MAP =
{
	{23,0},
	{21,0}
};

void imgcheck1 (void);
void imgcheck2 (void);
void imgcheck3 (void);
void imgcheck4 (void);
void Pantalla(void);
static void sd_mmc_resources_init(void);
void InicializarFiles (void);
void Writefile(char *Files);
void Readfile(char *Files,unsigned char *Buffer,int Tam);
void get_XY(void);
void filtro_movil(void);
void Brillo (void);
void Negativos (void);
void Luces (void);
void BlancoNegro (void);
void Autofilto (void);
void InicializarFondo(void);
void Desplegar (void);
void MenuFiltros(void);
void Autofilto (void);
void Timersetup (void);
void Dilatar (void);
void Blur (void);
void Erosion (void);

__attribute__((__interrupt__))
static void tc_irq(void)
{
	get_XY();
	filtro_movil();
	imgcheck1();
	imgcheck2();
	imgcheck3();
	imgcheck4();
	// Filtros Menu
	MenuFiltros();
	// Filtros
	Brillo();
	Negativos();
	BlancoNegro();
	Autofilto();
	Dilatar();
	Blur();
	Erosion();
}

__attribute__ ((__interrupt__))

void touch2 (void)
{
	gpio_clr_gpio_pin(LED0_GPIO);
	if(gpio_get_pin_interrupt_flag (Derecha)==1)
	{
		BrilloACT=false;
		NegACT=false;
		BNACT=false;
		AutoFiltroACT=false;
		dilatar=false;
		eros=false;
		blur=false;
		step=0;
		busy=false;
	entrada=true;
	forzada++;
	if (forzada>4)
	{
		forzada=1;
	}
	}
	gpio_clear_pin_interrupt_flag(Derecha);
	if(gpio_get_pin_interrupt_flag (Izquierda)==1)
	{
		entrada=true;
		BrilloACT=false;
		NegACT=false;
		BNACT=false;
		AutoFiltroACT=false;
		dilatar=false;
		eros=false;
		blur=false;
		step=0;
		busy=false;
		forzada--;
		if (forzada<1)
		{
			forzada=4;
		}	
	}
	gpio_clear_pin_interrupt_flag(Izquierda);
	if(gpio_get_pin_interrupt_flag (Enter)==1)
	{
	if (reset==false)
	{
		reset=true;
		adc_configure(&AVR32_ADC);
		Desplegar();
		tc_start(&AVR32_TC,0);
	}
	else 
	{
	InicializarFondo();
	Desplegar();
	BrilloACT=false;
	NegACT=false;
	BNACT=false;
	AutoFiltroACT=false;
	dilatar=false;
	eros=false;
	blur=false;
	step=0;
	busy=false;
	}
	}
	gpio_clear_pin_interrupt_flag(Enter);
	gpio_set_gpio_pin(LED0_GPIO);
}


int main(void)
{
	pcl_configure_clocks(&pcl_freq_param);
	et024006_Init( pcl_freq_param.cpu_f, pcl_freq_param.cpu_f );
	gpio_enable_gpio_pin(LED0_GPIO);
	gpio_enable_gpio_pin(LED1_GPIO);
	gpio_enable_gpio_pin(LED2_GPIO);
	gpio_enable_gpio_pin(LED3_GPIO);
	gpio_set_gpio_pin(LED0_GPIO);
	gpio_set_gpio_pin(LED1_GPIO);
	gpio_set_gpio_pin(LED2_GPIO);
	gpio_set_gpio_pin(LED3_GPIO);
	init_dbg_rs232(pcl_freq_param.pba_f);
	sdramc_init(pcl_freq_param.cpu_f);
	gpio_enable_module(ADC_GPIO_MAP, sizeof(ADC_GPIO_MAP) / sizeof(ADC_GPIO_MAP[0]));
	sd_mmc_resources_init();
	while (!sd_mmc_spi_mem_check());
	InicializarFiles();
	InicializarFondo();
	Pantalla();
	Luces();
	Disable_global_interrupt();
	// Habilitamos las interrupciones del
	INTC_init_interrupts();		
	INTC_register_interrupt(&touch2,71, 1); 
	INTC_register_interrupt(&tc_irq, AVR32_TC_IRQ0, AVR32_INTC_INT0);
	gpio_enable_pin_interrupt(Derecha,	GPIO_FALLING_EDGE);
	gpio_enable_pin_interrupt(Izquierda,GPIO_FALLING_EDGE);
	gpio_enable_pin_interrupt(Enter,	GPIO_FALLING_EDGE);
	Timersetup();
	Enable_global_interrupt();
	while(1);
}

void MenuFiltros(void)
{

	if (X>200 && X<=320 && Y>200 && Y<=240 && xytot<filtro && busy==true && step==1)
	{
		gpio_clr_gpio_pin(LED0_GPIO);
		et024006_PutPixmap(RawImg, 320, 0, 0, 0,0, 320, 240);
		char brillo[]="Brillo";
		char Negativo[]="Negativo";
		char BlNe[]="B/N";
		et024006_DrawFilledRect(20,20,80,20,WHITE);
		et024006_PrintString(brillo,(const unsigned char *)&FONT8x16,20,20,BLACK,WHITE);
		et024006_DrawFilledRect(20,50,80,20,WHITE);
		et024006_PrintString(Negativo,(const unsigned char *)&FONT8x16,20,50,BLACK,WHITE);
		et024006_DrawFilledRect(20,80,80,20,WHITE);
		et024006_PrintString(BlNe,(const unsigned char *)&FONT8x16,20,80,BLACK,WHITE);
		et024006_DrawFilledRect(20,110,80,20,WHITE);
		char AutoFiltro[]="AutoFiltro";
		et024006_PrintString(AutoFiltro,(const unsigned char *)&FONT8x16,20,110,BLACK,WHITE);
		et024006_DrawFilledRect(20,140,80,20,WHITE);
		char BlNes[]="Dilatar";
		et024006_PrintString(BlNes,(const unsigned char *)&FONT8x16,20,140,BLACK,WHITE);
		et024006_DrawFilledRect(20,170,80,20,WHITE);
		char BlNs[]="Blur";
		et024006_PrintString(BlNs,(const unsigned char *)&FONT8x16,20,170,BLACK,WHITE);
		et024006_DrawFilledRect(20,200,80,20,WHITE);
		char Eros[]="Erosion";
		et024006_PrintString(Eros,(const unsigned char *)&FONT8x16,20,200,BLACK,WHITE);
		char Exit[]="SALIR";
		et024006_DrawFilledRect(240,0,80,40,WHITE);
		et024006_PrintString(Exit,(const unsigned char *)&FONT8x16,260,16,BLACK,WHITE);
		step=2;
		delay_ms(250);
		gpio_set_gpio_pin(LED0_GPIO);
	}
	else if(X>200 && X<=320 && Y>0 && Y<=40 && xytot<filtro && busy==true && step==1 && BrilloACT==false && NegACT==false && BNACT == false && AutoFiltroACT==false && blur==false && dilatar==false)
	{
		et024006_PutPixmap(RawImg, 320, 0, 0, 0,0, 320, 240);
		char save[]="Guardando";
		et024006_PrintString(save,(const unsigned char *)&FONT8x16,120,115,BLACK,WHITE);
		busy=false;
		static char Imgnum [11]= {'O','u','t','p','u','t',49,'.','b','m','p'};
		Writefile(Imgnum);
		Imgnum[6]+=1;
		InicializarFondo();
		Desplegar();
	}
	else if(X>240 && X<=320 && Y>40 && Y<=80 && xytot<filtro && busy==true && step==1 && BrilloACT==false && NegACT==false && BNACT == false && blur==false && dilatar==false && AutoFiltroACT==false)
	{
		busy=false;
		InicializarFondo();
		Desplegar();
	}

	else if (X>20 && X<=120 && Y>110 && Y<=130 && xytot<filtro && busy==true && step==2)
	{
		gpio_clr_gpio_pin(LED0_GPIO);
		AutoFiltroACT = true;
		et024006_PutPixmap(RawImg, 320, 0, 0, 0,0, 320, 240);
		char brillo[]="Auto Filtro Activado";
		et024006_PrintString(brillo,(const unsigned char *)&FONT8x16,96,220,BLACK,WHITE);
		char NEG2[]="AutoF";
		et024006_DrawFilledRect(240,0,80,40,WHITE);
		et024006_PrintString(NEG2,(const unsigned char *)&FONT8x16,260,16,BLACK,WHITE);
		step=3;
		delay_ms(250);
		gpio_set_gpio_pin(LED0_GPIO);
	}
		else if (X>20 && X<=120 && Y>170 && Y<=190 && xytot<filtro && busy==true && step==2)
		{
			gpio_clr_gpio_pin(LED0_GPIO);
			blur= true;
			et024006_PutPixmap(RawImg, 320, 0, 0, 0,0, 320, 240);
			char brillo[]="Blur Activado";
			et024006_PrintString(brillo,(const unsigned char *)&FONT8x16,90,220,BLACK,WHITE);
			char NEG2[]="Blur";
			et024006_DrawFilledRect(240,0,80,40,WHITE);
			et024006_PrintString(NEG2,(const unsigned char *)&FONT8x16,250,16,BLACK,WHITE);
			step=3;
			delay_ms(250);
			gpio_set_gpio_pin(LED0_GPIO);
		}
				else if (X>20 && X<=120 && Y>200 && Y<=220 && xytot<filtro && busy==true && step==2)
				{
					gpio_clr_gpio_pin(LED0_GPIO);
					eros= true;
					et024006_PutPixmap(RawImg, 320, 0, 0, 0,0, 320, 240);
					char brillo[]="Erosion Activado";
					et024006_PrintString(brillo,(const unsigned char *)&FONT8x16,90,220,BLACK,WHITE);
					char NEG2[]="Erosion";
					et024006_DrawFilledRect(240,0,80,40,WHITE);
					et024006_PrintString(NEG2,(const unsigned char *)&FONT8x16,250,16,BLACK,WHITE);
					step=3;
					delay_ms(250);
					gpio_set_gpio_pin(LED0_GPIO);
				}
		else if (X>20 && X<=120 && Y>140 && Y<=160 && xytot<filtro && busy==true && step==2)
		{
			gpio_clr_gpio_pin(LED0_GPIO);
			dilatar = true;
			et024006_PutPixmap(RawImg, 320, 0, 0, 0,0, 320, 240);
			char brillo[]="Dilatar Activado";
			et024006_PrintString(brillo,(const unsigned char *)&FONT8x16,90,220,BLACK,WHITE);
			char NEG2[]="Dilatar";
			et024006_DrawFilledRect(240,0,80,40,WHITE);
			et024006_PrintString(NEG2,(const unsigned char *)&FONT8x16,250,16,BLACK,WHITE);
			step=3;
			delay_ms(250);
			gpio_set_gpio_pin(LED0_GPIO);
		}

	else if (X>20 && X<=120 && Y>10 && Y<=40 && xytot<filtro && busy==true && step==2)
	{
		gpio_clr_gpio_pin(LED0_GPIO);
		BrilloACT = true;
		et024006_PutPixmap(RawImg, 320, 0, 0, 0,0, 320, 240);
		char NEG[]="Brillo Activado";
		et024006_PrintString(NEG,(const unsigned char *)&FONT8x16,96,220,BLACK,WHITE);
		step=3;
		delay_ms(200);
		gpio_set_gpio_pin(LED0_GPIO);
	}

	else if (X>20 && X<=120 && Y>50 && Y<=70 && xytot<filtro && busy==true && step==2)
	{
		gpio_clr_gpio_pin(LED0_GPIO);
		NegACT = true;
		et024006_PutPixmap(RawImg, 320, 0, 0, 0,0, 320, 240);
		char NEG[]="Negativo Activado";
		et024006_PrintString(NEG,(const unsigned char *)&FONT8x16,96,220,BLACK,WHITE);
		char NEG2[]="NEGAR";
		et024006_DrawFilledRect(240,0,80,40,WHITE);
		et024006_PrintString(NEG2,(const unsigned char *)&FONT8x16,260,16,BLACK,WHITE);
		step=3;
		delay_ms(200);
		gpio_set_gpio_pin(LED0_GPIO);
	}
	else if (X>20 && X<=120 && Y>80 && Y<=100 && xytot<filtro && busy==true && step==2)
	{
		gpio_clr_gpio_pin(LED0_GPIO);
		BNACT = true;
		et024006_PutPixmap(RawImg, 320, 0, 0, 0,0, 320, 240);
		char BN[]="B/N Activado";
		et024006_PrintString(BN,(const unsigned char *)&FONT8x16,96,220,BLACK,WHITE);
		char BN2[]="B/N";
		et024006_DrawFilledRect(240,0,80,40,WHITE);
		et024006_PrintString(BN2,(const unsigned char *)&FONT8x16,260,16,BLACK,WHITE);
		step=3;
		delay_ms(250);
		gpio_set_gpio_pin(LED0_GPIO);
	}
	else if(X>240 && X<=320 && Y>0 && Y<=40 && xytot<filtro && busy==true && step==2)
	{
		gpio_clr_gpio_pin(LED0_GPIO);
		et024006_PutPixmap(RawImg, 320, 0, 0, 0,0, 320, 240);
		char menuTXT[]="Menu Filtros";
		et024006_DrawFilledRect(200,200,120,40,WHITE);
		et024006_PrintString(menuTXT,(const unsigned char *)&FONT8x16,210,210,BLACK,WHITE);
		char Salir[]="Salir y Salvar";
		et024006_DrawFilledRect(200,0,140,40,WHITE);
		et024006_PrintString(Salir,(const unsigned char *)&FONT8x16,200,16,BLACK,WHITE);
		char Salirr[]="Salir";
		et024006_DrawFilledRect(240,40,80,40,WHITE);
		et024006_PrintString(Salirr,(const unsigned char *)&FONT8x16,260,56,BLACK,WHITE);
		delay_ms(250);
		step=1;
		gpio_set_gpio_pin(LED0_GPIO);
	}
}

void Autofilto (void)
{
	if (X>240 && X<=320 && Y>0 && Y<=40 && xytot<filtro && busy==true && step==3 && AutoFiltroACT==true)
	{
		gpio_clr_gpio_pin(LED0_GPIO);
		int R1,G1,B1,promedio;
		int Value1;
		U16 *puntero;
		puntero = RawImg;
		for (int i=0; i<76800;i++)
		{
			Value1=*puntero&0xFFFF;
			B1=Value1&BLUE;
			G1=(Value1&GREEN)>>5;
			R1=(Value1&RED)>>11;
			promedio=(B1+(G1/2)+R1)/3;
			if(promedio>24)
			{
				B1-=1;
				if (B1<=0)
				{
					B1=0;
				}
				G1-=2;
				if (G1<=0)
				{
					G1=0;
				}
				R1-=1;
				if (R1<=0)
				{
					R1=0;
				}
				G1=G1<<5;
				R1=R1<<11;
				*puntero=B1+G1+R1;
				puntero++;
			}
			else if (promedio<10)
			{
				B1+=1;
				if (B1>=32)
				{
					B1=BLUE;
				}
				G1+=2;
				if (G1>=64)
				{
					G1=63;
				}
				R1+=1;
				if (R1>=32)
				{
					R1=31;
				}
				G1=G1<<5;
				R1=R1<<11;
				*puntero=B1+G1+R1;
				puntero++;
			}
			else
			{
				G1=(G1)<<5;
				R1=(R1)<<11;
				B1=(B1);
				*puntero=B1+G1+R1;
				puntero++;
			}
		}
		et024006_PutPixmap(RawImg, 320, 0, 0, 0,0, 320, 240);
		char NEG[]="Autofiltro Activado";
		et024006_PrintString(NEG,(const unsigned char *)&FONT8x16,96,220,BLACK,WHITE);
		char NEG2[]="AutoF";
		et024006_DrawFilledRect(240,0,80,40,WHITE);
		et024006_PrintString(NEG2,(const unsigned char *)&FONT8x16,260,16,BLACK,WHITE);
		delay_ms(250);
		gpio_set_gpio_pin(LED0_GPIO);
	}
	else if(AutoFiltroACT== true && X>96 && X<=232 && Y>220 && Y<=240 && xytot<filtro && busy==true && step==3)
	{
		AutoFiltroACT=false;
gpio_clr_gpio_pin(LED0_GPIO);
et024006_PutPixmap(RawImg, 320, 0, 0, 0,0, 320, 240);
char brillo[]="Brillo";
char Negativo[]="Negativo";
char BlNe[]="B/N";
char dila[]="Dilatar";
et024006_DrawFilledRect(20,20,80,20,WHITE);
et024006_PrintString(brillo,(const unsigned char *)&FONT8x16,20,20,BLACK,WHITE);
et024006_DrawFilledRect(20,50,80,20,WHITE);
et024006_PrintString(Negativo,(const unsigned char *)&FONT8x16,20,50,BLACK,WHITE);
et024006_DrawFilledRect(20,80,80,20,WHITE);
et024006_PrintString(BlNe,(const unsigned char *)&FONT8x16,20,80,BLACK,WHITE);
et024006_DrawFilledRect(20,110,80,20,WHITE);
char AutoFiltro[]="AutoFiltro";
et024006_PrintString(AutoFiltro,(const unsigned char *)&FONT8x16,20,110,BLACK,WHITE);
et024006_DrawFilledRect(20,140,80,20,WHITE);
et024006_PrintString(dila,(const unsigned char *)&FONT8x16,20,140,BLACK,WHITE);
et024006_DrawFilledRect(20,170,80,20,WHITE);
char BlNs[]="Blur";
et024006_PrintString(BlNs,(const unsigned char *)&FONT8x16,20,170,BLACK,WHITE);
et024006_DrawFilledRect(20,200,80,20,WHITE);
char Eros[]="Erosion";
et024006_PrintString(Eros,(const unsigned char *)&FONT8x16,20,200,BLACK,WHITE);
char Exit[]="SALIR";
et024006_DrawFilledRect(240,0,80,40,WHITE);
et024006_PrintString(Exit,(const unsigned char *)&FONT8x16,260,16,BLACK,WHITE);
step=2;
delay_ms(250);
gpio_set_gpio_pin(LED0_GPIO);
	}
}

void Erosion (void)
{
		if (X>240 && X<=320 && Y>0 && Y<=40 && xytot<filtro && busy==true && step==3 && eros==true)
		{
			gpio_clr_gpio_pin(LED0_GPIO);
			int R1,G1,B1, R2,G2, B2, promedio;
			int Value1, Value2;
			U16 *puntero;
			puntero = RawImg;
			for (int i=0; i<76800;i++)
			{
				Value1=*puntero&0xFFFF;
				B1=Value1&BLUE;
				G1=(Value1&GREEN)>>5;
				R1=(Value1&RED)>>11;
				promedio=(B1+(G1/2)+R1)/3;
				B1=promedio;
				G1=(promedio*2)<<5;
				R1=promedio<<11;
				*puntero=B1+G1+R1;
				puntero++;
			}
			puntero = RawImg;
			for (int i=0; i<76480;i++)
			{
				Value1=*puntero&0xFFFF;
				B1=Value1&BLUE;
				G1=(Value1&GREEN)>>5;
				R1=(Value1&RED)>>11;
				
				if(B1==R1){
					
					if(R1>0b01111){
						if(i>=320){
							Value2 = *(puntero-320);
							B2=Value2&BLUE;
							G2=(Value2&GREEN)>>5;
							R2=(Value2&RED)>>11;
							if(R1>R2){
								
								R2 = R1;
								
								B2 = 0b00000;
								
								R2 = R2<<11;
								G2 = G2<<5;
								
								*(puntero-320) = R2+G2+B2;
								
							}
						}
						if((i%319)!=0){
							
							Value2 = *(puntero+1);
							
							B2=Value2&BLUE;
							G2=(Value2&GREEN)>>5;
							R2=(Value2&RED)>>11;
							
							if(R1>R2){
								
								R2 = R1;
								
								B2 = 0b00000;
								
								R2 = R2<<11;
								G2 = G2<<5;
								
								*(puntero+1) = R2+G2+B2;
								
							}
						}
						if(i<=76480){
							
							Value2 = *(puntero+320);
							
							B2=Value2&BLUE;
							G2=(Value2&GREEN)>>5;
							R2=(Value2&RED)>>11;
							
							if(R1>R2){
								
								R2 = R1;
								
								B2 = 0b00000;
								
								R2 = R2<<11;
								G2 = G2<<5;
								
								*(puntero+320) = R2+G2+B2;
								
							}
							
						}
						if((i!=0)||((i%320)!=0)){
							
							Value2 = *(puntero-1);
							
							B2=Value2&BLUE;
							G2=(Value2&GREEN)>>5;
							R2=(Value2&RED)>>11;
							
							if(R1>R2){
								
								R2 = R1;
								
								B2 = 0b00000;
								
								R2 = R2<<11;
								G2 = G2<<5;
								
								*(puntero-1) = R2+G2+B2;
								
							}
							
						}

					}
					
				}
				
				if(i>320){
					
					Value2 = *(puntero - 321);
					
					B2=Value2&BLUE;
					G2=(Value2&GREEN)>>5;
					R2=(Value2&RED)>>11;
					
					if(B2==0b00000){
						
						B2 = R2;
						
						G2 = R2*2;
						
						R2 = R2<<11;
						G2 = G2<<5;
						
						*(puntero - 321) = R2+G2+B2;
						
					}
					
				}

				puntero++;
			}
			et024006_PutPixmap(RawImg, 320, 0, 0, 0,0, 320, 240);
			char brillo[]="Erosion Activado";
			et024006_PrintString(brillo,(const unsigned char *)&FONT8x16,90,220,BLACK,WHITE);
			char NEG2[]="Erosion";
			et024006_DrawFilledRect(240,0,80,40,WHITE);
			et024006_PrintString(NEG2,(const unsigned char *)&FONT8x16,250,16,BLACK,WHITE);
			delay_ms(250);
			gpio_set_gpio_pin(LED0_GPIO);
		}
	
	else if(eros== true && X>96 && X<=232 && Y>220 && Y<=240 && xytot<filtro && busy==true && step==3)
	{
	eros=false;
	gpio_clr_gpio_pin(LED0_GPIO);
	et024006_PutPixmap(RawImg, 320, 0, 0, 0,0, 320, 240);
	char brillo[]="Brillo";
	char Negativo[]="Negativo";
	char BlNe[]="B/N";
	et024006_DrawFilledRect(20,20,80,20,WHITE);
	et024006_PrintString(brillo,(const unsigned char *)&FONT8x16,20,20,BLACK,WHITE);
	et024006_DrawFilledRect(20,50,80,20,WHITE);
	et024006_PrintString(Negativo,(const unsigned char *)&FONT8x16,20,50,BLACK,WHITE);
	et024006_DrawFilledRect(20,80,80,20,WHITE);
	et024006_PrintString(BlNe,(const unsigned char *)&FONT8x16,20,80,BLACK,WHITE);
	et024006_DrawFilledRect(20,110,80,20,WHITE);
	char AutoFiltro[]="AutoFiltro";
	et024006_PrintString(AutoFiltro,(const unsigned char *)&FONT8x16,20,110,BLACK,WHITE);
	et024006_DrawFilledRect(20,140,80,20,WHITE);
	char BlNes[]="Dilatar";
	et024006_PrintString(BlNes,(const unsigned char *)&FONT8x16,20,140,BLACK,WHITE);
	et024006_DrawFilledRect(20,170,80,20,WHITE);
	char BlNs[]="Blur";
	et024006_PrintString(BlNs,(const unsigned char *)&FONT8x16,20,170,BLACK,WHITE);
	et024006_DrawFilledRect(20,200,80,20,WHITE);
	char Eros[]="Erosion";
	et024006_PrintString(Eros,(const unsigned char *)&FONT8x16,20,200,BLACK,WHITE);
	char Exit[]="SALIR";
	et024006_DrawFilledRect(240,0,80,40,WHITE);
	et024006_PrintString(Exit,(const unsigned char *)&FONT8x16,260,16,BLACK,WHITE);
	step=2;
	delay_ms(250);
	gpio_set_gpio_pin(LED0_GPIO);
	
	}
}

void Blur (void)
{
	//Prueba de blur
	if(blur == true && X>240 && X<=320 && Y>0 && Y<=40 && xytot<filtro && busy==true && step==3)
	{
		gpio_clr_gpio_pin(LED0_GPIO);
	int R1,G1,B1, R2,G2, B2, promedio;
	int Value1, Value2;
	U16 *puntero;
	puntero = RawImg;
	promedio = 1;
	for (int i=0; i<76800;i++)
	{
		Value1=*puntero&0xFFFF;
		B1=Value1&BLUE;
		G1=(Value1&GREEN)>>5;
		R1=(Value1&RED)>>11;
		
		if(i>320){
			Value2 = *(puntero-321);
			B2=Value2&BLUE;
			G2=(Value2&GREEN)>>5;
			R2=(Value2&RED)>>11;
			
			B1+=B2;
			G1+=G2;
			R1+=R2;
			
			promedio++;
			
			Value2 = *(puntero-320);
			B2=Value2&BLUE;
			G2=(Value2&GREEN)>>5;
			R2=(Value2&RED)>>11;
			
			B1+=B2;
			G1+=G2;
			R1+=R2;
			
			promedio++;
			
		}
		if((i%319)!=0){
			
			Value2 = *(puntero+1);
			
			B2=Value2&BLUE;
			G2=(Value2&GREEN)>>5;
			R2=(Value2&RED)>>11;
			
			B1+=B2;
			G1+=G2;
			R1+=R2;
			
			promedio++;
			
			Value2 = *(puntero-319);
			
			B2=Value2&BLUE;
			G2=(Value2&GREEN)>>5;
			R2=(Value2&RED)>>11;
			
			B1+=B2;
			G1+=G2;
			R1+=R2;
			
			promedio++;
			
			
		}
		if(i<=76480){
			
			Value2 = *(puntero+320);
			
			B2=Value2&BLUE;
			G2=(Value2&GREEN)>>5;
			R2=(Value2&RED)>>11;
			
			B1+=B2;
			G1+=G2;
			R1+=R2;
			
			promedio++;
			
			
			
		}
		if((i!=0)||((i%320)!=0)){
			
			Value2 = *(puntero-1);
			
			B2=Value2&BLUE;
			G2=(Value2&GREEN)>>5;
			R2=(Value2&RED)>>11;
			
			B1+=B2;
			G1+=G2;
			R1+=R2;
			
			promedio++;
			
		}
		
		if(((i%319)!=0)&&((i!=0)||((i%320)!=0))&&(i<=76480)){
			
			Value2 = *(puntero+319);
			
			B2=Value2&BLUE;
			G2=(Value2&GREEN)>>5;
			R2=(Value2&RED)>>11;
			
			B1+=B2;
			G1+=G2;
			R1+=R2;
			
			promedio++;
			
			Value2 = *(puntero+321);
			
			B2=Value2&BLUE;
			G2=(Value2&GREEN)>>5;
			R2=(Value2&RED)>>11;
			
			B1+=B2;
			G1+=G2;
			R1+=R2;
			
			promedio++;
			
		}
		
		B1/=promedio;
		G1/=promedio;
		R1/=promedio;
		
		G1=G1<<5;
		R1=R1<<11;
		
		*puntero = R1+G1+B1;

		puntero++;
		promedio = 1;
	}
	et024006_PutPixmap(RawImg, 320, 0, 0, 0,0, 320, 240);
	char brillo[]="Blur Activado";
	et024006_PrintString(brillo,(const unsigned char *)&FONT8x16,90,220,BLACK,WHITE);
	char NEG2[]="Blur";
	et024006_DrawFilledRect(240,0,80,40,WHITE);
	et024006_PrintString(NEG2,(const unsigned char *)&FONT8x16,250,16,BLACK,WHITE);
	delay_ms(250);
	gpio_set_gpio_pin(LED0_GPIO);
	}
		else if(blur== true && X>96 && X<=232 && Y>220 && Y<=240 && xytot<filtro && busy==true && step==3)
		{
			blur=false;
			gpio_clr_gpio_pin(LED0_GPIO);
			et024006_PutPixmap(RawImg, 320, 0, 0, 0,0, 320, 240);
			char brillo[]="Brillo";
			char Negativo[]="Negativo";
			char BlNe[]="B/N";
			char dila[]="Dilatar";
			et024006_DrawFilledRect(20,20,80,20,WHITE);
			et024006_PrintString(brillo,(const unsigned char *)&FONT8x16,20,20,BLACK,WHITE);
			et024006_DrawFilledRect(20,50,80,20,WHITE);
			et024006_PrintString(Negativo,(const unsigned char *)&FONT8x16,20,50,BLACK,WHITE);
			et024006_DrawFilledRect(20,80,80,20,WHITE);
			et024006_PrintString(BlNe,(const unsigned char *)&FONT8x16,20,80,BLACK,WHITE);
			et024006_DrawFilledRect(20,110,80,20,WHITE);
			char AutoFiltro[]="AutoFiltro";
			et024006_PrintString(AutoFiltro,(const unsigned char *)&FONT8x16,20,110,BLACK,WHITE);
			et024006_DrawFilledRect(20,140,80,20,WHITE);
			et024006_PrintString(dila,(const unsigned char *)&FONT8x16,20,140,BLACK,WHITE);
			et024006_DrawFilledRect(20,200,80,20,WHITE);
			char Eros[]="Erosion";
			et024006_PrintString(Eros,(const unsigned char *)&FONT8x16,20,200,BLACK,WHITE);
			et024006_DrawFilledRect(20,170,80,20,WHITE);
			char BlNs[]="Blur";
			et024006_PrintString(BlNs,(const unsigned char *)&FONT8x16,20,170,BLACK,WHITE);
			char Exit[]="SALIR";
			et024006_DrawFilledRect(240,0,80,40,WHITE);
			et024006_PrintString(Exit,(const unsigned char *)&FONT8x16,260,16,BLACK,WHITE);
			step=2;
			delay_ms(250);
			gpio_set_gpio_pin(LED0_GPIO);
		}
	//Prueba de blur
}

void BlancoNegro (void)
{
	if(BNACT == true && X>240 && X<=320 && Y>0 && Y<=40 && xytot<filtro && busy==true && step==3)
	{
		gpio_clr_gpio_pin(LED0_GPIO);
		int R1,G1,B1,promedio;
		int Value1;
		U16 *puntero;
		puntero = RawImg;
		for (int i=0; i<76800;i++)
		{
			Value1=*puntero&0xFFFF;
			B1=Value1&BLUE;
			G1=(Value1&GREEN)>>5;
			R1=(Value1&RED)>>11;
			promedio=(B1+(G1/2)+R1)/3;
			B1=promedio;
			G1=(promedio*2)<<5;
			R1=promedio<<11;
			*puntero=B1+G1+R1;
			puntero++;
		}
		et024006_PutPixmap(RawImg, 320, 0, 0, 0,0, 320, 240);
		char NEG[]="B/N Activado";
		et024006_PrintString(NEG,(const unsigned char *)&FONT8x16,96,220,BLACK,WHITE);
		char NEG2[]="B/N";
		et024006_DrawFilledRect(240,0,80,40,WHITE);
		et024006_PrintString(NEG2,(const unsigned char *)&FONT8x16,260,16,BLACK,WHITE);
		delay_ms(250);
		gpio_set_gpio_pin(LED0_GPIO);
	}
	else if(BNACT== true && X>96 && X<=232 && Y>220 && Y<=240 && xytot<filtro && busy==true && step==3)
	{
		BNACT=false;
gpio_clr_gpio_pin(LED0_GPIO);
et024006_PutPixmap(RawImg, 320, 0, 0, 0,0, 320, 240);
char brillo[]="Brillo";
char Negativo[]="Negativo";
char BlNe[]="B/N";
char dila[]="Dilatar";
et024006_DrawFilledRect(20,20,80,20,WHITE);
et024006_PrintString(brillo,(const unsigned char *)&FONT8x16,20,20,BLACK,WHITE);
et024006_DrawFilledRect(20,50,80,20,WHITE);
et024006_PrintString(Negativo,(const unsigned char *)&FONT8x16,20,50,BLACK,WHITE);
et024006_DrawFilledRect(20,80,80,20,WHITE);
et024006_PrintString(BlNe,(const unsigned char *)&FONT8x16,20,80,BLACK,WHITE);
et024006_DrawFilledRect(20,110,80,20,WHITE);
char AutoFiltro[]="AutoFiltro";
et024006_PrintString(AutoFiltro,(const unsigned char *)&FONT8x16,20,110,BLACK,WHITE);
et024006_DrawFilledRect(20,140,80,20,WHITE);
et024006_PrintString(dila,(const unsigned char *)&FONT8x16,20,140,BLACK,WHITE);
et024006_DrawFilledRect(20,200,80,20,WHITE);
char Eros[]="Erosion";
et024006_PrintString(Eros,(const unsigned char *)&FONT8x16,20,200,BLACK,WHITE);
et024006_DrawFilledRect(20,170,80,20,WHITE);
char BlNs[]="Blur";
et024006_PrintString(BlNs,(const unsigned char *)&FONT8x16,20,170,BLACK,WHITE);
char Exit[]="SALIR";
et024006_DrawFilledRect(240,0,80,40,WHITE);
et024006_PrintString(Exit,(const unsigned char *)&FONT8x16,260,16,BLACK,WHITE);
step=2;
delay_ms(250);
gpio_set_gpio_pin(LED0_GPIO);
	}
}

void Dilatar (void)
{

	if(dilatar == true && X>240 && X<=320 && Y>0 && Y<=40 && xytot<filtro && busy==true && step==3)
	{gpio_clr_gpio_pin(LED0_GPIO);
			U16 *puntero;
			puntero = RawImg+76160;
			for (int i=0; i<640;i++)
			{
				*puntero=0;
				puntero++;
			}
			int R1,G1,B1, R2,G2, B2, promedio;
			int Value1, Value2;
			puntero = RawImg;
			for (int i=0; i<76800;i++)
			{
				Value1=*puntero&0xFFFF;
				B1=Value1&BLUE;
				G1=(Value1&GREEN)>>5;
				R1=(Value1&RED)>>11;
				promedio=(B1+(G1/2)+R1)/3;
				B1=promedio;
				G1=(promedio*2)<<5;
				R1=promedio<<11;
				*puntero=B1+G1+R1;
				puntero++;
			}
			puntero = RawImg;
			for (int i=0; i<76480;i++)
			{
				Value1=*puntero&0xFFFF;
				B1=Value1&BLUE;
				G1=(Value1&GREEN)>>5;
				R1=(Value1&RED)>>11;
				
				if(B1==R1){
					
					if(R1<0b01111){
						if(i>=320){
							Value2 = *(puntero-320);
							B2=Value2&BLUE;
							G2=(Value2&GREEN)>>5;
							R2=(Value2&RED)>>11;
							if(R1<R2){
								
								R2 = R1;
								
								B2 = 0b11111;
								
								R2 = R2<<11;
								G2 = G2<<5;
								
								*(puntero-320) = R2+G2+B2;
								
							}
						}
						if((i%319)!=0){
							
							Value2 = *(puntero+1);
							
							B2=Value2&BLUE;
							G2=(Value2&GREEN)>>5;
							R2=(Value2&RED)>>11;
							
							if(R1<R2){
								
								R2 = R1;
								
								B2 = 0b11111;
								
								R2 = R2<<11;
								G2 = G2<<5;
								
								*(puntero+1) = R2+G2+B2;
								
							}
						}
						if(i<=76480){
							
							Value2 = *(puntero+320);
							
							B2=Value2&BLUE;
							G2=(Value2&GREEN)>>5;
							R2=(Value2&RED)>>11;
							
							if(R1<R2){
								
								R2 = R1;
								
								B2 = 0b11111;
								
								R2 = R2<<11;
								G2 = G2<<5;
								
								*(puntero+320) = R2+G2+B2;
								
							}
							
						}
						if((i!=0)||((i%320)!=0)){
							
							Value2 = *(puntero-1);
							
							B2=Value2&BLUE;
							G2=(Value2&GREEN)>>5;
							R2=(Value2&RED)>>11;
							
							if(R1<R2){
								
								R2 = R1;
								
								B2 = 0b11111;
								
								R2 = R2<<11;
								G2 = G2<<5;
								
								*(puntero-1) = R2+G2+B2;
								
							}
							
						}

					}
					
				}
				
				if(i>320){
					
					Value2 = *(puntero - 321);
					
					B2=Value2&BLUE;
					G2=(Value2&GREEN)>>5;
					R2=(Value2&RED)>>11;
					
					if(B2==0b11111){
						
						B2 = R2;
						
						G2 = R2*2;
						
						R2 = R2<<11;
						G2 = G2<<5;
						
						*(puntero - 321) = R2+G2+B2;
						
					}
					
				}

				puntero++;
			}
			et024006_PutPixmap(RawImg, 320, 0, 0, 0,0, 320, 240);
			et024006_DrawFilledRect(0,238,320,2,BLACK);
			char NEG[]="Dilatar Activado";
			et024006_PrintString(NEG,(const unsigned char *)&FONT8x16,90,220,BLACK,WHITE);
			char NEG2[]="Dilatar";
			et024006_DrawFilledRect(240,0,80,40,WHITE);
			et024006_PrintString(NEG2,(const unsigned char *)&FONT8x16,250,16,BLACK,WHITE);
			delay_ms(250);
			gpio_set_gpio_pin(LED0_GPIO);	
	}
	else if(dilatar== true && X>96 && X<=232 && Y>220 && Y<=240 && xytot<filtro && busy==true && step==3)
	{
	dilatar=false;
gpio_clr_gpio_pin(LED0_GPIO);
et024006_PutPixmap(RawImg, 320, 0, 0, 0,0, 320, 240);
char brillo[]="Brillo";
char Negativo[]="Negativo";
char BlNe[]="B/N";
char dila[]="Dilatar";
et024006_DrawFilledRect(20,20,80,20,WHITE);
et024006_PrintString(brillo,(const unsigned char *)&FONT8x16,20,20,BLACK,WHITE);
et024006_DrawFilledRect(20,50,80,20,WHITE);
et024006_PrintString(Negativo,(const unsigned char *)&FONT8x16,20,50,BLACK,WHITE);
et024006_DrawFilledRect(20,80,80,20,WHITE);
et024006_PrintString(BlNe,(const unsigned char *)&FONT8x16,20,80,BLACK,WHITE);
et024006_DrawFilledRect(20,110,80,20,WHITE);
char AutoFiltro[]="AutoFiltro";
et024006_PrintString(AutoFiltro,(const unsigned char *)&FONT8x16,20,110,BLACK,WHITE);
et024006_DrawFilledRect(20,140,80,20,WHITE);
et024006_PrintString(dila,(const unsigned char *)&FONT8x16,20,140,BLACK,WHITE);
et024006_DrawFilledRect(20,170,80,20,WHITE);
et024006_DrawFilledRect(20,200,80,20,WHITE);
char Eros[]="Erosion";
et024006_PrintString(Eros,(const unsigned char *)&FONT8x16,20,200,BLACK,WHITE);
char BlNs[]="Blur";
et024006_PrintString(BlNs,(const unsigned char *)&FONT8x16,20,170,BLACK,WHITE);
char Exit[]="SALIR";
et024006_DrawFilledRect(240,0,80,40,WHITE);
et024006_PrintString(Exit,(const unsigned char *)&FONT8x16,260,16,BLACK,WHITE);
step=2;
delay_ms(250);
gpio_set_gpio_pin(LED0_GPIO);
	}
	
	
}
void Negativos (void)
{
	if(NegACT == true && X>240 && X<=320 && Y>0 && Y<=40 && xytot<filtro && busy==true && step==3)
	{
		gpio_clr_gpio_pin(LED0_GPIO);
		while(gpio_get_pin_value(Arriba)==1);
		int R1,G1,B1;
		int Value1;
		U16 *puntero;
		puntero = RawImg;
		for (int i=0; i<76800;i++)
		{
			Value1=*puntero&0xFFFF;
			B1=(~Value1)&BLUE;
			G1=((~Value1)&GREEN);
			R1=((~Value1)&RED);
			*puntero=B1+G1+R1;
			puntero++;
		}
		et024006_PutPixmap(RawImg, 320, 0, 0, 0,0, 320, 240);
		char NEG[]="Negativo Activado";
		et024006_PrintString(NEG,(const unsigned char *)&FONT8x16,96,220,BLACK,WHITE);
		char NEG2[]="NEGAR";
		et024006_DrawFilledRect(240,0,80,40,WHITE);
		et024006_PrintString(NEG2,(const unsigned char *)&FONT8x16,260,16,BLACK,WHITE);
		delay_ms(250);
		gpio_set_gpio_pin(LED0_GPIO);
	}
	else if(NegACT== true && X>96 && X<=232 && Y>220 && Y<=240 && xytot<filtro && busy==true && step==3)
	{
		NegACT=false;
gpio_clr_gpio_pin(LED0_GPIO);
et024006_PutPixmap(RawImg, 320, 0, 0, 0,0, 320, 240);
char brillo[]="Brillo";
char Negativo[]="Negativo";
char BlNe[]="B/N";
char dila[]="Dilatar";
et024006_DrawFilledRect(20,20,80,20,WHITE);
et024006_PrintString(brillo,(const unsigned char *)&FONT8x16,20,20,BLACK,WHITE);
et024006_DrawFilledRect(20,50,80,20,WHITE);
et024006_PrintString(Negativo,(const unsigned char *)&FONT8x16,20,50,BLACK,WHITE);
et024006_DrawFilledRect(20,80,80,20,WHITE);
et024006_PrintString(BlNe,(const unsigned char *)&FONT8x16,20,80,BLACK,WHITE);
et024006_DrawFilledRect(20,110,80,20,WHITE);
char AutoFiltro[]="AutoFiltro";
et024006_PrintString(AutoFiltro,(const unsigned char *)&FONT8x16,20,110,BLACK,WHITE);
et024006_DrawFilledRect(20,140,80,20,WHITE);
et024006_PrintString(dila,(const unsigned char *)&FONT8x16,20,140,BLACK,WHITE);
et024006_DrawFilledRect(20,170,80,20,WHITE);
et024006_DrawFilledRect(20,200,80,20,WHITE);
char Eros[]="Erosion";
et024006_PrintString(Eros,(const unsigned char *)&FONT8x16,20,200,BLACK,WHITE);
char BlNs[]="Blur";
et024006_PrintString(BlNs,(const unsigned char *)&FONT8x16,20,170,BLACK,WHITE);
char Exit[]="SALIR";
et024006_DrawFilledRect(240,0,80,40,WHITE);
et024006_PrintString(Exit,(const unsigned char *)&FONT8x16,260,16,BLACK,WHITE);
step=2;
delay_ms(250);
gpio_set_gpio_pin(LED0_GPIO);
	}
}

void Brillo (void)
{
	if(BrilloACT== true && gpio_get_pin_value(Abajo)==1 && busy==true && step==3)
	{
		gpio_clr_gpio_pin(LED0_GPIO);
		while(gpio_get_pin_value(Abajo)==1);
		int R1,G1,B1;
		int Value1;
		U16 *puntero;
		puntero = RawImg;
		for (int i=0; i<76800;i++)
		{
			Value1=*puntero&0xFFFF;
			B1=Value1&BLUE;
			G1=(Value1&GREEN)>>5;
			R1=(Value1&RED)>>11;
			B1-=1;
			if (B1<=0)
			{
				B1=0;
			}
			G1-=2;
			if (G1<=0)
			{
				G1=0;
			}
			R1-=1;
			if (R1<=0)
			{
				R1=0;
			}
			G1=G1<<5;
			R1=R1<<11;
			*puntero=B1+G1+R1;
			puntero++;
		}
		et024006_PutPixmap(RawImg, 320, 0, 0, 0,0, 320, 240);
		char brillo[]="Brillo Activado";
		et024006_PrintString(brillo,(const unsigned char *)&FONT8x16,96,220,BLACK,WHITE);
		gpio_set_gpio_pin(LED0_GPIO);
	}
	else if(BrilloACT== true && gpio_get_pin_value(Arriba)==1 && busy==true && step==3)
	{
		gpio_clr_gpio_pin(LED0_GPIO);
		while(gpio_get_pin_value(Arriba)==1);
		int R1,G1,B1;
		int Value1;
		U16 *puntero;
		puntero = RawImg;
		for (int i=0; i<76800;i++)
		{
			Value1=*puntero&0xFFFF;
			B1=Value1&BLUE;
			G1=(Value1&GREEN)>>5;
			R1=(Value1&RED)>>11;
			B1+=1;
			if (B1>=32)
			{
				B1=BLUE;
			}
			G1+=2;
			if (G1>=64)
			{
				G1=63;
			}
			R1+=1;
			if (R1>=32)
			{
				R1=31;
			}
			G1=G1<<5;
			R1=R1<<11;
			*puntero=B1+G1+R1;
			puntero++;
		}
		et024006_PutPixmap(RawImg, 320, 0, 0, 0,0, 320, 240);
		char brillo[]="Brillo Activado";
		et024006_PrintString(brillo,(const unsigned char *)&FONT8x16,96,220,BLACK,WHITE);
		gpio_set_gpio_pin(LED0_GPIO);
	}
	else if(BrilloACT== true && X>96 && X<=232 && Y>220 && Y<=240 && xytot<filtro && busy==true && step==3)
	{
		BrilloACT=false;
gpio_clr_gpio_pin(LED0_GPIO);
et024006_PutPixmap(RawImg, 320, 0, 0, 0,0, 320, 240);
char brillo[]="Brillo";
char Negativo[]="Negativo";
char BlNe[]="B/N";
char dila[]="Dilatar";
et024006_DrawFilledRect(20,20,80,20,WHITE);
et024006_PrintString(brillo,(const unsigned char *)&FONT8x16,20,20,BLACK,WHITE);
et024006_DrawFilledRect(20,50,80,20,WHITE);
et024006_PrintString(Negativo,(const unsigned char *)&FONT8x16,20,50,BLACK,WHITE);
et024006_DrawFilledRect(20,80,80,20,WHITE);
et024006_PrintString(BlNe,(const unsigned char *)&FONT8x16,20,80,BLACK,WHITE);
et024006_DrawFilledRect(20,110,80,20,WHITE);
char AutoFiltro[]="AutoFiltro";
et024006_PrintString(AutoFiltro,(const unsigned char *)&FONT8x16,20,110,BLACK,WHITE);
et024006_DrawFilledRect(20,140,80,20,WHITE);
et024006_PrintString(dila,(const unsigned char *)&FONT8x16,20,140,BLACK,WHITE);
et024006_DrawFilledRect(20,170,80,20,WHITE);
et024006_DrawFilledRect(20,200,80,20,WHITE);
char Eros[]="Erosion";
et024006_PrintString(Eros,(const unsigned char *)&FONT8x16,20,200,BLACK,WHITE);
char BlNs[]="Blur";
et024006_PrintString(BlNs,(const unsigned char *)&FONT8x16,20,170,BLACK,WHITE);
char Exit[]="SALIR";
et024006_DrawFilledRect(240,0,80,40,WHITE);
et024006_PrintString(Exit,(const unsigned char *)&FONT8x16,260,16,BLACK,WHITE);
step=2;
delay_ms(250);
gpio_set_gpio_pin(LED0_GPIO);
	}
}

void imgcheck1 (void)
{
	if ((X>0 && X<=80 && Y>0 && Y<=60 && xytot<filtro && busy==false)|| (forzada==1 &&entrada==true))
	{
		entrada=false;
		forzada=1;
		busy=true;
		if (!jpeg_lib_init())						// JPEG IJG lib initialization
		{
			print_dbg("\r\n Initialization failed");
			while (1);
		}
		unsigned char Buffer[Filessize[0]];
		Readfile(filenames[0],&Buffer,(sizeof(Buffer)/sizeof(Buffer[0])));
		stream_jpeg_src_ptr = Buffer;
		stream_src_size = sizeof Buffer/sizeof Buffer[0];
		// main decoder
		U16 width;
		U16 height;
		width= 320;
		height= 240;
		RawImg = (et024006_color_t const *)jpeg_lib_decode_ex(0, &width, &height);
		et024006_PutPixmap(RawImg, width, 0, 0, 0,0, width, height);
		jpeg_lib_exit();
		gpio_clr_gpio_pin(LED0_GPIO);
		et024006_PutPixmap(RawImg, 320, 0, 0, 0,0, 320, 240);
		char menuTXT[]="Menu Filtros";
		et024006_DrawFilledRect(200,200,120,40,WHITE);
		et024006_PrintString(menuTXT,(const unsigned char *)&FONT8x16,210,210,BLACK,WHITE);
		char Salir[]="Salir y Salvar";
		et024006_DrawFilledRect(200,0,140,40,WHITE);
		et024006_PrintString(Salir,(const unsigned char *)&FONT8x16,200,16,BLACK,WHITE);
		char Salirr[]="Salir";
		et024006_DrawFilledRect(240,40,80,40,WHITE);
		et024006_PrintString(Salirr,(const unsigned char *)&FONT8x16,260,56,BLACK,WHITE);
		delay_ms(250);
		step=1;
		gpio_set_gpio_pin(LED0_GPIO);
		delay_ms(500);
	}
}
void imgcheck2 (void)
{
	if ((X>0 && X<=80 && Y>60 && Y<=120 && xytot<filtro && busy==false)|| (forzada==2&&entrada==true))
	{
		entrada=false;
		forzada=2;
		busy=true;
		if (!jpeg_lib_init())						// JPEG IJG lib initialization
		{
			print_dbg("\r\n Initialization failed");
			while (1);
		}
		unsigned char Buffer[Filessize[1]];
		Readfile(filenames[1],&Buffer,(sizeof(Buffer)/sizeof(Buffer[0])));
		stream_jpeg_src_ptr = Buffer;
		stream_src_size = sizeof Buffer/sizeof Buffer[0];
		// main decoder
		U16 width;
		U16 height;
		width= 320;
		height= 240;
		RawImg  = (et024006_color_t const *)jpeg_lib_decode_ex(0, &width, &height);
		et024006_PutPixmap(RawImg , width, 0, 0, 0,0, width, height);
		jpeg_lib_exit();
		gpio_clr_gpio_pin(LED0_GPIO);
		et024006_PutPixmap(RawImg, 320, 0, 0, 0,0, 320, 240);
		char menuTXT[]="Menu Filtros";
		et024006_DrawFilledRect(200,200,120,40,WHITE);
		et024006_PrintString(menuTXT,(const unsigned char *)&FONT8x16,210,210,BLACK,WHITE);
		char Salir[]="Salir y Salvar";
		et024006_DrawFilledRect(200,0,140,40,WHITE);
		et024006_PrintString(Salir,(const unsigned char *)&FONT8x16,200,16,BLACK,WHITE);
		char Salirr[]="Salir";
		et024006_DrawFilledRect(240,40,80,40,WHITE);
		et024006_PrintString(Salirr,(const unsigned char *)&FONT8x16,260,56,BLACK,WHITE);
		delay_ms(250);
		step=1;
		gpio_set_gpio_pin(LED0_GPIO);
		delay_ms(500);
	}
}
void imgcheck3 (void)
{
	if ((X>0 && X<=80 && Y>120 && Y<=180 && xytot<filtro && busy==false)|| (forzada==3&&entrada==true))
	{
		entrada=false;
		forzada=3;
		busy=true;
		if (!jpeg_lib_init())						// JPEG IJG lib initialization
		{
			print_dbg("\r\n Initialization failed");
			while (1);
		}
		unsigned char Buffer[Filessize[2]];
		Readfile(filenames[2],&Buffer,(sizeof(Buffer)/sizeof(Buffer[0])));
		stream_jpeg_src_ptr = Buffer;
		stream_src_size = sizeof Buffer/sizeof Buffer[0];
		// main decoder
		U16 width;
		U16 height;
		width= 320;
		height= 240;
		RawImg  = (et024006_color_t const *)jpeg_lib_decode_ex(0, &width, &height);
		et024006_PutPixmap(RawImg , width, 0, 0, 0,0, width, height);
		jpeg_lib_exit();
		gpio_clr_gpio_pin(LED0_GPIO);
		et024006_PutPixmap(RawImg, 320, 0, 0, 0,0, 320, 240);
		char menuTXT[]="Menu Filtros";
		et024006_DrawFilledRect(200,200,120,40,WHITE);
		et024006_PrintString(menuTXT,(const unsigned char *)&FONT8x16,210,210,BLACK,WHITE);
		char Salir[]="Salir y Salvar";
		et024006_DrawFilledRect(200,0,140,40,WHITE);
		et024006_PrintString(Salir,(const unsigned char *)&FONT8x16,200,16,BLACK,WHITE);
		char Salirr[]="Salir";
		et024006_DrawFilledRect(240,40,80,40,WHITE);
		et024006_PrintString(Salirr,(const unsigned char *)&FONT8x16,260,56,BLACK,WHITE);
		delay_ms(250);
		step=1;
		gpio_set_gpio_pin(LED0_GPIO);
		delay_ms(500);
	}
}
void imgcheck4 (void)
{
	if ((X>0 && X<=80 && Y>180 && Y<=240 && xytot<filtro && busy==false)|| (forzada==4&&entrada==true))
	{
		entrada=false;
		forzada=4;
		busy=true;
		if (!jpeg_lib_init())						// JPEG IJG lib initialization
		{
			print_dbg("\r\n Initialization failed");
			while (1);
		}
		unsigned char Buffer[Filessize[3]];
		Readfile(filenames[3],&Buffer,(sizeof(Buffer)/sizeof(Buffer[0])));
		stream_jpeg_src_ptr = Buffer;
		stream_src_size = sizeof Buffer/sizeof Buffer[0];
		// main decoder
		U16 width;
		U16 height;
		width= 320;
		height= 240;
		RawImg  = (et024006_color_t const *)jpeg_lib_decode_ex(0, &width, &height);
		et024006_PutPixmap(RawImg , width, 0, 0, 0,0, width, height);
		jpeg_lib_exit();
		gpio_clr_gpio_pin(LED0_GPIO);
		et024006_PutPixmap(RawImg, 320, 0, 0, 0,0, 320, 240);
		char menuTXT[]="Menu Filtros";
		et024006_DrawFilledRect(200,200,120,40,WHITE);
		et024006_PrintString(menuTXT,(const unsigned char *)&FONT8x16,210,210,BLACK,WHITE);
		char Salir[]="Salir y Salvar";
		et024006_DrawFilledRect(200,0,140,40,WHITE);
		et024006_PrintString(Salir,(const unsigned char *)&FONT8x16,200,16,BLACK,WHITE);
		char Salirr[]="Salir";
		et024006_DrawFilledRect(240,40,80,40,WHITE);
		et024006_PrintString(Salirr,(const unsigned char *)&FONT8x16,260,56,BLACK,WHITE);
		step=1;
		gpio_set_gpio_pin(LED0_GPIO);
		delay_ms(500);
	}
}

void Desplegar (void)
{
	nav_filelist_reset();
	int counter=0;
	if (!jpeg_lib_init())						// JPEG IJG lib initialization
	{
		print_dbg("\r\n Initialization failed");
		while (1);
	}
	while (nav_filelist_set(0, FS_FIND_NEXT))
	{
		if(counter>0 && counter<5)
		{
			nav_file_name((FS_STRING)str_buff, MAX_FILE_PATH_LENGTH, FS_NAME_GET, true);
			nav_file_name((FS_STRING)filenames[counter-1], MAX_FILE_PATH_LENGTH, FS_NAME_GET, true);
			Filessize[counter-1]=nav_file_lgt();
			unsigned char Buffer[Filessize[counter-1]];
			Readfile(str_buff,&Buffer,(sizeof(Buffer)/sizeof(Buffer[0])));
			stream_jpeg_src_ptr = Buffer;
			stream_src_size = sizeof Buffer/sizeof Buffer[0];
			// main decoder
			U16 width;
			U16 height;
			width= 80;
			height= 60;
			RawImg = (et024006_color_t const *)jpeg_lib_decode_ex(0, &width, &height);
			et024006_PutPixmap(RawImg, width, 0, 0, 0,0+height*(counter-1), width, height );
			et024006_PrintString(str_buff,FONT8x16,width,height*(counter-1),BLACK,WHITE);
		}
		counter++;
	}
	jpeg_lib_exit();
}

void Readfile(char *Files,unsigned char *Buffer,int Tam)
{
	nav_setcwd(Files, true, false);
	file_open(FOPEN_MODE_R);
	for (int i=0;i<Tam;i++)
	{
		Buffer[i]=0;
	}
	int k=0;
	while (!file_eof())
	{
		Buffer[k]=file_getc();
		k++;
	}
	// Close the file.
	file_close();

}

void Writefile(char *Files)
{
	nav_setcwd(Files, true, true);
	file_open(FOPEN_MODE_APPEND);
	U16 *puntero;
	puntero = RawImg;
	int Value1;
	uint8_t r;
	uint8_t g;
	uint8_t b;
	const unsigned char Header[]={0x42, 0x4D,0x36,0x84, 0x03,0x00, 0x00, 0x00, 0x00, 0x00, 0x36, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x40, 0x01, 0x00,0x00 ,0xF0 ,0x00 ,0x00 ,0x00, 0x01 ,0x00 ,0x18 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x84 ,0x03 ,0x00 ,0xC4 ,0x0E ,0x00 ,0x00 ,0xC4 ,0x0E ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00};
	for(int k=0;k<76480;k++)
	{
		puntero++;
	}
	for (int k=0;k<(sizeof(Header)/sizeof(Header[0]));k++)
	{
		file_putc(Header[k]);
	}
	int l=0;
	for (int k=0;k<76800;k++)
	{
		Value1=*puntero&0xFFFF;
		r = ((((Value1 >> 11) & 0x1F) * 527) + 23) >> 6;
		g = ((((Value1 >> 5) & 0x3F) * 259) + 33) >> 6;
		b = (((Value1 & 0x1F) * 527) + 23) >> 6;
		file_putc(b);
		file_putc(g);
		file_putc(r);
		puntero++;
		l++;
		if(l==320)
		{
			for(int j=0;j<640;j++)
			{
				puntero--;
			}
			l=0;
		}
	}
	file_close();

}

void Pantalla(void)
{
	avr32_pwm_channel_t pwm_channel6 =
	{
		.cdty = 0,
		.cprd = 100
	};
	
	pwm_opt_t opt =
	{
		.diva = 0,
		.divb = 0,
		.prea = 0,
		.preb = 0
	};
	pwm_init(&opt);
	pwm_channel6.CMR.calg = PWM_MODE_LEFT_ALIGNED;
	pwm_channel6.CMR.cpol = PWM_POLARITY_HIGH; //PWM_POLARITY_LOW;//PWM_POLARITY_HIGH;
	pwm_channel6.CMR.cpd = PWM_UPDATE_DUTY;
	pwm_channel6.CMR.cpre = AVR32_PWM_CMR_CPRE_MCK_DIV_2;
	pwm_channel_init(6, &pwm_channel6);
	pwm_start_channels(AVR32_PWM_ENA_CHID6_MASK);
	while(pwm_channel6.cdty < pwm_channel6.cprd)
	{
		pwm_channel6.cdty++;
		pwm_channel6.cupd = pwm_channel6.cdty;
		pwm_async_update_channel(AVR32_PWM_ENA_CHID6, &pwm_channel6);
		delay_ms(10);
	}
}

static void sd_mmc_resources_init(void)
{
	// GPIO pins used for SD/MMC interface
	static const gpio_map_t SD_MMC_SPI_GPIO_MAP =
	{
		{SD_MMC_SPI_SCK_PIN,  SD_MMC_SPI_SCK_FUNCTION },  // SPI Clock.
		{SD_MMC_SPI_MISO_PIN, SD_MMC_SPI_MISO_FUNCTION},  // MISO.
		{SD_MMC_SPI_MOSI_PIN, SD_MMC_SPI_MOSI_FUNCTION},  // MOSI.
		{SD_MMC_SPI_NPCS_PIN, SD_MMC_SPI_NPCS_FUNCTION}   // Chip Select NPCS.
	};

	// SPI options.
	spi_options_t spiOptions =
	{
		.reg          = 1,
		.baudrate     = pcl_freq_param.pba_f,  // Defined in conf_sd_mmc_spi.h.
		.bits         = 8,          // Defined in conf_sd_mmc_spi.h.
		.spck_delay   = 0,
		.trans_delay  = 0,
		.stay_act     = 1,
		.spi_mode     = 0,
		.modfdis      = 1
	};

	// Assign I/Os to SPI.
	gpio_enable_module(SD_MMC_SPI_GPIO_MAP,
	sizeof(SD_MMC_SPI_GPIO_MAP) / sizeof(SD_MMC_SPI_GPIO_MAP[0]));

	// Initialize as master.
	spi_initMaster(SD_MMC_SPI, &spiOptions);

	// Set SPI selection mode: variable_ps, pcs_decode, delay.
	spi_selectionMode(SD_MMC_SPI, 0, 0, 0);

	// Enable SPI module.
	spi_enable(SD_MMC_SPI);

	// Initialize SD/MMC driver with SPI clock (PBA).
	sd_mmc_spi_init(spiOptions, pcl_freq_param.pba_f);
}

void InicializarFiles (void)
{
	first_ls = true;
	if (nav_drive_get() >= nav_drive_nb() || first_ls)
	{
		first_ls = false;
		// Reset navigators .
		nav_reset();
		// Use the last drive available as default.
		nav_drive_set(nav_drive_nb() - 1);
		// Mount it.
		nav_partition_mount();
	}
	nav_dir_name((FS_STRING)str_buff, MAX_FILE_PATH_LENGTH);
	// Try to sort items by folders
	if (!nav_filelist_first(FS_DIR))
	{
		// Sort items by files
		nav_filelist_first(FS_FILE);
	}
	nav_filelist_reset();
}


void InicializarFondo(void)
{
	if (!jpeg_lib_init())						// JPEG IJG lib initialization
	{
		print_dbg("\r\n Initialization failed");
		while (1);
	}
	unsigned char Buffer[12888];
	Readfile("EVKinst.jpg",&Buffer,(sizeof(Buffer)/sizeof(Buffer[0])));
	stream_jpeg_src_ptr = Buffer;
	stream_src_size = sizeof Buffer/sizeof Buffer[0];
	// main decoder
	U16 width;
	U16 height;
	width= 320;
	height= 240;
	RawImg  = (et024006_color_t const *)jpeg_lib_decode_ex(0, &width, &height);
	et024006_PutPixmap(RawImg , width, 0, 0, 0,0, width, height);
	jpeg_lib_exit();
}

void filtro_movil(void){
	y4=y3;
	y3=y2;
	y2=ye1;
	ye1=Y;
	x4=x3;
	x3=x2;
	x2=x1;
	x1=X;
	if(x4>x3)
	{
		x43=x4-x3;
	}
	else
	{
		x43=x3-x4;
	}
	if(x3>x2)
	{
		x32=x3-x2;
	}
	else
	{
		x32=x2-x3;
	}
	if(x2>x1)
	{
		x21=x2-x1;
	}
	else
	{
		x21=x1-x2;
	}
	if(y4>y3)
	{
		y43=y4-y3;
	}
	else
	{
		y43=y3-y4;
	}
	if(y3>y2)
	{
		y32=y3-y2;
	}
	else
	{
		y32=y2-y3;
	}
	if(y2>ye1)
	{
		y21=y2-ye1;
	}
	else
	{
		y21=ye1-y2;
	}
	ytot=y43+y32+y21;
	xtot=x43+x32+x21;
	xytot=xtot+ytot;
}

void get_XY(void){
	gpio_enable_gpio_pin(T_l);	//Reinicializacion de pines
	gpio_enable_gpio_pin(T_r);
	gpio_set_gpio_pin(T_t);		//Polarizacion de la pantalla
	gpio_clr_gpio_pin(T_b);
	delay_ms(1);
	adc_enable(&AVR32_ADC,0);
	delay_ms(1);
	adc_start(&AVR32_ADC);
	delay_ms(1);
	X=adc_get_value(&AVR32_ADC,0)*.4;	//Lectura y linealizacion
	delay_ms(1);
	adc_disable(&AVR32_ADC,0);
	delay_ms(1);
	gpio_enable_gpio_pin(T_t);	//Reinicializacion de pines
	gpio_enable_gpio_pin(T_b);
	gpio_set_gpio_pin(T_l);		//Polarizacion de la pantalla
	gpio_clr_gpio_pin(T_r);
	adc_enable(&AVR32_ADC,2);
	delay_ms(1);
	adc_start(&AVR32_ADC);
	delay_ms(1);
	Y=adc_get_value(&AVR32_ADC,2)*.3;	//Lectura y linealizacion
	delay_ms(1);
	adc_disable(&AVR32_ADC,2);
	//Eliminacion de Offset
	Y=270-Y;
	X=X-50;
}

void Luces (void)
{
	for (int i=0; i<4;i++)
	{
		gpio_set_gpio_pin(LEDS[i]);
		delay_ms(100);
	}

	for (int i=0; i<4;i++)
	{
		gpio_clr_gpio_pin(LEDS[i]);
		delay_ms(100);
	}
	for (int i=0; i<4;i++)
	{
		gpio_set_gpio_pin(LEDS[i]);
		delay_ms(100);
	}

	for (int i=0; i<4;i++)
	{
		gpio_tgl_gpio_pin(LEDS[i]);
	}
	delay_ms(200);

	for (int i=0; i<4;i++)
	{
		gpio_tgl_gpio_pin(LEDS[i]);
	}
	delay_ms(200);
	for (int i=0; i<4;i++)
	{
		gpio_tgl_gpio_pin(LEDS[i]);
	}
	delay_ms(100);

	for (int i=0; i<4;i++)
	{
		gpio_tgl_gpio_pin(LEDS[i]);
	}
	delay_ms(100);

	for (int i=0; i<4;i++)
	{
		gpio_tgl_gpio_pin(LEDS[i]);
	}
	delay_ms(50);

	for (int i=0; i<4;i++)
	{
		gpio_tgl_gpio_pin(LEDS[i]);
	}
	delay_ms(50);

	for (int i=0; i<4;i++)
	{
		gpio_tgl_gpio_pin(LEDS[i]);
	}
	delay_ms(25);

	for (int i=0; i<4;i++)
	{
		gpio_tgl_gpio_pin(LEDS[i]);
	}
	delay_ms(25);
	gpio_set_gpio_pin(LED0_GPIO);
	gpio_set_gpio_pin(LED1_GPIO);
	gpio_set_gpio_pin(LED2_GPIO);
	gpio_set_gpio_pin(LED3_GPIO);
}

void Timersetup (void)
{
	static const tc_waveform_opt_t WAVEFORM_OPT =
	{
		.channel  = 0,                        	// Channel selection.

		.bswtrg   = TC_EVT_EFFECT_NOOP,                	// Software trigger effect on TIOB.
		.beevt    = TC_EVT_EFFECT_NOOP,                	// External event effect on TIOB.
		.bcpc     = TC_EVT_EFFECT_NOOP,               	// RC compare effect on TIOB.
		.bcpb     = TC_EVT_EFFECT_NOOP,                     // RB compare effect on TIOB.

		.aswtrg   = TC_EVT_EFFECT_NOOP,                     // Software trigger effect on TIOA.
		.aeevt    = TC_EVT_EFFECT_NOOP,                     // External event effect on TIOA.
		.acpc     = TC_EVT_EFFECT_NOOP,                     // RC compare effect on TIOA: toggle.
		.acpa     = TC_EVT_EFFECT_NOOP,                     // RA compare effect on TIOA: toggle (other possibilities are none, set and clear).

		.wavsel   = TC_WAVEFORM_SEL_UP_MODE,     // Waveform selection: Up mode with automatic trigger(reset) on RC compare.
		.enetrg   = false,                                  // External event trigger enable.
		.eevt     = 0,                                      // External event selection.
		.eevtedg  = TC_SEL_NO_EDGE,                         // External event edge selection.
		.cpcdis   = FALSE,                                  // Counter disable when RC compare.
		.cpcstop  = FALSE,                                  // Counter clock stopped with RC compare.

		.burst    = FALSE,                                  // Burst signal selection.
		.clki     = FALSE,                                  // Clock inversion.
		.tcclks   = TC_CLOCK_SOURCE_TC3                     // TC1=32Khz, TC2=FOSC/2, TC3=FOSC/8, TC4=FOSC/32, TC5=FOSC/128.
	};

	static const tc_interrupt_t TC_INTERRUPT =
	{
		.etrgs = 0,
		.ldrbs = 0,
		.ldras = 0,
		.cpcs  = 1,   // Habilitar interrupción por comparación con RC
		.cpbs  = 0,
		.cpas  = 0,
		.lovrs = 0,
		.covfs = 0
	};
	tc_configure_interrupts(&AVR32_TC, 0, &TC_INTERRUPT);
	tc_init_waveform(&AVR32_TC, &WAVEFORM_OPT);
}