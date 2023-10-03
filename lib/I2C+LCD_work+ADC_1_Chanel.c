#include <stm32f1xx.h>
#include <stdbool.h>
 double Vdd=0;
  double d0;
  char str1[10];
//---------------------------------------------------------------------
//---------------------------------------------------------------------
#define e_set() LCD_WriteByteI2CLCD(portlcd|=0x04)  ////установка линии Е в 1
#define e_reset() LCD_WriteByteI2CLCD(portlcd&=~0x04) //установка линии Е в 0
#define rs_set() LCD_WriteByteI2CLCD(portlcd|=0x01) //установка линии RS в 1
#define rs_reset() LCD_WriteByteI2CLCD(portlcd&=~0x01) //установка линии RS в 0
#define setled() LCD_WriteByteI2CLCD(portlcd|=0x08) //установка линии BL в 1
#define setwrite() LCD_WriteByteI2CLCD(portlcd&=~0x02) //установка линии RW в 0
#define setread() LCD_WriteByteI2CLCD(portlcd|=0x02) //установка линии RW в 1
//---------------------------------------------------------------------


#define I2C_REQUEST_WRITE                       0x00
#define I2C_REQUEST_READ                        0x01
uint8_t addr=0x3F; //3F


void i2c_init()
{
RCC->APB1ENR|=1<<22;
RCC->APB2ENR|=(1<<3);
GPIOB->CRH|=0x0000FF00;
I2C2->CR2|=36;
I2C2->CCR|=180;
I2C2->TRISE=37;
I2C2->CR1|=(1<<10);
I2C2->CR1 |= I2C_CR1_PE;
}

bool CMSIS_I2C_Adress_Device_Scan(uint8_t data ) 
{
  CLEAR_BIT(I2C2->CR1, I2C_CR1_POS); //Бит ACK управляет (N)ACK текущего байта, принимаемого в сдвиговом регистре.
  SET_BIT(I2C2->CR1, I2C_CR1_START); //Отправляем сигнал START
  while (READ_BIT(I2C2->SR1, I2C_SR1_SB) == 0) {
       }
    //Ожидаем до момента, пока не сработает Start condition generated
        //ВНИМАНИЕ!
  /* Бит I2C_SR1_SB очищается программно путем чтения регистра SR1 с последующей записью данных в регистр DR или когда PE=0*/
  I2C2->SR1;
  I2C2->DR = (addr << 1); //Адрес + Write 
  while ((READ_BIT(I2C2->SR1, I2C_SR1_AF) == 0) && (READ_BIT(I2C2->SR1, I2C_SR1_ADDR) == 0)) {
    }   //Ждем, пока адрес отзовется
  if (READ_BIT(I2C2->SR1, I2C_SR1_ADDR)) {
    }//Если устройство отозвалось
  //  SET_BIT(I2C->CR1, I2C_CR1_STOP); //Отправляем сигнал STOP
    /*Сброс бита ADDR производится чтением SR1, а потом SR2*/
    I2C2->SR1;
    I2C2->SR2;    
        //пишем данные  
  I2C2->DR = data;  
  while(!(I2C2->SR1 & I2C_SR1_BTF)){};  
  SET_BIT(I2C2->CR1, I2C_CR1_STOP); //Отправляем сигнал STOP
  CLEAR_BIT(I2C2->SR1, I2C_SR1_AF); //Сбрасываем бит AF
}


//__STATIC_INLINE void DelayMicro(__IO uint32_t micros)
volatile void DelayMicro(__IO uint32_t micros)
{
  micros *=(SystemCoreClock / 1000000) / 9;
  while (micros--);
}

uint8_t portlcd;
void sendhalfbyte(uint8_t c)
{
  c<<=4;
  CMSIS_I2C_Adress_Device_Scan(portlcd|c);
  CMSIS_I2C_Adress_Device_Scan((portlcd|=0x04)|c);
  DelayMicro(1);
  CMSIS_I2C_Adress_Device_Scan((portlcd&=~0x04)|c);
  DelayMicro(50);
}

void sendbyte(uint8_t c, uint8_t mode)
{
  if(mode==0) rs_reset();
  else rs_set();
  uint8_t hc=0;
  hc=c>>4;
  sendhalfbyte(hc);sendhalfbyte(c);
}

void LCD_WriteByteI2CLCD(uint8_t bt)
{
  CMSIS_I2C_Adress_Device_Scan(bt);
}


void LCD_ini()
{
 DelayMicro(20000);
  DelayMicro(16000);
  sendhalfbyte(0x03);
  DelayMicro(4200);
  sendhalfbyte(0x03);
    sendhalfbyte(0x03);
  DelayMicro(4200);
  DelayMicro(50000);
 DelayMicro(50000);
  sendhalfbyte(0x02);
   DelayMicro(500);
  sendhalfbyte(0x02);//ðåæèì 4 áèò, 2 ëèíèè (äëÿ íàøåãî áîëüøîãî äèñïëåÿ ýòî 4 ëèíèè, øðèôò 5õ8
  DelayMicro(500);
    sendhalfbyte(0x0F);//ðåæèì 4 áèò, 2 ëèíèè (äëÿ íàøåãî áîëüøîãî äèñïëåÿ ýòî 4 ëèíèè, øðèôò 5õ8
  DelayMicro(500);
    sendhalfbyte(0x0);//ðåæèì 4 áèò, 2 ëèíèè (äëÿ íàøåãî áîëüøîãî äèñïëåÿ ýòî 4 ëèíèè, øðèôò 5õ8
  DelayMicro(500);
  sendhalfbyte(0x0C);//äèñïëåé âêëþ÷àåì (D=1), êóðñîðû íèêàêèå íå íóæíû
  DelayMicro(500);
  sendbyte(0,0);// óáåðåì ìóñîð
 DelayMicro(500);
  sendbyte(0x01,0);// óáåðåì ìóñîð
 DelayMicro(500);
  sendbyte(0x00,0);// ïèøåì âëåâî
  DelayMicro(500);
    sendbyte(0x06,0);// ïèøåì âëåâî
  DelayMicro(500);
   sendbyte(0x0C,0);//дисплей включаем (D=1), курсоры никакие не нужны
}
void LCD_SendChar(char ch)
{
  sendbyte(ch,1);
}

void LCD_String(char* st)
{
  uint8_t i=0;
  while(st[i]!=0)
  {
    sendbyte(st[i],1);
    i++;
  }
}

void LCD_SetPos(uint8_t x, uint8_t y)
{
  switch(y)
  {
    case 0:
      sendbyte(x|0x80,0);
      break;
    case 1:
      sendbyte((0x40+x)|0x80,0);
      break;
    case 2:
      sendbyte((0x14+x)|0x80,0);
      break;
    case 3:
      sendbyte((0x54+x)|0x80,0);
      break;
  }
}

void ADC1_IRQHandler()
{
    // Анализируем флаги и выясняем причину прерывания,
    // выполняем какие-то действия...
    
    // Сбрасываем флаги - младшие 5 бит регистра.
    ADC1->SR&=~0x1F;
}

void ADC_mess()
{
   ADC1->SQR3&=~0x1F;
    ADC1->SQR3|=1;
    
    // Задаём продолжительность выборки для канала 17:
    // 239.5 тактов = 20мкс при тактовой частоте ADC 12МГц.
    ADC1->SMPR1|=7<<21;
    
    // Запустим фиктивное преобразование и дождёмся его завершения
    // для формирования достаточно продолжительной задержки.
    ADC1->CR2|=ADC_CR2_ADON;
    while((ADC1->SR&ADC_SR_EOC)==0);
    // Сброс флага завершения.
    ADC1->SR&=~ADC_SR_EOC;
    
    // Запускаем самокалибровку ADC и ждём её завершения.
    ADC1->CR2|=ADC_CR2_CAL;
    while(ADC1->CR2&ADC_CR2_CAL);
    // ADC готов к работе и откалиброван.
    // Измерим напряжение на встроенном источнике опорного напряжения.
    ADC1->CR2|=ADC_CR2_ADON;
    while((ADC1->SR&ADC_SR_EOC)==0);
    d0=ADC1->DR;  // Чтение из DR сбрасывает флаг EOC.    
    // Теперь можем определить напряжение питания микроконтроллера
    // (если Vref(+)=Vdd):
    Vdd=4069/d0;
    d0=3.24/Vdd;
    sprintf(str1,"%f",(char)d0);
   //sprintf(do);
LCD_print_const_d(d0); 
      LCD_String("V");   
    // Vdd=d0/Vdd;
    // Чтобы разрешить генерацию прерывания от ADC при установке
    // каких-либо флагов в регистре ADC1->SR, необходимо установить
    // соответствующий разрешающий бит, например:
    ADC1->CR1|=ADC_CR1_EOCIE;    
    // Завершение следующего преобразования приведёт к
    // генерации прерывания:
    ADC1->CR2|=ADC_CR2_ADON;
}

void LCD_print_const(int ini) //Выводим переменные типа int
{
  char in[20];
  sprintf(in,"%i",ini);
  LCD_String(in);
}


int LCD_print_const_d(double val) //Выводим переменные типа double
{
  int prescaler=10000;
  double fracPart, intPart;
  fracPart = modf(val, &intPart);
  LCD_print_const((int)val);
  LCD_String(".");
  LCD_print_const((int)(fracPart*prescaler));
  }


int main()
{
    RCC->APB2ENR|=RCC_APB2ENR_ADC1EN;
    RCC->APB2ENR|=RCC_APB2ENR_IOPAEN;
     GPIOA->CRL &= ~GPIO_CRL_CNF1;
    // Включаем ADC и внутренние аналоговые каналы:
    // термометр и встроенный источник опорного напряжения.
    ADC1->CR2|=ADC_CR2_ADON|ADC_CR2_TSVREFE;
i2c_init();
LCD_ini();


//LCD_String("String 4");
/*LCD_SetPos(5,1);
LCD_String("String 3");
*/

//volatile uint8_t state=0;

LCD_String("Voltage"); 
 LCD_SetPos(1,1);  
// LCD_print_const(1000);
    ADC_mess();
    while(1)
    {
    //LCD_String("String ");   
   
     
    }
    return 0;
}