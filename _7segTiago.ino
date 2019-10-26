#include <Shifter.h>
#include <EEPROM.h>   
#define SER_Pin 2 //SER_IN
#define RCLK_Pin 3 //L_CLOCK
#define SRCLK_Pin 4 //CLOCK
#define NUM_REGISTERS 3 //how many registers are in the chain
Shifter shifter(SER_Pin, RCLK_Pin, SRCLK_Pin, NUM_REGISTERS); 


 
  void gravar_memoria(byte memoria, int valor_para_gravar){EEPROM.write(memoria, highByte(valor_para_gravar)); 
  EEPROM.write(memoria + 256, lowByte(valor_para_gravar));}
  int memoria_lida[255];        
  //  le_memoria(x);                                                       // le um endereço de memoria "x" e salva na variavel   memoria_lida[x];
  //  for (int x = 0; x < 256; x++) { le_memoria(x);}                  // le todos os enderesos de memoria e salva na variavel  memoria_lida[255];
  void le_memoria(int endereco_memoria){ byte high = EEPROM.read(endereco_memoria);          
  byte low = EEPROM.read(endereco_memoria + 256);  memoria_lida[endereco_memoria] = (high << 8) + low; }
  
char SuaString[3] = "";
long previousMillis = 0, intervalo=1000;  
int numero = 16;  

boolean bot1ok=0,bot2ok=0,bot3ok=0, soma=0, dim=0, xxx=0;
byte rele1=5, rele2=6, somabot1=0, somabot2=0, somabot3=0;
byte pinSensor = A2, pinos[8]={7, 10, 1, 2, 3, 6, 15, 5},  leds[10]={8, 7, 9, 10, 11, 12, 13, A3, A4, A5}, dig[8] = {4,12,11,14,0,0,0,0};   
byte digito_set_seg[47][8] = { 
{ 0,0,0,0,0,0,1,1 }, // = 0
{ 1,0,0,1,1,1,1,1 }, // = 1
{ 0,0,1,0,0,1,0,1 }, // = 2
{ 0,0,0,0,1,1,0,1 }, // = 3
{ 1,0,0,1,1,0,0,1 }, // = 4
{ 0,1,0,0,1,0,0,1 }, // = 5
{ 0,1,0,0,0,0,0,1 }, // = 6
{ 0,0,0,1,1,1,1,1 }, // = 7
{ 0,0,0,0,0,0,0,1 }, // = 8
{ 0,0,0,0,1,0,0,1 }, // = 9
{ 0,0,0,1,0,0,0,1 }, // = A
{ 1,1,0,0,0,0,0,1 }, // = b   11
{ 0,1,1,0,0,0,1,1 }, // = C   12
{ 1,0,0,0,0,1,0,1 }, // = d   13
{ 0,1,1,0,0,0,0,1 }, // = E   14
{ 0,1,1,1,0,0,0,1 }, // = F   15
{ 0,1,0,0,0,0,1,1 }, // = G   16
{ 1,0,0,1,0,0,0,1 }, // = H   17
{ 1,1,1,1,0,1,1,1 }, // = i   18
{ 1,0,0,0,0,1,1,1 }, // = J   19
{ 1,1,1,0,0,0,1,1 }, // = L   20
{ 0,0,0,1,0,0,1,1 }, // = M   21
{ 1,1,0,1,0,1,0,1 }, // = n   22
{ 1,1,0,0,0,1,0,1 }, // = o   23
{ 0,0,1,1,0,0,0,1 }, // = P   24
{ 0,0,0,0,0,1,1,0 }, // = Q   25
{ 1,1,1,1,0,1,0,1 }, // = r   26
{ 0,1,1,1,0,0,0,0 },
{ 1,1,1,1,1,1,0,1 }, // = t
{ 1,1,1,1,1,1,1,1 }, // = _   29
{ 0,1,1,1,1,1,1,1 }, // = =   30
{ 1,0,1,1,1,1,1,1 }, // = L
{ 1,1,0,1,1,1,1,1 }, // = P   32
{ 1,1,1,0,1,1,1,1 }, // = A   33
{ 1,1,1,1,0,1,1,1 }, // = E   34
{ 1,1,1,1,1,0,1,1 }, //=F 17
{ 1,0,1,1,1,1,0,1 }, // = r   36
{ 0,1,1,1,1,1,1,1 }, // = -   37
{ 1,1,1,1,1,1,1,0 }, // = .
{ 1,1,1,1,1,1,0,1 }, // = -   39
{ 1,1,1,0,0,0,1,0 }, // = l.  40
{ 1,0,0,0,1,1,1,0 }, // = t   41
{ 1,0,0,0,1,0,1,0 }, // = ti   42
{ 1,0,0,0,0,0,0,1 }, // = A   43
{ 0,0,0,0,1,0,1,1 }, // = G   44
{ 0,0,1,1,1,0,0,1 }, // = o   45 
{ 1,1,1,1,1,1,1,1 }};// =     46
char lido='Z', lid[3]={'0', '0', '0'};
byte leituras=0, x=0, espera=0;
int valorSensor = 0, valorSensorcont=0, valorPor=0, ma1=0, ma2=0, mp1=0, mp2=0;
int  digito2=0, digito3=0, digito4=0, valormax=0, tempMax=0, segundo=0;




void tresDig(int valorSens){ digito2 = valorSens / 100;  digito3 = (valorSens - digito2 * 100) / 10; digito4 = valorSens - digito2 * 100 - digito3 * 10;
                escreve_set_seg(2, digito2);     escreve_set_seg(3, digito3); escreve_set_seg(4, digito4); 
                
                
             
}
void escreve_set_seg(byte x, byte digit) { shifter.clear();  shifter.write();   byte pin=0; for (byte segCount = 0; segCount < 8; ++segCount) { 
                                           shifter.setPin(pinos[pin], digito_set_seg[digit][segCount]); ++pin;} shifter.setPin(dig[x-1], HIGH); shifter.write(); 
                                           valorSensorcont = valorSensorcont + analogRead(pinSensor);  leituras++;
                                           if(leituras>=30){ valorSensor = valorSensorcont/30; valorSensorcont=0;
                                           valorSensor = map(valorSensor, mp1, 1023, 0, mp2); leituras=0;}}

void setup(){ Serial.begin(9600); //  gravar_memoria(12,0); gravar_memoria(13,400);  gravar_memoria(10,100); gravar_memoria(11,200);
shifter.clear();  shifter.write();
le_memoria(10);le_memoria(11);le_memoria(12);le_memoria(13);le_memoria(20);le_memoria(21);

ma1=memoria_lida[10];ma2=memoria_lida[11];mp1=memoria_lida[12];mp2=memoria_lida[13];
valormax=memoria_lida[20];
for (int x = 0; x < 10; x++)  {pinMode(leds[x], OUTPUT); digitalWrite(leds[x], LOW); }

Serial.print("pico maximo de pressao: "); Serial.print(memoria_lida[20]); Serial.print(", tempo com presao acima do valor definido: ");Serial.print(memoria_lida[21]);
Serial.println(" minutos...");
pinMode(rele1,OUTPUT); pinMode(3,OUTPUT); pinMode(2,OUTPUT); pinMode(4,OUTPUT); 
pinMode(rele2,OUTPUT);
digitalWrite(rele1,LOW);
digitalWrite(rele2,LOW); 


}
void loop(){ 

  unsigned long currentMillis = millis();
 
  if(currentMillis - previousMillis > intervalo) {
    previousMillis = currentMillis; numero++; // digitalWrite(leds[numero], xxx); 
  }
  
  
   if (Serial.available() > 0) {     for (int i=0;i<3;i++){    SuaString[i] = Serial.read();   
 numero = atoi(SuaString);   delay(10);  escreve_set_seg(4, numero); }
  
}

xxx=!xxx;
if(numero==1){intervalo=1000;
escreve_set_seg(1, 20);} 
if(numero==2){
escreve_set_seg(1, 42);} 
if(numero==3){
escreve_set_seg(2, 42); 
escreve_set_seg(1, 43);}
if(numero==4){
escreve_set_seg(3, 42); 
escreve_set_seg(2, 43);
escreve_set_seg(1, 44);}
if(numero==5){
escreve_set_seg(4, 42); 
escreve_set_seg(3, 43);
escreve_set_seg(2, 44);
escreve_set_seg(1, 45);
}
if(numero==6){
escreve_set_seg(4, 42); 
delay(10);
escreve_set_seg(3, 43);
delay(10);
escreve_set_seg(2, 44);
delay(10);
escreve_set_seg(1, 45);
delay(10);}
if(numero==7){
escreve_set_seg(4, 42); 
escreve_set_seg(3, 43);
escreve_set_seg(2, 44);
escreve_set_seg(1, 45);
}
if(numero==8){intervalo=500;
escreve_set_seg(4, 43);
escreve_set_seg(3, 44);
escreve_set_seg(2, 45);}
if(numero==9){
escreve_set_seg(4, 44);
escreve_set_seg(3, 45);}
if(numero==10){
escreve_set_seg(4, 45);}
if(numero==11){
escreve_set_seg(1, 5);
}
if(numero==12){
escreve_set_seg(2, 5);
escreve_set_seg(1, 3);}
if(numero==13){
escreve_set_seg(3, 5);
escreve_set_seg(2, 3);
escreve_set_seg(1, 36);
}
if(numero==14){
escreve_set_seg(4, 5);
escreve_set_seg(3, 3);
escreve_set_seg(2, 36);
escreve_set_seg(1, 13);
}
if(numero==15){
escreve_set_seg(4, 3);
escreve_set_seg(3, 36);
escreve_set_seg(2, 13);
escreve_set_seg(1, 43);
}
if(numero>=16){
escreve_set_seg(4, 29);escreve_set_seg(3, 29);escreve_set_seg(2, 29);escreve_set_seg(1, 29);delay(50);
escreve_set_seg(4, 39);escreve_set_seg(3, 39);escreve_set_seg(2, 39);delay(50);escreve_set_seg(1, 39);
escreve_set_seg(4, 37);escreve_set_seg(3, 37);delay(50);escreve_set_seg(2, 37);escreve_set_seg(1, 37);
escreve_set_seg(4, 38);delay(50);escreve_set_seg(3, 38);escreve_set_seg(2, 38);escreve_set_seg(1, 38);
}
if(numero==18){
  escreve_set_seg(4, 46);
numero=0;}
}






 
