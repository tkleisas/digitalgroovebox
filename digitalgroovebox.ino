
#include <SPI.h>

#include "adafruitSSD1306mod.h"

// Arduino synth library basic example

//Hardware connections:

//                    +10µF 
//PIN 11 ---[ 1k ]--+---||--->> Audio out
//                  |
//                 === 10nF
//                  |
//                 GND

// DZL 2014
// HTTP://dzlsevilgeniuslair.blogspot.dk
// HTTP://illutron.dk


#include "synth.h"

#define OLED_MOSI      8
#define OLED_CLK      10
#define OLED_DC       11
#define OLED_CS       12
#define OLED_RESET    13
#define ROT1_BTN       7
#define ROT2_BTN       6
#define ENC_PORT    PINC // for rotary encoder
#define BUTTON_1      12
#define BUTTON_2       9
#define BUTTON_3       5
#define BUTTON_4       2
#define BUTTON_SHIFT   4
#define BUTTON_MODE   A4

#define STATE_STOP     0
#define STATE_PLAY     1
#define STATE_REC      3
#define STATE_STEP_REC 4

#define CHAR_PLAY     27;
Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

synth edgar;    //-Make a synth
int bpm = 80;
uint8_t encv1 = 0;
uint8_t encv2 = 0;
uint8_t rv1 = 0;
uint8_t rv2 = 0;
uint8_t state = STATE_STOP;

int interval;
const unsigned char wavnames[6][3]=
{
  {'S','I','N'},
  {'T','R','I'},
  {'S','Q','R'},
  {'S','A','W'},
  {'R','M','P'},
  {'N','Z','E'}
  
};
unsigned char durwav[8][4][16] =
{
  {
    {128 | SQUARE,128 | SQUARE,128 | SQUARE,128 | SQUARE,128 | SQUARE,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW},
    {128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW},
    {128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW},
    {128 | NOISE,128 | NOISE,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW},
    
  },
    {
    {128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW},
    {128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW},
    {128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW},
    {128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW},
    
  },
  {
    {128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW},
    {128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW},
    {128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW},
    {128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW},
    
  },
  {
    {128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW},
    {128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW},
    {128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW},
    {128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW},
    
  },
  {
     {128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW},
    {128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW},
    {128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW},
    {128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW},
   
  },
  {
    {128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW},
    {128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW},
    {128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW},
    {128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW},
    
  },
  {
    {128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW},
    {128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW},
    {128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW},
    {128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW},
    
  },
  {
    {128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW},
    {128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW},
    {128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW},
    {128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW,128 | SAW},
    
  }

 
};
typedef struct voiceParams
{
  char wave;
  char duration;
  char note;
  char envelope;
  char mod;
}; 
voiceParams currVoiceParams[4];

char note[8][4][16] = 
{
  {
    {0,-1,2,-1,4,-1,6,-1,8,-1,10,-1,12,-1,14,-1},
    {0,0,-1,-1,12,12,-1,24,27,-1,33,-1,44,44,55,55},
    {-1,-1,-1,-1,36,4,67,68,32,-1,-1,-1,-1,4,67,68},
    {0,-1,-1,-1,16,-1,-1,-1,0,-1,0,-1,16,-1,-1,-1}
  },
  {
    {0,-1,2,-1,4,-1,6,-1,8,-1,10,-1,12,-1,14,-1},
    {0,0,-1,-1,12,12,-1,24,27,-1,33,-1,44,44,55,55},
    {-1,-1,-1,-1,36,4,67,68,32,-1,-1,-1,-1,4,67,68},
    {0,-1,-1,-1,16,-1,-1,-1,0,-1,0,-1,16,-1,-1,-1}
  },
{
    {0,-1,2,-1,4,-1,6,-1,8,-1,10,-1,12,-1,14,-1},
    {0,0,-1,-1,12,12,-1,24,27,-1,33,-1,44,44,55,55},
    {-1,-1,-1,-1,36,4,67,68,32,-1,-1,-1,-1,4,67,68},
    {0,-1,-1,-1,16,-1,-1,-1,0,-1,0,-1,16,-1,-1,-1}
  },
{
    {0,-1,2,-1,4,-1,6,-1,8,-1,10,-1,12,-1,14,-1},
    {0,0,-1,-1,12,12,-1,24,27,-1,33,-1,44,44,55,55},
    {-1,-1,-1,-1,36,4,67,68,32,-1,-1,-1,-1,4,67,68},
    {0,-1,-1,-1,16,-1,-1,-1,0,-1,0,-1,16,-1,-1,-1}
  },
{
    {0,-1,2,-1,4,-1,6,-1,8,-1,10,-1,12,-1,14,-1},
    {0,0,-1,-1,12,12,-1,24,27,-1,33,-1,44,44,55,55},
    {-1,-1,-1,-1,36,4,67,68,32,-1,-1,-1,-1,4,67,68},
    {0,-1,-1,-1,16,-1,-1,-1,0,-1,0,-1,16,-1,-1,-1}
  },
{
    {0,-1,2,-1,4,-1,6,-1,8,-1,10,-1,12,-1,14,-1},
    {0,0,-1,-1,12,12,-1,24,27,-1,33,-1,44,44,55,55},
    {-1,-1,-1,-1,36,4,67,68,32,-1,-1,-1,-1,4,67,68},
    {0,-1,-1,-1,16,-1,-1,-1,0,-1,0,-1,16,-1,-1,-1}
  },
{
    {0,-1,2,-1,4,-1,6,-1,8,-1,10,-1,12,-1,14,-1},
    {0,0,-1,-1,12,12,-1,24,27,-1,33,-1,44,44,55,55},
    {-1,-1,-1,-1,36,4,67,68,32,-1,-1,-1,-1,4,67,68},
    {0,-1,-1,-1,16,-1,-1,-1,0,-1,0,-1,16,-1,-1,-1}
  },
{
    {0,-1,2,-1,4,-1,6,-1,8,-1,10,-1,12,-1,14,-1},
    {0,0,-1,-1,12,12,-1,24,27,-1,33,-1,44,44,55,55},
    {-1,-1,-1,-1,36,4,67,68,32,-1,-1,-1,-1,4,67,68},
    {0,-1,-1,-1,16,-1,-1,-1,0,-1,0,-1,16,-1,-1,-1}
  }  
};

/*
char pattrn1[16] = {0,-1,2,-1,4,-1,6,-1,8,-1,10,-1,12,-1,14,-1};
char pattrn2[16] = {0,0,-1,-1,12,12,-1,24,27,-1,33,-1,44,44,55,55};
char pattrn3[16] = {-1,-1,-1,-1,36,4,67,68,32,-1,-1,-1,-1,4,67,68};
char pattrn4[16] = {0,-1,-1,-1,16,-1,-1,-1,0,-1,0,-1,16,-1,-1,-1};
*/
void setup() {


  edgar.begin(CHB);  //-Start it up
display.begin(SSD1306_SWITCHCAPVCC);
  
  display.display();
  delay(2000);
  setupUI();
  display.display();
  interval = calc_interval(bpm);
  //Set up the voices:
  //setupVoice( voice[0-3] , waveform[SINE,TRIANGLE,SQUARE,SAW,RAMP,NOISE] , pitch[0-127], envelope[ENVELOPE0-ENVELOPE3], length[0-127], mod[0-127, 64=no mod])
  
  edgar.setupVoice(0,SINE,60,ENVELOPE1,60,64);
  currVoiceParams[0].wave = SINE;
  currVoiceParams[0].duration = 64;
  currVoiceParams[0].note = 18;
  currVoiceParams[0].envelope = ENVELOPE1;
  currVoiceParams[0].mod = 64;
  edgar.setupVoice(1,RAMP,0,ENVELOPE3,64,64);
  currVoiceParams[1].wave = RAMP;
  currVoiceParams[1].duration = 64;
  currVoiceParams[1].note = 36;
  currVoiceParams[1].envelope = ENVELOPE3;
  currVoiceParams[1].mod = 64;
  edgar.setupVoice(2,TRIANGLE,0,ENVELOPE2 ,70,64);
  currVoiceParams[2].wave = TRIANGLE;
  currVoiceParams[2].duration = 64;
  currVoiceParams[2].note = 67;
  currVoiceParams[2].envelope = ENVELOPE2;
  currVoiceParams[2].mod = 64;
  edgar.setupVoice(3,NOISE,0,ENVELOPE3,20,64);
  currVoiceParams[3].wave = NOISE;
  currVoiceParams[3].duration = 64;
  currVoiceParams[3].note = 68;
  currVoiceParams[3].envelope = ENVELOPE3;
  currVoiceParams[3].mod = 64;
  
  pinMode(A0, INPUT);
  digitalWrite(A0, HIGH);
  pinMode(A1, INPUT);
  digitalWrite(A1, HIGH);
  pinMode(A2, INPUT);
  digitalWrite(A2, HIGH);
  pinMode(A3, INPUT);
  digitalWrite(A3, HIGH);
  pinMode(ROT1_BTN,INPUT_PULLUP);
  pinMode(ROT2_BTN,INPUT_PULLUP);
  pinMode(BUTTON_1,INPUT_PULLUP);
  pinMode(BUTTON_2,INPUT_PULLUP);
  pinMode(BUTTON_3,INPUT_PULLUP);
  pinMode(BUTTON_4,INPUT_PULLUP);
  pinMode(BUTTON_SHIFT,INPUT_PULLUP);
  pinMode(BUTTON_MODE,INPUT_PULLUP);
  pciSetup(A0);
  pciSetup(A1);
  pciSetup(A2);
  pciSetup(A3);
  
  encv2 = bpm;
  state = STATE_STOP;  
}
ISR (PCINT1_vect) // handle pin change interrupt for A0 to A5 here
{
  static int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  static uint8_t old_AB = 0;
  static uint8_t old_AB2 = 0;
  /**/
  old_AB <<= 2;                   //remember previous state
  old_AB |= ( (ENC_PORT & 0x3)  );  //add current state
  
  old_AB2 <<= 2;                   //remember previous state
  old_AB2 |= ( (ENC_PORT & 12) >> 2 );  //add current state
  rv2 = rv2 + ( enc_states[( old_AB2 & 0x0f )]);
  
  rv1 =  rv1+( enc_states[( old_AB & 0x0f )]);
}  
int calc_interval(int bpm)
{
  return (int)15000.0f / (((float)bpm)) ;
}
char* num2screen(uint8_t n)
{
  static char nbuff[4] = {32,32,32,32};
  itoa(n,nbuff,10);
  if(nbuff[0]==0)
  {
    nbuff[0] = 32;
    nbuff[1] = 32;
    nbuff[2] = 32;
    return nbuff;
  }
  if(nbuff[2]==0)
  {
    nbuff[2] = nbuff[1];
    nbuff[1] = nbuff[0];
    nbuff[0] = 32;
  }
  if(nbuff[1]==0)
  {
    nbuff[2] = nbuff[0];
    nbuff[1] = 32;
    nbuff[0] = 32;
  }
  
  return nbuff;
}

void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

void loop()
{
  //-Plays some notes on the voices:
  
  //const char notes [] PROGMEM= "0123456789101112131415161718192021222324252627282930313233343536373839404142434445464748495051525354555657585960616263646566676869707172737475767778798081828384858687888990919293949596979899";
  static int curr = 0;
  static int i = 47;
  static int cnt  = 0;
  static uint8_t ifield = 0;
  static int pindex = 0;
  uint8_t rot1_btn = 0;
  uint8_t rot2_btn = 0;
  static uint8_t btn1;
  static uint8_t btn2;
  static uint8_t btn3;
  static uint8_t btn4;
  static uint8_t btnshft;
  static uint8_t btnmode;
  static uint8_t prev_btn1 = HIGH;
  static uint8_t prev_btn2 = HIGH;
  static uint8_t prev_btn3  = HIGH;
  static uint8_t prev_btn4  = HIGH;
  static uint8_t prev_btnshft = HIGH;
  static uint8_t prev_mode = HIGH;
/*
  uint8_t rv1 = read_encoder_1();
  uint8_t rv2 = read_encoder_2();
  */
  uint8_t vm1,vm2;
  static int currtime = millis();
  static int prevtime = currtime;
  char* nbuf;
  
  currtime = millis();
  rot1_btn = digitalRead(ROT1_BTN);
  
  vm1 = (rot1_btn == LOW)?128:0;
  
  rot2_btn = digitalRead(ROT2_BTN);
  vm2 = (rot2_btn == LOW)?128:0;
  static int currpattern = 0;
  btn4 = digitalRead(BUTTON_4);
  btn1 = digitalRead(BUTTON_1);
  btn2 = digitalRead(BUTTON_2);
  btn3 = digitalRead(BUTTON_3);
  
  btnshft = digitalRead(BUTTON_SHIFT);
  btnmode = digitalRead(BUTTON_MODE);
  switch(state)
  {
    case STATE_STOP:
    if(btn1==LOW )
    {
      
      display.charbuffer[1]= CHAR_PLAY;
      if(prev_btn1 == HIGH)
      {
        edgar.setWave(0,currVoiceParams[0].wave);
            //edgar.setPitch(v,note[currpattern][v][pindex]);
        edgar.setLength(0,currVoiceParams[0].duration);
        
        edgar.setEnvelope(0,currVoiceParams[0].envelope);
        edgar.setMod(0,currVoiceParams[0].mod);
        edgar.mTrigger(0,currVoiceParams[0].note);
      }
      
    }
    else
    {
      display.charbuffer[1] = ' ';
    }
    if(btn2==LOW )
    {
      display.charbuffer[1+16]=CHAR_PLAY;
      if(prev_btn2 == HIGH)
      {
        edgar.setWave(1,currVoiceParams[1].wave);
            //edgar.setPitch(v,note[currpattern][v][pindex]);
        edgar.setLength(1,currVoiceParams[1].duration);
        
        edgar.setEnvelope(1,currVoiceParams[1].envelope);
        edgar.setMod(1,currVoiceParams[1].mod);

        edgar.mTrigger(1,currVoiceParams[1].note);
      }
    }
    else
    {
      display.charbuffer[1+16]=' ';
    }
    if(btn3==LOW )
    {
      display.charbuffer[1+32]=CHAR_PLAY;
      if(prev_btn3==HIGH)
      {
        edgar.setWave(2,currVoiceParams[2].wave);
            //edgar.setPitch(v,note[currpattern][v][pindex]);
        edgar.setLength(2,currVoiceParams[2].duration);
        
        edgar.setEnvelope(2,currVoiceParams[2].envelope);
        edgar.setMod(2,currVoiceParams[2].mod);

        
        edgar.mTrigger(2,currVoiceParams[2].note);
      }
      
    }
    else
    {
      display.charbuffer[1+32] = ' ';
    }
    if(btn4==LOW )
    {
      display.charbuffer[1+48] = CHAR_PLAY;
      if(prev_btn4==HIGH)
      {
        edgar.setWave(3,currVoiceParams[3].wave);
            //edgar.setPitch(v,note[currpattern][v][pindex]);
        edgar.setLength(3,currVoiceParams[3].duration);
        
        edgar.setEnvelope(3,currVoiceParams[3].envelope);
        edgar.setMod(3,currVoiceParams[3].mod);

        
        edgar.mTrigger(3,currVoiceParams[3].note);
      }
    }
    else
    {
      display.charbuffer[1+48] = ' ';
    }
    
    
    break;
    case STATE_PLAY:
        if((currtime-prevtime)>interval)
        {
          display.charbuffer[64+pindex] = 0x1f;
          for(uint8_t v= 0;v<4;v++)
          { 
            if(note[currpattern][v][pindex]!=-1)
            {
            display.charbuffer[(v<<4) + 1] = 27;
            nbuf = num2screen(note[currpattern][v][pindex]);
            display.charbuffer[(v<<4) + 2] = nbuf[0];
            display.charbuffer[(v<<4) + 3] = nbuf[1];
            display.charbuffer[(v<<4) + 4] = nbuf[2];
            display.charbuffer[(v<<4) + 5] = ' ';
            display.charbuffer[(v<<4) + 6] = wavnames[durwav[currpattern][v][pindex] & 0x7][0];
            display.charbuffer[(v<<4) + 7] = wavnames[durwav[currpattern][v][pindex] & 0x7][1];
            display.charbuffer[(v<<4) + 8] = wavnames[durwav[currpattern][v][pindex] & 0x7][2];
            
            edgar.setWave(v,durwav[currpattern][v][pindex] & 0x7);
            //edgar.setPitch(v,note[currpattern][v][pindex]);
            edgar.setLength(v,64/*(durwav[currpattern][pindex][v] & 0xf0)*/);
            edgar.setEnvelope(v,ENVELOPE0);
            edgar.mTrigger(v,(note[currpattern][v][pindex]+encv1)%128);
            
            }
            else
            {
              display.charbuffer[(v<<4) +1] = ' ';
              display.charbuffer[(v<<4) +2] = ' ';
              display.charbuffer[(v<<4)+3] = '-';
              display.charbuffer[(v<<4)+4] = '-';
              display.charbuffer[(v<<4) +5] = ' ';
              display.charbuffer[(v<<4) +6] = '-';
              display.charbuffer[(v<<4)+7] = '-';
              display.charbuffer[(v<<4)+8] = '-';
              
            }
          }
          
          pindex++;
          if(pindex>15)
          {
            pindex = 0;
          }
          edgar.filter->setResonance(encv1);
          edgar.filter->setCutoffFreq(encv2);
          
          display.charbuffer[64+pindex] = 0x1e;
          prevtime = currtime;   
        }

    break;
    case STATE_REC:
    if(btn1==LOW && prev_btn1==HIGH)
        {
          if(btnshft == LOW)
          {
            note[currpattern][0][pindex] = -1;
          }
          else
          {
            
            note[currpattern][0][pindex] = currVoiceParams[0].note;
            durwav[currpattern][0][pindex] = durwav[currpattern][0][pindex] | currVoiceParams[0].wave;
          }
          
        }
        if(btn2==LOW && prev_btn2==HIGH)
        {
          if(btnshft == LOW)
          {
            note[currpattern][1][pindex] = -1;
          }
          else
          {
            note[currpattern][1][pindex] = currVoiceParams[1].note;
            durwav[currpattern][1][pindex] = durwav[currpattern][1][pindex] | currVoiceParams[1].wave; 

          }
        }
        if(btn3==LOW && prev_btn3==HIGH)
        {
          if(btnshft == LOW)
          {
            note[currpattern][2][pindex] = -1;
          }
          else
          {
            note[currpattern][2][pindex] = currVoiceParams[2].note;
            durwav[currpattern][2][pindex] = durwav[currpattern][2][pindex] | currVoiceParams[2].wave; 
          }
        }
        if(btn4==LOW && prev_btn4==HIGH)
        {
          if(btnshft == LOW)
          {
            note[currpattern][3][pindex] = -1;
          }
          else
          {
            note[currpattern][3][pindex] = currVoiceParams[3].note;
            durwav[currpattern][3][pindex] = durwav[currpattern][3][pindex] | currVoiceParams[3].wave; 

          }
        }
      if((currtime-prevtime)>interval)
      {
        display.charbuffer[64+pindex] = 0x1f;
        
        for(uint8_t v= 0;v<4;v++)
        { 
          if(note[currpattern][v][pindex]!=-1)
          {
          display.charbuffer[(v<<4) + 1] = 27;
          nbuf = num2screen(note[currpattern][v][pindex]);
          display.charbuffer[(v<<4) + 2] = nbuf[0];
          display.charbuffer[(v<<4) + 3] = nbuf[1];
          display.charbuffer[(v<<4) + 4] = nbuf[2];
          display.charbuffer[(v<<4) + 5] = ' ';
          display.charbuffer[(v<<4) + 6] = wavnames[durwav[currpattern][v][pindex] & 0x7][0];
          display.charbuffer[(v<<4) + 7] = wavnames[durwav[currpattern][v][pindex] & 0x7][1];
          display.charbuffer[(v<<4) + 8] = wavnames[durwav[currpattern][v][pindex] & 0x7][2];
          
          edgar.setWave(v,durwav[currpattern][v][pindex] & 0x7);
          //edgar.setPitch(v,note[currpattern][v][pindex]);
          edgar.setLength(v,64/*(durwav[currpattern][pindex][v] & 0xf0)*/);
          edgar.setEnvelope(v,ENVELOPE0);
          edgar.mTrigger(v,(note[currpattern][v][pindex]+encv1)%128);
          
          }
          else
          {
            display.charbuffer[(v<<4) +1] = ' ';
            display.charbuffer[(v<<4) +2] = ' ';
            display.charbuffer[(v<<4)+3] = '-';
            display.charbuffer[(v<<4)+4] = '-';
            display.charbuffer[(v<<4) +5] = ' ';
            display.charbuffer[(v<<4) +6] = '-';
            display.charbuffer[(v<<4)+7] = '-';
            display.charbuffer[(v<<4)+8] = '-';
            
          }
        }
        
        pindex++;
        if(pindex>15)
        {
          pindex = 0;
        }
        edgar.filter->setResonance(encv1);
        edgar.filter->setCutoffFreq(encv2);
        
        display.charbuffer[64+pindex] = 0x1e;
        prevtime = currtime;
      }
    break;
    default:
      
    break;
  }
    
  if(btnmode==LOW && btn1 == LOW)
  {
    state = STATE_PLAY;
  }
  if(btnmode==LOW && btn2 == LOW)
  {
    state = STATE_STOP;
  }
  if(btnmode==LOW && btn3 == LOW)
  {
    state = STATE_REC;
  }
  if(btnmode==LOW && btn4 == LOW)
  {
    state = STATE_STEP_REC;
    
  }
  prev_btn1 = btn1;
  prev_btn2 = btn2;
  prev_btn3 = btn3;
  prev_btn4 = btn4;

  //if(rv1>0 || vm1 == HIGH)
  {
    
    encv1 = encv1+rv1;
    rv1 = 0;
    nbuf = num2screen(encv1);
    
    display.charbuffer[112] = nbuf[0] | vm1;
    display.charbuffer[113] = nbuf[1] | vm1;
    display.charbuffer[114] = nbuf[2] | vm1;
    
        
  }
  
  {
    
    encv2 = encv2+rv2;
    rv2 = 0;
    
    
    
    if(encv2>250)
    {
      encv2 = 250;
    }
    if(encv2<20)
    {
      encv2 = 20;
    }
    bpm = encv2;
    interval = calc_interval(bpm);
    nbuf = num2screen(bpm);
    display.charbuffer[116] = nbuf[0] | vm2;
    display.charbuffer[117] = nbuf[1] | vm2;
    display.charbuffer[118] = nbuf[2] | vm2;
    
        
  }
  /*
  switch(curr)
  {
    case 20:
      edgar.mTrigger(0,i+encv1%72);
     edgar.mTrigger(1,i);
    break;
    case 40:
     edgar.setLength(3,30);
     edgar.mTrigger(3,i+encv1%72);
    break;
    case 60:
      edgar.mTrigger(2,i+encv2%72);
       edgar.setLength(3,20);
       edgar.mTrigger(3,i+encv2%72);
    break;
    case 80:
    break;
    default:
    break;
  }
  curr++;
  if(curr>80)
  {
    curr = 0;
    i++;
    if(i>56)
    {
      i = 47;
    }
  }

   */
   
  /* 
   display.begin_display();
       display.display_row(0,ifield);
       display.display_row(1,ifield);
       display.display_row(2,ifield);
       display.display_row(3,ifield);
       display.display_row(4,ifield);
       display.display_row(5,ifield);
       display.display_row(6,ifield);
       display.display_row(7,ifield);
       display.end_display();
       if(ifield ==0)
       {
         ifield = 1;
       }
       else
       {
         ifield = 0;
       }
       */

  switch(cnt)
  {
    case 0:
      display.begin_display();
      
      break;
     case 1:
       display.display_row(0);
       break;
     case 2:
       display.display_row(1);
       break;
     case 3:
       display.display_row(2);
       break;
     case 4:
       display.display_row(3);
       break;
     case 5:
       display.display_row(4);
       break;
     case 6:
       display.display_row(5);
       break;
     case 7:
       display.display_row(6);
       break;
     case 8:
       display.display_row(7);
       break;
     case 9:
       display.end_display();
       if(ifield ==0)
       {
         ifield = 1;
       }
       else
       {
         ifield = 0;
       }
       break;
      default:
      break; 
     
  }
  cnt++;
  if(cnt>10)
  {
    cnt = 0;
  }
   
}

void setupUI()
{
  for(int y = 0;y<4;y++)
 {
   display.charbuffer[y*16] = (uint8_t)(0x30+y+1);
   display.charbuffer[y*16+1] = (uint8_t)' ';
   display.charbuffer[y*16+2] = (uint8_t)' ';
   display.charbuffer[y*16+3] = (uint8_t)'S';
   display.charbuffer[y*16+4] = (uint8_t)' ';
   display.charbuffer[y*16+5] = (uint8_t)' ';
   display.charbuffer[y*16+6] = (uint8_t)' ';
   display.charbuffer[y*16+7] = (uint8_t)' ';
   display.charbuffer[y*16+8] = (uint8_t)'E';
   display.charbuffer[y*16+9] = (uint8_t)' ';
   display.charbuffer[y*16+10] = (uint8_t)' ';
   display.charbuffer[y*16+11] = (uint8_t)' ';
   display.charbuffer[y*16+12] = (uint8_t)' ';
   display.charbuffer[y*16+13] = (uint8_t)'P';
   display.charbuffer[y*16+14] = (uint8_t)' ';
   display.charbuffer[y*16+15] = (uint8_t)' ';
   
   
 }
   for(int y=0;y<16;y++)
  {
    display.charbuffer[64+y] = 0x1f; // not filled circle
  } 
}

int8_t read_encoder_1()
{
  static int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  static uint8_t old_AB = 0;
  /**/
  old_AB <<= 2;                   //remember previous state
  old_AB |= ( (ENC_PORT & 0x3)  );  //add current state
  return ( enc_states[( old_AB & 0x0f )]);
}

int8_t read_encoder_2()
{
  static int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  static uint8_t old_AB = 0;
  /**/
  old_AB <<= 2;                   //remember previous state
  old_AB |= ( (ENC_PORT & 12) >> 2 );  //add current state
  return ( enc_states[( old_AB & 0x0f )]);
}

