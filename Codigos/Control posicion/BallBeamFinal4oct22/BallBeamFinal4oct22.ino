const int A = D4;
const int Button = D3;
const int B = D5;
const int Pw = D6;
const int DirA = D8;
const int DirB = D7;

int Pw_val = 0;
int Pw_vo = 0;
int t = 0;
int tm = 4;
float tms = 0.004;
float itms = 250;
int i = 0;
long int b = 0;
int timems = 0;

float pi = 3.141592;
float ppr = 4180;//817; //motor dario reales
float p2pos =(2*pi)/ppr;
float Pos = 0;
float ref = 0;
float c = 0;
float c_o = 0;
float err = 0;
float err_d = 0;
float err_s = 0;
float err_a = 0;
float Kp = 79.2;
float Kd = 1.03;
float Ki = 0;

ICACHE_RAM_ATTR void ButtonPress()
  {
   if(digitalRead(Button) == LOW)
   {b = 0;}  
  }

ICACHE_RAM_ATTR void ARise()
  {
   if(digitalRead(B) == digitalRead(A))
   {b--;}
   else
   {b++;}   
  }

ICACHE_RAM_ATTR void BRise()
  {
   if(digitalRead(A) != digitalRead(B))
   {b--;}
   else
   {b++;}   
  }

void setup() {
  Serial.begin(115200);
  pinMode(Button, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(Button), ButtonPress, FALLING);
  pinMode(A, INPUT_PULLUP);
  pinMode(B, INPUT_PULLUP);
  pinMode(DirA, OUTPUT);
  pinMode(DirB, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(A), ARise, RISING);
  attachInterrupt(digitalPinToInterrupt(B), BRise, RISING);
}

void loop() { 

  timems = millis();

 
  switch (timems) {
   case 30000:
     ref = 0.087;
    break;
  }
  
  if(timems - t >= tm)
  {
    Pos = b * p2pos;
    err = ref - Pos;
    err_d = (err - err_a)*itms;
    err_s = (err_s + err);
    c = err * Kp + err_d * Kd + err_s*Ki*(float)tms;
    err_a = err;
    
    if(c > 12){
      c = 12;
    }
    else if (c < -12){
      c = -12;
    }
    else if(c == 0){
      delay(1);
      c = 0;
    }

    c_o = (c * 1024)/12;
    c_o = ChangeDir((int)c_o);
    analogWrite(Pw,c_o);
    
    Serial.print(timems);
    Serial.print(" ");
    Serial.print(err,4);
    Serial.print(" ");
    Serial.print(ref,4);
    Serial.print(" ");
    Serial.print(Pos,4);
    Serial.print(" ");
    Serial.println(c);

    t = millis();    
  }
}


int ChangeDir(int P)
{
  if(P > 0)
  {
    digitalWrite(DirA,LOW);   //D8
    digitalWrite(DirB,HIGH);  //D7
    
  }
  else
  {
    //digitalWrite(DirA,LOW);
    digitalWrite(DirB,LOW);
    digitalWrite(DirA,HIGH); 
    P = P*-1; 
  }
  return P;
}
