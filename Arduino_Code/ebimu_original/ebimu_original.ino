////////////////////////////////////////////////////////////////////
/////////////////////// EBIMU FUNCTION /////////////////////////////
 
//자이로는 각속도 계산( rad/s )
//오일러는 각도 계산 ( 
#define SBUF_SIZE 64

char sbuf[SBUF_SIZE];
signed int sbuf_cnt=0;

int EBimuAsciiParser(float *item, int number_of_item)
{
  int n,i;
  int rbytes;
  char *addr; 
  int result = 0;
  
  rbytes = Serial.available();
  for(n=0;n<rbytes;n++)
  {
    sbuf[sbuf_cnt] = Serial.read();
    if(sbuf[sbuf_cnt]==0x0a)
       {
           addr = strtok(sbuf,",");
           for(i=0;i<number_of_item;i++)
           {
              item[i] = atof(addr);
              addr = strtok(NULL,",");
           }

           result = 1;

         // Serial.print("\n\r");
         // for(i=0;i<number_of_item;i++)  {  Serial.print(item[i]);  Serial.print(" "); }
       }
     else if(sbuf[sbuf_cnt]=='*')
       {   sbuf_cnt=-1;
       }

     sbuf_cnt++;
     if(sbuf_cnt>=SBUF_SIZE) sbuf_cnt=0;
  }
  
  return result;
}
/////////////////////// EBIMU FUNCTION /////////////////////////////
////////////////////////////////////////////////////////////////////


void setup() {
  Serial.begin(115200);
}

void loop() {
  float euler[6];
  
  if(EBimuAsciiParser(euler,6))
  {
     Serial.print("\n\r");
     Serial.print(euler[3]);   Serial.print(" ");//ax값
     Serial.print(euler[4]);   Serial.print(" ");//ay값
     Serial.print(euler[5]);   Serial.print(" ");//az값
     Serial.print(euler[0]);   Serial.print(" ");//Roll값 
     Serial.print(euler[1]);   Serial.print(" ");//Pitch값
     Serial.print(euler[2]);   Serial.print(" ");//Yaw값
     
  }
}
