// String.toCharArray(buf, length,index);
volatile long p1,p2;
volatile long v1,v2;
 
void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);
}

void Serial_Event();

void loop() {
  //Serial_Event();
  Serial3.println("co=1");
  Serial3.println("co2=1");
  Serial3.println("mvc=200,200");
}

void Serial_Event(){
if(Serial.available()){
    delay(1);  //데이터 송신 다 받을수 있도록
    String recv_str = ""; //초기화
    String recv_str3 = "";
    String str_p1,str_p2;
    int find_equal,find_comma;
    
    do{
      recv_str += (char)Serial.read();
      }while(Serial.available());//
      
      Serial.print("recvData : ");
      Serial.print(recv_str);
      Serial3.println(recv_str);
      delay(10); ////데이터 송신후 수신 delay
      
    do{
      recv_str3 += (char)Serial3.read();
      }while(Serial3.available());
      Serial.print("recv_str3 : ");
      Serial.print(recv_str3);
      
    //'=' 인덱스 넘버 찾기
    find_equal = recv_str3.indexOf('=',0);//from 0
    //',' 인덱스 넘버 찾기
    find_comma = recv_str3.indexOf(',',0);
    
    //엔코더 1,2 값 분리
    for(int i=find_equal+1;i<find_comma;i++){str_p1 += recv_str3[i];}
    for(int i=find_comma+1;i<recv_str3.length();i++){str_p2 += recv_str3[i];}
    p1=str_p1.toInt();
    p2=str_p2.toInt();
    Serial.print("엔코더 1 값 :");
    Serial.println(p1);
    Serial.print("엔코더 2 값 :");
    Serial.println(p2);
   }
  
}
