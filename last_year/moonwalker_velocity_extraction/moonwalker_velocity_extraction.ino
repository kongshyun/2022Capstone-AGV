volatile double v1, v2;
volatile double velocity_Right,velocity_Left;
const double radius = 0.032;

void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);
}

void Serial_Event();
void extract_velocity_Right();
void extract_velocity_Left();
void loop() {
  String cmd_str = "mvc=";
  String buf_velocity_R = "";
  String buf_velocity_L = ",";
  
  int speed_req_right = 0;//1*60/(2*3.14)/radius;
  int speed_req_left = 0;//1*60/(2*3.14)/radius;
  
  buf_velocity_R += String(speed_req_right);
  buf_velocity_L += String(speed_req_left);

  
  cmd_str +=  buf_velocity_R + buf_velocity_L;
  
  Serial_Event();
  extract_velocity_Left();
  extract_velocity_Right(); 

  Serial.println(cmd_str);
  Serial.print(velocity_Right);
  Serial.print(',');
  Serial.println(velocity_Left);
  Serial3.println(cmd_str);
  delay(500);

}

void Serial_Event() {
  if (Serial.available()) {
    delay(5);  //데이터 송신 다 받을수 있도록
    String recv_str = ""; //초기화
    String recv_str3 = "";


    do {
      recv_str += (char)Serial.read();
    } while (Serial.available()); //

    Serial.print("recvData : ");
    Serial.print(recv_str);
    //if(recv_str = "123")extract_velocity_Right();
    Serial3.println(recv_str);

    //Serial3.println("v1");
    delay(5); ////데이터 송신후 수신 delay
    do {
      recv_str3 += (char)Serial3.read();
    } while (Serial3.available());

  }
}
void extract_velocity_Right() {
    String recv_str3 = "";
    String str_v1, str_v2;
    Serial3.println("v");
    delay(5); ////데이터 송신후 수신 delay
    do {
      recv_str3 += (char)Serial3.read();
    } while (Serial3.available());
    //'=', '.' 인덱스 넘버 찾기
    int find_equal = recv_str3.indexOf('=', 0); //from 0
    int find_dot = recv_str3.indexOf('.', 0);
    int find_positive = recv_str3.indexOf('-', 0);
    //속도 값 추출
    for (int i = find_equal + 1; i < recv_str3.length(); i++) {str_v1 += recv_str3[i];}
    /*Serial.print("find_equal : ");
    Serial.println(find_equal);
    Serial.print("find_dot : ");
    Serial.println(find_dot);
    Serial.print("find_pos : ");
    Serial.println(find_positive);
    Serial.print("str_v1 : ");
    Serial.println(str_v1);*/
    bool positive;
    if (find_positive < 0) positive = true;
    else if (find_positive > 0) positive = false;
    double v1_1 = str_v1.toInt();
    double v1_2 = str_v1.substring(find_dot - 2, find_dot).toInt() / 100.;
    v1 = positive ? v1_1 + v1_2 : v1_1 - v1_2;
    Serial.print("velocity_R : ");
    Serial.println(v1);
    velocity_Right = v1*radius*2*3.14/60;
}
void extract_velocity_Left(){
    String recv_str3 = "";
    String str_v1, str_v2;
    Serial3.println("v2");
    delay(5); ////데이터 송신후 수신 delay
    do {
      recv_str3 += (char)Serial3.read();
    } while (Serial3.available());
    //'=', '.' 인덱스 넘버 찾기
    int find_equal = recv_str3.indexOf('=', 0); //from 0
    int find_dot = recv_str3.indexOf('.', 0);
    int find_positive = recv_str3.indexOf('-', 0);
    //속도 값 추출
    for (int i = find_equal + 1; i < recv_str3.length(); i++) {str_v1 += recv_str3[i];}
    bool positive;
    if (find_positive < 0) positive = true;
    else if (find_positive > 0) positive = false;
    double v1_1 = str_v1.toInt();
    double v1_2 = str_v1.substring(find_dot - 2, find_dot).toInt() / 100.;
    v2 = positive ? v1_1 + v1_2 : v1_1 - v1_2;
    Serial.print("velocity_L : ");
    Serial.println(v2);
    Serial.println("mmmmmmmmmmmmmmmmmmm");
    velocity_Left = v2*radius*2*3.14/60;
  }
