#pragma once
#include <cstdint>
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <string>
#include <cctype>
#include <cstdio>
using std::min; using std::max; using std::isfinite;
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define RISING 3
#define ARDUINO_ISR_ATTR
using portMUX_TYPE = int;
#define portMUX_INITIALIZER_UNLOCKED 0
inline void portENTER_CRITICAL(portMUX_TYPE*){}
inline void portEXIT_CRITICAL(portMUX_TYPE*){}
inline void portENTER_CRITICAL_ISR(portMUX_TYPE*){}
inline void portEXIT_CRITICAL_ISR(portMUX_TYPE*){}
inline void attachInterrupt(int,void(*)(),int){}
inline int digitalPinToInterrupt(int pin){return pin;}
extern uint32_t fakeNow;
extern int pins[40];
inline uint32_t millis(){return fakeNow;}
inline uint32_t micros(){return fakeNow*1000;}
inline int digitalRead(int p){return pins[p];}
inline void digitalWrite(int,int){}
inline void pinMode(int,int){}
inline void ledcWrite(int,int){}
inline bool ledcAttach(int,int,int){return true;}
inline void delayMicroseconds(int){}
inline int &pulseHighAfterTimeout(){static int pin=-1;return pin;}
inline unsigned long pulseIn(int pin,int,uint32_t){
  if(pin==pulseHighAfterTimeout())pins[pin]=HIGH;
  return 0;
}
template<class T> T constrain(T x,T a,T b){return min(max(x,a),b);}
inline bool isDigit(char c){return std::isdigit(static_cast<unsigned char>(c));}
class String {
 std::string s;
 public:
 String(const char* x=""):s(x){}
 void trim(){auto a=s.find_first_not_of(" \r\n\t"); if(a==std::string::npos){s.clear();return;} auto b=s.find_last_not_of(" \r\n\t");s=s.substr(a,b-a+1);}
 void toUpperCase(){for(auto &c:s)c=std::toupper(c);}
 unsigned length()const{return s.size();}
 char operator[](unsigned i)const{return s[i];}
 long toInt()const{return std::strtol(s.c_str(),nullptr,10);}
 bool operator==(const char* x)const{return s==x;}
};
struct SerialStub {
 std::string output;
 std::string input;size_t readAt=0;
 void begin(int){} void setTimeout(int){} void setTxBufferSize(size_t){}
 void println(const char* s = ""){output+=s;output+='\n';}
 template<class... T> void printf(const char* format,T... args){
   char buffer[4096];int n=std::snprintf(buffer,sizeof(buffer),format,args...);
   if(n>0)output.append(buffer,(size_t)min(n,(int)sizeof(buffer)-1));
 }
 int available(){return (int)(input.size()-readAt);}
 int read(){return available()?input[readAt++]:-1;}
 String readStringUntil(char){return String();}
};
extern SerialStub Serial;
