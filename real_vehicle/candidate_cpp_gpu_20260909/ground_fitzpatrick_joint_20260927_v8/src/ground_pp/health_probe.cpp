#include "health_core.hpp"
#include <iostream>
#include <sstream>
int main(){
  health::Core c;std::string line;
  while(std::getline(std::cin,line)){
    Json::Value j,out;Json::CharReaderBuilder b;b["allowSpecialFloats"]=true;std::string err;std::istringstream s(line);
    if(!Json::parseFromStream(b,s,&j,&err))return 2;
    if(j["op"].asString()=="reset"){c=health::Core();continue;}
    bool value=j["op"].asString()=="update"?c.update(j["sample"],j["mono"].asDouble(),j["ros"].asDouble()):c.healthy(j["mono"].asDouble(),j["ros"].asDouble());
    out["value"]=value;out["reason"]=c.reason.empty()?Json::Value():Json::Value(c.reason);
    Json::StreamWriterBuilder w;w["indentation"]="";std::cout<<Json::writeString(w,out)<<"\n";
  }
}
