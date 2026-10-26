#pragma once
#include <cerrno>
#include <csignal>
#include <cstring>
#include <poll.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>
#include <json/json.h>
#include <chrono>
#include <stdexcept>
#include <memory>
#include <string>

namespace takeover {
// A bounded local socket to a persistent CPU-only numerical worker. It has no
// ROS subscriptions, network sockets, truth input or actuator access.
class SqpWorker {
 public:
  SqpWorker(const std::string &script, const std::string &source) {
    int pair[2];
    if(socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, pair))
      throw std::runtime_error("sqp_socket_failed");
    pid_=fork();
    if(pid_<0) {close(pair[0]);close(pair[1]);throw std::runtime_error("sqp_fork_failed");}
    if(pid_==0) {
      close(pair[0]); dup2(pair[1],STDIN_FILENO); dup2(pair[1],STDOUT_FILENO); close(pair[1]);
      execlp("python3","python3","-u",script.c_str(),source.c_str(),static_cast<char*>(nullptr));
      _exit(127);
    }
    close(pair[1]); fd_=pair[0];
    Json::Value hello;
    if(!receive(hello,5000) || !hello["ready"].asBool()) {
      stop(); throw std::runtime_error("sqp_worker_not_ready_check_external_source");
    }
  }
  ~SqpWorker(){stop();}
  SqpWorker(const SqpWorker&)=delete;
  SqpWorker& operator=(const SqpWorker&)=delete;
  Json::Value solve(const Json::Value &request) {
    ++calls;
    Json::Value response;
    if(fd_<0) {++failures;response["ok"]=false;response["message"]="sqp_worker_unavailable";last=response;return response;}
    // Discard a late result; it belongs to an old vehicle state. A single slow
    // solve must not permanently disable the worker or be reused as fresh.
    if(pending_) {
      Json::Value stale;
      if(!receive(stale,1)) {
        ++failures;response["ok"]=false;response["message"]="sqp_worker_still_busy";last=response;return response;
      }
      pending_=false;
    }
    Json::StreamWriterBuilder writer;writer["indentation"]="";
    const std::string line=Json::writeString(writer,request)+"\n";
    if(line.size()>32000 ||
       send(fd_,line.data(),line.size(),MSG_NOSIGNAL|MSG_DONTWAIT)!=ssize_t(line.size())) {
      stop();response["ok"]=false;response["message"]="sqp_worker_io_failure";
    } else if(!receive(response,40)) {
      ++timeouts;pending_=true;
      response["ok"]=false;response["message"]="sqp_worker_deadline";
    }
    if(!response["ok"].asBool()) ++failures;
    last=response;
    return response;
  }
  int calls=0,failures=0,timeouts=0;
  Json::Value last;
 private:
  int fd_=-1;pid_t pid_=-1;
  bool pending_=false;
  std::string buffered_;
  bool receive(Json::Value &value,int timeout_ms) {
    const auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(timeout_ms);
    while(buffered_.size()<32000) {
      const auto newline=buffered_.find('\n');
      if(newline!=std::string::npos) {
        const std::string line=buffered_.substr(0,newline);
        buffered_.erase(0,newline+1);
        Json::CharReaderBuilder builder;std::string errors;
        std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
        return reader->parse(line.data(),line.data()+line.size(),&value,&errors);
      }
      const auto remaining=end-std::chrono::steady_clock::now();
      if(remaining<=std::chrono::steady_clock::duration::zero()) return false;
      // Round UP: truncating a one-ms poll to zero prevented draining late
      // results forever. Keep partial frames across deadlines as well.
      int left=1+std::chrono::duration_cast<std::chrono::milliseconds>(remaining).count();
      pollfd p{fd_,POLLIN,0};
      if(poll(&p,1,left)<=0 || !(p.revents&POLLIN)) return false;
      char chunk[4096];ssize_t n=recv(fd_,chunk,sizeof(chunk),MSG_DONTWAIT);
      if(n<=0)return false;
      buffered_.append(chunk,n);
    }
    return false;
  }
  void stop() {
    if(fd_>=0){close(fd_);fd_=-1;}
    if(pid_>0){kill(pid_,SIGKILL);waitpid(pid_,nullptr,0);pid_=-1;}
  }
};
} // namespace takeover
