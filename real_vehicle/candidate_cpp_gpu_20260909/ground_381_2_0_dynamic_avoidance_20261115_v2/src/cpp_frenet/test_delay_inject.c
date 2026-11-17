/* Test-only LD_PRELOAD interceptor. Never linked into runtime binaries.
 * Delay extraction of the last predicted state, after successful residual
 * checks but before swept-path/timing checks. ROS test domain 83 only. */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "acados_c/ocp_nlp_interface.h"
void ocp_nlp_out_get(ocp_nlp_config *c, ocp_nlp_dims *d, ocp_nlp_out *o,
                    int stage, const char *field, void *value) {
  typedef void (*fn)(ocp_nlp_config*,ocp_nlp_dims*,ocp_nlp_out*,int,const char*,void*);
  static fn real = NULL;
  static double begin = 0;
  if(!real) real=(fn)dlsym(RTLD_NEXT,"ocp_nlp_out_get");
  if(!real) abort();
  real(c,d,o,stage,field,value);
  struct timespec ts; clock_gettime(CLOCK_MONOTONIC,&ts);
  double now=ts.tv_sec+ts.tv_nsec*1e-9;
  if(stage==0 && strcmp(field,"u")==0) begin=now;
  if(stage!=24 || strcmp(field,"x")!=0) return;
  const char *domain=getenv("ROS_DOMAIN_ID"), *path=getenv("TEST_COMPUTE_DELAY_FILE");
  if(!domain || strcmp(domain,"83") || !path) return;
  FILE *f=fopen(path,"r"); double delay_ms=0;
  if(!f) return;
  int read=fscanf(f,"%lf",&delay_ms);fclose(f);
  if(read!=1 || delay_ms<=0 || delay_ms>200) return;
  double remaining=delay_ms*.001-(now-begin);
  if(remaining>0) usleep((useconds_t)(remaining*1e6));
}
