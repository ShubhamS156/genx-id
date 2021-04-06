#include "data_defs.h"
void genx_uuid_data_parse(const uint8_t **value,uint8_t *stud_arr,uint16_t length){
  uint8_t *p_data=*(value);
  printf("Bytes of data: %d\n",length);
  for(int i=0;i<length;i++){
    stud_arr[i]=*(p_data)&0xff;
    printf("stud_arr[%d]-> 0x%04x\n",i,*p_data);
    p_data=p_data+1;
  }
}
