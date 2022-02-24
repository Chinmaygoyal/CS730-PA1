#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <crypter.h>

int main()
{
  DEV_HANDLE cdev;
  // char *msg = "Hello CS730!";
  // 5MB
  char* msg = (char*)malloc(5*1024*1024 + 5);
  for(int i = 0;i<5*1024*1024;i++) msg[i] = 'a';
  msg[1024*1024*5] = '\0';
  char* op_text = (char*)malloc(5*1024*1024 + 5);
  
  // char op_text[1024*1024*5+2];

  KEY_COMP a=1, b=1;
  uint64_t size = strlen(msg);
  printf("%lld\n",size);
  // return 0;
  strcpy(op_text, msg);
  cdev = create_handle();
  
  if(cdev == ERROR)
  {
    printf("Unable to create handle for device\n");
    exit(0);
  }

  if(set_key(cdev, a, b) == ERROR){
    printf("Unable to set key\n");
    exit(0);
  }


  set_config(cdev,1,1);
  // set_config(cdev,0,1);

  // printf("Original Text: %s\n", msg);

  encrypt(cdev, op_text, size, 0);
  // printf("Encrypted Text: %s\n", op_text);

  for(int i = 0;i<5*1024*1024;i++){
    if(op_text[i] != 'c'){
      printf("%c\n",op_text[i]);
      printf("TEST FAILED\n");
      return 0;
    }
  }

  printf("TEST PASSED\n");
  // decrypt(cdev, op_text, size, 0);
  // // printf("Decrypted Text: %s\n", op_text);

  // for(int i = 0;i<5*1024*1024;i++){
  //   if(op_text[i] != 'a'){
  //     printf("TEST FAILED\n");
  //     return 0;
  //   }
  // }
  // printf("TEST PASSED\n");


  // set_config(cdev,1,0);

  // printf("Original Text: %s\n", msg);

  // encrypt(cdev, op_text, size, 0);
  // printf("Encrypted Text: %s\n", op_text);

  // decrypt(cdev, op_text, size, 0);
  // printf("Decrypted Text: %s\n", op_text);
  close_handle(cdev);
  return 0;
}
