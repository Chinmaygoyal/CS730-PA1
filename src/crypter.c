#include<crypter.h>
#include <sys/mman.h>

/*Function template to create handle for the CryptoCard device.
On success it returns the device handle as an integer*/

struct crypt_data initializa_crypt(){
    struct crypt_data cd;
    cd.address = NULL;
    cd.length = 0;
    cd.operation = 1;
    cd.a = 0;
    cd.b = 0;
    cd.interrupt = 0;
    cd.mmio = 1;
    cd.ismapped = 0;
    return cd;
}

DEV_HANDLE create_handle(){
    int fd = open("/dev/crypt_chardev",O_RDWR);
    return fd;
}

/*Function template to close device handle.
Takes an already opened device handle as an arguments*/
void close_handle(DEV_HANDLE cdev){
    close(cdev);
    return;
}

/*Function template to encrypt a message using MMIO/DMA/Memory-mapped.
Takes four arguments
  cdev: opened device handle
  addr: data address on which encryption has to be performed
  length: size of data to be encrypt
  isMapped: TRUE if addr is memory-mapped address otherwise FALSE
*/
int encrypt(DEV_HANDLE cdev, ADDR_PTR addr, uint64_t length, uint8_t isMapped){
    int fd = cdev;
    struct crypt_data data = initializa_crypt();
    data.address = addr;
    data.operation = 1;
    data.length = length;
    data.ismapped = isMapped;
    int err = write(fd,(void*)&data,sizeof(struct crypt_data));
    return err;
}

/*Function template to decrypt a message using MMIO/DMA/Memory-mapped.
Takes four arguments
  cdev: opened device handle
  addr: data address on which decryption has to be performed
  length: size of data to be decrypt
  isMapped: TRUE if addr is memory-mapped address otherwise FALSE
*/
int decrypt(DEV_HANDLE cdev, ADDR_PTR addr, uint64_t length, uint8_t isMapped){
    int fd = cdev;
    struct crypt_data data = initializa_crypt();
    data.address = addr;
    data.operation = 2;
    data.length = length;
    data.ismapped = isMapped;
    int err = write(fd,(void*)&data,sizeof(struct crypt_data));
    return err;
}

/*Function template to set the key pair.
Takes three arguments
  cdev: opened device handle
  a: value of key component a
  b: value of key component b
Return 0 in case of key is set successfully*/
int set_key(DEV_HANDLE cdev, KEY_COMP a, KEY_COMP b)
{
    int fd = cdev;
    struct crypt_data data = initializa_crypt();
    data.operation = 3;
    data.a = a;
    data.b = b;
    int err = write(fd,(void*)&data,sizeof(struct crypt_data));
    return err;
}

/*Function template to set configuration of the device to operate.
Takes three arguments
  cdev: opened device handle
  type: type of configuration, i.e. set/unset DMA operation, interrupt
  value: SET/UNSET to enable or disable configuration as described in type
Return 0 in case of key is set successfully*/
int set_config(DEV_HANDLE cdev, config_t type, uint8_t value)
{
    int fd = cdev;
    struct crypt_data data = initializa_crypt();
    if(type == 0){
        data.operation = 4;
        if(value) data.interrupt = 1;
        else data.interrupt = 0;
    }else{
        data.operation = 5;
        if(value) data.mmio = 0;
        else data.mmio = 1;
    }
    int err = write(fd,(void*)&data,sizeof(struct crypt_data));

    return err;
}

/*Function template to device input/output memory into user space.
Takes three arguments
  cdev: opened device handle
  size: amount of memory-mapped into user-space (not more than 1MB strict check)
Return virtual address of the mapped memory*/
ADDR_PTR map_card(DEV_HANDLE cdev, uint64_t size)
{
    if(size >= 1024*1024) return NULL;
    void* ret_val = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_POPULATE, cdev,0);
    return ret_val;
}

/*Function template to device input/output memory into user space.
Takes three arguments
  cdev: opened device handle
  addr: memory-mapped address to unmap from user-space*/
void unmap_card(DEV_HANDLE cdev, ADDR_PTR addr)
{
  return;
}
