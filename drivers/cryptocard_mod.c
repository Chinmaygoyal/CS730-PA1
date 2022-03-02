#include<linux/module.h>
#include<linux/kernel.h>
#include<linux/sched.h>
#include<linux/init.h>
#include<linux/pci.h>
#include<linux/mm.h>
#include<linux/mm_types.h>
#include<linux/file.h>
#include<linux/io.h>
#include<linux/fs.h>
#include<linux/wait.h>
#include<linux/sched.h>
#include<linux/path.h>
#include<linux/slab.h>
#include<linux/dcache.h>
#include<linux/uaccess.h>
#include<linux/fs_struct.h>
#include<asm/tlbflush.h>
#include<linux/uaccess.h>
#include<linux/device.h>
#include<asm-generic/io.h>
#include <linux/delay.h>
#include "cryptcard.h"

static int major;
atomic_t  device_opened;
static struct class *demo_class;
struct device *demo_device;
static int command;

static unsigned long gptr;

unsigned long mmio_start, mmio_len;
u8 __iomem *hwmem; /* Memory pointer for the I/O operations */
u8 ir_value;
struct pci_dev *p_dev;
dma_addr_t dma_handle;
void* kernel_add;
DECLARE_WAIT_QUEUE_HEAD(wq);
static DEFINE_SPINLOCK(sp_lock);
int g_a = 1, g_b = 1, g_mmio = 1, g_interrupt = 0;

void iowrite64(u64 value, void __iomem* addr){
    writeq(value,addr);
    return;
}

u64 ioread64(const void __iomem *addr){
	return readq(addr);
}

void set_a_b(uint8_t a, uint8_t b){
    // printk("The values of a and b are: %u,%u\n",a,b);
    int val = (a << 8) | b;
    g_a = a;
    g_b = b;
    iowrite32(val,hwmem+0x8);
    return;
}

void __encrypt(u64 address, uint64_t length, struct file_pvt *pvt_data, int decrypt, int ismapped){
    int mmio = pvt_data->mmio;
    int interrupt = pvt_data->interrupt;
    set_a_b(pvt_data->a,pvt_data->b);
    void* destination_addr;

    g_mmio = mmio;
    g_interrupt = interrupt;

    if(mmio){
        // printk("MMIO\n");
        while(1){
            if((ioread32(hwmem + MMIO_STATUS) & (u32)1) == 1);
            else break;
        }
        destination_addr = hwmem + MMIO_DATA;
        if(!ismapped){
            memcpy(destination_addr ,address,length);
            iowrite32((u32)length,hwmem+MMIO_LENGTH);
            iowrite32(0 | (decrypt << DECRYPT_BIT) | (interrupt << MMIO_INTERRUPT),hwmem+MMIO_STATUS);
            iowrite64(MMIO_DATA,hwmem+MMIO_ADDRESS);
        }else{
            iowrite32((u32)length,hwmem+MMIO_LENGTH);
            iowrite32(0 | (decrypt << DECRYPT_BIT) | (interrupt << MMIO_INTERRUPT),hwmem+MMIO_STATUS);
            iowrite64(MMIO_DATA,hwmem+MMIO_ADDRESS);
        }
    }else{
        // printk("DMA\n");
        while(1){
            if((ioread64(hwmem + DMA_STATUS) & (u64)1) == 1) usleep_range(100,200);
            else break;
        }
        destination_addr = kernel_add; 
        memcpy(destination_addr ,address,length);
        iowrite64(length,hwmem + DMA_LENGTH);
        iowrite64(dma_handle,hwmem + DMA_ADDRESS);
        iowrite64(((u64)1 | (decrypt << DECRYPT_BIT) | (interrupt << DMA_INTERRUPT)),hwmem + DMA_STATUS);
    }

    if(interrupt){
        // process goes into sleep over here, need to wake the process from interrupt handler
        // interruptible_sleep_on(&wq);
        if(mmio){
            wait_event_interruptible(wq,(ioread32(hwmem + MMIO_STATUS) & (u32)1) == 0);
        }else{
            wait_event_interruptible(wq,(ioread64(hwmem + DMA_STATUS) & (u64)1) == 0);
        }
    }else{
        if(mmio){
            while(1){
                if((ioread32(hwmem + MMIO_STATUS) & (u32)1) == 0) break;
                else{
                    usleep_range(1000,1100);
                }
            }
        }else{
            while(1){
                if((ioread64(hwmem + DMA_STATUS) & (u64)1) == 0) break;
                else{
                    usleep_range(1000,1100);
                }
            }
        }
    }
    if(mmio && ismapped){
        // memcpy(address,destination_addr,length);
    }else
        memcpy(address,destination_addr,length);
    return;
}

struct file_pvt* private_data_init(void){
    struct file_pvt* pvt = (struct file_pvt*)kmalloc(sizeof(struct file_pvt),GFP_USER);
    pvt->mmio = g_mmio;
    pvt->a = g_a;
    pvt->b = g_b;
    pvt->interrupt = g_interrupt;
    return pvt;
}

void private_data_release(struct file* file){
    kvfree(file->private_data);
    file->private_data = NULL;
    return;
}

static int demo_open(struct inode *inode, struct file *file)
{
        atomic_inc(&device_opened);
        try_module_get(THIS_MODULE);
        if(!file){
            printk("The given file structure is NULL\n");
        }
        file->private_data = private_data_init();
        printk(KERN_INFO "Device opened successfully\n");
        return 0;
}

static int demo_release(struct inode *inode, struct file *file)
{
        atomic_dec(&device_opened);
        module_put(THIS_MODULE);
        private_data_release(file);
        printk(KERN_INFO "Device closed successfully\n");
        return 0;
}

static ssize_t demo_read(struct file *filp, char *buffer, size_t length, loff_t * offset)
{           
        printk(KERN_INFO "In read\n");
        if (copy_to_user(buffer,&gptr,sizeof(unsigned long)) != 0)
             return sizeof(unsigned long);
        return -1;
}

static ssize_t demo_write(struct file *filp, const char *buff, size_t len, loff_t * off)
{

        spin_lock(&sp_lock);
        printk(KERN_INFO "In write\n");
        struct crypt_data data;
        if(copy_from_user(&data,buff,len) != 0)
            return -1;

        if(data.operation == 1 || data.operation == 2){
            // encrypt
            long remaining = data.length;
            int chunk_num = 0;
            if(((struct file_pvt *)filp->private_data)->mmio == 1){
                while(remaining > 0){
                    u64 temp_address = (u64)data.address + MMIO_LIMIT*(chunk_num);
                    chunk_num++;
                    long chunk_size = min(remaining,(long)MMIO_LIMIT);
                    __encrypt(temp_address,chunk_size,filp->private_data,(data.operation == 1) ? 0 : 1,data.ismapped);
                    remaining -= chunk_size;
                }
            }else{
                while(remaining > 0){
                    u64 temp_address = (u64)data.address + DMA_LIMIT*(chunk_num);
                    chunk_num++;
                    long chunk_size = min(remaining,(long)DMA_LIMIT);
                    __encrypt(temp_address,(u64)min(remaining,DMA_LIMIT),filp->private_data,(data.operation == 1) ? 0 : 1,data.ismapped);
                    remaining -= chunk_size;
                }
            }
            // __encrypt(data.address,data.length,filp->private_data,(data.operation == 1) ? 0 : 1);
        
        }else if(data.operation == 3){
            // set key values
            ((struct file_pvt *)filp->private_data)->a = data.a;
            ((struct file_pvt *)filp->private_data)->b = data.b;
        }else if(data.operation == 4){
            // this is for interrupt
            ((struct file_pvt *)filp->private_data)->interrupt = data.interrupt;
        }else if(data.operation == 5){
            // this is for mmio
            ((struct file_pvt *)filp->private_data)->mmio = data.mmio;
        }else{
            printk("Invalid operation\n");
            spin_unlock(&sp_lock);
            return -1;
        }
        spin_unlock(&sp_lock);
        return sizeof(struct crypt_data);
}

void simple_vma_close(struct vm_area_struct *vma){
    printk("Calling the close function\n");
    return;
}

static struct vm_operations_struct simple_remap_vm_ops = {
    .close = simple_vma_close,
};

static int demo_mmap(struct file *filp, struct vm_area_struct * vma){
        io_remap_pfn_range(vma, vma->vm_start, pci_resource_start(p_dev, 0) >> PAGE_SHIFT, vma->vm_end - vma->vm_start,vma->vm_page_prot);
        return 0;
}

static struct file_operations fops = {
        .read = demo_read,
        .write = demo_write,
        .open = demo_open,
        .release = demo_release,
        .mmap = demo_mmap,
};

static char *demo_devnode(struct device *dev, umode_t *mode)
{
        if (mode && dev->devt == MKDEV(major, 0))
                *mode = 0666;
        return NULL;
}

static ssize_t memtrack_command_show(struct kobject *kobj,
                                  struct kobj_attribute *attr, char *buf)
{
        return 0;
}

static ssize_t memtrack_command_set(struct kobject *kobj,
                                   struct kobj_attribute *attr,
                                   const char *buf, size_t count)
{
        int x;
        // printk("Setting the value of sysfs variable command\n");
        sscanf(buf,"%d",&x);
        g_a = (0xff) & x;
        g_b = (0xff) & (x >> 8);
        g_interrupt = (1) & (x >> 16);
        g_mmio = (1) & (x >> 17);
        return count;
}

static struct kobj_attribute memtrack_command_attribute = __ATTR(command,0220,memtrack_command_show, memtrack_command_set);


int init_char_dev(void)
{
    int err;
	printk(KERN_INFO "Hello kernel. I am char_dev module.\n");
            
    major = register_chrdev(0, CHAR_DEVNAME, &fops);
    err = major;
    if (err < 0) {      
            printk(KERN_ALERT "Registering char device failed with %d\n", major);   
            goto error_regdev;
    }
    
    demo_class = class_create(THIS_MODULE, CHAR_DEVNAME);
    err = PTR_ERR(demo_class);
    if (IS_ERR(demo_class))
            goto error_class;

    demo_class->devnode = demo_devnode;

    demo_device = device_create(demo_class, NULL, MKDEV(major, 0), NULL, CHAR_DEVNAME);
    err = PTR_ERR(demo_device);
    if (IS_ERR(demo_device))
            goto error_device;

    /*sysfs creation*/
    err = sysfs_create_group (kernel_kobj, &memtrack_attr_group);
    if(unlikely(err)){
        printk(KERN_INFO "memtracker: can't create sysfs\n");
        goto error_device;
    }


    // printk(KERN_INFO "I was assigned major number %d. To talk to\n", major);
    atomic_set(&device_opened, 0);

	return 0;

error_device:
    class_destroy(demo_class);
error_class:
    unregister_chrdev(major, CHAR_DEVNAME);
error_regdev:
    return  err;
}

static struct attribute *memtrack_attrs[] = {
        &memtrack_command_attribute.attr,
        NULL,
};
struct attribute_group memtrack_attr_group = {
        .attrs = memtrack_attrs,
        .name = "cryptcard",
};


void cleanup_char_dev(void)
{
    device_destroy(demo_class, MKDEV(major, 0));
    class_destroy(demo_class);
    unregister_chrdev(major, CHAR_DEVNAME);
    sysfs_remove_group (kernel_kobj, &memtrack_attr_group);
    printk(KERN_INFO "Goodbye kernel\n");
}

static irqreturn_t irq_handler(int irq, void *cookie)
{
    u32 value = ioread32(hwmem + 0x24);
    iowrite32((u32)value, hwmem + 0x64);
    
    // waking up the process in the interrupt
    wake_up_interruptible(&wq);
    return IRQ_HANDLED;
}

int set_interrupts(struct pci_dev *dev)
{
    return request_irq(dev->irq, irq_handler, 0, "Cryptcard IRQ", dev);
}


int crypt_probe(struct pci_dev *dev, const struct pci_device_id *id){
    // printk(KERN_INFO "Hi!! This is the cryptcard driver. New device inserted\n");
    
    pci_enable_device(dev);
    // pci_request_region(dev, 0, DEVNAME);
    mmio_start = pci_resource_start(dev, 0);
    mmio_len = pci_resource_len(dev, 0);
    // struct resource *res = request_mem_region((long)mmio_start, (long)mmio_len, "Crypt Mem");
    hwmem = ioremap(mmio_start, mmio_len);
    // printk("Start of memory location: %llx, Length is: %lx\n",mmio_start,mmio_len);
    // printk("The value of hwmwm is %x\n",hwmem);
    // u32 value = ioread32(hwmem + 0x24);
    set_interrupts(dev);
    p_dev = dev;
    kernel_add = dma_alloc_coherent(&(p_dev->dev), 32*KB, &dma_handle, GFP_ATOMIC);
    return 0;
}

void crypt_remove(struct pci_dev *dev){
    free_irq (dev->irq, dev);
    dma_free_coherent(&(p_dev->dev), 32*KB, kernel_add, dma_handle);
    // pci_release_selected_regions(dev,0);
    // printk(KERN_INFO "Hi!! This is the cryptcard driver. New device removed\n");
    return;
}

// Vendor ID - 1234
// Product ID - deba
static struct pci_device_id driver_id_table[] = {
    {PCI_DEVICE(0x1234, 0xdeba)},
    {0,}
};

MODULE_DEVICE_TABLE(pci, driver_id_table);

static struct pci_driver crypt_driver = {
    .name = DEVNAME,
    .probe = crypt_probe,
    .remove = crypt_remove,
    .id_table = driver_id_table
};

int init_module(void){
    printk("Loading the pci driver module\n");
    int err = init_char_dev();
    if(err < 0){
      printk("Failed to register driver\n");
      return err;
    }
    return pci_register_driver(&crypt_driver);
}

void cleanup_module(void){
    printk("Unloading the pci driver module\n");
    cleanup_char_dev();
    pci_unregister_driver(&crypt_driver);
    return;
}

MODULE_DESCRIPTION("Cryptcard");
MODULE_AUTHOR("cg2chinmay@gmail.com");
MODULE_LICENSE("GPL");
