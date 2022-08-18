#ifndef GPIO_SYSFS_H
#define GPIO_SYSFS_H


#define GPIO_IN              0
#define GPIO_OUT             1
#define GPIO_LOW             0
#define GPIO_HIGH            1


int gpio_export(int pin);
int gpio_unexport(int pin);
int gpio_direction(int pin, int direction);
int gpio_read(int pin);
int gpio_write(int pin, int value);


#endif // GPIO_SYSFS_H
