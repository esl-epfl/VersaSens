#include "../drivers/twim_inst.h"
#include "../drivers/MLX90632.h"
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#define NUM_REGISTERS 9

LOG_MODULE_REGISTER(mlx90632_tb, LOG_LEVEL_INF);

uint16_t swap_endianness(uint16_t value) {
    return (value << 8) | (value >> 8);
}

int main(void)
{
    MLX90632_REG mlx_reg;
    twim_inst_init();
    MLX90632Initialize();
    LOG_INF("TWIM BUSY: %d\n", twim_is_busy());
    uint8_t data[NUM_REGISTERS*2] = {0, 0};
    uint8_t data2[NUM_REGISTERS*2] = {0, 0};
    uint16_t addr = RAM_1_Addr;
    k_sleep(K_MSEC(100));
    MLX90632Reset();
    k_sleep(K_MSEC(100));
    MLX90632Configure();
    k_sleep(K_MSEC(1000));
    // MLX90632ReadSequentialRegs(addr, data, NUM_REGISTERS*2);
    // k_sleep(K_MSEC(10));
    // mlx_reg.REG_CONTROL.w = (data[0] << 8) | data[1];
    // for (int i = 0; i < NUM_REGISTERS; i++)
    // {
    //     LOG_INF("RAM[%d]: %04x\n", i, mlx_reg.REG_CONTROL.w);
    // }
    // MLX90632WriteReg(addr, 0x0000);
    // k_sleep(K_MSEC(10));
    // MLX90632ReadSequentialRegs(addr, data, NUM_REGISTERS*2);
    // k_sleep(K_MSEC(1000));
    // MLX90632ReadSequentialRegs(addr, data2, NUM_REGISTERS*2);
    // LOG_INF("TWIM BUSY: %d\n", twim_is_busy());
    // LOG_INF("Hello, world\n");
    k_sleep(K_MSEC(100));
    // LOG_INF("TWIM BUSY: %d\n", twim_is_busy());
    // LOG_INF("transfer succeeded: %d\n", twim_transfer_succeeded());
    // for (int i = 0; i < NUM_REGISTERS; i++)
    // {
    //     LOG_INF("RAM[%d]: %02x%02x\n", i, data[i*2], data[i*2+1]);
    // }
    // k_sleep(K_MSEC(300));

    MLX90632StartContinuousRead();
    k_sleep(K_MSEC(3000));
    // MLX90632StopContinuousRead();
    // for (int i = 0; i < NUM_REGISTERS; i++)
    // {
    //     LOG_INF("RAM[%d]: %02x%02x\n", i, data2[i*2], data2[i*2+1]);
    // }
    while(1){
        k_sleep(K_MSEC(1000));
    }
    return 0;
}