#include "bmi088_middleware.h"

#include "stm32f407xx.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include "../communication_topic/spi_topics.hpp"

namespace {
orb::SpiDev s_active = orb::SpiDev::None;

osEventFlagsId_t s_spi_evt = nullptr;
StaticEventGroup_t s_spi_evt_cb{};
osEventFlagsAttr_t s_spi_evt_attr{};

constexpr uint32_t kSpiDoneBit = 1u << 0;

inline void spi_publish(const orb::SpiTxFrame& f)
{
    orb::spi_tx.publish(f);
}

inline bool spi_transceive_byte(orb::SpiDev dev, uint8_t tx, uint8_t& rx, uint32_t timeout_ms)
{
    if (dev == orb::SpiDev::None || s_spi_evt == nullptr) {
        return false;
    }

    bool ok = false;
    rx = 0;

    (void)osEventFlagsClear(s_spi_evt, kSpiDoneBit);

    orb::SpiTxFrame f{};
    f.dev = dev;
    f.op = orb::SpiOp::Transceive;
    f.tx = &tx;
    f.rx = &rx;
    f.len = 1;
    f.timeout_ms = timeout_ms;
    f.done_evt = s_spi_evt;
    f.done_mask = kSpiDoneBit;
    f.ok = &ok;
    spi_publish(f);

    // 事件等待单位为 kernel ticks。这里用一个保守的上限，避免 SPI task 未启动时卡死。
    // 事务本身在 SpiTxTask 内是阻塞调用，通常会很快完成。
    constexpr uint32_t kWaitTicks = 20;
    const uint32_t r = osEventFlagsWait(s_spi_evt, kSpiDoneBit, osFlagsWaitAny, kWaitTicks);
    if ((r & kSpiDoneBit) == 0u) {
        return false;
    }
    return ok;
}
}  // namespace

/**
************************************************************************
* @brief:      	BMI088_GPIO_init(void)
* @param:       void
* @retval:     	void
* @details:    	BMI088������GPIO��ʼ������
************************************************************************
**/
void BMI088_GPIO_init(void)
{
    // GPIO 已由 Board 层完成初始化
}
/**
************************************************************************
* @brief:      	BMI088_com_init(void)
* @param:       void
* @retval:     	void
* @details:    	BMI088������ͨ�ų�ʼ������
************************************************************************
**/
void BMI088_com_init(void)
{
    if (s_spi_evt != nullptr) {
        return;
    }

    s_spi_evt_attr = osEventFlagsAttr_t{
        .name = "bmi088_spi_evt",
        .cb_mem = &s_spi_evt_cb,
        .cb_size = sizeof(s_spi_evt_cb),
    };
    s_spi_evt = osEventFlagsNew(&s_spi_evt_attr);
    configASSERT(s_spi_evt != nullptr);

}
/**
************************************************************************
* @brief:      	BMI088_delay_ms(uint16_t ms)
* @param:       ms - Ҫ�ӳٵĺ�����
* @retval:     	void
* @details:    	�ӳ�ָ���������ĺ���������΢���ӳ�ʵ��
************************************************************************
**/
void BMI088_delay_ms(uint16_t ms)
{
    while(ms--)
    {
        BMI088_delay_us(1000);
    }
}
/**
************************************************************************
* @brief:      	BMI088_delay_us(uint16_t us)
* @param:       us - Ҫ�ӳٵ�΢����
* @retval:     	void
* @details:    	΢�뼶�ӳٺ�����ʹ��SysTick��ʱ��ʵ��
************************************************************************
**/
void BMI088_delay_us(uint16_t us)
{

    uint32_t ticks = 0;
    uint32_t told = 0;
    uint32_t tnow = 0;
    uint32_t tcnt = 0;
    uint32_t reload = 0;
    reload = SysTick->LOAD;
    ticks = us * 168;
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
}
/**
************************************************************************
* @brief:      	BMI088_ACCEL_NS_L(void)
* @param:       void
* @retval:     	void
* @details:    	��BMI088���ٶȼ�Ƭѡ�ź��õͣ�ʹ�䴦��ѡ��״̬
************************************************************************
**/
void BMI088_ACCEL_NS_L(void)
{
    s_active = orb::SpiDev::BMI088_ACCEL;

    orb::SpiTxFrame f{};
    f.dev = s_active;
    f.op = orb::SpiOp::Select;
    spi_publish(f);
}
/**
************************************************************************
* @brief:      	BMI088_ACCEL_NS_H(void)
* @param:       void
* @retval:     	void
* @details:    	��BMI088���ٶȼ�Ƭѡ�ź��øߣ�ʹ�䴦�ڷ�ѡ��״̬
************************************************************************
**/
void BMI088_ACCEL_NS_H(void)
{
    if (s_active != orb::SpiDev::None) {
        orb::SpiTxFrame f{};
        f.dev = s_active;
        f.op = orb::SpiOp::Deselect;
        spi_publish(f);
    }
    s_active = orb::SpiDev::None;
}
/**
************************************************************************
* @brief:      	BMI088_GYRO_NS_L(void)
* @param:       void
* @retval:     	void
* @details:    	��BMI088������Ƭѡ�ź��õͣ�ʹ�䴦��ѡ��״̬
************************************************************************
**/
void BMI088_GYRO_NS_L(void)
{
    s_active = orb::SpiDev::BMI088_GYRO;

    orb::SpiTxFrame f{};
    f.dev = s_active;
    f.op = orb::SpiOp::Select;
    spi_publish(f);
}
/**
************************************************************************
* @brief:      	BMI088_GYRO_NS_H(void)
* @param:       void
* @retval:     	void
* @details:    	��BMI088������Ƭѡ�ź��øߣ�ʹ�䴦�ڷ�ѡ��״̬
************************************************************************
**/
void BMI088_GYRO_NS_H(void)
{
    if (s_active != orb::SpiDev::None) {
        orb::SpiTxFrame f{};
        f.dev = s_active;
        f.op = orb::SpiOp::Deselect;
        spi_publish(f);
    }
    s_active = orb::SpiDev::None;
}
/**
************************************************************************
* @brief:      	BMI088_read_write_byte(uint8_t txdata)
* @param:       txdata - Ҫ���͵�����
* @retval:     	uint8_t - ���յ�������
* @details:    	ͨ��BMI088ʹ�õ�SPI���߽��е��ֽڵĶ�д����
************************************************************************
**/
uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data = 0;
    (void)spi_transceive_byte(s_active, txdata, rx_data, 1);
    return rx_data;
}

