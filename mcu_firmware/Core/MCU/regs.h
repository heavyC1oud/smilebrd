/**
 * @file regs.h
 */

#ifndef STM32_REGS_H_
#define STM32_REGS_H_

#include <cstdint>

/**
 * @brief Тип описывающий размерность регистра контроллера.
 */
typedef uint32_t mcu_reg_t;

/**
 * @brief Проверка что размер битовой структуры не
 *        превышает размер регистра контроллера (mcu_reg_t)
 */
#define CHECK_REG_SIZE(type_name)                           \
static_assert(sizeof(type_name) == sizeof(mcu_reg_t),       \
        "Size of bits struct is not correct")               \

/**
 * @brief Макрос для описания типов регистров контроллера.
 *        в результате выполнения макроса создается структура вида:
 *        typedef union
 *        {
 *            mcu_reg_t d32;
 *            __packed struct
 *                <описание битовой структуры регистра>
 *        } <имя типа регистра>;
 *
 *        также выполняется проверка, что размер битовой структуры не
 *        превышает размер регистра контроллера (mcu_reg_t)
 *
 * @param type_name: имя типа регистра
 * @param bits_struct: битовая структура регистра -
 *              { <описание битовых полей> },
 *              в формате: <имя поля> : <количество бит>,
 */
#define MCU_REG_TYPE(type_name, bits_struct)                \
typedef union                                               \
{                                                           \
    mcu_reg_t d32;                                          \
    __packed struct                                         \
    bits_struct;                                            \
} type_name;                                                \
CHECK_REG_SIZE(type_name);                                  \


/**
 * @brief Проверка размера структуры описывающей регистры модуля
 */
#define CHECK_REG_MAP(reg_map_type, reg_num)                \
static_assert(sizeof(reg_map_type) ==                       \
    (reg_num * sizeof(mcu_reg_t)),                          \
    "Size is not correct");                                 \

#endif  // STM32_REGS_H_
