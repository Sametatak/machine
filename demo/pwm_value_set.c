void user_pwm_setvalue_mosfet4(int16_t new_target_value, int16_t limit)
{
    static int16_t previous_value = 0; // Static variable to store the last set value
    int16_t step_size = 30; // Define the step size for gradual adjustment

    // Get the target value from modbusDevices[01].data[0]
    new_target_value = modbusDevices[01].data[0] * 2 - limit;

    // Ensure the new target value is within the specified limit
    new_target_value = (new_target_value < limit) ? new_target_value : limit;
    new_target_value = (new_target_value > -limit) ? new_target_value : -limit;

    int16_t difference = new_target_value - previous_value;

    if (difference > step_size) {
        previous_value += step_size; // Increase the value by step size
    } else if (difference < -step_size) {
        previous_value -= step_size; // Decrease the value by step size
    } else {
        previous_value = new_target_value; // Set to target value if difference is within step size
    }

    // If the new target value is 0, set the previous value to 0
    if (new_target_value == 0) {
        previous_value = 0;
    }

    // Set the PWM value using the adjusted previous_value
    int16_t pwm_value = previous_value;
    pwm_value = (pwm_value < limit) ? pwm_value : limit;
    pwm_value = (pwm_value > -limit) ? pwm_value : -limit;

    if (previous_value == 0) {
        pwm_value = 0;
    }

    // Set the GPIO pins and PWM channels based on the calculated pwm_value
    if (pwm_value > 0) {
        HAL_GPIO_WritePin(MOSFET_4_A_H_GPIO_Port, MOSFET_4_A_H_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOSFET_4_B_H_GPIO_Port, MOSFET_4_B_H_Pin, GPIO_PIN_RESET);
    } else if (pwm_value < 0) {
        HAL_GPIO_WritePin(MOSFET_4_A_H_GPIO_Port, MOSFET_4_A_H_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOSFET_4_B_H_GPIO_Port, MOSFET_4_B_H_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(MOSFET_4_A_H_GPIO_Port, MOSFET_4_A_H_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOSFET_4_B_H_GPIO_Port, MOSFET_4_B_H_Pin, GPIO_PIN_RESET);
    }

    TIM_OC_InitTypeDef sConfigOC;
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.Pulse = (pwm_value > 0) ? default_pwm : -pwm_value;

    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

    TIM_OC_InitTypeDef sConfigOC2;
    sConfigOC2.OCMode = TIM_OCMODE_PWM1;
    sConfigOC2.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC2.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC2.Pulse = (pwm_value > 0) ? pwm_value : default_pwm;

    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC2, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}

void user_pwm_setvalue_mosfet3(int16_t new_target_value, int16_t limit)
{
    static int16_t previous_value = 0; // Static variable to store the last set value
    int16_t step_size = 30; // Define the step size for gradual adjustment

    // Get the target value from modbusDevices[01].data[1]
    new_target_value = modbusDevices[01].data[1] * 2 - limit;

    // Ensure the new target value is within the specified limit
    new_target_value = (new_target_value < limit) ? new_target_value : limit;
    new_target_value = (new_target_value > -limit) ? new_target_value : -limit;

    int16_t difference = new_target_value - previous_value;

    if (difference > step_size) {
        previous_value += step_size; // Increase the value by step size
    } else if (difference < -step_size) {
        previous_value -= step_size; // Decrease the value by step size
    } else {
        previous_value = new_target_value; // Set to target value if difference is within step size
    }

    // If the new target value is 0, set the previous value to 0
    if (new_target_value == 0) {
        previous_value = 0;
    }

    // Set the PWM value using the adjusted previous_value
    int16_t pwm_value = previous_value;
    pwm_value = (pwm_value < limit) ? pwm_value : limit;
    pwm_value = (pwm_value > -limit) ? pwm_value : -limit;

    if (previous_value == 0) {
        pwm_value = 0;
    }

    // Set the GPIO pins and PWM channels based on the calculated pwm_value
    if (pwm_value > 0) {
        HAL_GPIO_WritePin(MOSFET_3_A_H_GPIO_Port, MOSFET_3_A_H_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOSFET_3_B_H_GPIO_Port, MOSFET_3_B_H_Pin, GPIO_PIN_RESET);
    } else if (pwm_value < 0) {
        HAL_GPIO_WritePin(MOSFET_3_A_H_GPIO_Port, MOSFET_3_A_H_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOSFET_3_B_H_GPIO_Port, MOSFET_3_B_H_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(MOSFET_3_A_H_GPIO_Port, MOSFET_3_A_H_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOSFET_3_B_H_GPIO_Port, MOSFET_3_B_H_Pin, GPIO_PIN_RESET);
    }

    TIM_OC_InitTypeDef sConfigOC;
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.Pulse = (pwm_value > 0) ? default_pwm : -pwm_value;

    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

    TIM_OC_InitTypeDef sConfigOC2;
    sConfigOC2.OCMode = TIM_OCMODE_PWM1;
    sConfigOC2.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC2.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC2.Pulse = (pwm_value > 0) ? pwm_value : default_pwm;

    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
}

void user_pwm_setvalue_mosfet2(int16_t new_target_value, int16_t limit)
{
    static int16_t previous_value = 0; // Static variable to store the last set value
    int16_t step_size = 30; // Define the step size for gradual adjustment

    // Get the target value from modbusDevices[01].data[2]
    new_target_value = modbusDevices[01].data[2] * 2 - limit;

    // Ensure the new target value is within the specified limit
    new_target_value = (new_target_value < limit) ? new_target_value : limit;
    new_target_value = (new_target_value > -limit) ? new_target_value : -limit;

    int16_t difference = new_target_value - previous_value;

    if (difference > step_size) {
        previous_value += step_size; // Increase the value by step size
    } else if (difference < -step_size) {
        previous_value -= step_size; // Decrease the value by step size
    } else {
        previous_value = new_target_value; // Set to target value if difference is within step size
    }

    // If the new target value is 0, set the previous value to 0
    if (new_target_value == 0) {
        previous_value = 0;
    }

    // Set the PWM value using the adjusted previous_value
    int16_t pwm_value = previous_value;
    pwm_value = (pwm_value < limit) ? pwm_value : limit;
    pwm_value = (pwm_value > -limit) ? pwm_value : -limit;

    if (previous_value == 0) {
        pwm_value = 0;
    }

    // Set the GPIO pins and PWM channels based on the calculated pwm_value
    if (pwm_value > 0) {
        HAL_GPIO_WritePin(MOSFET_2_A_H_GPIO_Port, MOSFET_2_A_H_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOSFET_2_B_H_GPIO_Port, MOSFET_2_B_H_Pin, GPIO_PIN_RESET);
    } else if (pwm_value < 0) {
        HAL_GPIO_WritePin(MOSFET_2_A_H_GPIO_Port, MOSFET_2_A_H_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOSFET_2_B_H_GPIO_Port, MOSFET_2_B_H_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(MOSFET_2_A_H_GPIO_Port, MOSFET_2_A_H_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOSFET_2_B_H_GPIO_Port, MOSFET_2_B_H_Pin, GPIO_PIN_RESET);
    }

    TIM_OC_InitTypeDef sConfigOC;
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.Pulse = (pwm_value > 0) ? default_pwm : -pwm_value;

    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

    TIM_OC_InitTypeDef sConfigOC2;
    sConfigOC2.OCMode = TIM_OCMODE_PWM1;
    sConfigOC2.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC2.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC2.Pulse = (pwm_value > 0) ? pwm_value : default_pwm;

    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC2, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

void user_pwm_setvalue_mosfet1(int16_t new_target_value, int16_t limit)
{
    static int16_t previous_value = 0; // Static variable to store the last set value
    int16_t step_size = 30; // Define the step size for gradual adjustment

    // Get the target value from modbusDevices[01].data[3]
    new_target_value = modbusDevices[01].data[3] * 2 - limit;

    // Ensure the new target value is within the specified limit
    new_target_value = (new_target_value < limit) ? new_target_value : limit;
    new_target_value = (new_target_value > -limit) ? new_target_value : -limit;

    int16_t difference = new_target_value - previous_value;

    if (difference > step_size) {
        previous_value += step_size; // Increase the value by step size
    } else if (difference < -step_size) {
        previous_value -= step_size; // Decrease the value by step size
    } else {
        previous_value = new_target_value; // Set to target value if difference is within step size
    }

    // If the new target value is 0, set the previous value to 0
    if (new_target_value == 0) {
        previous_value = 0;
    }

    // Set the PWM value using the adjusted previous_value
    int16_t pwm_value = previous_value;
    pwm_value = (pwm_value < limit) ? pwm_value : limit;
    pwm_value = (pwm_value > -limit) ? pwm_value : -limit;

    if (previous_value == 0) {
        pwm_value = 0;
    }

    // Set the GPIO pins and PWM channels based on the calculated pwm_value
    if (pwm_value > 0) {
        HAL_GPIO_WritePin(MOSFET_1_A_H_GPIO_Port, MOSFET_1_A_H_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOSFET_1_B_H_GPIO_Port, MOSFET_1_B_H_Pin, GPIO_PIN_RESET);
    } else if (pwm_value < 0) {
        HAL_GPIO_WritePin(MOSFET_1_A_H_GPIO_Port, MOSFET_1_A_H_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOSFET_1_B_H_GPIO_Port, MOSFET_1_B_H_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(MOSFET_1_A_H_GPIO_Port, MOSFET_1_A_H_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOSFET_1_B_H_GPIO_Port, MOSFET_1_B_H_Pin, GPIO_PIN_RESET);
    }

    TIM_OC_InitTypeDef sConfigOC;
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.Pulse = (pwm_value > 0) ? defaut_pwm : -pwm_value;

    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    TIM_OC_InitTypeDef sConfigOC2;
    sConfigOC2.OCMode = TIM_OCMODE_PWM1;
    sConfigOC2.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC2.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC2.Pulse = (pwm_value > 0) ? pwm_value : default_pwm;

    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}

