### Project Description: STM32F446RE PWM Control for LED and Motor (Direction and Speed Control)

#### Overview:
This project demonstrates the use of Pulse Width Modulation (PWM) on the STM32F446RE microcontroller to control both an LED and a DC motor. The PWM technique is used to vary the brightness of the LED and the speed of the motor. Additionally, the direction of the motor is controlled using GPIO pins configured as outputs.
### What is PWM?
PWM stands for Pulse Width Modulation. It’s a technique used to control the amount of power delivered to an electrical device by rapidly switching it on and off. By varying the width of the "on" time (duty cycle) relative to the "off" time, PWM effectively simulates an analog signal using a digital output.
### How Does PWM Work?
## Duty Cycle:

The duty cycle is the percentage of time the signal is "on" (high) compared to the total period of the signal.

For example, a 50% duty cycle means the signal is high for half the time and low for the other half.

## Frequency:

The frequency of the PWM signal determines how fast the signal switches between on and off states.

Higher frequencies result in smoother control but may require more processing power.

## Analog Simulation:

By adjusting the duty cycle, PWM can simulate varying voltage levels. For example:

A 25% duty cycle simulates a lower voltage.

A 75% duty cycle simulates a higher voltage.
![image](https://github.com/user-attachments/assets/606b649f-2a59-40e3-94e8-de2814b02aad)


### Why is PWM Important?
# Efficiency: 
PWM is highly efficient because it minimizes power loss by switching devices fully on or off, rather than operating them in a partially on state.

# Precision: 
It allows for precise control of devices like motors and LEDs.

# Versatility: 
PWM can be used in a wide range of applications, from simple LED dimming to complex motor control systems.

### PWM in Microcontrollers
Microcontrollers like the STM32 have built-in hardware timers that can generate PWM signals. This makes it easy to implement PWM without needing external components. 
#### Key Features:
1. **PWM Generation**: The STM32F446RE's timer peripheral is used to generate PWM signals.
2. **LED Control**: The PWM signal is applied to an LED to control its brightness.
3. **Motor Control**:
   - **Speed Control**: The PWM signal controls the speed of the DC motor.
   - **Direction Control**: GPIO pins are used to control the direction of the motor (forward/reverse).
4. **User Input**: Buttons or a potentiometer can be used to adjust the duty cycle of the PWM signal, thereby controlling the brightness of the LED and the speed of the motor.

#### Hardware Components:
- **STM32F446RE Nucleo Board**: The main microcontroller unit.
- **LED**: Connected to a PWM-capable GPIO pin.
- **DC Motor**: Connected to an H-Bridge motor driver (e.g., L298N) for direction and speed control.
- **Potentiometer**: To adjust the PWM duty cycle (optional).
- **Power Supply**: Appropriate power supply for the motor and microcontroller.(i have just used my computer as a power supply!!!)

#### Software Implementation:
1. **PWM Configuration**:
   - Configure one of the STM32F446RE's timers (e.g., TIM2) in PWM mode.
   - Set the timer's prescaler and auto-reload register to achieve the desired PWM frequency.
   - Configure a GPIO pin as an alternate function to output the PWM signal.

2. **LED Control**:
   - Connect the LED to the PWM output pin.
   - Vary the duty cycle of the PWM signal to control the LED brightness.

3. **Motor Control**:
   - **Speed Control**: Connect the PWM signal to the enable pin of the H-Bridge motor driver.
   - **Direction Control**: Use two GPIO pins to control the input pins of the H-Bridge, determining the motor's direction (forward/reverse).

4. **User Input**:
   - If using a potentiometer, configure an ADC channel to read the analog value and map it to the PWM duty cycle.
   - If using buttons, configure GPIO pins as inputs to change the motor direction.

#### Example Code Snippets:

1. **PWM Initialization**:
```c
void PWM_Init(void) {
    TIM_HandleTypeDef htim2;
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 84 - 1; // 84 MHz / 84 = 1 MHz
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 1000 - 1; // 1 MHz / 1000 = 1 kHz PWM frequency
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim2);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 500; // 50% duty cycle
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}
```

2. **Motor Direction Control**:
```c
void Motor_Forward(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);  // IN1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // IN2
}

void Motor_Reverse(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); // IN1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);  // IN2
}
```

3. **Main Loop**:
```c
int main(void) {
    HAL_Init();
    SystemClock_Config();
    PWM_Init();
    GPIO_Init();

    while (1) {
        // Adjust PWM duty cycle based on user input (e.g., potentiometer)
        uint16_t adc_value = HAL_ADC_GetValue(&hadc1);
        uint16_t duty_cycle = (adc_value * 1000) / 4095; // Map ADC value to 0-1000
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty_cycle);

        // Control motor direction based on button press
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET) {
            Motor_Forward();
        } else {
            Motor_Reverse();
        }
    }
}
```

#### Conclusion:
This project showcases the versatility of the STM32F446RE microcontroller in generating PWM signals for controlling both an LED and a DC motor.
By adjusting the PWM duty cycle, the brightness of the LED and the speed of the motor can be controlled. Additionally, the direction of the motor can be easily managed using GPIO pins.
This project serves as a foundation for more complex applications involving motor control and PWM-based signal generation.
If you’re working with microcontrollers or electronics, understanding PWM is essential!
