# Host-side tests

The test uses a fake motor and an injected clock, so it does not require STM32 HAL.
Build it with any C99 compiler from the repository root:

```text
cc -std=c99 -ICore/Inc Core/Src/motor_driver.c Tests/test_motor_runtime.c -o test_motor_runtime
```

The external-input test supplies a fake GPIO port reader and exercises the same
debounce core used by TIM6 and the future FreeRTOS sensor task:

```text
cc -std=c99 -ICore/Inc Core/Src/external_input.c Tests/test_external_input.c -o test_external_input
```
