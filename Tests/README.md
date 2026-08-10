# Host-side motor runtime test

The test uses a fake motor and an injected clock, so it does not require STM32 HAL.
Build it with any C99 compiler from the repository root:

```text
cc -std=c99 -ICore/Inc Core/Src/motor_driver.c Tests/test_motor_runtime.c -o test_motor_runtime
```
