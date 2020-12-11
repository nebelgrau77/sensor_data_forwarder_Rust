//! print accelerometer and gyroscope data over serial in a format required by EdgeImpulse data forwarder

#![no_std]
#![no_main]

use panic_halt as _;

use rtic::app;

use stm32f4xx_hal::{
    gpio::*,
    i2c::I2c, 
    prelude::*, 
    stm32,
    stm32::USART2,
    delay::Delay,
    timer::{Event as TimerEvent, Timer},    
    serial,
    serial::{
        Serial,
        config::Config},
    };

use mpu6050::*;

use core::fmt::Write;

const BOOT_DELAY_MS: u16 = 100; 

const CONVERT_G_TO_MS2: f32 = 9.80665;

const FREQUENCY: u32 = 50; //data acquisition frequency in Hertz

type Sensor = mpu6050::Mpu6050<stm32f4xx_hal::i2c::I2c<stm32::I2C1, (gpiob::PB8<AlternateOD<stm32f4xx_hal::gpio::AF4>>, gpiob::PB9<AlternateOD<stm32f4xx_hal::gpio::AF4>>)>, Delay>;

#[app(device = stm32f4xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        led: gpioc::PC13<Output<PushPull>>,
        timer: Timer<stm32::TIM2>,
        tx: serial::Tx<USART2>,
        mpu: Sensor,        
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        let device = cx.device;
        let core = cx.core;

        //set up the clocks
        let rcc = device.RCC.constrain();
        let clocks = rcc.cfgr.use_hse(25.mhz()).sysclk(100.mhz()).pclk1(25.mhz()).freeze();

        // set up GPIOs
        let gpioa = device.GPIOA.split();
        let gpiob = device.GPIOB.split();
        let gpioc = device.GPIOC.split();
    
        //LED for blinking as an indicator of UART activity
        let led = gpioc.pc13.into_push_pull_output();

        //configure serial USART2
        let tx_pin = gpioa.pa2.into_alternate_af7();
        let rx_pin = gpioa.pa3.into_alternate_af7();

        let serial = Serial::usart2(
            device.USART2,
            (tx_pin, rx_pin),
            Config::default().baudrate(115200.bps()),
            clocks,
            ).unwrap();
    
        let (tx, mut _rx) = serial.split();

        //delay provider
        let mut delay = Delay::new(core.SYST, clocks);

        //small delay for a correct I2C bus initialization
        delay.delay_ms(BOOT_DELAY_MS);

        //set up I2C
        let scl = gpiob.pb8.into_alternate_af4().set_open_drain();
        let sda = gpiob.pb9.into_alternate_af4().set_open_drain();
        let i2c = I2c::i2c1(device.I2C1, (scl, sda), 400.khz(), clocks);

        //set up the MPU6050 sensor
        let mut mpu = Mpu6050::new(i2c, delay);
        mpu.init().unwrap();
        mpu.soft_calib(Steps(100)).unwrap();
        mpu.calc_variance(Steps(50)).unwrap();

        //set up the timer
        let mut timer = Timer::tim2(device.TIM2, FREQUENCY.hz(), clocks);
        timer.listen(TimerEvent::TimeOut);

        init::LateResources{led, tx, timer, mpu}

    }

    #[task(binds = TIM2, resources = [led, timer, tx, mpu])]
    fn TIM2(cx: TIM2::Context) {
        
        //clear the interrupt flag
        cx.resources.timer.clear_interrupt(TimerEvent::TimeOut);

        //blink the LED
        cx.resources.led.toggle().unwrap();
            
        //get sensor values
        let accel = cx.resources.mpu.get_acc().unwrap();
        let gyro = cx.resources.mpu.get_gyro().unwrap();

        //print sensor values to serial in a format required by EdgeImpulse data forwarder
        writeln!(cx.resources.tx, "{}\t{}\t{}\t{}\t{}\t{}\r",
                accel[0]*CONVERT_G_TO_MS2, 
                accel[1]*CONVERT_G_TO_MS2,
                accel[2]*CONVERT_G_TO_MS2,
                gyro[0],
                gyro[1],
                gyro[2],
                ).unwrap();

    }

};
