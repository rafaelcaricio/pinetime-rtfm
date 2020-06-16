#![no_main]
#![no_std]

// Panic handler
#[cfg(not(test))]
use panic_rtt_target as _;

use debouncr::{debounce_6, Debouncer, Edge, Repeat6};
use embedded_graphics::prelude::*;
use lvgl::{UI, Align, Widget, Part, State, Color};
use lvgl::widgets::{Label, LabelAlign, Spinner};
use lvgl::style::Style as LvStyle;
use embedded_graphics::{
    fonts::{Font12x16, Text},
    image::{Image, ImageRawLE},
    pixelcolor::Rgb565,
    primitives::rectangle::Rectangle,
    style::{PrimitiveStyleBuilder, TextStyleBuilder},
};
use nrf52832_hal::gpio::{p0, Floating, Input, Level, Output, Pin, PushPull};
use nrf52832_hal::prelude::*;
use nrf52832_hal::{self as hal, pac};
use numtoa::NumToA;
use rtfm::app;
use rtt_target::{rprintln, rtt_init_print};
use rubble::config::Config;
use rubble::gatt::BatteryServiceAttrs;
use rubble::l2cap::{BleChannelMap, L2CAPState};
use rubble::link::ad_structure::AdStructure;
use rubble::link::queue::{PacketQueue, SimpleQueue};
use rubble::link::{LinkLayer, Responder, MIN_PDU_BUF};
use rubble::security::NoSecurity;
use rubble::time::{Duration as RubbleDuration, Timer};
use rubble_nrf5x::radio::{BleRadio, PacketBuffer};
use rubble_nrf5x::timer::BleTimer;
use rubble_nrf5x::utils::get_device_address;
use st7789::{self, Orientation};

mod backlight;
mod battery;
mod delay;
mod monotonic_nrf52;

use monotonic_nrf52::U32Ext;

const LCD_W: u16 = 240;
const LCD_H: u16 = 240;

const FERRIS_W: u16 = 86;
const FERRIS_H: u16 = 64;

const MARGIN: u16 = 10;

const BACKGROUND_COLOR: Rgb565 = Rgb565::new(0, 0b000111, 0);

pub struct AppConfig {}

impl Config for AppConfig {
    type Timer = BleTimer<hal::target::TIMER2>;
    type Transmitter = BleRadio;
    type ChannelMapper = BleChannelMap<BatteryServiceAttrs, NoSecurity>;
    type PacketQueue = &'static mut SimpleQueue;
}

#[app(device = nrf52832_hal::pac, peripherals = true, monotonic = crate::monotonic_nrf52::Tim1)]
const APP: () = {
    struct Resources {
        // LCD
        // lcd: st7789::ST7789<
        //     hal::spim::Spim<pac::SPIM1>,
        //     p0::P0_18<Output<PushPull>>,
        //     p0::P0_26<Output<PushPull>>,
        //     delay::TimerDelay,
        // >,
        backlight: backlight::Backlight,

        // Battery
        battery: battery::BatteryStatus,

        // Button
        button: Pin<Input<Floating>>,
        button_debouncer: Debouncer<u8, Repeat6>,

        // Counter resources
        #[init(0)]
        counter: usize,

        // LVGL GUI
        ui: UI<
            st7789::ST7789<
                hal::spim::Spim<pac::SPIM1>,
                p0::P0_18<Output<PushPull>>,
                p0::P0_26<Output<PushPull>>,
                delay::TimerDelay,
            >,
            Rgb565>,
        label: Label,
        batt_label: Label,

        // BLE
        #[init([0; MIN_PDU_BUF])]
        ble_tx_buf: PacketBuffer,
        #[init([0; MIN_PDU_BUF])]
        ble_rx_buf: PacketBuffer,
        #[init(SimpleQueue::new())]
        tx_queue: SimpleQueue,
        #[init(SimpleQueue::new())]
        rx_queue: SimpleQueue,
        radio: BleRadio,
        ble_ll: LinkLayer<AppConfig>,
        ble_r: Responder<AppConfig>,
    }

    #[init(
        resources = [ble_tx_buf, ble_rx_buf, tx_queue, rx_queue],
        // spawn = [write_counter, write_ferris, poll_button, show_battery_status, update_battery_status],
        spawn = [write_counter, poll_button, lvgl_tick_inc, lvgl_task_handler, show_battery_status, update_battery_status],
    )]
    fn init(cx: init::Context) -> init::LateResources {
        // Destructure device peripherals
        let pac::Peripherals {
            CLOCK,
            FICR,
            P0,
            RADIO,
            SAADC,
            SPIM1,
            TIMER0,
            TIMER1,
            TIMER2,
            ..
        } = cx.device;

        // Init RTT
        rtt_init_print!();
        rprintln!("Initializing…");

        // Set up clocks. On reset, the high frequency clock is already used,
        // but we also need to switch to the external HF oscillator. This is
        // needed for Bluetooth to work.
        let _clocks = hal::clocks::Clocks::new(CLOCK).enable_ext_hfosc();

        // Set up delay provider on TIMER0
        let delay = delay::TimerDelay::new(TIMER0);

        // Initialize monotonic timer on TIMER1 (for RTFM)
        monotonic_nrf52::Tim1::initialize(TIMER1);

        // Initialize BLE timer on TIMER2
        let ble_timer = BleTimer::init(TIMER2);

        // Set up GPIO peripheral
        let gpio = hal::gpio::p0::Parts::new(P0);

        // Enable backlight
        let backlight = backlight::Backlight::init(
            gpio.p0_14.into_push_pull_output(Level::High).degrade(),
            gpio.p0_22.into_push_pull_output(Level::High).degrade(),
            gpio.p0_23.into_push_pull_output(Level::High).degrade(),
            1,
        );

        // Battery status
        let battery = battery::BatteryStatus::init(
            gpio.p0_12.into_floating_input(),
            gpio.p0_31.into_floating_input(),
            SAADC,
        );

        // Enable button
        gpio.p0_15.into_push_pull_output(Level::High);
        let button = gpio.p0_13.into_floating_input().degrade();

        // Get bluetooth device address
        let device_address = get_device_address();
        rprintln!("Bluetooth device address: {:?}", device_address);

        // Initialize radio
        let mut radio = BleRadio::new(
            RADIO,
            &FICR,
            cx.resources.ble_tx_buf,
            cx.resources.ble_rx_buf,
        );

        // Create bluetooth TX/RX queues
        let (tx, tx_cons) = cx.resources.tx_queue.split();
        let (rx_prod, rx) = cx.resources.rx_queue.split();

        // Create the actual BLE stack objects
        let mut ble_ll = LinkLayer::<AppConfig>::new(device_address, ble_timer);
        let ble_r = Responder::<AppConfig>::new(
            tx,
            rx,
            L2CAPState::new(BleChannelMap::with_attributes(BatteryServiceAttrs::new())),
        );

        // Send advertisement and set up regular interrupt
        let next_update = ble_ll
            .start_advertise(
                RubbleDuration::from_millis(200),
                &[AdStructure::CompleteLocalName("Rusty PineTime")],
                &mut radio,
                tx_cons,
                rx_prod,
            )
            .unwrap();
        ble_ll.timer().configure_interrupt(next_update);

        // Set up SPI pins
        let spi_clk = gpio.p0_02.into_push_pull_output(Level::Low).degrade();
        let spi_mosi = gpio.p0_03.into_push_pull_output(Level::Low).degrade();
        let spi_miso = gpio.p0_04.into_floating_input().degrade();
        let spi_pins = hal::spim::Pins {
            sck: spi_clk,
            miso: Some(spi_miso),
            mosi: Some(spi_mosi),
        };

        // Set up LCD pins
        // LCD_CS (P0.25): Chip select
        let mut lcd_cs = gpio.p0_25.into_push_pull_output(Level::Low);
        // LCD_RS (P0.18): Data/clock pin
        let lcd_dc = gpio.p0_18.into_push_pull_output(Level::Low);
        // LCD_RESET (P0.26): Display reset
        let lcd_rst = gpio.p0_26.into_push_pull_output(Level::Low);

        // Initialize SPI
        let spi = hal::Spim::new(
            SPIM1,
            spi_pins,
            // Use SPI at 8MHz (the fastest clock available on the nRF52832)
            // because otherwise refreshing will be super slow.
            hal::spim::Frequency::M8,
            // SPI must be used in mode 3. Mode 0 (the default) won't work.
            hal::spim::MODE_3,
            0,
        );

        // Chip select must be held low while driving the display. It must be high
        // when using other SPI devices on the same bus (such as external flash
        // storage) so that the display controller won't respond to the wrong
        // commands.
        lcd_cs.set_low().unwrap();

        // Initialize LCD
        let mut lcd = st7789::ST7789::new(spi, lcd_dc, lcd_rst, LCD_W, LCD_H, delay);
        lcd.init().unwrap();
        lcd.set_orientation(&Orientation::Portrait).unwrap();

        let mut ui = UI::init().unwrap();

        ui.disp_drv_register(lcd);

        let mut scr = ui.scr_act().unwrap();
        
        let mut screen_style = LvStyle::default();
        screen_style.set_bg_color(State::DEFAULT, Color::from_rgb((0, 50, 140)));
        scr.add_style(Part::Main, screen_style).unwrap();

        let mut spinner = Spinner::new(&mut scr).unwrap();
        spinner.set_size(100, 100).unwrap();
        spinner.set_align(&mut scr, Align::Center, 0, 0).unwrap();

        let mut label = Label::new(&mut scr).unwrap();
        label.set_text(cstr_core::CStr::from_bytes_with_nul("It's alive!\0".as_bytes()).unwrap()).unwrap();
        label.set_align(&mut spinner, Align::OutTopMid, 0, -15).unwrap();

        let mut counter_style = LvStyle::default();
        counter_style.set_text_color(State::DEFAULT, Color::from_rgb((255, 255, 255)));
        label.add_style(Part::Main, counter_style).unwrap();

        let mut device_label_style = LvStyle::default();
        device_label_style.set_text_color(State::DEFAULT, Color::from_rgb((255, 255, 255)));

        let mut device_label = Label::new(&mut scr).unwrap();
        device_label.set_text(cstr_core::CStr::from_bytes_with_nul("Pinetime\0".as_bytes()).unwrap()).unwrap();
        device_label.add_style(Part::Main, device_label_style).unwrap();
        device_label.set_align(&mut scr, Align::InTopLeft, 0, 0).unwrap();

        let mut batt_label_style = LvStyle::default();
        batt_label_style.set_text_color(State::DEFAULT, Color::from_rgb((255, 255, 255)));

        let mut batt_label = Label::new(&mut scr).unwrap();
        batt_label.set_text(cstr_core::CStr::from_bytes_with_nul("Bat LVL\0".as_bytes()).unwrap()).unwrap();
        batt_label.add_style(Part::Main, batt_label_style).unwrap();
        batt_label.set_align(&mut scr, Align::InTopRight, 0, 0).unwrap();


        // Schedule tasks immediately
        cx.spawn.write_counter().unwrap();
        cx.spawn.lvgl_tick_inc().unwrap();
        cx.spawn.lvgl_task_handler().unwrap();
        cx.spawn.poll_button().unwrap();
        cx.spawn.show_battery_status().unwrap();
        cx.spawn.update_battery_status().unwrap();

        init::LateResources {
            battery,
            backlight,
            button,
            button_debouncer: debounce_6(),
            //text_style,
            //ferris,
            ui,
            label,
            batt_label,

            radio,
            ble_ll,
            ble_r,
        }
    }

    /// Hook up the RADIO interrupt to the Rubble BLE stack.
    #[task(binds = RADIO, resources = [radio, ble_ll], spawn = [ble_worker], priority = 3)]
    fn radio(cx: radio::Context) {
        let ble_ll: &mut LinkLayer<AppConfig> = cx.resources.ble_ll;
        if let Some(cmd) = cx
            .resources
            .radio
            .recv_interrupt(ble_ll.timer().now(), ble_ll)
        {
            cx.resources.radio.configure_receiver(cmd.radio);
            ble_ll.timer().configure_interrupt(cmd.next_update);

            if cmd.queued_work {
                // If there's any lower-priority work to be done, ensure that happens.
                // If we fail to spawn the task, it's already scheduled.
                cx.spawn.ble_worker().ok();
            }
        }
    }

    /// Hook up the TIMER2 interrupt to the Rubble BLE stack.
    #[task(binds = TIMER2, resources = [radio, ble_ll], spawn = [ble_worker], priority = 3)]
    fn timer2(cx: timer2::Context) {
        let timer = cx.resources.ble_ll.timer();
        if !timer.is_interrupt_pending() {
            return;
        }
        timer.clear_interrupt();

        let cmd = cx.resources.ble_ll.update_timer(&mut *cx.resources.radio);
        cx.resources.radio.configure_receiver(cmd.radio);

        cx.resources
            .ble_ll
            .timer()
            .configure_interrupt(cmd.next_update);

        if cmd.queued_work {
            // If there's any lower-priority work to be done, ensure that happens.
            // If we fail to spawn the task, it's already scheduled.
            cx.spawn.ble_worker().ok();
        }
    }

    /// Lower-priority task spawned from RADIO and TIMER2 interrupts.
    #[task(resources = [ble_r], priority = 2)]
    fn ble_worker(cx: ble_worker::Context) {
        // Fully drain the packet queue
        while cx.resources.ble_r.has_work() {
            cx.resources.ble_r.process_one().unwrap();
        }
    }

    #[task(resources = [ui], schedule = [lvgl_tick_inc])]
    fn lvgl_tick_inc(cx: lvgl_tick_inc::Context) {
        let ui = cx.resources.ui;
        ui.tick_inc(core::time::Duration::from_millis(5));

        cx.schedule.lvgl_tick_inc(cx.scheduled + 5.millis()).unwrap();
    }

    #[task(resources = [ui], schedule = [lvgl_task_handler])]
    fn lvgl_task_handler(cx: lvgl_task_handler::Context) {
        let ui = cx.resources.ui;
        ui.task_handler();

        cx.schedule.lvgl_task_handler(cx.scheduled + 1.millis()).unwrap();
    }

    #[task(resources = [label, counter], schedule = [write_counter])]
    fn write_counter(cx: write_counter::Context) {
        rprintln!("Counter is {}", cx.resources.counter);

        // Write counter to the display
        // let mut buf = [0u8; 20];
        // let text = cx.resources.counter.numtoa_str(10, &mut buf);

        // let counter: &mut lvgl::widgets::Label = cx.resources.label;
        // counter.set_text(cstr_core::CStr::from_bytes_with_nul(text.as_bytes()).unwrap()).unwrap();

        // Increment counter
        *cx.resources.counter += 1;

        // Re-schedule the timer interrupt
        cx.schedule.write_counter(cx.scheduled + 1.secs()).unwrap();
    }

    #[task(resources = [button, button_debouncer], spawn = [button_pressed], schedule = [poll_button])]
    fn poll_button(cx: poll_button::Context) {
        // Poll button
        let pressed = cx.resources.button.is_high().unwrap();
        let edge = cx.resources.button_debouncer.update(pressed);

        // Dispatch event
        if edge == Some(Edge::Rising) {
            cx.spawn.button_pressed().unwrap();
        }

        // Re-schedule the timer interrupt in 2ms
        cx.schedule.poll_button(cx.scheduled + 2.millis()).unwrap();
    }

    /// Called when button is pressed without bouncing for 12 (6 * 2) ms.
    #[task(resources = [backlight])]
    fn button_pressed(cx: button_pressed::Context) {
        if cx.resources.backlight.get_brightness() < 7 {
            cx.resources.backlight.brighter();
        } else {
            cx.resources.backlight.off();
        }
    }

    /// Fetch the battery status from the hardware. Update the text if
    /// something changed.
    #[task(resources = [batt_label, battery], spawn = [show_battery_status], schedule = [update_battery_status])]
    fn update_battery_status(cx: update_battery_status::Context) {
        rprintln!("Update battery status");
    
        let changed = cx.resources.battery.update();
        if changed {
            rprintln!("Battery status changed");
            cx.spawn.show_battery_status().unwrap();
        }
    
        // Re-schedule the timer interrupt in 1s
        cx.schedule
            .update_battery_status(cx.scheduled + 1.secs())
            .unwrap();
    }

    /// Show the battery status on the LCD.
    #[task(resources = [batt_label, battery])]
    fn show_battery_status(cx: show_battery_status::Context) {
        let voltage = cx.resources.battery.voltage();
        let charging = cx.resources.battery.is_charging();
    
        rprintln!(
            "Battery status: {} ({})",
            voltage,
            if charging { "charging" } else { "discharging" },
        );
    
        // Show battery status in top right corner
        let mut buf = [0u8; 7];
        (voltage / 10).numtoa(10, &mut buf[0..1]);
        buf[1] = b'.';
        (voltage % 10).numtoa(10, &mut buf[2..3]);
        buf[3] = b'V';
        buf[4] = b'/';
        buf[5] = if charging { b'C' } else { b'D' };
        buf[6] = b'\0';
        //let status = core::str::from_utf8(&buf).unwrap();
        unsafe {
            cx.resources.batt_label.set_text(cstr_core::CStr::from_bytes_with_nul_unchecked(&buf)).unwrap();
        };
    }

    // Provide unused interrupts to RTFM for its scheduling
    extern "C" {
        fn SWI0_EGU0();
        fn SWI1_EGU1();
        fn SWI2_EGU2();
        fn SWI3_EGU3();
        fn SWI4_EGU4();
        fn SWI5_EGU5();
    }
};
