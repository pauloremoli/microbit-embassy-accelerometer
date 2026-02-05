#![no_std]
#![no_main]

use defmt::{error, info};
use embassy_executor::Spawner;
use embassy_nrf::twim::{self, Config, Twim};
use embassy_nrf::{bind_interrupts, peripherals::TWISPI0};
use embassy_time::{Delay, Duration, Timer};
use lsm303agr::{AccelMode, AccelOutputDataRate};
use {defmt_rtt as _, panic_probe as _};

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;

use static_cell::{ConstStaticCell, StaticCell};

static I2C_BUS: StaticCell<Mutex<NoopRawMutex, Twim>> = StaticCell::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_nrf::init(Default::default());
    bind_interrupts!(struct Irqs {
        TWISPI0 => twim::InterruptHandler<TWISPI0>;
    });

    static RAM_BUFFER: ConstStaticCell<[u8; 16]> = ConstStaticCell::new([0; 16]);

    info!("Configuring I2C shared bus for accelerometer");
    let i2c = embassy_nrf::twim::Twim::new(
        p.TWISPI0,
        Irqs,
        p.P0_16,
        p.P0_08,
        Config::default(),
        RAM_BUFFER.take(),
    );

    let i2c_bus = I2C_BUS.init(Mutex::new(i2c));
    let mut accelerometer = lsm303agr::Lsm303agr::new_with_i2c(I2cDevice::new(i2c_bus));
    accelerometer
        .init()
        .await
        .expect("Failed to init accelerometer senser");

    accelerometer
        .set_accel_mode_and_odr(&mut Delay, AccelMode::Normal, AccelOutputDataRate::Hz100)
        .await
        .expect("Failed to set mode and odr");

    loop {
        match accelerometer.accel_status().await {
            Ok(status) => {
                if status.xyz_new_data() {
                    let data = accelerometer.acceleration().await.unwrap();

                    info!("{:?}", data.xyz_mg());
                }
            }
            Err(_) => {
                error!("Failed to retrieve accelerometer status");
            }
        }
        Timer::after(Duration::from_millis(1000)).await;
    }
}
