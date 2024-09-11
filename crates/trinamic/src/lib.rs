#[macro_use]
extern crate structure;

use eyre::{self as anyhow, Result, WrapErr};

pub type FloatType = f64;

#[derive(Debug)]
pub struct Message {
    /// `address` for request, `reply_address` for response
    byte0: u8,
    /// `command` for request, `module_address` for response
    byte1: u8,
    /// `command_type` for request, `status` for response
    byte2: u8,
    /// `motor_bank` for request, `command_number` for response
    byte3: u8,
    value: i32,
    checksum: u8,
}

impl Message {
    pub fn new(byte0: u8, byte1: u8, byte2: u8, byte3: u8, value: i32) -> Self {
        let mut result = Self {
            byte0,
            byte1,
            byte2,
            byte3,
            value,
            checksum: 0,
        };
        result.calculate_checksum();
        result
    }

    pub fn ax_param(byte0: u8, arg: AxisParameters, byte3: u8, value: i32) -> Self {
        let byte1 = 6;
        let byte2 = arg.into();
        Self::new(byte0, byte1, byte2, byte3, value)
    }

    pub fn set_ax_param(byte0: u8, arg: AxisParameters, byte3: u8, value: i32) -> Self {
        let byte1 = 5;
        let byte2 = arg.into();
        Self::new(byte0, byte1, byte2, byte3, value)
    }

    pub fn from_buffer(data: &[u8; 9]) -> Result<Self> {
        #[allow(clippy::too_many_arguments)]
        let request_struct = structure!(">BBBBiB").unpack(data)?;
        let expected_checksum = request_struct.5;
        let decoded = Message::new(
            request_struct.0,
            request_struct.1,
            request_struct.2,
            request_struct.3,
            request_struct.4,
        );
        if decoded.checksum != expected_checksum {
            anyhow::bail!("checksum mismatch");
        }
        Ok(decoded)
    }

    fn calculate_checksum(&mut self) -> u8 {
        let buf = self.to_buffer();
        let data = &buf[..buf.len() - 1];
        let mut checksum: u8 = 0;
        for d in data.iter() {
            checksum = checksum.wrapping_add(*d);
        }
        self.checksum = checksum;
        checksum
    }

    pub fn to_buffer(&self) -> Vec<u8> {
        #[allow(clippy::too_many_arguments)]
        structure!(">BBBBiB")
            .pack(
                self.byte0,
                self.byte1,
                self.byte2,
                self.byte3,
                self.value,
                self.checksum,
            )
            .unwrap()
    }
}

impl std::fmt::Display for Message {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> Result<(), std::fmt::Error> {
        write!(
            f,
            "TMCL Message: {:02X},{:02X},{:02X},{:02X},{:08X},{:02X}",
            self.byte0, self.byte1, self.byte2, self.byte3, self.value, self.checksum
        )
    }
}

pub enum AxisParameters {
    MaximumPositioningSpeed,
    MaximumAcceleration,
    MaximumCurrent,
    StandbyCurrent,
    AccelerationA1,
    VelocityV1,
    MaximumDeceleration,
    DecelerationD1,
    ActualLoadValue,
    SGThreshold,
    StopOnStallVelocity,
    StartVelocity,
    StopVelocity,
    ActualPosition,
    EncoderPosition,
    MicrostepResolution,
}

impl From<AxisParameters> for u8 {
    fn from(val: AxisParameters) -> Self {
        use AxisParameters::*;
        match val {
            ActualPosition => 1,
            MaximumPositioningSpeed => 4,
            MaximumAcceleration => 5,
            MaximumCurrent => 6,
            StandbyCurrent => 7,
            AccelerationA1 => 15,
            VelocityV1 => 16,
            MaximumDeceleration => 17,
            DecelerationD1 => 18,
            ActualLoadValue => 206,
            SGThreshold => 174,
            StopOnStallVelocity => 181,
            StartVelocity => 19,
            StopVelocity => 20,
            EncoderPosition => 209,
            MicrostepResolution => 140,
        }
    }
}

#[derive(Clone)]
pub struct TMCM1240Parameters {
    /// microsteps per second, max = 7_999_774
    pub int_speed_limit: i32,
    /// microsteps per square-second, max=7_629_278
    pub int_acceleration: i32,
    pub idle_current: i32,
    pub max_current: i32,
}
#[derive(Clone)]
pub struct TMCM1141Parameters {
    /// [it's complicated], max = 2047
    pub int_speed_limit: i32,
    /// [it's complicted], max = 2047
    pub int_accelertion: i32,
    pub pulse_div: i32,
    pub ramp_div: i32,
    pub idle_current: i32,
    pub max_current: i32,
    pub microstep_resolution: i32,
}

#[derive(Clone)]
pub enum MotorParameters {
    TMCM1240(TMCM1240Parameters),
    TMCM1141(TMCM1141Parameters),
}

impl Default for TMCM1240Parameters {
    fn default() -> Self {
        Self {
            int_speed_limit: 1_000_000,  //focus: 1300
            int_acceleration: 5_000_000, //focus: 2000
            idle_current: 50,            //focus: 110
            max_current: 150,            //focus: 110
        }
    }
}

impl Default for TMCM1141Parameters {
    fn default() -> Self {
        Self {
            int_speed_limit: 1300,
            int_accelertion: 2000,
            pulse_div: 3,
            ramp_div: 7,
            idle_current: 110,
            max_current: 110,
            microstep_resolution: 3, // 3 <-> 1/8 microstepping
        }
    }
}
pub trait TMCMParameters {
    ///returns speed limit in microsteps per second
    fn speed_limit(&self) -> FloatType;
    ///returns acceleration in microsteps per s^2
    fn acceleration(&self) -> FloatType;
    //pub async fn apply(&self, &mut device: Motor) -> Result<()>; //not supported by rust
}

impl TMCMParameters for TMCM1141Parameters {
    fn speed_limit(&self) -> FloatType {
        //from tmcm-1141-hardware-manual-hw1.30-rev1.07, section 9.1
        let f_clk = 16.0e6 as FloatType;
        f_clk * (self.int_speed_limit as FloatType)
            / ((2.0 as FloatType).powi(self.pulse_div) * 2048.0 * 32.0)
    }
    fn acceleration(&self) -> FloatType {
        //from tmcm-1141-hardware-manual-hw1.30-rev1.07, section 9.1
        let f_clk = 16.0e6 as FloatType;
        f_clk.powi(2) * (self.int_accelertion as FloatType)
            / ((2.0 as FloatType).powi(self.pulse_div + self.ramp_div + 29))
    }
}

impl TMCM1141Parameters {
    pub async fn apply(&self, device: &mut Motor) -> Result<()> {
        use AxisParameters::*;

        device
            .request(&Message::set_ax_param(
                1,
                MaximumAcceleration,
                0,
                self.int_accelertion,
            ))
            .await?;

        device
            .request(&Message::set_ax_param(
                1,
                MaximumPositioningSpeed,
                0,
                self.int_speed_limit,
            ))
            .await?;

        device
            .request(&Message::set_ax_param(
                1,
                MaximumCurrent,
                0,
                self.max_current,
            ))
            .await?;

        device
            .request(&Message::set_ax_param(
                1,
                StandbyCurrent,
                0,
                self.idle_current,
            ))
            .await?;

        device
            .request(&Message::set_ax_param(1, SGThreshold, 0, 0))
            .await?;

        device
            .request(&Message::set_ax_param(
                1,
                MicrostepResolution,
                0,
                self.microstep_resolution,
            ))
            .await?;
        Ok(())
    }
}

impl TMCMParameters for TMCM1240Parameters {
    fn speed_limit(&self) -> FloatType {
        self.int_speed_limit as FloatType
    }
    fn acceleration(&self) -> FloatType {
        self.int_acceleration as FloatType
    }
}
impl TMCM1240Parameters {
    pub async fn apply(&self, device: &mut Motor) -> Result<()> {
        use AxisParameters::*;

        // See https://www.trinamic.com/fileadmin/assets/Products/Modules_Documents/TMCM-1240_TMCL_firmware_manual_Fw1.45_Rev1.02.pdf

        device
            .request(&Message::set_ax_param(
                1,
                MaximumDeceleration,
                0,
                self.int_acceleration,
            ))
            .await?;

        device
            .request(&Message::set_ax_param(
                1,
                DecelerationD1,
                0,
                self.int_acceleration,
            ))
            .await?;

        device
            .request(&Message::set_ax_param(
                1,
                MaximumAcceleration,
                0,
                self.int_acceleration,
            ))
            .await?;

        device
            .request(&Message::set_ax_param(
                1,
                AccelerationA1,
                0,
                self.int_acceleration,
            ))
            .await?;

        device
            .request(&Message::set_ax_param(
                1,
                VelocityV1,
                0,
                self.int_speed_limit,
            ))
            .await?;

        device
            .request(&Message::set_ax_param(
                1,
                MaximumPositioningSpeed,
                0,
                self.int_speed_limit,
            ))
            .await?;

        device
            .request(&Message::set_ax_param(
                1,
                MaximumCurrent,
                0,
                self.max_current,
            ))
            .await?;

        device
            .request(&Message::set_ax_param(
                1,
                StandbyCurrent,
                0,
                self.idle_current,
            ))
            .await?;

        device
            .request(&Message::set_ax_param(1, SGThreshold, 0, 0))
            .await?;

        device
            .request(&Message::set_ax_param(1, StopOnStallVelocity, 0, 0))
            .await?;

        device
            .request(&Message::set_ax_param(1, StartVelocity, 0, 0))
            .await?;

        device
            .request(&Message::set_ax_param(1, StopVelocity, 0, 1))
            .await?;

        Ok(())
    }
}

//#FIXME: there must be a way to avoid this block
impl MotorParameters {
    pub fn acceleration(&self) -> FloatType {
        match self {
            Self::TMCM1240(params) => params.acceleration(),
            Self::TMCM1141(params) => params.acceleration(),
        }
    }
    pub fn speed_limit(&self) -> FloatType {
        match self {
            Self::TMCM1240(params) => params.speed_limit(),
            Self::TMCM1141(params) => params.speed_limit(),
        }
    }
    pub async fn apply(&self, device: &mut Motor) -> Result<()> {
        match self {
            Self::TMCM1240(params) => params.apply(device).await,
            Self::TMCM1141(params) => params.apply(device).await,
        }
    }
}

pub struct Motor {
    device: tokio_serial::SerialStream,
    pub config: MotorParameters,
}

impl Motor {
    pub async fn new(device_name: &str, baud_rate: u32, config: MotorParameters) -> Result<Self> {
        use tokio_serial::SerialPortBuilderExt;

        let device = tokio_serial::new(device_name, baud_rate)
            .open_native_async()
            .with_context(|| format!("Failed to open pan serial device {}", device_name))?;

        let mut device = Self {
            device,
            config: config.clone(),
        };

        config.apply(&mut device).await?;

        Ok(device)
    }

    pub async fn request(&mut self, rq: &Message) -> Result<Message> {
        use tokio::io::{AsyncReadExt, AsyncWriteExt};

        // write to the device
        self.device.write_all(&rq.to_buffer()).await?;
        let message = &rq;

        // Read from the device. The manual "TMCM-1240 TMCL™ Firmware Manual • Firmware Version V1.45"
        // says "Do not send the next command before having received the reply!"
        // so we wait for the command to finish here.
        let mut response = [0u8; 9];
        self.device.read_exact(&mut response[..]).await?;

        let response = Message::from_buffer(&response)?;
        // FIXME!: really bad hack to include the 03 ('wrong type') status bit in here.
        //  Focus motor doesnt accept the same axis parameters as the trinamic motors... to make
        // it work we simply ignore that.
        if !(response.byte2 == 100 || response.byte2 == 101 || response.byte2 == 3) {
            anyhow::bail!("Unexpected status byte: {response}, message sent: {message} ");
        }

        Ok(response)
    }

    pub async fn query_position(&mut self) -> Result<i32> {
        let arg = AxisParameters::ActualPosition;
        let rq = Message::ax_param(1, arg, 0, 0);
        let response = self.request(&rq).await?;
        Ok(response.value)
    }

    pub async fn move_to_absolute_position(&mut self, value: i32) -> Result<()> {
        let rq = Message::new(1, 4, 0, 0, value);
        self.request(&rq).await?;
        Ok(())
    }

    pub async fn ax_param(&mut self, arg: AxisParameters) -> Result<i32> {
        let rq = Message::ax_param(1, arg, 0, 0);
        let response = self.request(&rq).await?;
        Ok(response.value)
    }
}
