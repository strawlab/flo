//! copied from spec pdf and then edited using
//! regex replace
#![allow(unused)]
pub const CMD_READ_PARAMS: u8 = 82;
pub const CMD_WRITE_PARAMS: u8 = 87;
pub const CMD_REALTIME_DATA: u8 = 68;
pub const CMD_BOARD_INFO: u8 = 86;
pub const CMD_CALIB_ACC: u8 = 65;
pub const CMD_CALIB_GYRO: u8 = 103;
pub const CMD_CALIB_EXT_GAIN: u8 = 71;
pub const CMD_USE_DEFAULTS: u8 = 70;
pub const CMD_CALIB_POLES: u8 = 80;
pub const CMD_RESET: u8 = 114;
pub const CMD_HELPER_DATA: u8 = 72;
pub const CMD_CALIB_OFFSET: u8 = 79;
pub const CMD_CALIB_BAT: u8 = 66;
pub const CMD_MOTORS_ON: u8 = 77;
pub const CMD_MOTORS_OFF: u8 = 109;
pub const CMD_CONTROL: u8 = 67;
pub const CMD_TRIGGER_PIN: u8 = 84;
pub const CMD_EXECUTE_MENU: u8 = 69;
pub const CMD_GET_ANGLES: u8 = 73;
pub const CMD_CONFIRM: u8 = 67;
pub const CMD_BOARD_INFO_3: u8 = 20;
pub const CMD_READ_PARAMS_3: u8 = 21;
pub const CMD_WRITE_PARAMS_3: u8 = 22;
pub const CMD_REALTIME_DATA_3: u8 = 23;
pub const CMD_REALTIME_DATA_4: u8 = 25;
pub const CMD_SELECT_IMU_3: u8 = 24;
pub const CMD_READ_PROFILE_NAMES: u8 = 28;
pub const CMD_WRITE_PROFILE_NAMES: u8 = 29;
pub const CMD_QUEUE_PARAMS_INFO_3: u8 = 30;
pub const CMD_SET_ADJ_VARS_VAL: u8 = 31;
pub const CMD_SAVE_PARAMS_3: u8 = 32;
pub const CMD_READ_PARAMS_EXT: u8 = 33;
pub const CMD_WRITE_PARAMS_EXT: u8 = 34;
pub const CMD_AUTO_PID: u8 = 35;
pub const CMD_SERVO_OUT: u8 = 36;
pub const CMD_I2C_WRITE_REG_BUF: u8 = 39;
pub const CMD_I2C_READ_REG_BUF: u8 = 40;
pub const CMD_WRITE_EXTERNAL_DATA: u8 = 41;
pub const CMD_READ_EXTERNAL_DATA: u8 = 42;
pub const CMD_READ_ADJ_VARS_CFG: u8 = 43;
pub const CMD_WRITE_ADJ_VARS_CFG: u8 = 44;
pub const CMD_API_VIRT_CH_CONTROL: u8 = 45;
pub const CMD_ADJ_VARS_STATE: u8 = 46;
pub const CMD_EEPROM_WRITE: u8 = 47;
pub const CMD_EEPROM_READ: u8 = 48;
pub const CMD_CALIB_INFO: u8 = 49;
pub const CMD_SIGN_MESSAGE: u8 = 50;
pub const CMD_BOOT_MODE_3: u8 = 51;
pub const CMD_SYSTEM_STATE: u8 = 52;
pub const CMD_READ_FILE: u8 = 53;
pub const CMD_WRITE_FILE: u8 = 54;
pub const CMD_FS_CLEAR_ALL: u8 = 55;
pub const CMD_AHRS_HELPER: u8 = 56;
pub const CMD_RUN_SCRIPT: u8 = 57;
pub const CMD_SCRIPT_DEBUG: u8 = 58;
pub const CMD_CALIB_MAG: u8 = 59;
pub const CMD_GET_ANGLES_EXT: u8 = 61;
pub const CMD_READ_PARAMS_EXT2: u8 = 62;
pub const CMD_WRITE_PARAMS_EXT2: u8 = 63;
pub const CMD_GET_ADJ_VARS_VAL: u8 = 64;
pub const CMD_CALIB_MOTOR_MAG_LINK: u8 = 74;
pub const CMD_GYRO_CORRECTION: u8 = 75;
pub const CMD_DATA_STREAM_INTERVAL: u8 = 85;
pub const CMD_REALTIME_DATA_CUSTOM: u8 = 88;
pub const CMD_BEEP_SOUND: u8 = 89;
pub const CMD_ENCODERS_CALIB_OFFSET_4: u8 = 26;
pub const CMD_ENCODERS_CALIB_FLD_OFFSET_4: u8 = 27;
pub const CMD_CONTROL_CONFIG: u8 = 90;
pub const CMD_CALIB_ORIENT_CORR: u8 = 91;
pub const CMD_COGGING_CALIB_INFO: u8 = 92;
pub const CMD_CALIB_COGGING: u8 = 93;
pub const CMD_CALIB_ACC_EXT_REF: u8 = 94;
pub const CMD_PROFILE_SET: u8 = 95;
pub const CMD_CAN_DEVICE_SCAN: u8 = 96;
pub const CMD_CAN_DRV_HARD_PARAMS: u8 = 97;
pub const CMD_CAN_DRV_STATE: u8 = 98;
pub const CMD_CAN_DRV_CALIBRATE: u8 = 99;
pub const CMD_READ_RC_INPUTS: u8 = 100;
pub const CMD_REALTIME_DATA_CAN_DRV: u8 = 101;
pub const CMD_EVENT: u8 = 102;
pub const CMD_READ_PARAMS_EXT3: u8 = 104;
pub const CMD_WRITE_PARAMS_EXT3: u8 = 105;
pub const CMD_EXT_IMU_DEBUG_INFO: u8 = 106;
pub const CMD_SET_DEVICE_ADDR: u8 = 107;
pub const CMD_AUTO_PID2: u8 = 108;
pub const CMD_EXT_IMU_CMD: u8 = 110;
pub const CMD_READ_STATE_VARS: u8 = 111;
pub const CMD_WRITE_STATE_VARS: u8 = 112;
pub const CMD_SERIAL_PROXY: u8 = 113;
pub const CMD_IMU_ADVANCED_CALIB: u8 = 115;
pub const CMD_API_VIRT_CH_HIGH_RES: u8 = 116;
pub const CMD_SET_DEBUG_PORT: u8 = 249;
pub const CMD_MAVLINK_INFO: u8 = 250;
pub const CMD_MAVLINK_DEBUG: u8 = 251;
pub const CMD_DEBUG_VARS_INFO_3: u8 = 253;
pub const CMD_DEBUG_VARS_3: u8 = 254;
pub const CMD_ERROR: u8 = 255;
