extern crate libc;

#[cfg(target_os = "windows")]
extern crate winreg;

#[cfg(target_os = "windows")]
mod windows;
#[cfg(target_os = "windows")]
pub use crate::windows::get_uuid;

#[cfg(target_os = "macos")]
mod macos;
#[cfg(target_os = "macos")]
pub use crate::macos::get_uuid;

#[cfg(target_os = "linux")]
mod linux;
#[cfg(target_os = "linux")]
pub use linux::get_uuid;

// For windows, see
// https://stackoverflow.com/questions/49488624/how-to-get-a-computer-specific-id-number-using-java
// also
// https://social.technet.microsoft.com/Forums/en-US/c92e2d2d-3c19-4cb8-a153-1e4abcadfe6c/platform-uuid-os-activation-cd-keys-and-stuff-?forum=windowsinternals

#[derive(Debug)]
pub struct Error {}

impl std::error::Error for Error {}
impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> Result<(), std::fmt::Error> {
        write!(f, "platform-uuid error")
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        let uuid = crate::get_uuid().unwrap();
        println!("uuid = {}", String::from_utf8_lossy(&uuid));
    }
}
