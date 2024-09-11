use winreg::enums::*;
use winreg::RegKey;

fn strip_err(_orig: std::io::Error) -> crate::Error {
    crate::Error {}
}

pub fn get_uuid() -> Result<Vec<u8>, crate::Error> {
    let hklm = RegKey::predef(HKEY_LOCAL_MACHINE);
    let crypt = hklm
        .open_subkey("SOFTWARE\\Microsoft\\Cryptography")
        .map_err(strip_err)?;
    let guid: String = crypt.get_value("MachineGuid").map_err(strip_err)?;
    Ok(guid.as_bytes().to_vec())
}
