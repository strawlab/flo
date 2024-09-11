use std::io::Read;

pub fn get_uuid() -> Result<Vec<u8>, crate::Error> {
    let mut buf = Vec::new();
    let mut fd = std::fs::File::open("/etc/machine-id").map_err(|_e| crate::Error {})?;
    fd.read_to_end(&mut buf).map_err(|_e| crate::Error {})?;
    Ok(buf)
}
