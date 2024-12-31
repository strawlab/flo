// TODO: use https://crates.io/crates/cocoa instead of C code?

extern "C" {
    // get_platform_uuid is defined in uuid-macos.cpp
    fn get_platform_uuid(buf: *mut u8, buf_size: libc::c_int) -> u8;
}

pub fn get_uuid() -> Result<Vec<u8>, crate::Error> {
    let mut buf: Vec<u8> = vec![0; 256];
    let len = buf.len() as i32;
    unsafe {
        if get_platform_uuid(buf.as_mut_ptr(), len) == 0 {
            return Err(crate::Error {});
        }
    }

    // trim from first null
    for i in 0..buf.len() {
        if buf[i] == 0 {
            buf.truncate(i);
            return Ok(buf);
        }
    }
    Ok(buf)
}
