extern crate platform_uuid;

fn main() {
    let uuid = platform_uuid::get_uuid().expect("failed getting uuid");
    println!("{}", String::from_utf8_lossy(&uuid));
}
