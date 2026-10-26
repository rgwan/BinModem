fn main() {
    let set = datapump::v92::codegrid::grid(64).unwrap();
    println!("{:?}", set);
    println!("max {}", set[63]);
}
