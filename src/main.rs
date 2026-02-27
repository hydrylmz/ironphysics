mod math;
use math::vec2::Vector2;

fn main() {
    let mut vec1 = Vector2::new(2.0, 7.0);
    let _vec2 = Vector2::new(6.0, 3.0);
    let length = vec1.length();
    println!("The initial length of vec1: {}", length);
    vec1.normalize();
    let length = vec1.length();
    println!("The length of vec1 after normalization: {}", length);
}
