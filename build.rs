fn main() {
    // Указываем Cargo пересобирать проект, если изменился файл настроек
    println!("cargo:rerun-if-changed=sdkconfig.defaults");
    
    embuild::espidf::sysenv::output();
}