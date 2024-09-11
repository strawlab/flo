REM Install wasm-pack from here https://rustwasm.github.io/wasm-pack/installer/
wasm-pack build --target web

REM Install grass with: cargo install grass
grass scss/style.scss pkg/style.css

copy static\index.html pkg
