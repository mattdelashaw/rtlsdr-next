fn main() {
    // Only compile the V4 bridge C source code if the `bench-c` feature is enabled.
    // This prevents breaking normal builds on systems without a C compiler and
    // ensures the production binary remains lean.
    if std::env::var("CARGO_FEATURE_BENCH_C").is_ok() {
        cc::Build::new()
            .file("benches/librtlsdr_v4.c")
            .compile("librtlsdr_v4");

        // Re-run the build ONLY if the C source changes
        println!("cargo:rerun-if-changed=benches/librtlsdr_v4.c");
    }
}
