use criterion::{BenchmarkId, Criterion, black_box, criterion_group, criterion_main};
use rtlsdr_next::converter;
use rtlsdr_next::dsp::Decimator;

fn bench_converter(c: &mut Criterion) {
    let size = 256 * 1024; // 256KB block
    let src = vec![127u8; size];
    let mut dst = vec![0.0f32; size];

    let mut group = c.benchmark_group("Converter");
    group.throughput(criterion::Throughput::Bytes(size as u64));

    group.bench_function("u8_to_f32", |b| {
        b.iter(|| {
            converter::convert(black_box(&src), black_box(&mut dst));
        })
    });
    group.finish();
}

fn bench_decimator(c: &mut Criterion) {
    let size = 256 * 1024;
    let input = vec![0.0f32; size];

    let mut group = c.benchmark_group("Decimator");
    group.throughput(criterion::Throughput::Elements(size as u64));

    for factor in [4, 8, 16] {
        let mut dec = Decimator::with_factor(factor);
        group.bench_with_input(BenchmarkId::new("fir_decimate", factor), &factor, |b, _| {
            b.iter(|| {
                let _ = dec.process(black_box(&input));
            })
        });
    }
    group.finish();
}

criterion_group!(benches, bench_converter, bench_decimator);
criterion_main!(benches);
