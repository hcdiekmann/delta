import { runBenchmark, type BenchmarkRequest } from '@/core/benchmark';

// Runs the headless simulations off the main thread so the 3D view stays smooth.
self.onmessage = (e: MessageEvent<BenchmarkRequest>) => {
  const results = runBenchmark(e.data, (done, total) => self.postMessage({ type: 'progress', done, total }));
  self.postMessage({ type: 'done', results });
};
