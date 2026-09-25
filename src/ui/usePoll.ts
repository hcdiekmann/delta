import { useEffect, useState } from 'react';

/** Re-render at a fixed rate to show values that live outside React (the running simulation). */
export function usePoll(hz = 10) {
  const [, setTick] = useState(0);
  useEffect(() => {
    const id = setInterval(() => setTick((t) => t + 1), 1000 / hz);
    return () => clearInterval(id);
  }, [hz]);
}
