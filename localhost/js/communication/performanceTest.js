// record all times
const times = [];

function call() {
    // record starting time
    const startFetch = performance.now();
    fetch('/')
        .then((response) => {
            // compute fetch duration
            const elapsedFetch = performance.now() - startFetch;

            // record result
            console.log(elapsedFetch);
            times.push(elapsedFetch);

            if (times.length<100) {
                // start next call
                call();
            } else {
                // report statistics
                const totalFetch = times.reduce((a, b) => a + b, 0);
                const averageFetch = totalFetch/times.length;
                const standardDeviation = Math.sqrt(times.reduce((a, b) => a + (b-averageFetch) ** 2, 0)/times.length);
                const totalElapsed = performance.now() - startTime;
                console.log("Average fetch time:", averageFetch, '+-', standardDeviation);
                console.log("Percentage of overall elapsed:", totalFetch/totalElapsed)
            }
        });
}

startTime = performance.now();
//call(); // uncomment here to test the connection performance to server