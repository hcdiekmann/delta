/*-------------- Imports --------------*/
const { Client } = require('ads-client');
const express = require('express');
const path = require('path');
const Datastore = require('@seald-io/nedb');

/* webserver hosting the simulation (localhost/index.html) */
const PORT = process.env.PORT || 3000;
const app = express();
app.use(express.static(path.join(__dirname, 'localhost'), { index: 'index.HTML' }));
app.use(express.json({ limit: '1mb' })); // json parser for fetch requests limit could be increased

/* database file for logging server status and ads errors */
const logger = new Datastore({ filename: path.join(__dirname, 'serverLog.db'), autoload: true });
const log = (entry) => logger.insert({ time: new Date(), entry: entry instanceof Error ? entry.message : entry });

/* ads client, created on /connect */
let client = null;
const isConnected = () => client !== null && client.connection.connected;

/*----------------------------------------------------------------------------------------------------*/

// handle frontend connect post request
app.post('/connect', async (request, response) => {
    client = new Client({
        targetAmsNetId: request.body.AmsNetID,
        targetAdsPort: Number(request.body.AdsPort)
    });
    try {
        const res = await client.connect();
        const info = `Connected to ${res.targetAmsNetId}, local assigned router has AmsNetId ${res.localAmsNetId} at port ${res.localAdsPort}`;
        console.log(info);
        log(info);
        response.sendStatus(200);
    }
    catch (err) {
        console.log('Error:', err.message);
        log(err);
        client = null;
        response.status(500).send('Could not connect to PLC, check AmsNetID and port');
    }
});

// send configuration file
app.get('/config', (request, response) => {
    response.sendFile(path.join(__dirname, 'config files', 'installationsettings.xml'));
});

app.get('/connected', (request, response) => {
    response.sendStatus(isConnected() ? 200 : 500);
});

app.get('/disconnect', async (request, response) => {
    if (client === null) return response.sendStatus(200);
    try {
        await client.disconnect();
        log('Disconnected from PLC');
        response.sendStatus(200);
    }
    catch (err) {
        console.log('Error:', err.message);
        log(err);
        response.status(500).send(err.message);
    }
    finally {
        client = null;
    }
});

// runtime state of the PLC (Run, Config, Stop...)
app.get('/state', async (request, response) => {
    if (!isConnected()) return response.status(500).send('Client not connected to PLC');
    try {
        response.json(await client.readPlcRuntimeState());
    }
    catch (err) {
        log(err);
        response.status(500).json({ error: err.message });
    }
});

// motor positions MAIN.POS0 - MAIN.POS2
for (let i = 0; i < 3; i++) {
    app.get(`/pos${i}`, async (request, response) => {
        if (!isConnected()) return response.status(500).send('Client not connected to PLC');
        try {
            const res = await client.readValue(`MAIN.POS${i}`);
            response.json({ value: res.value });
        }
        catch (err) {
            console.log('Error:', err.message);
            log(err);
            response.status(500).json({ error: err.message });
        }
    });
}

app.listen(PORT, (err) => {
    if (err) {
        console.error(`Could not start server on port ${PORT}: ${err.message}`);
        console.error('Choose another port, e.g. PORT=3001 npm start');
        process.exit(1);
    }
    console.log(`Server running at http://localhost:${PORT}/`);
});
