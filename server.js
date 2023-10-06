/*-------------- Imports --------------*/
const ads = require('ads-client');      
const { response } = require('express');
const express = require('express');     
const fs = require('fs');               
const xmlparser = require('express-xml-bodyparser');
const Datastore = require('nedb');     
const socket = require('socket.io');   

/* webserver hosting index.html at port 3000 locally */
const PORT = 8000;
const app = express();
const server = app.listen(PORT, function() {
    console.log(`Starting server at http://localhost:${PORT}/`);
    console.log(`Listening on port ${PORT}`) });
app.use(express.static('localhost'));   // folder to for static files
app.use(express.json({ limit: '1mb'})); // json parser for fetch requests limit could be increased

/* websocket setup */
const io = socket(server);

/* database file for logging server status and ads errors */
const logger = new Datastore('serverLog.db');
logger.loadDatabase();

/*----------------------------------------------------------------------------------------------------*/


// handle frontend connect post request
app.post('/connect', (request, response) => {
    global.client = new ads.Client({            // global client variable, not sure if this is smart?
        targetAmsNetId: request.body.AmsNetID,  
        targetAdsPort: request.body.AdsPort
    });
    client.connect()
        .then(res => {
            let info = `Connected to ${res.targetAmsNetId} , local assigned router has AmsNetId ${res.localAmsNetId} at port ${res.localAdsPort}`;
            console.log(info);
            logger.insert(info);
            //Subscribe();
            response.sendStatus(200, "Connected successfully");
        })
        .catch(err => {
            console.log('Error:', err);
            logger.insert(err);
            response.sendStatus(500, "Could not connect to PLC, check AmsNetID and port");
        })
});

//send configuration file 
app.get('/config', (req, res) => {
    res.contentType('application/xml');
    res.sendFile(__dirname + '/config files/installationsettings.xml')
});

/* io.sockets.on("connection", function (socket) {
    console.log("Made socket connection");
    socket.on()
}); */

/* Subscribe to PLC variables (called once after connecting) */
/* async function Subscribe() {
    try {
        let PosXSub = await client.subscribe('MAIN.POSX', subCallback, 10, true);
        let PosYSub = await client.subscribe('MAIN.POSY', subCallback, 10, true);
        let PosZSub = await client.subscribe('MAIN.POSZ', subCallback, 10, true);

        console.log(`Subscribed to ${PosXSub.target}`);
        console.log(`Subscribed to ${PosYSub.target}`);
        console.log(`Subscribed to ${PosZSub.target}`);
    }
    catch (err) { 
        console.log('Error:', err);
        logger.insert(err);
    }
} */

// variable subscriptions callback function
const subCallback = (data, sub) => {
    try {
        let info = `${data.timeStamp}: ${sub.target} changed to ${data.value}`;
        console.log(info);
        logger.insert(info);
    }
    catch (err) {
        console.log('Error:', err)
    }
    finally {
        //We can call sub.unsubscribe() here to remove the subscriptions if we want
    }
}

app.get('/connected', (request, response) => {
    try {
       (client.connection.connected ? response.sendStatus(200) : response.sendStatus(500) );
    }
    catch {
        console.log("Connection status read without client object");
        response.sendStatus(500, "ads client does not exist, try connecting first");
    }
});

app.get('/disconnect', (request, response) => {
    client.disconnect()
    .then(res =>{
        response.sendStatus(200, "Disconnected from PLC successfully");
        logger.insert(res);
    })
    .catch(err =>{
        console.log('Error:', err);
        logger.insert(err);
        response.sendStatus(500, err);
    })
});

// handle frontend fetch get requests
app.get('/state', (request, response) => {
    if (client.connection.connected) {
        client.readPlcRuntimeState() //send the ads response as the servers response
        .then(res =>{
            response.json(res);
        })
        .catch(err =>{
            logger.insert(err);
            response.json(err);
        })
    }
    else {
        response.sendStatus(500, "Client not connected to PLC");
    }
});

app.get('/pos0', (request, response) => {
    client.readSymbol("MAIN.POS0")
    .then(res =>{
        response.json(res);
    })
    .catch(err =>{
        console.log('Error:', err);
        logger.insert(err);
    })
});

app.get('/pos1', (request, response) => {
    client.readSymbol("MAIN.POS1")
    .then(res =>{
        response.json(res);
    })
    .catch(err =>{
        console.log('Error:', err);
        logger.insert(err);
    })
});

app.get('/pos2', (request, response) => {
    client.readSymbol("MAIN.POS2")
    .then(res =>{
        response.json(res);
    })
    .catch(err =>{
        console.log('Error:', err);
        logger.insert(err);
    })
});

