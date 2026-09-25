const MAX_NUM_ROBOTS = 10;

//read the configuration file
async function readConfig(){
    const robot_list = [];
    let parser = new DOMParser();
    await fetch('/config')
    .then(res => res.text())
    .then(data=>{
        let xml = parser.parseFromString(data, "text/xml");
        for(let i=1; i<MAX_NUM_ROBOTS; i++){
            let pos = xml.querySelector(`PositionOfRobots${i}`);
            if(pos == undefined){
                break;
            }
            let reach = xml.querySelector(`RobotReach${i}`);
            let accPltoPi = xml.querySelector(`AccelerationPlaceToPick${i}`);
            let accPitoPi = xml.querySelector(`AccelerationPickToPick${i}`);
            let accPitoPl = xml.querySelector(`AccelerationPickToPlace${i}`);
            let robot = {
                "id": i,
                "type": "deltaRobot",
                "pos": pos.attributes.value.nodeValue,
                "reach":  reach.attributes.value.nodeValue,
                "AccelerationPlaceToPick": accPltoPi.attributes.value.nodeValue,
                "AccelerationPickToPick" : accPitoPi.attributes.value.nodeValue,
                "AccelerationPickToPlace" : accPitoPl.attributes.value.nodeValue
            };
            robot_list.push(robot);
        }
    });
    console.log(robot_list);
    return robot_list;
}

// connect or disconnect PLC (linked to btn click)
async function ConnectPLC(){
  let state = document.getElementById('connectState').textContent;
  if(state == "🟢online") {
    let response = await fetch('/disconnect');
    if(response.ok){
      ConnectionStatus();
    }
  }
  else {
    AmsNetID = document.getElementById('netId').value;
    AdsPort = document.getElementById('port').value;
    if (AmsNetID == '' || AdsPort == ''){ return; }

    const connectInfo = { AmsNetID, AdsPort };
      const options = {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(connectInfo)
      };
      let response = await fetch('/connect', options);
      if(response.ok){
        readConfig();
        ConnectionStatus();
      }
  }
}

// get connection state of ads client
async function ConnectionStatus(){
  let onlinelbl = document.getElementById('connectState');
  let statelbl = document.getElementById('AdsState');
  let response = await fetch('/connected');
  if(response.ok){
    onlinelbl.textContent  = "🟢online";
    hideConnectInputUI();
    ADSstate();
  }
  else{
    onlinelbl.textContent = "🔴offline";
    statelbl.textContent = "STATE: "
    statelbl.style.display = "none"; 
    showConnectInputUI();
  }
}

// get ads/runtime state of PLC
async function ADSstate(){
  let response = await fetch('/state');
  let data = await response.json(); // parse response to json
  let lbl = document.getElementById('AdsState');
  if(response.ok) {
    lbl.textContent = "STATE: " + data.adsStateStr;
    lbl.style.display = "inline-block"; 
  }
}

async function GetPos0(){
  const response = await fetch('/pos0');
  const data = await response.json(); // parse response to json
  let pos = Number(data.value);
  return pos;
}

async function GetPos1(){
  const response = await fetch('/pos1');
  const data = await response.json(); // parse response to json
  let pos = Number(data.value);
  return pos;
}

async function GetPos2(){
  const response = await fetch('/pos2');
  const data = await response.json(); // parse response to json
  let pos = Number(data.value);
  return pos;
}

