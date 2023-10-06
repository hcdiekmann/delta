const display = new Display();
const inputBox = new Case({x:-0.5,y:-1.5,z:-1.25},{},{x:0.75,y:0.75,z:0.35});
const outputbox = new Case({x:4,y:-1.5,z:-1.5},{},{});

const amountProducts = 100;

const curved = false;
const boxOnConveyor = false;

const conveyor1 = new Conveyor({x:0,y:-1.5,z:-1},{x:1.5,y:0,vector:true},{x:1,y:3.5});
const conveyor2 = new Conveyor({x:1.5},{x:1,y:0.5,vector:true},{x:1,y:1});
const conveyor3 = new Conveyor({x:2.375,y:0.45},{x:1,y:1,vector:true},{x:1,y:1});
const conveyor4 = new Conveyor({x:3.05,y:1.15},{x:0.5,y:1,vector:true},{x:1,y:1});
const conveyor5 = new Conveyor({x:3.5,y:2},{x:0,y:1,vector:true},{x:1,y:1});
const conveyor6 = new Conveyor({x:4,y:3.5,z:-0.6},{x:-1,y:0,vector:true},{x:1,y:10});

function setup() {  // called on system startup
    //frameRate(5);

    display.addObject(new Origin({},{},{}));  // 3 empty objects as parameters, default value's will be 0

    display.addObject(new DeltaRobot({},{},{},{upperArmLength:0.6,lowerArmLength:1}));
    display.deltaRobotArray[0].setColor(0, 0, 255);
	display.addObject(new DeltaRobot({},{},{},{upperArmLength:0.4,lowerArmLength:0.65}));
    display.deltaRobotArray[1].setColor(0, 255, 0);

    // display.deltaRobotArray[0].position.set(0,0,0); // to put the deltarobot on the conveyor
    // display.deltaRobotArray[0].Connect(conveyor1);

    //let stepperMotor = new StepperMotor({x:-3,z:-0.5},{},{});
    //display.addObject(stepperMotor);
    //stepperMotor.enable = true;

    display.addObject(conveyor1);
    if (curved)
    {
        conveyor1.nextParent = conveyor2;
        display.addObject(conveyor2);
        conveyor2.nextParent = conveyor3;
        display.addObject(conveyor3);
        conveyor3.nextParent = conveyor4;
        display.addObject(conveyor4);
        conveyor4.nextParent = conveyor5;
        display.addObject(conveyor5);
    }
    display.addObject(inputBox);
    display.addObject(outputbox);
    
    if (!curved)
    {
        conveyor1.nextParent = outputbox;
    }else
    {
        conveyor5.nextParent = outputbox;
        outputbox.position.x = 3.5;
        outputbox.position.y = 3.5;
    }

    if (boxOnConveyor)
    {
        display.addObject(conveyor6);
        outputbox.Connect(conveyor6);
        conveyor6.speed = 1.5;
    }

    for (let i = 0; i < amountProducts; i++) {
        const clone = new Product({},{},{});
        clone.Connect(inputBox);
        display.addObject(clone);
    }
}

const interval = setInterval(function() {
    document.getElementById("framerate").innerHTML = "FPS: " + Math.floor(frameRate() *10) / 10;
}, 100);

let conveyorPos = 0;
function addProduct() {
    const product = new Product({cPos:conveyorPos},{vector:true},{});
    display.addObject(product);
    product.Connect(conveyor1);  // connect product to conveyor
}

// places it on the conveyor
function MoveProduct()
{
    if (inputBox.connectedProducts.length > 0)
    {
        inputBox.connectedProducts[inputBox.connectedProducts.length - 1].Connect(conveyor1);
    }
}

function DemoCSVMoveRobot()
{
    if (display.deltaRobotArray[0].movementState === movementState.Idle) {
        display.deltaRobotArray[0].movementState = movementState.ReadCSV;	
    }
    else {
        display.deltaRobotArray[0].movementState = movementState.Idle;	
    }
}

function DemoRepeatMoveRobot() {
    // toggle
    if (display.deltaRobotArray[0].movementState === movementState.Idle) {
        display.deltaRobotArray[0].movementState = movementState.Repeat;	
    }
    else {
        display.deltaRobotArray[0].movementState = movementState.Idle;	
    }
}

function DemoSingularMoveRobot() {
    let slider = null, 
    slider3 = null,
    slider2 = null;
    
    if (slider === null)
    {
        slider = document.getElementById("motor1");
    }
    if (slider2 === null)
    {
        slider2 = document.getElementById("motor2");
    }
    if (slider3 === null)
    {
        slider3 = document.getElementById("motor3");
    }

    console.log("pos0: " + display.deltaRobotArray[0].demoTargetAngle[0]);
    console.log("pos1: " + display.deltaRobotArray[0].demoTargetAngle[1]);
    console.log("pos2: " + display.deltaRobotArray[0].demoTargetAngle[2]);
    display.deltaRobotArray[0].demoTargetAngle[0] = (slider.value / 100 * 3 - 1.5);
    display.deltaRobotArray[0].demoTargetAngle[1] = (slider2.value / 100 * 3 - 1.5);
    display.deltaRobotArray[0].demoTargetAngle[2] = (slider3.value / 100 * 3 - 1.5);
    display.deltaRobotArray[0].movementState = movementState.Singular;
}

async function DemoPLCMoveRobot(){
    let state = document.getElementById('connectState').textContent;
    if(state == "🟢online") {
        //let config = await readConfig();
        //LoadConfiguration(config);
        let pos0 = await GetPos0();
        let pos1 = await GetPos1();
        let pos2 = await GetPos2();
        display.deltaRobotArray[1].actuatorList[0].SetValue(pos0, 250);
        display.deltaRobotArray[1].actuatorList[1].SetValue(pos1, 250);
        display.deltaRobotArray[1].actuatorList[2].SetValue(pos2, 250, DemoPLCMoveRobot); //callback recursion
    }
    else {
        return;
    }
}

function SetProductSpeed(speed)
{
    productspeed = speed;
}

function SetConveyorSpeed(speed)
{
    conveyor1.speed = speed;
    conveyor2.speed = speed;
    conveyor3.speed = speed;
    conveyor4.speed = speed;
    conveyor5.speed = speed;
}

function SetConveyorPos(pos)
{
    conveyorPos = pos;
}

function toggleReadCSVState()
{
    if(modeCSV === ModeCSV.Repeat) {
        modeCSV = ModeCSV.Singular;
        document.getElementById("btn-changeCSVState").innerText = "Singular";
    } else if(modeCSV === ModeCSV.Singular){
        modeCSV = ModeCSV.Repeat;
        document.getElementById("btn-changeCSVState").innerText = "Repeat";
    }
}

function changeShowReach() {
    if(showKinematicReach) {
        showKinematicReach = false;
        document.getElementById("btn-changeSHOWReachState").innerText = "No Kinematic Reach";
    } else {
        showKinematicReach = true;
        document.getElementById("btn-changeSHOWReachState").innerText = "Kinematic Reach";
    }
}